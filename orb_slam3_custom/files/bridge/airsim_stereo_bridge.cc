#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <cstring>      // ✅ added (strlen)
#include <cstdlib>      // getenv
#include <filesystem>
#include <cstdio>
#include <fcntl.h>   // ✅ REQUIRED for fcntl, F_GETFL, O_NONBLOCK
#include <opencv2/opencv.hpp>

#include <Eigen/Core>   // ✅ added (Vector3f)

#include "System.h"

// AirSim
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

using std::cout;
using std::cerr;
using std::endl;
constexpr float MAP_SCALE = 20.0f; // pixels per meter
bool g_localization_only = false;

// ---------- Global stop flag (Ctrl+C safe shutdown) ----------
static std::atomic<bool> g_stop(false);
static void signalHandler(int)
{
    g_stop = true;
}
int createUdpRecvSocket(int port)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("cmd socket"); exit(1); }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("cmd bind"); exit(1);
    }

    // non-blocking
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    return sock;
}


// ---------- Convert AirSim image to OpenCV ----------
static cv::Mat airsimResponseToMat(
    const msr::airlib::ImageCaptureBase::ImageResponse& resp)
{
    if (resp.image_data_uint8.empty() || resp.width == 0 || resp.height == 0)
        return cv::Mat();

    cv::Mat img(resp.height, resp.width, CV_8UC3,
                (void*)resp.image_data_uint8.data());

    cv::Mat img_clone = img.clone();
    cv::cvtColor(img_clone, img_clone, cv::COLOR_RGB2BGR);
    return img_clone;
}

int createUdpSocket(const std::string& ip, int port, sockaddr_in& addr)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("UDP socket creation failed");
        exit(1);
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);

    return sock;
}
bool has_origin = false;
Eigen::Vector3f origin;

// Print stereo baseline once (computed from AirSim-reported camera positions in ImageResponse).
static bool g_printed_baseline = false;

int main(int argc, char** argv)
{
    static const std::string LIVE_KF_FILE = "live_keyframes.tum";
    static const std::string LIVE_KF_TMP  = "live_keyframes.tum.tmp";

    if (argc < 3)
    {
        cerr << "Usage:\n"
             << argv[0] << " <ORBvoc.txt> <settings.yaml> [vehicle_name]\n";
        return 1;
    }

    std::signal(SIGINT, signalHandler);

    const std::string vocab_path    = argv[1];
    const std::string settings_path = argv[2];
    const std::string vehicle_name  = (argc >= 4) ? argv[3] : "";

    // ---------- Start ORB-SLAM3 (STEREO, no viewer) ----------
    ORB_SLAM3::System SLAM(
        vocab_path,
        settings_path,
        ORB_SLAM3::System::STEREO,
        false   // viewer disabled
    );

    sockaddr_in udp_addr;
    int udp_sock = createUdpSocket("127.0.0.1", 5005, udp_addr);

    // ✅ Step 2.1: stable origin (set once on first valid pose)
    

    // ---------- Connect to AirSim ----------
    msr::airlib::MultirotorRpcLibClient client("172.20.144.1");
    client.confirmConnection();

    cout << "Connected to AirSim" << endl;

    // AirSim camera selection:
    // Prefer explicit names (stable) over numeric indices (order-dependent).
    // You can override with env vars if your settings.json uses different names.
    const char* left_cam_env = std::getenv("AIRSIM_LEFT_CAM");
    const char* right_cam_env = std::getenv("AIRSIM_RIGHT_CAM");
    std::string left_cam  = (left_cam_env && std::strlen(left_cam_env)) ? std::string(left_cam_env) : "StereoCameraLeft";
    std::string right_cam = (right_cam_env && std::strlen(right_cam_env)) ? std::string(right_cam_env) : "StereoCameraRight";
    bool cams_fallback_to_indices = false;
    std::cout << "AirSim cameras (preferred): left='" << left_cam << "' right='" << right_cam << "'\n";

    auto build_requests = [&](const std::string& l, const std::string& r) {
        msr::airlib::vector<msr::airlib::ImageCaptureBase::ImageRequest> req;
        req.emplace_back(l, msr::airlib::ImageCaptureBase::ImageType::Scene, false, false);
        req.emplace_back(r, msr::airlib::ImageCaptureBase::ImageType::Scene, false, false);
        return req;
    };

    auto requests = build_requests(left_cam, right_cam);

    auto t0 = std::chrono::steady_clock::now();
    auto last_print = t0;
    int frame_count = 0;
    int cmd_sock = createUdpRecvSocket(6007);
bool localization_only = false;
auto last_dump = std::chrono::steady_clock::now();

    // ---------- Main SLAM loop ----------
    while (!g_stop)
    {
        char cmd_buf[256];
int n = recv(cmd_sock, cmd_buf, sizeof(cmd_buf)-1, 0);
if (n > 0) {
    cmd_buf[n] = '\0';
    std::string cmd(cmd_buf);

    if (cmd.find("localization") != std::string::npos) {
        localization_only = true;
        SLAM.ActivateLocalizationMode();
        std::cout << "🔒 SLAM localization-only\n";
    } 
    else if (cmd.find("mapping") != std::string::npos) {
        localization_only = false;
        SLAM.DeactivateLocalizationMode();
        std::cout << "🗺️ SLAM mapping mode\n";
    }
}

        msr::airlib::vector<msr::airlib::ImageCaptureBase::ImageResponse> responses;
        try {
            responses = client.simGetImages(requests, vehicle_name);
        } catch (const std::exception& e) {
            // Common cause: invalid camera name (AirSim settings.json doesn't contain it).
            // Fall back to numeric indices once to preserve old behavior.
            if (!cams_fallback_to_indices) {
                cams_fallback_to_indices = true;
                left_cam = "0";
                right_cam = "1";
                requests = build_requests(left_cam, right_cam);
                std::cerr << "AirSim simGetImages failed for named cameras; falling back to indices left='0' right='1'. Error: " << e.what() << "\n";
            } else {
                std::cerr << "AirSim simGetImages failed (still). Error: " << e.what() << "\n";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
            continue;
        }

        if (responses.size() < 2)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }

        if (!g_printed_baseline) {
            try {
                const auto& p0 = responses[0].camera_position;
                const auto& p1 = responses[1].camera_position;
                // AirSim client in this build exposes camera_position as an Eigen vector.
                const double dx = double(p1.x()) - double(p0.x());
                const double dy = double(p1.y()) - double(p0.y());
                const double dz = double(p1.z()) - double(p0.z());
                const double b = std::sqrt(dx * dx + dy * dy + dz * dz);
                std::cout
                    << "AirSim stereo baseline (from ImageResponse camera_position): "
                    << b << " m"
                    << " (dx=" << dx << ", dy=" << dy << ", dz=" << dz << ")\n";
                g_printed_baseline = true;
            } catch (...) {
                // ignore
            }
        }

        cv::Mat left  = airsimResponseToMat(responses[0]);
        cv::Mat right = airsimResponseToMat(responses[1]);
        if (left.empty() || right.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - t0).count();

        Sophus::SE3f Tcw = SLAM.TrackStereo(left, right, t);

        // Keep your “valid pose” check style
	if (!Tcw.matrix().isZero())
{
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f p = Twc.translation();

    // Set origin once (first valid pose)
    if (!has_origin)
    {
        origin = p;
        has_origin = true;
    }

    Eigen::Vector3f p_local = p - origin;

    float x = p_local.x();
    float y = p_local.y();
    float z = p_local.z();
    float map_x = x * MAP_SCALE;
    float map_y = z * MAP_SCALE;


	    Eigen::Matrix3f R = Twc.rotationMatrix();
	    float yaw = atan2(R(1,0), R(0,0));

	    char buffer[128];
	    // IMPORTANT:
	    // Send an epoch timestamp for time-aligning SLAM poses with AirSim poses.
	    // Format expected by slam_web UDP listener:
	    //   sx sy sz yaw slam_ts [optional extras...]
	    // slam_ts is seconds since epoch (double), comparable to Python time.time().
	    double slam_ts = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
	    snprintf(buffer, sizeof(buffer),
	         "%.3f %.3f %.3f %.3f %.6f %.1f %.1f",
	         x, y, z, yaw,
	         slam_ts,
	         map_x, map_y);


    sendto(
        udp_sock,
        buffer,
        strlen(buffer),
        0,
        (sockaddr*)&udp_addr,
        sizeof(udp_addr)
    );
auto now_dump = std::chrono::steady_clock::now();

// dump every 1 second (tune 200ms–1000ms)
if (std::chrono::duration_cast<std::chrono::milliseconds>(now_dump - last_dump).count() >= 1000)
{
    // Write to temp first
    SLAM.SaveKeyFrameTrajectoryTUM(LIVE_KF_TMP);

    // Atomic-ish replace (rename is atomic on same filesystem)
    try {
        std::filesystem::rename(LIVE_KF_TMP, LIVE_KF_FILE);
    } catch (...) {
        // fallback: remove then rename
        std::remove(LIVE_KF_FILE.c_str());
        std::filesystem::rename(LIVE_KF_TMP, LIVE_KF_FILE);
    }

    last_dump = now_dump;
}

}

        frame_count++;

        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count() >= 2)
        {
            cout << "[PROGRESS] frames=" << frame_count
                 << " time=" << t << "s" << endl;
            last_print = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // ---------- Shutdown & save ----------
    cout << "\nStopping SLAM..." << endl;
    SLAM.Shutdown();

    cout << "Saving trajectories..." << endl;
    SLAM.SaveTrajectoryTUM("airsim_trajectory_tum.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("airsim_keyframes_tum.txt");

    cout << "Saved files:" << endl;
    cout << "  airsim_trajectory_tum.txt" << endl;
    cout << "  airsim_keyframes_tum.txt" << endl;

    close(udp_sock);
    return 0;
}
