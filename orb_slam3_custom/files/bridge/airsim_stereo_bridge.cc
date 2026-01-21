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
#include <filesystem>
#include <cstdio>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>   // ✅ added (Vector3f)

#include "System.h"

// AirSim
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

using std::cout;
using std::cerr;
using std::endl;
constexpr float MAP_SCALE = 20.0f; // pixels per meter
// ---------- Global stop flag (Ctrl+C safe shutdown) ----------
static std::atomic<bool> g_stop(false);
static void signalHandler(int)
{
    g_stop = true;
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

    const std::string left_cam  = "0";
    const std::string right_cam = "1";

    msr::airlib::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests;
    requests.emplace_back(
        left_cam,
        msr::airlib::ImageCaptureBase::ImageType::Scene,
        false,
        false
    );
    requests.emplace_back(
        right_cam,
        msr::airlib::ImageCaptureBase::ImageType::Scene,
        false,
        false
    );

    auto t0 = std::chrono::steady_clock::now();
    auto last_print = t0;
    int frame_count = 0;

    // ---------- Main SLAM loop ----------
    while (!g_stop)
    {
        auto responses = client.simGetImages(requests, vehicle_name);

        if (responses.size() < 2)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
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
    snprintf(buffer, sizeof(buffer),
         "%.3f %.3f %.3f %.3f %.1f %.1f",
         x, y, z, yaw,
         map_x, map_y);


    sendto(
        udp_sock,
        buffer,
        strlen(buffer),
        0,
        (sockaddr*)&udp_addr,
        sizeof(udp_addr)
    );
static auto last_dump = std::chrono::steady_clock::now();
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
