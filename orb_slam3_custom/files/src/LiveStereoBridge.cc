#include <iostream>
#include <string>
#include <vector>
#include <csignal>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "System.h"

static bool g_should_exit = false;

void sigint_handler(int)
{
    g_should_exit = true;
}

// Helper: read exactly N bytes from stdin
bool read_exact(std::istream& in, char* buffer, size_t n)
{
    size_t total = 0;
    while (total < n && in.good())
    {
        in.read(buffer + total, n - total);
        size_t got = static_cast<size_t>(in.gcount());
        if (got == 0) break;
        total += got;
    }
    return (total == n);
}

int main(int argc, char** argv)
{
    if (argc < 4)
    {
        std::cerr << "Usage: " << argv[0]
                  << " path_to_vocabulary path_to_settings output_trajectory.txt\n";
        return 1;
    }

    std::string vocab_path    = argv[1];
    std::string settings_path = argv[2];
    std::string traj_path     = argv[3];

    std::signal(SIGINT, sigint_handler);

    // ORB-SLAM3 system in stereo mode, with / without viewer
    ORB_SLAM3::System SLAM(
        vocab_path,
        settings_path,
        ORB_SLAM3::System::STEREO,
        /*bUseViewer =*/ false
    );

    // Protocol:
    // For each frame, Python sends:
    // [uint64 timestamp_ns][uint32 left_size][uint32 right_size][left_bytes][right_bytes]
    // When timestamp_ns == 0 and sizes==0, terminate.

    std::cerr << "[LiveStereoBridge] Ready. Waiting for frames on stdin...\n";

    while (!g_should_exit && std::cin.good())
    {
        // 8 bytes timestamp (uint64)
        uint64_t ts_ns;
        if (!read_exact(std::cin, reinterpret_cast<char*>(&ts_ns), sizeof(ts_ns)))
            break;

        // Exit sentinel: all zero
        if (ts_ns == 0)
        {
            std::cerr << "[LiveStereoBridge] Received sentinel timestamp 0. Exiting loop.\n";
            break;
        }

        // 4 bytes each for left/right sizes (uint32)
        uint32_t left_size = 0, right_size = 0;
        if (!read_exact(std::cin, reinterpret_cast<char*>(&left_size), sizeof(left_size)))
            break;
        if (!read_exact(std::cin, reinterpret_cast<char*>(&right_size), sizeof(right_size)))
            break;

        if (left_size == 0 || right_size == 0)
        {
            std::cerr << "[LiveStereoBridge] Got zero-sized image. Skipping.\n";
            continue;
        }

        std::vector<uchar> left_buf(left_size), right_buf(right_size);
        if (!read_exact(std::cin, reinterpret_cast<char*>(left_buf.data()), left_size))
            break;
        if (!read_exact(std::cin, reinterpret_cast<char*>(right_buf.data()), right_size))
            break;

        cv::Mat left_img  = cv::imdecode(left_buf, cv::IMREAD_UNCHANGED);
        cv::Mat right_img = cv::imdecode(right_buf, cv::IMREAD_UNCHANGED);

        if (left_img.empty() || right_img.empty())
        {
            std::cerr << "[LiveStereoBridge] Failed to decode images. Skipping.\n";
            continue;
        }

        // Convert timestamp to seconds (double) as in EuRoC
        double t_sec = static_cast<double>(ts_ns) * 1e-9;

        SLAM.TrackStereo(left_img, right_img, t_sec);
    }

    std::cerr << "[LiveStereoBridge] Shutting down SLAM...\n";
    SLAM.Shutdown();

    // Save trajectory + map (atlas)
    std::cerr << "[LiveStereoBridge] Saving trajectory to " << traj_path << "\n";
    SLAM.SaveTrajectoryEuRoC(traj_path);

    // Atlas saving is controlled by settings (System.SaveAtlasToFile)
    // so ORB-SLAM3 will write that automatically on Shutdown().

    std::cerr << "[LiveStereoBridge] Done.\n";
    return 0;
}

