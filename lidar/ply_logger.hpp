// ply_logger.hpp — jednoduchý asynchronní logger point‑cloudů do PLY
// ------------------------------------------------------------------
// Použití:
//   PLYLogger logger("/data/lidar"); // adresář
//   logger.push(cloud);               // z vlákna loopRead()
//   logger.stop();                    // při destrukci LidarControlleru
// ------------------------------------------------------------------
#pragma once
#include <filesystem>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include "unitree_lidar_sdk.h"   // PointCloudUnitree

class PLYLogger {
public:
    explicit PLYLogger(const std::string &dir)
        : directory_(dir), running_(true), worker_(&PLYLogger::loop, this) {
        std::filesystem::create_directories(directory_);
    }
    ~PLYLogger() { stop(); }

    void push(const unilidar_sdk2::PointCloudUnitree &cloud) {
        std::lock_guard<std::mutex> lg(mtx_);
        buffer_.push_back(cloud);
        cv_.notify_one();
    }

    void stop() {
        if (!running_.exchange(false)) return;
        cv_.notify_one();
        if (worker_.joinable()) worker_.join();
    }

private:
    static std::string timestamp() {
        auto t = std::chrono::system_clock::now();
        auto tt = std::chrono::system_clock::to_time_t(t);
        std::tm tm = *std::localtime(&tt);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
        return std::string(buf);
    }

    void writePLY(const std::vector<unilidar_sdk2::PointCloudUnitree> &clouds) {
        size_t total = 0;
        for (const auto &c : clouds) total += c.points.size();
        if (total == 0) return;

        std::string fname = directory_ + "/cloud_" + timestamp() + ".ply";
        FILE *f = std::fopen(fname.c_str(), "w");
        if (!f) return;
        // header
        std::fprintf(f, "ply\nformat ascii 1.0\n");
        std::fprintf(f, "element vertex %zu\n", total);
        std::fprintf(f, "property float x\nproperty float y\nproperty float z\n");
        std::fprintf(f, "property float intensity\nproperty uint ring\nend_header\n");
        // data
        for (const auto &c : clouds) {
            for (const auto &pt : c.points) {
                std::fprintf(f, "%f %f %f %f %u\n", pt.x, pt.y, pt.z, pt.intensity, pt.ring);
            }
        }
        std::fclose(f);
    }

    void loop() {
        using namespace std::chrono_literals;
        while (running_) {
            std::vector<unilidar_sdk2::PointCloudUnitree> local;
            {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.wait_for(lk, 10s, [&]{ return !buffer_.empty() || !running_; });
                buffer_.swap(local);
            }
            if (!local.empty()) writePLY(local);
        }
        // flush remaining
        std::vector<unilidar_sdk2::PointCloudUnitree> local;
        {
            std::lock_guard<std::mutex> lg(mtx_);
            buffer_.swap(local);
        }
        if (!local.empty()) writePLY(local);
    }

    std::string directory_;
    std::atomic<bool> running_;
    std::thread worker_;

    std::mutex mtx_;
    std::condition_variable cv_;
    std::vector<unilidar_sdk2::PointCloudUnitree> buffer_;
};
