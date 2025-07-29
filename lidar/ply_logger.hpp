// ply_logger.hpp — asynchronní logger point‑cloudů do PLY po 10 s blocích
// --------------------------------------------------------------------------
// Nové: možnost vlastního prefixu (cloud_, trans_, …)
// --------------------------------------------------------------------------
#pragma once
#include <filesystem>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cstdio>
#include "unitree_lidar_sdk.h"   // PointCloudUnitree

class PLYLogger {
public:
    // dir  … cílový adresář
    // pref … prefix názvu souboru (např. "cloud_" nebo "trans_")
    explicit PLYLogger(const std::string &dir, std::string pref = "cloud_")
        : directory_(dir), prefix_(std::move(pref)), running_(true), last_flush_(std::chrono::steady_clock::now()) {
        std::filesystem::create_directories(directory_);
        worker_ = std::thread(&PLYLogger::loop, this);
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
    static std::string ts_now() {
        auto t = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(t);
        std::tm tm = *std::localtime(&tt);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
        return buf;
    }

    void writePLY(const std::vector<unilidar_sdk2::PointCloudUnitree> &clouds) {
        size_t total = 0; for (auto &c:clouds) total += c.points.size();
        if (total==0) return;
        std::string fname = directory_ + "/" + prefix_ + ts_now() + ".ply";
        FILE *f = std::fopen(fname.c_str(), "w"); if (!f) return;
        std::fprintf(f,"ply\nformat ascii 1.0\n");
        std::fprintf(f,"element vertex %zu\n", total);
        std::fprintf(f,"property float x\nproperty float y\nproperty float z\nproperty float intensity\nproperty uint ring\nend_header\n");
        for (auto &c:clouds) {
            for (auto &p:c.points) {
                std::fprintf(f,"%f %f %f %f %u\n", p.x,p.y,p.z,p.intensity,p.ring);
            }
        }
        std::fclose(f);
        std::cout << "[ply_logger] saved " << fname.c_str() << std::endl;
    }

    void loop() {
        using namespace std::chrono;
        constexpr auto FLUSH_INTERVAL = seconds(10);
        while (running_) {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait_for(lk, seconds(1));
            auto now = steady_clock::now();
            if (now - last_flush_ >= FLUSH_INTERVAL && !buffer_.empty()) {
                std::vector<unilidar_sdk2::PointCloudUnitree> local;
                buffer_.swap(local);
                lk.unlock();
                writePLY(local);
                last_flush_ = now;
            }
        }
        // flush při stop
        std::lock_guard<std::mutex> lg(mtx_);
        if (!buffer_.empty()) writePLY(buffer_);
    }

    std::string directory_;
    std::string prefix_;
    std::atomic<bool> running_;
    std::thread worker_;

    std::mutex mtx_;
    std::condition_variable cv_;
    std::vector<unilidar_sdk2::PointCloudUnitree> buffer_;
    std::chrono::steady_clock::time_point last_flush_;
};
