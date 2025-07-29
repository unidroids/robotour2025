// ply_logger.hpp — asynchronní logger point‑cloudů do PLY po 10 s blocích
// --------------------------------------------------------------------------
// • push(cloud) jen ukládá do bufferu a upozorní vlákno (kvůli shutdownu).
// • Worker každou 1 s kontroluje, zda uplynulo FLUSH_INTERVAL (10 s)
//   od posledního zápisu. Pak vyprázdní buffer do nového souboru.
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
    explicit PLYLogger(const std::string &dir)
        : directory_(dir), running_(true), last_flush_(std::chrono::steady_clock::now()) {
        std::filesystem::create_directories(directory_);
        worker_ = std::thread(&PLYLogger::loop, this);
    }
    ~PLYLogger() { stop(); }

    void push(const unilidar_sdk2::PointCloudUnitree &cloud) {
        std::lock_guard<std::mutex> lg(mtx_);
        buffer_.push_back(cloud);
        cv_.notify_one();          // pro rychlé ukončení
    }

    void stop() {
        if (!running_.exchange(false)) return;
        cv_.notify_one();
        if (worker_.joinable()) worker_.join();
    }

private:
    static std::string timestamp() {
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
        std::string fname = directory_ + "/cloud_" + timestamp() + ".ply";
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
    }

    void loop() {
        using namespace std::chrono;
        constexpr auto FLUSH_INTERVAL = seconds(10);
        while (running_) {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait_for(lk, seconds(1));          // wake každý 1 s nebo při push
            auto now = steady_clock::now();
            if (now - last_flush_ >= FLUSH_INTERVAL && !buffer_.empty()) {
                std::vector<unilidar_sdk2::PointCloudUnitree> local;
                buffer_.swap(local);
                lk.unlock();
                writePLY(local);
                last_flush_ = now;
            }
        }
        // flush zbytek při stop
        std::lock_guard<std::mutex> lg(mtx_);
        if (!buffer_.empty()) writePLY(buffer_);
    }

    std::string directory_;
    std::atomic<bool> running_;
    std::thread worker_;

    std::mutex mtx_;
    std::condition_variable cv_;
    std::vector<unilidar_sdk2::PointCloudUnitree> buffer_;
    std::chrono::steady_clock::time_point last_flush_;
};
