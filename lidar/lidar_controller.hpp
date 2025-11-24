// lidar_controller.hpp — řadič Unitree L2 LiDARu (SDK2)
// ---------------------------------------------------------------------------
// Rev.8
// • Dva PLY loggery:
//     raw_logger_  → /data/robot/lidar/cloud_*.ply (syrový cloud)
//     proc_logger_ → /data/robot/lidar/trans_*.ply (transform + ořez)
// • loopRead():
//     1. uloží syrový cloud (raw_logger_)
//     2. vytvoří transformovaný cloud (pointproc::transformCloud)
//     3. uloží transformovaný cloud (proc_logger_)
//     4. minimum počítá z transformovaného cloudu (pointproc::minDistance)
// ---------------------------------------------------------------------------

#pragma once

#include <atomic>
#include <thread>
#include <memory>
#include <cmath>
#include <iostream>
#include <chrono>
#include <mutex>
#include <limits>

#include "unitree_lidar_sdk.h"
#include "unitree_lidar_protocol.h"

#include "point_processing.hpp"
#include "ply_logger.hpp"

namespace unilidar = unilidar_sdk2;

class LidarController {
public:
    LidarController()
        : raw_logger_("/data/robot/lidar", "cloud_"),
          proc_logger_("/data/robot/lidar", "trans_")
    {
        resetDistance();
    }
    ~LidarController() { stop(); }

    bool start() {
        std::lock_guard<std::mutex> lg(mtx_);
        if (running_) { std::cout << "[LIDAR] already running" << std::endl; return true; }

        try {
            resetDistance();
            if (!reader_) {
                reader_.reset(unilidar::createUnitreeLidarReader());
                if (!reader_) throw std::runtime_error("factory returned nullptr");

                std::string lidar_ip  = "192.168.10.62";
                std::string local_ip  = "192.168.10.2";
                uint16_t lidar_port   = 6101;
                uint16_t local_port   = 6201;
                uint16_t cloud_scan_num = 3;

                int rc = reader_->initializeUDP(lidar_port, lidar_ip, local_port, local_ip, cloud_scan_num);
                if (rc != 0) {
                    std::cerr << "[LIDAR] initializeUDP rc="<<rc<<std::endl;
                    reader_.reset();
                    return false;
                }
            }

            reader_->startLidarRotation();

            // flush 2 s
            auto t_end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (std::chrono::steady_clock::now() < t_end) reader_->runParse();
            reader_->clearBuffer();
            resetDistance();

            running_ = true;
            worker_ = std::thread(&LidarController::loopRead, this);
            std::cout << "[LIDAR] started (flushed)" << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "[LIDAR] start exc: " << e.what() << std::endl;
            return false;
        }
        return true;
    }

    void stop() {
        std::lock_guard<std::mutex> lg(mtx_);
        if (!running_) return;
        running_ = false;
        if (worker_.joinable()) worker_.join();
        try { reader_->stopLidarRotation(); } catch (...) {}
        resetDistance();
        std::cout << "[LIDAR] stopped" << std::endl;
    }

    bool getDistance(uint64_t &seq_out, float &dist_out) const {
        seq_out = seq_.load();
        if (seq_out == 0 || !running_) return false;
        dist_out = latest_.load();
        return true;
    }

private:
    void resetDistance() {
        latest_.store(-1.f);
        seq_.store(-1);
    }

    void loopRead() {
        //const int REV_CLOUDS = 10;
        float rev_min = std::numeric_limits<float>::infinity();
        //int   clouds  = 0;
        auto t_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
        while (running_) {
            int pkt = reader_->runParse();
            if (pkt == LIDAR_POINT_DATA_PACKET_TYPE) {
                unilidar::PointCloudUnitree cloud;
                if (reader_->getPointCloud(cloud)) {
                    // --- RAW log ---
                    raw_logger_.push(cloud);

                    // --- Transformace + log ---
                    auto proc = pointproc::transformCloud(cloud);
                    proc_logger_.push(proc);

                    float cloud_min = pointproc::minDistance(proc);
                    if (cloud_min >= 0 && cloud_min < rev_min) rev_min = cloud_min;

                    if (std::chrono::steady_clock::now() > t_end || cloud_min < 50) {
                        latest_.store(rev_min);
                        seq_.fetch_add(1);
                        t_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
                        rev_min = std::numeric_limits<float>::infinity();
                        std::cerr << "[loopRead] data: " << latest_.load() << " s:" << seq_.load() << std::endl;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    struct RD { void operator()(unilidar::UnitreeLidarReader *p) const { delete p; } };

    std::unique_ptr<unilidar::UnitreeLidarReader, RD> reader_;
    std::thread worker_;
    PLYLogger raw_logger_;   // syrový cloud
    PLYLogger proc_logger_;  // transformovaný cloud

    std::atomic<bool>     running_{false};
    std::atomic<float>    latest_;
    std::atomic<uint64_t> seq_;

    mutable std::mutex mtx_;
};
