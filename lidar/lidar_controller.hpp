// lidar_controller.hpp — řadič Unitree L2 LiDARu (SDK2)
// --------------------------------------------------------
// Rev.2 — "persistent socket"
// • Reader se vytváří a initializeUDP() provede JEN POPRVÉ.
// • Příkaz STOP pouze zastaví rotaci a vlákno, ale socket zůstává
//   => další START už jen startLidarRotation() + nové vlákno → žádné bind() fail.
// --------------------------------------------------------

#pragma once

#include <atomic>
#include <thread>
#include <memory>
#include <cmath>
#include <iostream>
#include <chrono>
#include <mutex>

#include "unitree_lidar_sdk.h"
#include "unitree_lidar_protocol.h"

namespace unilidar = unilidar_sdk2; // alias

class LidarController {
public:
    LidarController() : running_(false), min_distance_(9999.0f) {}
    ~LidarController() { stop(); }

    // ------------------------------------------------------------------
    // START — vytvoří reader při prvním volání; další START jen restartuje.
    // ------------------------------------------------------------------
    bool start() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (running_) { std::cout << "[LIDAR] already running" << std::endl; return true; }

        try {
            if (!reader_) {
                reader_.reset(unilidar::createUnitreeLidarReader());
                if (!reader_) throw std::runtime_error("factory returned nullptr");

                // IP/porty — upravte dle vašeho setupu
                std::string lidar_ip  = "192.168.10.62";
                std::string local_ip  = "192.168.10.2";
                uint16_t lidar_port   = 6101;
                uint16_t local_port   = 6201;
                uint16_t cloud_scan_num = 3;

                int rc = reader_->initializeUDP(lidar_port, lidar_ip,
                                                local_port, local_ip,
                                                cloud_scan_num);
                if (rc != 0) {
                    std::cerr << "[LIDAR] initializeUDP() failed (rc=" << rc << ")" << std::endl;
                    reader_.reset();
                    return false;
                }
            }

            reader_->startLidarRotation();
            running_ = true;
            worker_ = std::thread(&LidarController::loopRead, this);
            std::cout << "[LIDAR] started" << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "[LIDAR] start() exception: " << e.what() << std::endl;
            reader_.reset();
            return false;
        }
        return true;
    }

    // ------------------------------------------------------------------
    // STOP — zastaví rotaci + vlákno, ale reader ponechá připravený.
    // ------------------------------------------------------------------
    void stop() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!running_) return;           // nic nespustit
        running_ = false;

        if (reader_) {
            try { reader_->stopLidarRotation(); } catch (...) {}
        }
        if (worker_.joinable()) worker_.join();
        min_distance_.store(9999.0f);
        std::cout << "[LIDAR] stopped" << std::endl;
    }

    float lastDistance() const { return min_distance_.load(); }

private:
    void loopRead() {
        while (running_) {
            int pkt_type = reader_->runParse();
            if (pkt_type == LIDAR_POINT_DATA_PACKET_TYPE) {
                unilidar::PointCloudUnitree cloud;
                if (reader_->getPointCloud(cloud)) {
                    float local_min = 9999.0f;
                    for (const auto &pt : cloud.points) {
                        float d = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
                        if (d < local_min) local_min = d;
                    }
                    min_distance_.store(local_min);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    struct ReaderDeleter { void operator()(unilidar::UnitreeLidarReader *p) const { delete p; } };

    std::unique_ptr<unilidar::UnitreeLidarReader, ReaderDeleter> reader_;
    std::thread worker_;

    std::atomic<bool>  running_;
    std::atomic<float> min_distance_;
    std::mutex         mtx_;          // chrání start/stop + reader_
};
