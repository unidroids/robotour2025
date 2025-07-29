// lidar_controller.hpp — řadič Unitree L2 LiDARu (SDK2)
// --------------------------------------------------------
// • Jediná globální instance používaná TCP serverem.
// • START  → inicializuje SDK, spustí LiDAR, rozběhne vlákno loopRead().
// • STOP   → zastaví smyčku, vypne rotaci, uvolní SDK.
// • lastDistance() → thread‑safe minimální vzdálenost (m).
// --------------------------------------------------------

#pragma once

#include <atomic>
#include <thread>
#include <memory>
#include <cmath>
#include <iostream>
#include <chrono>

// SDK hlavní hlavička
#include "unitree_lidar_sdk.h"
#include "unitree_lidar_protocol.h"   // pro LIDAR_POINT_DATA_PACKET_TYPE

namespace unilidar_sdk2 {
    // forward declarations už máme v unitree_lidar_sdk.h
}

class LidarController {
public:
    LidarController() : running_(false), min_distance_(9999.0f) {}
    ~LidarController() { stop(); }

    // Spustí LiDAR a čtecí vlákno. Vrátí true, když se povedlo (nebo už běží).
    bool start() {
        if (running_.exchange(true)) {
            std::cout << "[LIDAR] already running" << std::endl;
            return true;
        }

        try {
            // vytvoření readeru (factory funkce z SDK)
            using namespace unilidar_sdk2;
            reader_.reset(createUnitreeLidarReader());
            if (!reader_) throw std::runtime_error("createUnitreeLidarReader() returned nullptr");

            // výchozí IP adresy – přizpůsobte podle sítě robota
            std::string lidar_ip  = "192.168.10.62";
            std::string local_ip  = "192.168.10.2";
            unsigned short lidar_port  = 6101;
            unsigned short local_port  = 6201;
            uint16_t cloud_scan_num = 3;   // naše strategie 3 cloudy na 360°

            int rc = reader_->initializeUDP(lidar_port, lidar_ip, local_port, local_ip, cloud_scan_num);
            if (rc != 0) {
                std::cerr << "[LIDAR] initializeUDP() failed (rc=" << rc << ")" << std::endl;
                reader_.reset();
                running_ = false;
                return false;
            }

            reader_->startLidarRotation();
            worker_ = std::thread(&LidarController::loopRead, this);
            std::cout << "[LIDAR] started" << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "[LIDAR] start() exception: " << e.what() << std::endl;
            running_ = false;
            return false;
        }
        return true;
    }

    // Graceful stop (blokující join)
    void stop() {
        if (!running_.exchange(false)) return;  // nebyl spuštěn
        try {
            if (reader_) reader_->stopLidarRotation();
        } catch (...) {}
        if (worker_.joinable()) worker_.join();
        reader_.reset();
        min_distance_.store(9999.0f);
        std::cout << "[LIDAR] stopped" << std::endl;
    }

    float lastDistance() const { return min_distance_.load(); }

private:
    void loopRead() {
        using namespace unilidar_sdk2;
        while (running_.load()) {
            int pkt_type = reader_->runParse();
            if (pkt_type == LIDAR_POINT_DATA_PACKET_TYPE) {
                PointCloudUnitree cloud;
                if (reader_->getPointCloud(cloud)) {
                    float local_min = 9999.0f;
                    for (const auto &pt : cloud.points) {
                        float d = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
                        if (d < local_min) local_min = d;
                    }
                    min_distance_.store(local_min);
                }
            }
            // malá pauza, když není co číst (snižuje CPU při chybě)
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    // custom deleter pro unique_ptr
    struct ReaderDeleter {
        void operator()(unilidar_sdk2::UnitreeLidarReader *p) const { delete p; }
    };

    std::unique_ptr<unilidar_sdk2::UnitreeLidarReader, ReaderDeleter> reader_;
    std::thread worker_;
    std::atomic<bool>  running_;
    std::atomic<float> min_distance_;
};
