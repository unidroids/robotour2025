// lidar_controller.hpp — řadič Unitree L2 LiDARu (SDK2)
// --------------------------------------------------------
// Rev.3 — distance with sequence & unknown = -1
// • Udržuje minimalní vzdálenost za 3 cloudy (plná otočka) + pořadové id.
// • DISTANCE => (-1) pokud zatím neznámé, jinak "seq dist".
// --------------------------------------------------------

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

namespace unilidar = unilidar_sdk2;

class LidarController {
public:
    LidarController() { resetDistance(); }
    ~LidarController() { stop(); }

    bool start() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (running_) { std::cout << "[LIDAR] already running" << std::endl; return true; }

        try {
            resetDistance();            // vzdálenost začíná neznámá
            if (!reader_) {
                reader_.reset(unilidar::createUnitreeLidarReader());
                if (!reader_) throw std::runtime_error("factory nullptr");
                std::string lidar_ip = "192.168.10.62";
                std::string local_ip = "192.168.10.2";
                uint16_t lidar_port = 6101, local_port = 6201, cloud_scan_num = 3;
                int rc = reader_->initializeUDP(lidar_port, lidar_ip, local_port, local_ip, cloud_scan_num);
                if (rc) { std::cerr << "[LIDAR] initializeUDP() rc="<<rc<<std::endl; reader_.reset(); return false; }
            }
            reader_->startLidarRotation();
            running_ = true;
            worker_ = std::thread(&LidarController::loopRead, this);
            std::cout << "[LIDAR] started" << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "[LIDAR] start() exc: "<<e.what()<<std::endl; reader_.reset(); return false; }
        return true;
    }

    void stop() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!running_) return;
        running_ = false;
        if (reader_) { try { reader_->stopLidarRotation(); } catch (...) {} }
        if (worker_.joinable()) worker_.join();
        resetDistance();
        std::cout << "[LIDAR] stopped" << std::endl;
    }

    // Returns true if valid distance available; seq & dist filled.
    bool getDistance(uint64_t &seq_out, float &dist_out) const {
        std::cout << "[getDistance] seq=" << seq_.load() << " min=" << latest_.load() << " m\n";
        seq_out = seq_.load();
        if (seq_out == 0) return false;
        dist_out = latest_.load();
        return true;
    }

private:
    void resetDistance() {
        latest_.store(-1.f);
        seq_.store(0);
    }

    void loopRead() {
        const int REV_CLOUDS = 3;
        float rev_min = std::numeric_limits<float>::infinity();
        int   clouds = 0;
        while (running_) {
            int pkt_type = reader_->runParse();
            if (pkt_type == LIDAR_POINT_DATA_PACKET_TYPE) {
                unilidar::PointCloudUnitree cloud;
                if (reader_->getPointCloud(cloud)) {
                    float cloud_min = std::numeric_limits<float>::infinity();
                    for (const auto &pt : cloud.points) {
                        float d = std::sqrt(pt.x*pt.x+pt.y*pt.y+pt.z*pt.z);
                        if (d < cloud_min) cloud_min = d;
                    }
                    // Early critical (example threshold 0.1 m)
                    if (cloud_min < rev_min) rev_min = cloud_min;
                    if (++clouds >= REV_CLOUDS) {
                        latest_.store(rev_min);
                        seq_.fetch_add(1);
                        rev_min = std::numeric_limits<float>::infinity();
                        clouds = 0;
                        std::cout << "[loopRead] seq=" << seq_.load() << " min=" << latest_.load() << " m\n";
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    struct ReaderDel { void operator()(unilidar::UnitreeLidarReader *p) const { delete p; }};
    std::unique_ptr<unilidar::UnitreeLidarReader, ReaderDel> reader_;

    std::thread worker_;
    std::atomic<bool>  running_{false};
    std::atomic<float> latest_;          // -1 = unknown
    std::atomic<uint64_t> seq_;          // 0 = unknown

    std::mutex mtx_;
};

#include <sys/socket.h>
#include <fcntl.h>

// util na konci souboru
static void drainSocket(int fd) {
    char buf[1500];
    for (;;) {
        ssize_t n = ::recv(fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (n <= 0) break;   // nic dalšího
    }
}