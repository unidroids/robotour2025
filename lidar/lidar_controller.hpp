// lidar_controller.hpp — řadič Unitree L2 LiDARu (SDK2)
// ---------------------------------------------------------
// Rev.6 – Jednorázová inicializace UDP + 2 s vyčerpání bufferu
// • `reader_` se vytváří jen poprvé. Port zůstává otevřený.
// • Každý `START` po roztočení LiDARu volá 2 s "flush", která opakovaně
//   čte `runParse()` a zahazuje data ➜ fronta kernelu i dekodéru se vyčistí.
// • Teprve poté se spustí `worker_` vlákno.
// ---------------------------------------------------------

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

namespace unilidar = unilidar_sdk2;

class LidarController {
public:
    LidarController() { resetDistance(); }
    ~LidarController() { stop(); }

    bool start() {
        std::lock_guard<std::mutex> lg(mtx_);
        if (running_) { std::cout << "[LIDAR] already running" << std::endl; return true; }

        try {
            resetDistance();             // údaje budou nové až od workeru
            if (!reader_) {
                // --- PRVNÍ SPUŠTĚNÍ: vytvoř reader & inicializuj UDP ---
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

            // --- FLUSH 2 sekundy ------------------------------------------------
            auto t_end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (std::chrono::steady_clock::now() < t_end) {
                reader_->runParse();    // ignorujeme návratovou hodnotu
            }
            reader_->clearBuffer();      // pro jistotu vynuluj dekodér
            resetDistance();             // údaje budou nové až od workeru
            // -------------------------------------------------------------------

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

        if (worker_.joinable()) worker_.join();   // nevolá reader po join
        try { reader_->stopLidarRotation(); } catch (...) {}
        resetDistance();
        std::cout << "[LIDAR] stopped" << std::endl;
    }

    bool getDistance(uint64_t &seq_out, float &dist_out) const {
        std::cout << "[getDistance] seq=" << seq_.load() << " min=" << latest_.load() << " m" << std::endl;
        seq_out = seq_.load();
        if (seq_out == 0 || running_ == false) return false;
        dist_out = latest_.load();
        return true;
    }

private:
    void resetDistance() {
        latest_.store(-1.f);
        seq_.store(0);
    }

    void loopRead() {
        const int REV_CLOUDS = 6;
        float rev_min = std::numeric_limits<float>::infinity();
        int   clouds  = 0;
        while (running_) {
            int pkt = reader_->runParse();
            if (pkt == LIDAR_POINT_DATA_PACKET_TYPE) {
                unilidar::PointCloudUnitree cloud;
                if (reader_->getPointCloud(cloud)) {
                    float cloud_min = pointproc::minDistanceTransformed(cloud);
                    if (cloud_min < rev_min) rev_min = cloud_min;
                    if (++clouds >= REV_CLOUDS) {
                        latest_.store(rev_min);
                        uint64_t newSeq = seq_.fetch_add(1) + 1;
                        //std::cout << "[loopRead] seq=" << newSeq << " min=" << rev_min << " m" << std::endl;
                        clouds = 0;
                        rev_min = std::numeric_limits<float>::infinity();
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    struct RD { void operator()(unilidar::UnitreeLidarReader *p) const { delete p; } };

    std::unique_ptr<unilidar::UnitreeLidarReader, RD> reader_;
    std::thread worker_;

    std::atomic<bool>     running_{false};
    std::atomic<float>    latest_;
    std::atomic<uint64_t> seq_;

    mutable std::mutex mtx_;
};
