#pragma once

// lidar_controller.hpp — řadič Unitree L2 LiDARu (SDK2)
// ---------------------------------------------------------------------------
// • Dva PLY loggery:
//     raw_logger_  → /data/robot/lidar/cloud_*.ply (syrový cloud)
//     proc_logger_ → /data/robot/lidar/trans_*.ply (transform + ořez)
// • loopRead():
//     1. uloží syrový cloud (raw_logger_)
//     2. vytvoří transformovaný cloud (pointproc::transformCloud)
//     3. uloží transformovaný cloud (proc_logger_)
//     4. minimum počítá z transformovaného cloudu (pointproc::minDistance)
//
// Design:
//   - reader_ + UDP socket se inicializují jen jednou (initializeUDP).
//   - STOP/START pouze start/stop rotace + vlákna, ne UDP.
//   - MODE mění konfiguraci přes setLidarWorkMode(), ale nesahá na UDP / resetLidar.
// ---------------------------------------------------------------------------

#include <atomic>
#include <thread>
#include <memory>
#include <cmath>
#include <iostream>
#include <chrono>
#include <mutex>
#include <limits>
#include <cstdint>

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

    ~LidarController() {
        // Bezpečné: stop() shodí vlákno, reader_ zůstane
        // a na konci destruktoru se korektně delete-ne.
        stop();
    }

    // Volitelný helper – jen zajistí vytvoření readeru a initializeUDP.
    // Nespouští rotaci ani vlákno.
    bool connect() {
        std::lock_guard<std::mutex> lg(mtx_);
        if (reader_) {
            std::cout << "[CONNECT] already connected" << std::endl;
            return true;
        }
        return ensureReaderLocked();
    }

    // Spustí LiDAR (rotaci) a čtecí vlákno.
    bool start() {
        {
            std::lock_guard<std::mutex> lg(mtx_);
            if (running_) {
                std::cout << "[LIDAR] already running" << std::endl;
                return true;
            }

            resetDistance();

            // Pokud reader_ ještě neexistuje, vytvoříme ho + initializeUDP
            if (!ensureReaderLocked()) {
                return false;
            }
        } // mtx_ uvolněn

        unilidar::UnitreeLidarReader* r = reader_.get();
        if (!r) {
            std::cerr << "[LIDAR] start: reader_ is null after ensureReaderLocked" << std::endl;
            return false;
        }

        try {
            // start rotace + 2s flush mimo zámek
            r->startLidarRotation();

            auto t_end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (std::chrono::steady_clock::now() < t_end) {
                r->runParse();
            }
            r->clearBuffer();

            {
                std::lock_guard<std::mutex> lg(mtx_);
                resetDistance();
                running_.store(true, std::memory_order_relaxed);
                worker_ = std::thread(&LidarController::loopRead, this);
            }

            std::cout << "[LIDAR] started (flushed)" << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "[LIDAR] start exc: " << e.what() << std::endl;
            return false;
        } catch (...) {
            std::cerr << "[LIDAR] start: unknown exception" << std::endl;
            return false;
        }

        return true;
    }

    // Zastaví čtecí vlákno a rotaci,
    // UDP / reader_ nechá žít (re-use při dalším START).
    void stop() {
        // 1) signalizuj workeru konec (krátká kritická sekce)
        {
            std::lock_guard<std::mutex> lg(mtx_);
            if (!running_.load(std::memory_order_relaxed)) return;
            running_.store(false, std::memory_order_relaxed);
        }

        // 2) počkej, až worker skončí – bez držení zámku
        if (worker_.joinable()) {
            worker_.join();
        }

        // 3) zastav rotaci (reader_ stále žije, UDP necháme být)
        try {
            unilidar::UnitreeLidarReader* r = reader_.get();
            if (r) {
                r->stopLidarRotation();
            }
        } catch (...) {
            std::cerr << "[LIDAR] stop: exception in stopLidarRotation" << std::endl;
        }

        // 4) reset lokálního stavu
        {
            std::lock_guard<std::mutex> lg(mtx_);
            resetDistance();
        }

        std::cout << "[LIDAR] stopped" << std::endl;
    }

    // Vrací poslední změřenou vzdálenost a pořadové číslo "rev_min" měření.
    // true  = platná data, false = žádná / neběží.
    bool getDistance(uint64_t &seq_out, float &dist_out) const {
        const bool running = running_.load(std::memory_order_relaxed);
        seq_out = seq_.load(std::memory_order_relaxed);

        if (!running || seq_out == 0) {
            return false;
        }

        dist_out = latest_.load(std::memory_order_relaxed);
        return true;
    }

    // Nastaví pracovní mód LiDARu (bitová maska podle SDK).
    // Lze volat pouze, pokud LiDAR neběží (running_ == false).
    // Pokud ještě není reader_, nejdřív ho inicializuje (initializeUDP),
    // potom pošle konfigurační paket setLidarWorkMode(mode).
    bool setMode(uint32_t mode) {
        std::cout << "[setMode] request " << mode << std::endl;

        std::lock_guard<std::mutex> lock(mtx_);
        if (running_.load(std::memory_order_relaxed)) {
            std::cout << "[setMode] cannot change mode while running" << std::endl;
            return false;
        }

        // pokud reader_ ještě neexistuje, inicializuj ho
        if (!ensureReaderLocked()) {
            std::cerr << "[setMode] ensureReaderLocked/initReader failed" << std::endl;
            return false;
        }

        try {
            reader_->setLidarWorkMode(mode);
            std::cout << "[setMode] mode sent: " << mode << std::endl;
        } catch (...) {
            std::cerr << "[setMode] exception while setting mode" << std::endl;
            return false;
        }

        return true;
    }

private:
    // RAII deleter pro UnitreeLidarReader (SDK2)
    struct RD {
        void operator()(unilidar::UnitreeLidarReader *p) const noexcept {
            delete p;
        }
    };

    // Vytvoří reader_ a zavolá initializeUDP(), pokud ještě reader_ neexistuje.
    // PŘEDPOKLAD: volající drží mtx_.
    bool ensureReaderLocked() {
        if (reader_) return true;

        reader_.reset(unilidar::createUnitreeLidarReader());
        if (!reader_) {
            std::cerr << "[LIDAR] createUnitreeLidarReader returned nullptr" << std::endl;
            return false;
        }
        std::cout << "[initReader] reader_ instance is created" << std::endl;

        std::string lidar_ip  = "192.168.10.62";
        std::string local_ip  = "192.168.10.2";
        uint16_t lidar_port   = 6101;
        uint16_t local_port   = 6201;
        uint16_t cloud_scan_num = 3;

        int rc = reader_->initializeUDP(lidar_port, lidar_ip, local_port, local_ip, cloud_scan_num);
        std::cout << "[initReader] initializeUDP rc = " << rc << std::endl;
        if (rc != 0) {
            std::cerr << "[LIDAR] initializeUDP rc=" << rc << std::endl;
            reader_.reset();
            return false;
        }

        return true;
    }

    void resetDistance() {
        latest_.store(-1.0f, std::memory_order_relaxed);
        seq_.store(0u, std::memory_order_relaxed);
    }

    // Čtecí smyčka: parsuje pakety, ukládá point cloudy, počítá min. vzdálenost.
    void loopRead() {
        float rev_min = std::numeric_limits<float>::infinity();
        auto t_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);

        while (running_.load(std::memory_order_relaxed)) {
            unilidar::UnitreeLidarReader* r = reader_.get();
            if (!r) {
                std::cerr << "[loopRead] reader_ is null, exiting loop" << std::endl;
                break;
            }

            int pkt = r->runParse();
            if (pkt == LIDAR_POINT_DATA_PACKET_TYPE) {
                unilidar::PointCloudUnitree cloud;
                if (r->getPointCloud(cloud)) {
                    // --- RAW log ---
                    raw_logger_.push(cloud);

                    // --- Transformace + log ---
                    auto proc = pointproc::transformCloud(cloud);
                    proc_logger_.push(proc);

                    float cloud_min = pointproc::minDistance(proc);
                    if (cloud_min >= 0.0f && cloud_min < rev_min) {
                        rev_min = cloud_min;
                    }

                    if (std::chrono::steady_clock::now() > t_end || cloud_min < 50.0f) {
                        latest_.store(rev_min, std::memory_order_relaxed);
                        seq_.fetch_add(1u, std::memory_order_relaxed);
                        t_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
                        rev_min = std::numeric_limits<float>::infinity();

                        std::cerr << "[loopRead] data: " << latest_.load(std::memory_order_relaxed)
                                  << " seq: " << seq_.load(std::memory_order_relaxed) << std::endl;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    // ------------------------------------------------------------------------
    // Členské proměnné
    // ------------------------------------------------------------------------

    std::unique_ptr<unilidar::UnitreeLidarReader, RD> reader_;
    std::thread worker_;
    PLYLogger raw_logger_;   // syrový cloud
    PLYLogger proc_logger_;  // transformovaný cloud

    std::atomic<bool>     running_{false};
    std::atomic<float>    latest_;
    std::atomic<uint64_t> seq_;

    mutable std::mutex mtx_;
};
