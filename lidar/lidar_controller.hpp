#pragma once

// lidar_controller.hpp — řadič Unitree L2 LiDARu (SDK2)
// ---------------------------------------------------------------------------
// • Dva PLY loggery:
//     raw_logger_  → /data/robot/lidar/cloud_*.ply (syrový cloud)
//     proc_logger_ → /data/robot/lidar/trans_*.ply (transform + ořez)
// • loopRead():
//     - pro cloud:
//         1. uloží syrový cloud (raw_logger_)
//         2. vytvoří transformovaný cloud (pointproc::transformCloud)
//         3. uloží transformovaný cloud (proc_logger_)
//         4. minimum počítá z transformovaného cloudu (pointproc::minDistance)
//     - pro IMU:
//         1. po přijetí IMU paketu zavolá getImuData()
//         2. vypíše IMU hodnoty na stdout
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
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "unitree_lidar_sdk.h"
#include "unitree_lidar_protocol.h"

#include "point_processing.hpp"
//#include "ply_logger.hpp"
#include "raw_logger.hpp"

namespace unilidar = unilidar_sdk2;

class LidarController {
public:
    LidarController()
        //: points_(),
          //raw_logger_("/data/robot/lidar", "cloud_"),
          //proc_logger_("/data/robot/lidar", "trans_")
    {
        //resetDistance();
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
            
            //points_->clear();

            //resetDistance();

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
                //resetDistance();
                //points_->clear();
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

        point_processing_.clear();

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
            //resetDistance();
            point_processing_.clear();
        }

        std::cout << "[LIDAR] stopped" << std::endl;
    }


    // Vrací poslední změřenou vzdálenost a pořadové číslo "rev_min" měření.
    // true  = platná data, false = žádná / neběží.
    /*
    bool getDistance(uint64_t &seq_out, float &dist_out) const {
        const bool running = running_.load(std::memory_order_relaxed);
        seq_out = seq_.load(std::memory_order_relaxed);

        if (!running || seq_out == 0) {
            return false;
        }

        dist_out = latest_.load(std::memory_order_relaxed);
        return true;
    }
    */

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


    bool getDistance(float &dist_out) {
        dist_out = point_processing_.distance();
        return dist_out < 0 ? false : true;
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
        bool use_system_timestamp = true;

        int rc = reader_->initializeUDP(lidar_port, lidar_ip, local_port, local_ip, cloud_scan_num, use_system_timestamp);
        std::cout << "[initReader] initializeUDP rc = " << rc << std::endl;
        if (rc != 0) {
            std::cerr << "[LIDAR] initializeUDP rc=" << rc << std::endl;
            reader_.reset();
            return false;
        }

        return true;
    }

    /*
    void resetDistance() {
        latest_.store(-1.0f, std::memory_order_relaxed);
        seq_.store(0u, std::memory_order_relaxed);
    }
    */

    // ----------------------------- zpracování dat -----------------------------

    // Zpracování point cloudu (původní logika z loopRead)
    void processCloudData(unilidar::UnitreeLidarReader &r,
                          float &rev_min,
                          std::chrono::steady_clock::time_point &t_end)
    {
        unilidar::PointCloudUnitree cloud;
        if (!r.getPointCloud(cloud)) {
            return;
        }

        point_processing_.updateCloud(cloud);

        // --- RAW log ---
        //raw_logger_.push(cloud);

        // --- Transformace + log ---
        /*
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

            std::cerr << "[loopRead] data: "
                      << latest_.load(std::memory_order_relaxed)
                      << " seq: "
                      << seq_.load(std::memory_order_relaxed)
                      << std::endl;
        }
        */
    }

    void processIMUData(unilidar::UnitreeLidarReader &r)
    {
        unilidar::LidarImuData imu{};
        if (!r.getImuData(imu)) {
            return;
        }

        const auto &info = imu.info;
        const double imu_ts =
            static_cast<double>(info.stamp.sec) +
            static_cast<double>(info.stamp.nsec) / 1.0e9;
        const double sys_ts = unilidar::getSystemTimeStamp();

        // ---- Původní raw log (můžeš klidně ponechat nebo omezit) ----
        /*
        std::cout << std::fixed << std::setprecision(9);
        std::cout << "[IMU] seq=" << info.seq
                << " imu_ts=" << imu_ts
                << " sys_ts=" << sys_ts << '\n';

        std::cout << "      q      = ["
                << imu.quaternion[0] << ", "
                << imu.quaternion[1] << ", "
                << imu.quaternion[2] << ", "
                << imu.quaternion[3] << "]\n";

        std::cout << "      gyro   = ["
                << imu.angular_velocity[0] << ", "
                << imu.angular_velocity[1] << ", "
                << imu.angular_velocity[2] << "]\n";

        std::cout << "      acc    = ["
                << imu.linear_acceleration[0] << ", "
                << imu.linear_acceleration[1] << ", "
                << imu.linear_acceleration[2] << "]"
                << std::endl;
        */
        // ---- Statistika každých ~10 s ----
        struct ImuStats {
            double start_ts       = 0.0;
            double last_print_ts  = 0.0;
            std::uint64_t n       = 0;

            Eigen::Vector3d sum_acc      = Eigen::Vector3d::Zero();
            Eigen::Vector3d sum_acc_sq   = Eigen::Vector3d::Zero();

            Eigen::Vector3d sum_g_wxyz   = Eigen::Vector3d::Zero();
            Eigen::Vector3d sum_g_xyzw   = Eigen::Vector3d::Zero();

            Eigen::Vector3d sum_err_wxyz = Eigen::Vector3d::Zero();
            Eigen::Vector3d sum_err_xyzw = Eigen::Vector3d::Zero();

            double sum_err2_wxyz = 0.0;
            double sum_err2_xyzw = 0.0;
        };

        static ImuStats S;

        if (S.start_ts == 0.0) {
            S.start_ts      = imu_ts;
            S.last_print_ts = imu_ts;
        }

        Eigen::Vector3d acc_B(
            imu.linear_acceleration[0],
            imu.linear_acceleration[1],
            imu.linear_acceleration[2]
        );

        // varianta 1: předpoklad q = [w, x, y, z] (tak jak se tiskne)
        Eigen::Quaterniond q_wxyz(
            imu.quaternion[0], // w
            imu.quaternion[1], // x
            imu.quaternion[2], // y
            imu.quaternion[3]  // z
        );
        q_wxyz.normalize();

        // varianta 2: q = [x, y, z, w] (SDK často používá tento zápis)
        Eigen::Quaterniond q_xyzw(
            imu.quaternion[3], // w
            imu.quaternion[0], // x
            imu.quaternion[1], // y
            imu.quaternion[2]  // z
        );
        q_xyzw.normalize();

        const Eigen::Vector3d g_W(0.0, 0.0, -9.81);

        // gravitace v tělese pro obě varianty (body->world předpoklad)
        Eigen::Vector3d g_B_wxyz = q_wxyz.conjugate() * g_W;
        Eigen::Vector3d g_B_xyzw = q_xyzw.conjugate() * g_W;

        // očekávané měření akcelerometru ve statice je "specific force":
        // f_B ≈ -g_B
        Eigen::Vector3d acc_pred_wxyz = -g_B_wxyz;
        Eigen::Vector3d acc_pred_xyzw = -g_B_xyzw;

        Eigen::Vector3d err_wxyz = acc_B - acc_pred_wxyz;
        Eigen::Vector3d err_xyzw = acc_B - acc_pred_xyzw;

        // akumulace
        S.n++;
        S.sum_acc    += acc_B;
        S.sum_acc_sq += acc_B.cwiseProduct(acc_B);

        S.sum_g_wxyz   += g_B_wxyz;
        S.sum_g_xyzw   += g_B_xyzw;

        S.sum_err_wxyz += err_wxyz;
        S.sum_err_xyzw += err_xyzw;

        S.sum_err2_wxyz += err_wxyz.squaredNorm();
        S.sum_err2_xyzw += err_xyzw.squaredNorm();

        // každých ~10 s vytiskni statistiku
        if (imu_ts - S.last_print_ts >= 10.0) {
            const double dt_window = imu_ts - S.last_print_ts;
            S.last_print_ts = imu_ts;

            if (S.n > 0) {
                Eigen::Vector3d acc_mean    = S.sum_acc / double(S.n);
                Eigen::Vector3d acc_var     = (S.sum_acc_sq / double(S.n)) -
                                            acc_mean.cwiseProduct(acc_mean);
                Eigen::Vector3d acc_std     = acc_var.cwiseMax(0.0).cwiseSqrt();

                Eigen::Vector3d g_mean_wxyz = S.sum_g_wxyz / double(S.n);
                Eigen::Vector3d g_mean_xyzw = S.sum_g_xyzw / double(S.n);

                Eigen::Vector3d err_mean_wxyz = S.sum_err_wxyz / double(S.n);
                Eigen::Vector3d err_mean_xyzw = S.sum_err_xyzw / double(S.n);

                double rms_err_wxyz = std::sqrt(S.sum_err2_wxyz / double(S.n));
                double rms_err_xyzw = std::sqrt(S.sum_err2_xyzw / double(S.n));

                std::cout << "\n[IMU-STAT] window_len=" << dt_window
                        << "s N=" << S.n << "\n";

                std::cout << "  acc_mean = [" << acc_mean.transpose()
                        << "], |acc_mean|=" << acc_mean.norm() << "\n";
                std::cout << "  acc_std  = [" << acc_std.transpose() << "]\n";

                std::cout << "  g_mean_wxyz (body) = [" << g_mean_wxyz.transpose()
                        << "], |g_mean_wxyz|=" << g_mean_wxyz.norm() << "\n";
                std::cout << "  g_mean_xyzw (body) = [" << g_mean_xyzw.transpose()
                        << "], |g_mean_xyzw|=" << g_mean_xyzw.norm() << "\n";

                std::cout << "  err_mean_wxyz      = [" << err_mean_wxyz.transpose()
                        << "], RMS=" << rms_err_wxyz << "\n";
                std::cout << "  err_mean_xyzw      = [" << err_mean_xyzw.transpose()
                        << "], RMS=" << rms_err_xyzw << "\n";

                // pro poslední vzorek ještě roll/pitch/yaw (pro obě varianty)
                auto print_rpy = [](const char* label, const Eigen::Quaterniond& q) {
                    Eigen::Matrix3d R = q.toRotationMatrix();
                    double roll, pitch, yaw;
                    pitch = std::asin(-R(2,0));
                    if (std::cos(pitch) > 1e-6) {
                        roll  = std::atan2(R(2,1), R(2,2));
                        yaw   = std::atan2(R(1,0), R(0,0));
                    } else {
                        roll  = std::atan2(-R(1,2), R(1,1));
                        yaw   = 0.0;
                    }
                    std::cout << "  " << label << " rpy (rad) = ["
                            << roll << ", " << pitch << ", " << yaw << "]\n";
                };

                print_rpy("q_wxyz", q_wxyz);
                print_rpy("q_xyzw", q_xyzw);

                std::cout << "----------------------------------------\n";
            }

            // po výpisu můžeš buď:
            //  - vše vynulovat (nezávislá okna),
            //  - nebo nechat běžet kumulativně. Já tady vynuluju:
            S = ImuStats{};
            S.start_ts      = imu_ts;
            S.last_print_ts = imu_ts;
        }
    }

    inline uint64_t getMonotonicTimeNs() {
        using namespace std::chrono;
        return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
    }

    // Čtecí smyčka: parsuje pakety, deleguje na processCloudData/processIMUData.
    void loopRead() {
        LidarRawLogger raw_logger;
        float rev_min = std::numeric_limits<float>::infinity();
        auto t_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);

        while (running_.load(std::memory_order_relaxed)) {
            unilidar::UnitreeLidarReader* r = reader_.get();
            if (!r) {
                std::cerr << "[loopRead] reader_ is null, exiting loop" << std::endl;
                break;
            }

            int type = r->runParse();
            uint64_t mono_ts_ns = getMonotonicTimeNs();  

            if (type == LIDAR_POINT_DATA_PACKET_TYPE) {
                const auto& pkt = r->getLidarPointDataPacket();
                raw_logger.writePointPacket(pkt, mono_ts_ns);
                processCloudData(*r, rev_min, t_end);
            } else if (type == LIDAR_IMU_DATA_PACKET_TYPE) {
                const auto& pkt = r->getLidarImuDataPacket();
                raw_logger.writeImuPacket(pkt, mono_ts_ns);
                processIMUData(*r);
            } else if (type == LIDAR_VERSION_PACKET_TYPE) {
                const auto& pkt = r->getLidarVersionDataPacket();
                raw_logger.writeVersionPacket(pkt, mono_ts_ns);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            
        }
    }

    // ------------------------------------------------------------------------
    // Členské proměnné
    // ------------------------------------------------------------------------

    std::unique_ptr<unilidar::UnitreeLidarReader, RD> reader_;
    std::thread worker_;
    //PLYLogger raw_logger_;   // syrový cloud
    //PLYLogger proc_logger_;  // transformovaný cloud

    LidarPointProcessing point_processing_;

    std::atomic<bool>     running_{false};
    std::atomic<float>    latest_;
    std::atomic<uint64_t> seq_;

    mutable std::mutex mtx_;
};
