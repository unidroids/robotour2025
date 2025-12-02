#pragma once

#include <array>
#include <cstdint>
#include <cmath>
#include <limits>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <iostream>

#include <Eigen/Dense>
#include "unitree_lidar_utilities.h"   // PointCloudUnitree, PointUnitree :contentReference[oaicite:1]{index=1}

class LidarPointProcessing
{
public:
    struct Sample {
        float x;
        float y;
        float z;
        float intensity;
        double time;          // absolutní čas [s] (cloud.stamp + point.time)
        std::uint32_t ring;
    };

    static constexpr std::size_t kCapacity = 1u << 16; // 65536 bodů

    LidarPointProcessing() = default;

    // Aktualizace z nového cloud-u (v lidar frame, v metrech).
    void updateCloud(const unilidar_sdk2::PointCloudUnitree &cloud_in)
    {
        // 1) Transformace do rámce robota + odfiltrování kvádru robota.
        unilidar_sdk2::PointCloudUnitree cloud_robot = transformCloud(cloud_in);

        const double base_stamp = cloud_robot.stamp;  // absolutní čas začátku scanu

        // 2) Zápis bodů do ring bufferu.
        for (const auto &pt : cloud_robot.points) {
            Sample s;
            s.x = pt.x;           // už ve tvém měřítku (cm) díky Ms=100 v transformMatrix
            s.y = pt.y;
            s.z = pt.z;
            s.intensity = pt.intensity;
            s.time = base_stamp + static_cast<double>(pt.time); // point.time je relativní od cloud.stamp :contentReference[oaicite:2]{index=2}
            s.ring = pt.ring;

            pushSample(s);
        }
    }

    // Minimální vzdálenost překážky v rozsahu z∈[z_min,z_max] (v cm v rámci robota).
    // Vrací:
    //   - sqrt(x^2 + y^2) [cm]
    //   - 5000cm pokud v bufferu není žádný bod v z-intervalu.
    float distance(float z_min = -50.0f, float z_max = 80.0f) const
    {
        if (size_ < kCapacity) {
            return -1.0f;
        }

        //const std::size_t N = (size_ < kCapacity) ? size_ : kCapacity;

        float min_sq = 5000.0; // 5000 cm = 50m
        bool found = false;

        for (std::size_t i = 0; i < kCapacity; ++i) {
            const Sample &p = buffer_[i];

            if (p.z < z_min || p.z > z_max) {
                continue;
            }

            const float d2 = p.x * p.x + p.y * p.y;
            if (d2 < min_sq) {
                min_sq = d2;
                found = true;
            }
        }

        if (!found) {
            return 5000.0f;
        }

        return std::sqrt(min_sq);
    }

    // Volitelně: snapshot bufferu (např. pro debug / další algoritmy).
    std::vector<Sample> snapshot() const
    {
        std::vector<Sample> out;
        const std::size_t N = (size_ < kCapacity) ? size_ : kCapacity;
        out.reserve(N);
        for (std::size_t i = 0; i < N; ++i) {
            out.push_back(buffer_[i]);
        }
        return out;
    }

    void clear() {
        head_ = 0;
        size_ = 0;
        // buffer_ necháme jak je, stará data nám nevadí, stejně je size_==0
    }

private:
    // ---------- Geometrie / transformace -----------------------------------

    static const Eigen::Matrix4f &transformMatrix()
    {
        static const Eigen::Matrix4f M = [] {
            const float deg  = static_cast<float>(M_PI) / 180.0f;
            const float th_z = -25.5f * deg;
            const float th_y = -47.5f * deg;

            Eigen::Matrix4f Rz;
            Rz <<  std::cos(th_z),  std::sin(th_z), 0, 0,
                  -std::sin(th_z),  std::cos(th_z), 0, 0,
                                 0,             0, 1, 0,
                                 0,             0, 0, 1;

            Eigen::Matrix4f Ry;
            Ry <<  std::cos(th_y), 0, -std::sin(th_y), 0,
                                 0, 1,              0, 0,
                   std::sin(th_y), 0,  std::cos(th_y), 0,
                                 0, 0,              0, 1;

            Eigen::Matrix4f Mz = Eigen::Matrix4f::Identity();
            Mz(2,2) = 1.0f;   // případné zrcadlení Z vypnuto (1.0f)

            Eigen::Matrix4f Ms = Eigen::Matrix4f::Identity();
            Ms(0,0) = Ms(1,1) = Ms(2,2) = 100.0f;   // škálování 100× (m → cm)

            Eigen::Matrix4f T  = Eigen::Matrix4f::Identity();
            T(2,3) = 0.0f;   // případný posun v +z

            Eigen::Matrix4f Tx = T * Ms * Mz * Ry * Rz;  // aplikace na column vektory
            std::cout << "LidarPointProcessing::Tx =\n" << Tx << "\n\n";
            return Tx;
        }();
        return M;
    }

    static bool ignoreBox(float x, float y)
    {
        // Kvádr robota ve cm v rámce robota; body uvnitř ignorujeme.
        return (y > -20.0f && y <  20.0f &&
                x <  20.0f && x > -50.0f);
    }

    static unilidar_sdk2::PointCloudUnitree
    transformCloud(const unilidar_sdk2::PointCloudUnitree &src)
    {
        const Eigen::Matrix4f &T = transformMatrix();
        unilidar_sdk2::PointCloudUnitree dst;
        dst.stamp   = src.stamp;
        dst.id      = src.id;
        dst.ringNum = src.ringNum;
        dst.points.reserve(src.points.size());

        Eigen::Vector4f p(0.0f, 0.0f, 0.0f, 1.0f);

        for (const auto &pt : src.points) {
            p << pt.x, pt.y, pt.z, 1.0f;
            Eigen::Vector4f q4 = T * p;
            Eigen::Vector3f q  = q4.head<3>();

            if (ignoreBox(q.x(), q.y())) {
                continue;  // odfiltruj body robota
            }

            unilidar_sdk2::PointUnitree o;
            o.x = q.x();
            o.y = q.y();
            o.z = q.z();
            o.intensity = pt.intensity;
            o.time      = pt.time;   // stále relativní od cloud.stamp
            o.ring      = pt.ring;
            dst.points.push_back(o);
        }

        return dst;
    }

    // ---------- Ring buffer -------------------------------------------------

    void pushSample(const Sample &s)
    {
        buffer_[static_cast<std::size_t>(head_)] = s;

        // posun indexu (uint16_t overflow → mod 2^16)
        ++head_;

        if (size_ < kCapacity) {
            ++size_;
        }

        // Přetečení (head_ == 0) *a* buffer je plný → dump do PLY.
        if (size_ == kCapacity && head_ == 0) {
            dumpBufferToPly();
        }
    }

    // Čas → cesta pro PLY: /data/robot/lidar/<YYYY-mm-dd>/points-<hh>/ply-<mm-ss-mmm>.ply
    static std::string makePlyPath()
    {
        namespace fs = std::filesystem;
        using clock = std::chrono::system_clock;

        const auto now = clock::now();
        const auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now.time_since_epoch()) % 1000;

        std::time_t tt = clock::to_time_t(now);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &tt);
#else
        localtime_r(&tt, &tm);
#endif

        std::ostringstream date_ss;
        date_ss << std::put_time(&tm, "%Y-%m-%d");

        std::ostringstream hour_ss;
        hour_ss << "points-" << std::setw(2) << std::setfill('0') << tm.tm_hour;

        std::ostringstream file_ss;
        file_ss << "ply-"
                << std::setw(2) << std::setfill('0') << tm.tm_min << "-"
                << std::setw(2) << std::setfill('0') << tm.tm_sec << "-"
                << std::setw(3) << std::setfill('0') << ms.count()
                << ".ply";

        fs::path dir  = fs::path("/data/robot/lidar") / date_ss.str() / hour_ss.str();
        fs::create_directories(dir);

        fs::path path = dir / file_ss.str();
        return path.string();
    }

    void dumpBufferToPly() const
    {
        const std::size_t N = (size_ < kCapacity) ? size_ : kCapacity;
        if (N == 0) {
            return;
        }

        const std::string path = makePlyPath();
        std::ofstream ofs(path);
        if (!ofs) {
            std::cerr << "LidarPointProcessing: failed to open PLY file: " << path << "\n";
            return;
        }

        // PLY header
        ofs << "ply\n";
        ofs << "format ascii 1.0\n";
        ofs << "comment generated by LidarPointProcessing\n";
        ofs << "element vertex " << N << "\n";
        ofs << "property float x\n";
        ofs << "property float y\n";
        ofs << "property float z\n";
        ofs << "property float intensity\n";
        ofs << "property double time\n";
        ofs << "property uint32 ring\n";
        ofs << "end_header\n";

        // data: pro jednoduchost v pořadí [0..N-1] v bufferu
        ofs << std::setprecision(6) << std::fixed;
        for (std::size_t i = 0; i < N; ++i) {
            const Sample &p = buffer_[i];
            ofs << p.x << " "
                << p.y << " "
                << p.z << " "
                << p.intensity << " "
                << p.time << " "
                << p.ring << "\n";
        }
    }

private:
    std::array<Sample, kCapacity> buffer_{};
    std::uint16_t head_{0};   // index pro další zápis (automaticky přeteče mod 2^16)
    std::size_t   size_{0};   // počet platných prvků (<= kCapacity)
};
