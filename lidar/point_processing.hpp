// point_processing.hpp — transformace & filtrování bodů LiDARu
// ---------------------------------------------------------------------------
// Funkce:
//   • transformCloud(src)          → 4×4 rigid-body + scale + mirror + offset
//                                    + filtr: ignoruje kvádr robota (x ∈ [-50,15] & y ∈ [-20,20])
//   • minDistance(cloud)           → minimum v již transformovaném cloudu
//   • minDistanceTransformed(src)  → helper (transform + min)
// Konstantní transformační matice převzatá z uživatelského Python skriptu:
//   T @ Ms @ Mz @ Ry @ Rz
// ---------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>
#include <limits>
#include <cmath>
#include "unitree_lidar_sdk.h"

namespace pointproc {
    using namespace unilidar_sdk2;

    // ---------- 1. Konstantní 4×4 matice -----------------------------------
    inline const Eigen::Matrix4f &transformMatrix() {
        static const Eigen::Matrix4f M = [] {
            const float deg  = M_PI / 180.f;
            const float th_z = -25.5f * deg;    //25.5
            const float th_y = -47.5f * deg; //47.5

            Eigen::Matrix4f Rz;
            Rz <<  cosf(th_z),  sinf(th_z), 0, 0,
                 - sinf(th_z),  cosf(th_z), 0, 0,
                           0,            0, 1, 0,
                           0,            0, 0, 1;

            Eigen::Matrix4f Ry;
            Ry <<  cosf(th_y), 0, -sinf(th_y), 0,
                            0, 1,           0, 0,
                    sinf(th_y), 0,  cosf(th_y), 0,
                            0, 0,           0, 1;

            Eigen::Matrix4f Mz = Eigen::Matrix4f::Identity();
            Mz(2,2) = 1.f;  //-1.f              // mirror Z

            Eigen::Matrix4f Ms = Eigen::Matrix4f::Identity();
            Ms(0,0) = Ms(1,1) = Ms(2,2) = 100.f;   // scale 100×

            Eigen::Matrix4f T  = Eigen::Matrix4f::Identity();
            T(2,3) = 0.f; //90.                 // +z translation

            Eigen::Matrix4f Tx = T * Ms * Mz * Ry * Rz;  // column-major order
            //Eigen::Matrix4f Tx = Rz * Ry * Mz * Ms * T;  // column-major order
            std::cout << "Tx (final) =\n" << Tx << "\n\n";
            return Tx;
        }();
        return M;
    }

    // ---------- 2. Filtr – ignorovat body v kvádru robota ------------------
    inline bool ignoreBox(float x, float y) {
        return (y > -20.f && y < 20.f && x < 20.f && x > -50.f);
    }

    // ---------- 3. Transform + filtr --------------------------------------
    inline PointCloudUnitree transformCloud(const PointCloudUnitree &src) {
        const Eigen::Matrix4f &T = transformMatrix();
        PointCloudUnitree dst;
        dst.stamp   = src.stamp;
        dst.id      = src.id;
        dst.ringNum = src.ringNum;
        dst.points.reserve(src.points.size());

        Eigen::Vector4f p(0,0,0,1);
        for (const auto &pt : src.points) {
            p << pt.x, pt.y, pt.z, 1.f;
            Eigen::Vector4f q4 = T * p;
            Eigen::Vector3f q  = q4.head<3>();
            if (ignoreBox(q.x(), q.y())) continue;   // odfiltruj robot

            PointUnitree o;
            o.x = q.x();
            o.y = q.y();
            o.z = q.z();
            o.intensity = pt.intensity;
            o.time      = pt.time;
            o.ring      = pt.ring;
            dst.points.push_back(o);
        }
        return dst;
    }

    // ---------- 4. Minimum -----------------------------------------------
    inline float minDistance(const PointCloudUnitree &cloud) {
        float min_d = std::numeric_limits<float>::infinity();
        for (const auto &p : cloud.points) {
            float d = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
            if (d < min_d) min_d = d;
        }
        return min_d;
    }

    inline float minDistanceTransformed(const PointCloudUnitree &src) {
        auto dst = transformCloud(src);
        return minDistance(dst);
    }
}