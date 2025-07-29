// point_processing.hpp — transformace & filtrování bodů LiDARu
// -----------------------------------------------------------
// • Konstantní 4×4 matice převzatá z Python skriptu uživatele
//   T @ Ms @ Mz @ Ry @ Rz  (viz komentář níže)
// • Ignoruje body v kvádru (x ∈ [-50,15], y ∈ [-20,20]) – konstrukce robota
// • Funkce `minDistanceTransformed()` projde jeden PointCloudUnitree,
//   převede body, aplikuje filtr a vrátí minimální vzdálenost (m).
//   Pokud žádný bod nevyhoví, vrátí +∞
// -----------------------------------------------------------

#pragma once

#include <Eigen/Dense>
#include <limits>
#include "unitree_lidar_sdk.h"

namespace pointproc {

// === 1. Konstantní transformační matice ================================
// Python:  transform = T @ Ms @ Mz @ Ry @ Rz  ==> row‑major
inline const Eigen::Matrix4f &transformMatrix()
{
    static const Eigen::Matrix4f M = []{
        const float deg = M_PI / 180.f;
        float th_z = 25.5f * deg;
        float th_y = 47.5f * deg;

        Eigen::Matrix4f Rz;  Rz <<  cosf(th_z), -sinf(th_z), 0, 0,
                                   sinf(th_z),  cosf(th_z), 0, 0,
                                           0,          0, 1, 0,
                                           0,          0, 0, 1;

        Eigen::Matrix4f Ry;  Ry <<  cosf(th_y), 0, sinf(th_y), 0,
                                            0, 1,         0, 0,
                                   -sinf(th_y), 0, cosf(th_y), 0,
                                            0, 0,         0, 1;

        Eigen::Matrix4f Mz = Eigen::Matrix4f::Identity(); Mz(2,2) = -1.f;
        Eigen::Matrix4f Ms = Eigen::Matrix4f::Identity(); Ms.topLeftCorner<3,3>() *= 100.f;

        Eigen::Matrix4f T  = Eigen::Matrix4f::Identity(); T(2,3) = 90.f;

        return T * Ms * Mz * Ry * Rz;
    }();
    return M;
}

// === 2. Filtrační funkce (ignorujeme "robot box") =====================
inline bool ignoreBox(float x, float y)
{
    return (y > -20.f && y < 20.f && x < 15.f && x > -50.f);
}

// === 3. Hlavní funkce ===================================================
inline float minDistanceTransformed(const unilidar_sdk2::PointCloudUnitree &cloud)
{
    const Eigen::Matrix4f &T = transformMatrix();
    float d_min = std::numeric_limits<float>::infinity();

    Eigen::Vector4f p(0,0,0,1);
    for (const auto &pt : cloud.points) {
        p << pt.x, pt.y, pt.z, 1.f;
        Eigen::Vector4f q4 = T * p;
        Eigen::Vector3f q = q4.head<3>();

        if (ignoreBox(q.x(), q.y()))
            continue;                       // přeskakujeme robotický kvádr

        float d = q.norm();
        if (d < d_min) d_min = d;
    }
    return d_min;
}

} // namespace pointproc
