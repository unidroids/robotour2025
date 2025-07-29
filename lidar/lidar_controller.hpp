// lidar_controller.hpp — kostra řadiče Unitree L2 LiDARu
// ------------------------------------------------------
// Tento jednoduchý wrapper zatím **nevolá SDK** – jen udržuje proměnné.
// Cílem kroku A je mít jednu centrální instanci, bezpečnou pro více vláken.
// ------------------------------------------------------

#pragma once

#include <atomic>
#include <thread>
#include <memory>

// ------------------------------------------------------
// LidarController
// ------------------------------------------------------
// • volání start() — začne (zatím jen nastavit flag)
// • volání stop()  — zastaví (zatím jen reset flag)
// • lastDistance() — vrací minimální vzdálenost (dummy 9999)
// ------------------------------------------------------

class LidarController {
public:
    LidarController() : running_(false), min_distance_(9999.0f) {}
    ~LidarController() { stop(); }

    // Start LiDARu (zatím stub). Vrací true při úspěchu.
    bool start() {
        if (running_.exchange(true)) return true;  // už běží
        // TODO: zde později zavoláme reader_->startLidarRotation() a spustíme worker_
        return true;
    }

    // Stop LiDARu (zatím stub)
    void stop() {
        if (!running_.exchange(false)) return;     // už stojí
        // TODO: zde reader_->stopLidarRotation(); a graceful join worker_
    }

    // Poslední minimální vzdálenost (dummy)
    float lastDistance() const { return min_distance_.load(); }

private:
    std::atomic<bool>  running_;
    std::atomic<float> min_distance_;
};
