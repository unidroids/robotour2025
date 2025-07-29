// robot_lidar_tcp.cpp — TCP služba pro Robotour LiDAR
// -----------------------------------------------------------------
// • Poslouchá POUZE na 127.0.0.1:9002 (plain TCP)
// • Příkazy: PING, START, STOP, DISTANCE, EXIT, SHUTDOWN
// • START/STOP volají LidarController (globální instance)
// • DISTANCE vrací poslední minimální vzdálenost z LiDARu
// • Všechny příkazy se logují na stdout
// • Build: g++ -std=c++17 -pthread robot_lidar_tcp.cpp -o robot_lidar_tcp
// -----------------------------------------------------------------

#include "lidar_controller.hpp"   // náš wrapper

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <algorithm>

constexpr uint16_t kPort = 9002;
constexpr const char *kBindAddr = "127.0.0.1";

// ------------------------------------------------------
// Globální stav
// ------------------------------------------------------
static LidarController lidar;          // jediná instance (safe)
std::atomic<bool> shutting_down{false};
std::atomic<int>  listen_fd{-1};

std::mutex clients_mtx;
std::vector<int> client_socks;

// ------------------------------------------------------
// Utility
// ------------------------------------------------------
void send_line(int sock, const std::string &msg) {
    std::string out = msg + "\n";
    ::send(sock, out.data(), out.size(), MSG_NOSIGNAL);
}

void close_all_clients() {
    std::lock_guard<std::mutex> lg(clients_mtx);
    for (int s : client_socks) {
        ::shutdown(s, SHUT_RDWR);
        ::close(s);
    }
    client_socks.clear();
}

void stop_listener() {
    int fd = listen_fd.exchange(-1);
    if (fd >= 0) { ::shutdown(fd, SHUT_RDWR); ::close(fd); }
}

// ------------------------------------------------------
// Vlákno pro každého klienta
// ------------------------------------------------------
void handle_client(int sock) {
    {
        std::lock_guard<std::mutex> lg(clients_mtx);
        client_socks.push_back(sock);
    }

    //send_line(sock, "WELCOME");
    std::string buffer;
    char tmp[512];

    while (!shutting_down.load()) {
        ssize_t n = ::recv(sock, tmp, sizeof(tmp), 0);
        if (n <= 0) break; // klient zavřel
        buffer.append(tmp, n);

        size_t pos;
        while ((pos = buffer.find('\n')) != std::string::npos) {
            std::string line = buffer.substr(0, pos);
            buffer.erase(0, pos + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();

            std::cout << "CMD(" << sock << "): " << line << std::endl;

            if (line == "PING") {
                send_line(sock, "PONG");
            } else if (line == "START") {
                bool ok = lidar.start();
                send_line(sock, ok ? "OK STARTED" : "ERR START");
            } else if (line == "STOP") {
                lidar.stop();
                send_line(sock, "OK STOPPED");
            } else if (line == "DISTANCE") {
                send_line(sock, std::to_string(lidar.lastDistance()));
            } else if (line == "EXIT") {
                send_line(sock, "BYE");
                ::shutdown(sock, SHUT_RDWR);
                break;
            } else if (line == "SHUTDOWN") {
                send_line(sock, "SHUTTING DOWN");
                shutting_down.store(true);
                lidar.stop();
                stop_listener();
                break;
            } else {
                send_line(sock, "ERR UNKNOWN COMMAND");
            }
        }
    }

    ::close(sock);
    {
        std::lock_guard<std::mutex> lg(clients_mtx);
        client_socks.erase(std::remove(client_socks.begin(), client_socks.end(), sock), client_socks.end());
    }
}

// ------------------------------------------------------
// main()
// ------------------------------------------------------
int main() {
    signal(SIGINT, [](int){ shutting_down.store(true); stop_listener(); });

    int listen_sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) { std::perror("socket"); return 1; }
    listen_fd = listen_sock;

    sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_port = htons(kPort);
    if (inet_pton(AF_INET, kBindAddr, &addr.sin_addr) <= 0) { std::perror("inet_pton"); return 1; }
    int opt=1; setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if (bind(listen_sock, (sockaddr*)&addr, sizeof(addr))<0){ std::perror("bind"); return 1; }
    if (listen(listen_sock, 8)<0){ std::perror("listen"); return 1; }

    std::cout << "📡 robot-lidar TCP server naslouchá na " << kBindAddr << ":" << kPort << std::endl;

    while (!shutting_down.load()) {
        sockaddr_in cli{}; socklen_t len=sizeof(cli);
        int cs = accept(listen_sock, (sockaddr*)&cli, &len);
        if (cs<0){ if (shutting_down.load()) break; std::perror("accept"); continue; }
        std::thread(handle_client, cs).detach();
    }

    close_all_clients();
    std::cout << "🛑 robot-lidar server ukončen." << std::endl;
    return 0;
}

/* ------------------- CMakeLists.txt --------------------
cmake_minimum_required(VERSION 3.16)
project(robot_lidar_tcp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
add_executable(robot_lidar_tcp robot_lidar_tcp.cpp)
# Později přidáme knihovny SDK: target_link_libraries(robot_lidar_tcp PRIVATE unilidar_sdk2 pthread)
target_link_libraries(robot_lidar_tcp PRIVATE pthread)
*/
