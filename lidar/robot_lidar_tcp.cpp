// robot_lidar_tcp.cpp — minimální TCP socket služba pro Robotour LiDAR
// -----------------------------------------------------------------
// • Poslouchá POUZE na 127.0.0.1:9002 (plain TCP, žádný WebSocket)
// • Příkazy: PING, START, STOP, DISTANCE, EXIT, SHUTDOWN
// • SHUTDOWN nyní korektně zavírá posluchač (accept skončí)
// • Build: g++ -std=c++17 -pthread robot_lidar_tcp.cpp -o robot_lidar_tcp
// -----------------------------------------------------------------

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <arpa/inet.h>      // inet_pton
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <algorithm>        // std::remove

constexpr uint16_t kPort = 9002;
constexpr const char *kBindAddr = "127.0.0.1";

std::atomic<bool> shutting_down{false};
std::atomic<bool> lidar_running{false};
std::atomic<float> last_distance{9999.0f};
std::atomic<int>   listen_fd{-1};          // abychom jej mohli zavřít z libovolného threadu

std::mutex clients_mtx;
std::vector<int> client_socks;

void close_all_clients() {
    std::lock_guard<std::mutex> lock(clients_mtx);
    for (int s : client_socks) {
        ::shutdown(s, SHUT_RDWR);
        ::close(s);
    }
    client_socks.clear();
}

void stop_listener() {
    int fd = listen_fd.exchange(-1);
    if (fd >= 0) {
        ::shutdown(fd, SHUT_RDWR); // přeruší blocking accept()
        ::close(fd);
    }
}

void send_line(int sock, const std::string &msg) {
    std::string to_send = msg + "\n";
    ::send(sock, to_send.data(), to_send.size(), MSG_NOSIGNAL);
}

void handle_client(int sock) {
    {
        std::lock_guard<std::mutex> lock(clients_mtx);
        client_socks.push_back(sock);
    }

    //send_line(sock, "WELCOME");
    std::string buffer;
    char tmp[512];

    while (!shutting_down.load()) {
        ssize_t n = ::recv(sock, tmp, sizeof(tmp), 0);
        if (n <= 0) break; // client closed or error
        buffer.append(tmp, n);

        size_t pos;
        while ((pos = buffer.find('\n')) != std::string::npos) {
            std::string line = buffer.substr(0, pos);
            buffer.erase(0, pos + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();

            if (line == "PING") {
                send_line(sock, "PONG");
            } else if (line == "START") {
                lidar_running.store(true);
                send_line(sock, "OK STARTED");
            } else if (line == "STOP") {
                lidar_running.store(false);
                send_line(sock, "OK STOPPED");
            } else if (line == "DISTANCE") {
                send_line(sock, std::to_string(last_distance.load()));
            } else if (line == "EXIT") {
                send_line(sock, "BYE");
                ::shutdown(sock, SHUT_RDWR);
                break;
            } else if (line == "SHUTDOWN") {
                send_line(sock, "SHUTTING DOWN");
                shutting_down.store(true);
                stop_listener();        // přeruší accept()
                break;
            } else {
                send_line(sock, "ERR UNKNOWN COMMAND");
            }
        }
    }

    ::close(sock);
    {
        std::lock_guard<std::mutex> lock(clients_mtx);
        client_socks.erase(std::remove(client_socks.begin(), client_socks.end(), sock), client_socks.end());
    }
}

int main() {
    signal(SIGINT, [](int) {
        shutting_down.store(true);
        stop_listener();
    });

    int listen_sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) { std::perror("socket"); return 1; }
    listen_fd = listen_sock;          // uložíme pro jiné thready

    sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_port = htons(kPort);
    if (inet_pton(AF_INET, kBindAddr, &addr.sin_addr) <= 0) { std::perror("inet_pton"); return 1; }

    int opt = 1; setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if (bind(listen_sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) { std::perror("bind"); return 1; }
    if (listen(listen_sock, 8) < 0) { std::perror("listen"); return 1; }

    std::cout << "📡 robot-lidar TCP server naslouchá na " << kBindAddr << ":" << kPort << std::endl;

    while (!shutting_down.load()) {
        sockaddr_in cli_addr{}; socklen_t cli_len = sizeof(cli_addr);
        int client_sock = accept(listen_sock, reinterpret_cast<sockaddr *>(&cli_addr), &cli_len);
        if (client_sock < 0) {
            if (shutting_down.load()) break; // listener byl zavřen
            std::perror("accept"); continue;
        }
        std::thread(handle_client, client_sock).detach();
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
target_link_libraries(robot_lidar_tcp PRIVATE pthread)
*/
