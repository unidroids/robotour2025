#pragma once

#include <cstdint>
#include <string>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>   // C++17
#include <stdexcept>

#include "unitree_lidar_protocol.h"  // LidarPointDataPacket, LidarImuDataPacket, LidarVersionDataPacket

// Pokud by std::filesystem dělal problémy (starší g++), můžeš
// implementaci makeDefaultPath() přepsat na POSIX mkdir.

enum class RawRecordType : uint8_t {
    Point   = 1,
    Imu     = 2,
    Version = 3,
};

#pragma pack(push, 1)
struct LogRecordHeader
{
    uint8_t  type;           // viz RawRecordType
    uint8_t  reserved[3];    // zarovnání / future use
    uint64_t mono_ts_ns;     // monotonic timestamp hosta v ns
    uint32_t payload_size;   // velikost payloadu v bajtech (mělo by odpovídat header.packet_size)
};
#pragma pack(pop)

static_assert(sizeof(LogRecordHeader) == 1 + 3 + 8 + 4,
              "LogRecordHeader must be packed as 16 bytes");

class LidarRawLogger
{
public:
    /// Vytvoří logger a otevře nové logovací soubory.
    /// base_dir: root pro logy, defaultně "/data/robot/lidar".
    explicit LidarRawLogger(const std::string& base_dir = "/data/robot/lidar")
    {
        path_ = makeDefaultPath(base_dir);
        openStream();
    }

    ~LidarRawLogger()
    {
        if (ofs_.is_open()) {
            ofs_.flush();
            ofs_.close();
        }
    }

    // nekopírovatelné
    LidarRawLogger(const LidarRawLogger&) = delete;
    LidarRawLogger& operator=(const LidarRawLogger&) = delete;

    // přesun povolený (když bys chtěl)
    LidarRawLogger(LidarRawLogger&& other) noexcept
        : ofs_(std::move(other.ofs_))
        , path_(std::move(other.path_))
    {}

    LidarRawLogger& operator=(LidarRawLogger&& other) noexcept
    {
        if (this != &other) {
            if (ofs_.is_open()) {
                ofs_.close();
            }
            ofs_  = std::move(other.ofs_);
            path_ = std::move(other.path_);
        }
        return *this;
    }

    bool isOpen() const noexcept { return ofs_.is_open(); }
    const std::string& path() const noexcept { return path_; }

    /// Zápis 3D point packetu
    void writePointPacket(const unilidar_sdk2::LidarPointDataPacket& pkt,
                          uint64_t mono_ts_ns)
    {
        writeAnyPacket(RawRecordType::Point,
                       reinterpret_cast<const uint8_t*>(&pkt),
                       pkt.header.packet_size,
                       sizeof(pkt),
                       mono_ts_ns);
    }

    /// Zápis IMU packetu
    void writeImuPacket(const unilidar_sdk2::LidarImuDataPacket& pkt,
                        uint64_t mono_ts_ns)
    {
        writeAnyPacket(RawRecordType::Imu,
                       reinterpret_cast<const uint8_t*>(&pkt),
                       pkt.header.packet_size,
                       sizeof(pkt),
                       mono_ts_ns);
    }

    /// Zápis VERSION packetu
    void writeVersionPacket(const unilidar_sdk2::LidarVersionDataPacket& pkt,
                            uint64_t mono_ts_ns)
    {
        writeAnyPacket(RawRecordType::Version,
                       reinterpret_cast<const uint8_t*>(&pkt),
                       pkt.header.packet_size,
                       sizeof(pkt),
                       mono_ts_ns);
    }

private:
    std::ofstream ofs_;
    std::string   path_;

    static std::string makeDefaultPath(const std::string& base_dir)
    {
        namespace fs = std::filesystem;

        // systémový čas pro jméno souboru
        const auto now = std::chrono::system_clock::now();
        const std::time_t t = std::chrono::system_clock::to_time_t(now);

        std::tm tm{};
        // POSIX varianta; na Windows by bylo potřeba localtime_s
        localtime_r(&t, &tm);

        std::ostringstream date_dir_ss;
        date_dir_ss << base_dir << '/'
                    << std::put_time(&tm, "%Y-%m-%d");

        fs::path date_dir_path{date_dir_ss.str()};
        fs::create_directories(date_dir_path);

        std::ostringstream file_ss;
        file_ss << "raw-" << std::put_time(&tm, "%H-%M-%S") << ".dat";

        fs::path file_path = date_dir_path / file_ss.str();
        return file_path.string();
    }

    void openStream()
    {
        ofs_.open(path_, std::ios::out | std::ios::binary);
        if (!ofs_) {
            // Tady se dá místo výjimky jen nastavit "logging disabled".
            throw std::runtime_error("LidarRawLogger: failed to open log file: " + path_);
        }

        // (volitelné) souborová hlavička s "magic" a verzí formátu
        // Můžeš vynechat, jestli nechceš globální header.
        const char magic[8] = {'L','2','R','A','W','0','1','\0'};
        ofs_.write(magic, sizeof(magic));
    }

    void writeAnyPacket(RawRecordType type,
                        const uint8_t* pkt_data,
                        uint32_t packet_size_field,
                        size_t   packet_object_size,
                        uint64_t mono_ts_ns)
    {
        if (!ofs_.is_open()) {
            return; // logging vypnutý / neotevřený soubor
        }

        // Bezpečnostní kontrola: lidar tvrdí, kolik má mít packet bajtů.
        // Nemělo by nikdy přesáhnout velikost objektu v paměti.
        if (packet_size_field == 0 || packet_size_field > packet_object_size) {
            // Poškozená data – záznam přeskočíme.
            return;
        }

        LogRecordHeader hdr{};
        hdr.type         = static_cast<uint8_t>(type);
        hdr.mono_ts_ns   = mono_ts_ns;
        hdr.payload_size = packet_size_field;

        ofs_.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
        ofs_.write(reinterpret_cast<const char*>(pkt_data),
                   static_cast<std::streamsize>(packet_size_field));
    }
};
