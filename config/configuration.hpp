#pragma once

#include <filesystem>
#include <vector>
#include <optional>
#include <string>
#include <expected>
#include <tuple>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime> // for std::localtime

class Configuration {
public:
    struct GearConfig {
        std::vector<double> gear_ratios;
        std::optional<double> final_drive;
        std::optional<double> wheel_circumference;
        std::optional<int> min_rpm;
        std::optional<int> max_rpm;
    };

    struct AppConfig {
        std::filesystem::path config_path = "C:/Users/Will/Documents/rpmrevmatch/config.txt";
        std::filesystem::path output_path = make_output_path();
        std::filesystem::path raw_output_path = make_output_path();
        std::string serial_port = "COM9";
        int baud_rate = 38400;
        bool test_mode = false;
        bool debug_mode = false;
    };

    GearConfig gear;
    AppConfig app;
    static std::expected<Configuration, std::string> fromFile(const std::filesystem::path& configPath);
    static Configuration fromCommandLine(int argc, char* argv[]);

private:
    static std::string remove_whitespace(const std::string& str);
    static std::tuple<int, int> split_by_comma(const std::string& input);
    static inline std::filesystem::path make_output_path() {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm{};
    #ifdef _WIN32
            localtime_s(&local_tm, &t);
    #else
            localtime_r(&t, &local_tm);
    #endif
            std::ostringstream oss;
            oss << "logs/"
                << std::put_time(&local_tm, "%Y-%m-%d_%H-%M-%S")
                << ".csv";

            return oss.str();
    }


};
