#include "configuration.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <expected>

std::string Configuration::remove_whitespace(const std::string& str) {
    std::string result;
    std::copy_if(str.begin(), str.end(), std::back_inserter(result),
        [](char c) {
            return !std::isspace(static_cast<unsigned char>(c));
        });
    return result;
}

std::tuple<int, int> Configuration::split_by_comma(const std::string& input) {
    std::string cleaned = remove_whitespace(input);
    std::stringstream ss(cleaned);
    std::string part1, part2;

    if (!std::getline(ss, part1, ',') || !std::getline(ss, part2)) {
        throw std::invalid_argument("Invalid input format. Expected two comma-separated values.");
    }

    int val1 = std::stoi(part1);
    int val2 = std::stoi(part2);

    return std::make_tuple(val1, val2);
}

std::expected<Configuration, std::string> Configuration::fromFile(const std::filesystem::path& configPath) {
    Configuration config;
    
    std::ifstream configFile(configPath);
    if (!configFile) {
        return std::unexpected("Error opening config file: " + configPath.string());
    }

    std::string line;
    while (std::getline(configFile, line)) {
        if (line.empty() || line[0] == '#')
            continue;
            
        auto delimiterPos = line.find('=');
        if (delimiterPos == std::string::npos) 
            continue;
            
        std::string key = remove_whitespace(line.substr(0, delimiterPos));
        std::string value = remove_whitespace(line.substr(delimiterPos + 1));
        
        try {
            if (value.find(',') != std::string::npos) {
                auto [input, output] = split_by_comma(value);
                if (key == "final_drive") {
                    config.gear.final_drive = static_cast<double>(output) / input;
                }
                else if (key == "gear_ratio") {
                    config.gear.gear_ratios.push_back(static_cast<double>(output) / input);
                }
            }
            else if (key == "min_rpm") {
                config.gear.min_rpm = std::stoi(value);
            }
            else if (key == "max_rpm") {
                config.gear.max_rpm = std::stoi(value);
            }
            else if (key == "wheel_circumference") {
                config.gear.wheel_circumference = std::stod(value);
            }
        }
        catch (const std::exception& e) {
            return std::unexpected("Error parsing config line '" + line + "': " + e.what());
        }
    }

    // Validation
    std::ostringstream errorMsg;
    bool hasError = false;

    if (config.gear.gear_ratios.empty()) {
        hasError = true;
        errorMsg << "Missing: gear ratio data\n";
    }
    if (!config.gear.min_rpm.has_value() || !config.gear.max_rpm.has_value()) {
        hasError = true;
        errorMsg << "Missing: RPM range data\n";
    }
    if (!config.gear.wheel_circumference.has_value()) {
        hasError = true;
        errorMsg << "Missing: wheel circumference data\n";
    }
    if (!config.gear.final_drive.has_value()) {
        hasError = true;
        errorMsg << "Missing: final drive ratio\n";
    }

    if (hasError) {
        return std::unexpected(errorMsg.str());
    }

    return config;
}

Configuration Configuration::fromCommandLine(int argc, char* argv[]) {
    Configuration config;
    
    for (int i = 1; i < argc; ++i) {
        std::string_view arg = argv[i];
        if (arg == "--test" || arg == "-t") {
            config.app.test_mode = true;
        }
        else if (arg == "--debug" || arg == "-d") {
            config.app.debug_mode = true;
        }
        else if (arg == "--config" && i + 1 < argc) {
            config.app.config_path = argv[++i];
        }
        else if (arg == "--output" && i + 1 < argc) {
            config.app.output_path = argv[++i];
        }
        else if (arg == "--port" && i + 1 < argc) {
            config.app.serial_port = argv[++i];
        }
        else if (arg == "--baud" && i + 1 < argc) {
            config.app.baud_rate = std::stoi(argv[++i]);
        }
        else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Options:\n";
            std::cout << "  --test, -t          Use dummy ELM327 for testing\n";
            std::cout << "  --debug, -d         Enable debug output\n";
            std::cout << "  --config <file>     Specify config file path\n";
            std::cout << "  --output <file>     Specify output CSV file path\n";
            std::cout << "  --port <port>       Serial port (default: COM9)\n";
            std::cout << "  --baud <rate>       Baud rate (default: 38400)\n";
            std::cout << "  --help, -h          Show this help message\n";
            std::exit(0);
        }
    }
    
    return config;
}