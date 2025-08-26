#include <boost/asio.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <thread>
#include <string>
#include <iomanip>
#include <tuple>
#include <csignal>
#include <atomic>
#include <random>
#include <chrono>
#include <ctime>
#include <format>
#include <charconv>
#include <optional>
#include <boost/algorithm/string.hpp>
#include <filesystem>
#include <expected>
#include <stdexcept>
#include "../utils/error_handling.hpp"
#include "../utils/signal_handler.hpp"
#include "../config/configuration.hpp"
#include "../io/csv_writer.hpp"
#include "../io/elm327.hpp"
#include <SDKDDKVer.h>

using boost::asio::serial_port_base;
namespace asio = boost::asio;

class GearBox
{
public:
    explicit GearBox(const Configuration::GearConfig& config)
        : gearRatios(config.gear_ratios),
          finalDrive(config.final_drive.value()),
          wheelCircumference(config.wheel_circumference.value()),
          minRPM(config.min_rpm.value()),
          maxRPM(config.max_rpm.value())
    {
        std::cout << "Final Drive Ratio: " << finalDrive << "\nGear Ratios:\n";
        for (size_t i = 0; i < gearRatios.size(); ++i) {
            std::cout << "  Gear " << i+1 << ": " << gearRatios[i] << '\n';
        }
        std::cout << "RPM Range: " << minRPM << "-" << maxRPM << "\n";
        std::cout << "Wheel Circumference: " << wheelCircumference << " inches\n";
    }

    std::tuple<int, int> revMatcher(int MPH, int rpm)
    {
        int currentGear = getCurrentGear(rpm, MPH);
        if (currentGear < 2)
        {
            return {currentGear, -1};
        }
        else
        {
            int targetRPM = static_cast<int>((TIRE_CONVERSION * MPH * finalDrive * gearRatios[currentGear - 1]) / wheelCircumference);
            if (targetRPM < minRPM || targetRPM > maxRPM)
            {
                return {currentGear, -1};
            }
            else
            {
                return {currentGear, targetRPM};
            }
        }
    }

private:
    std::vector<double> gearRatios;
    double finalDrive;
    double wheelCircumference;
    int minRPM;
    int maxRPM;
    double TIRE_CONVERSION = 1056.0;
    double KMH_TO_MPH = 0.621371;
    
    int getCurrentGear(int rpm, int mph)
    {
        
        if (mph == 0) {
            return 1;
        }
        double currentRatioValue = ((rpm * wheelCircumference) / (mph * TIRE_CONVERSION)) / finalDrive;
        if (currentRatioValue >= gearRatios[0]) {
            return 1;
        }
        if (currentRatioValue <= gearRatios.back()) {
            return static_cast<int>(gearRatios.size() - 1);
        }

        int left = 0;
        while (left<gearRatios.size()-1) {
            double midpoint = (gearRatios[left] + gearRatios[left + 1]) / 2.0;
            if (currentRatioValue > midpoint) {
                return left+1;
            }
            left++;
        }
        return left+1;
    }
};



std::unique_ptr<ELM327Base> createELM327Interface(bool testMode, const std::string &port = "COM9", int baudRate = 38400)
{
    if (testMode)
    {
        return std::make_unique<DummyELM327>(true);
    }
    else
    {
        return std::make_unique<ELM327Interface>(port, baudRate);
    }
}


std::unique_ptr<BufferedCSVWriterBase> createBufferedCSVWriter(const std::filesystem::path& filename, bool testMode, bool debugMode) {
    if (testMode)
    {
        return std::make_unique<DummyCSVWriter>();
    }
    else
    {
        return std::make_unique<BufferedCSVWriter>(filename, debugMode);
    }
}



int main(int argc, char *argv[])
{
    SignalHandler handler;
    Configuration config = Configuration::fromCommandLine(argc, argv);
    auto fileConfigResult = Configuration::fromFile(config.app.config_path);

    if (!fileConfigResult) {
        std::cerr << "Configuration error: " << fileConfigResult.error() << std::endl;
        return 1;
    }

    Configuration finalConfig = *fileConfigResult;
    finalConfig.app = config.app;

    if (finalConfig.app.test_mode) {
        std::cout << "Running in TEST MODE with dummy ELM327\n";
    }

    try {
        GearBox gearBox(finalConfig.gear);
        auto csvWriter = createBufferedCSVWriter(finalConfig.app.output_path, finalConfig.app.test_mode, finalConfig.app.debug_mode);
        auto elm = createELM327Interface(finalConfig.app.test_mode, finalConfig.app.serial_port, finalConfig.app.baud_rate);

        if (!elm->isConnected()) {
            std::cerr << "Failed to connect to ELM327 device\n";
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        int sampleCount = 0;

        while (handler.isRunning()) {
            auto result = elm->getEngineData(handler, "01 0C 0D 04 11 05 4\r");
            if (!result) {
                if (finalConfig.app.debug_mode) {
                    std::cerr << "Parse error: " << result.error() << std::endl;
                }
                continue;
            }

            auto [rpm, speed, load, throttle] = result.value();

            if (rpm > 0 && speed >= 0) {
                auto [gear, revs] = gearBox.revMatcher(speed, rpm);
                auto now = std::chrono::system_clock::now();
                auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

                csvWriter->writeRow(rpm, speed, gear, revs, load, throttle, timestamp);
                sampleCount++;
            }
        }

        std::cout << "\nTest completed. Generated " << sampleCount << " samples.\n";
    }
    catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }

    std::cout << "Program terminated cleanly.\n";
    return 0;
}
