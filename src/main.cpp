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
#include <thread>  
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
    
    std::tuple<int, int> revMatcher(int MPH, int rpm, int64_t now)
    {
        auto [dR, dM] = getDerivatives(rpm, MPH, now);
        bool diverging = (dR * dM <= 0);
        bool rpmDecrease = (dR < 0);
        
        int currentGear = !(diverging || rpmDecrease) ? getCurrentGear(rpm, MPH) : previousGear;
        previousGear = currentGear;
        
        if (currentGear < 2)
        {
            return {currentGear, -1};
        }
        else
        {
            // for (auto g : gearRatios)
            // {
                //     std::cout << static_cast<int>((TIRE_CONVERSION * MPH * finalDrive * g) / wheelCircumference) << "  ";
                // }
                int cRPM = static_cast<int>((TIRE_CONVERSION * MPH * finalDrive * gearRatios[currentGear - 1]) / wheelCircumference);
                if (!(diverging || rpmDecrease)){

                    myfile.open("logs/temp.txt", std::ofstream::app);
                    myfile << currentGear << "  " << ((static_cast<double>(rpm-cRPM)/rpm))*100<< '\n';
                    myfile.close();
                }
                int targetRPM = static_cast<int>((TIRE_CONVERSION * MPH * finalDrive * gearRatios[currentGear - 2]) / wheelCircumference);
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
    std::ofstream myfile;
    std::vector<double> gearRatios;
    double finalDrive;
    double wheelCircumference;
    int minRPM;
    int maxRPM;
    double TIRE_CONVERSION = 1056.0;
    double KMH_TO_MPH = 0.621371;
    int previousGear = 1;
    struct DataPoint {
        int64_t timestamp;
        int rpm;
        int mph;
        
        DataPoint(int64_t t, int r, int m) : timestamp(t), rpm(r), mph(m) {}
    };
    static constexpr size_t MAX_POINTS = 100; // Adjust based on your needs
    std::vector<DataPoint> buffer;
    size_t start = 0;
    size_t count = 0;
    static constexpr int threshold = 250;

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
            if (currentRatioValue > gearRatios[left]) {
                return left+1;
            }
            left++;
        }
        return left+1;
    }
    std::pair<double, double> getDerivatives(int rpm, int mph, int64_t now) {
        updatePrevious(now, rpm, mph);
        
        if (count < 2) {
            return {0.0, 0.0};
        }
        
        double rpmSum = 0.0;
        double mphSum = 0.0;
        
        for (size_t i = 1; i < count; ++i) {
            const auto& p1 = buffer[(start + i - 1) % MAX_POINTS];
            const auto& p2 = buffer[(start + i) % MAX_POINTS];
            
            int64_t dt = p2.timestamp - p1.timestamp;
            if (dt > 0) {
                rpmSum += static_cast<double>(p2.rpm - p1.rpm) / dt;
                mphSum += static_cast<double>(p2.mph - p1.mph) / dt;
            }
        }
        
        return {rpmSum / (count - 1), mphSum / (count - 1)};
    }

    void updatePrevious(int64_t now, int rpm, int mph) {
        while (count > 0) {
            const auto& oldest = buffer[start];
            if (oldest.timestamp > now - threshold) {
                break;
            }
            start = (start + 1) % MAX_POINTS;
            --count;
        }
        
        if (count < MAX_POINTS) {
            size_t pos = (start + count) % MAX_POINTS;
            if (pos >= buffer.size()) {
                buffer.emplace_back(now, rpm, mph);
            } else {
                buffer[pos] = DataPoint(now, rpm, mph);
            }
            ++count;
        } else {
            buffer[start] = DataPoint(now, rpm, mph);
            start = (start + 1) % MAX_POINTS;
        }
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
        auto now = std::chrono::system_clock::now();
        auto oldtimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        while (handler.isRunning()) {
            auto result = elm->getEngineData(handler, "01 0C 0D 04 11 05 4\r");
            if (!result) {
                if (finalConfig.app.debug_mode) {
                    std::cerr << "Parse error: " << result.error() << std::endl;
                }
                continue;
            }

            auto [rpm, speed, load, throttle, timestamp] = result.value();
            if (finalConfig.app.test_mode){
                std::this_thread::sleep_for(std::chrono::milliseconds(timestamp - oldtimestamp)); 
                oldtimestamp = timestamp;
            }
            if (rpm > 0 && speed >= 0) {

                auto [gear, revs] = gearBox.revMatcher(speed, rpm, timestamp);
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
