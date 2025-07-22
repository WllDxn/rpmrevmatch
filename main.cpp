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

using boost::asio::serial_port_base;
namespace asio = boost::asio;
constexpr double TIRE_CONVERSION = 1056.0;
constexpr double KMH_TO_MPH = 0.621371;
constexpr const char *RPM_CMD = "010C";
constexpr const char *SPEED_CMD = "010D";
bool debug = false;
bool testMode = false;
std::atomic<bool> running{true};

void signalHandler(int signum)
{
    std::cout << "\nSIGINT received. Cleaning up...\n";
    running = false;
}

class ELM327Base
{
public:
    virtual ~ELM327Base() = default;
    virtual std::tuple<int, int> getRPMandSpeed() = 0;
    virtual bool isConnected() const = 0;
};

class GearBox
{
public:
    GearBox(const std::string &configPath = "C:\\Users\\Will\\Documents\\rpmrevmatch\\config.txt")
    {
        std::ifstream configFile(configPath);
        if (!configFile)
        {
            std::cerr << "Error opening config.txt\n";
            return;
        }

        std::string line;
        int postGearLines = 0;
        while (std::getline(configFile, line))
        {
            std::stringstream ss(line);
            if (line.find(',') != std::string::npos)
            {
                int input, output;
                char comma;
                if (ss >> input >> comma >> output && comma == ',')
                {
                    if (finalDrive == -1.0)
                    {
                        finalDrive = GearRatio{output, input}.ratio();
                    }
                    else
                    {
                        gearRatios.push_back(GearRatio{output, input}.ratio());
                    }
                }
                else
                {
                    std::cerr << "Invalid gear ratio line: " << line << '\n';
                }
            }
            else
            {
                if (postGearLines == 0)
                    ss >> minRPM;
                else if (postGearLines == 1)
                    ss >> maxRPM;
                else
                    ss >> wheelCircumference;

                if (!ss)
                {
                    std::cerr << "Invalid numeric value: " << line << '\n';
                }
                ++postGearLines;
            }
        }

        if (gearRatios.empty())
        {
            std::cerr << "No gear ratio data found.\n";
        }
        else
        {
            std::cout << "Final Drive Ratio: " << gearRatios[0] << "\nGear Ratios:\n";
            for (size_t i = 1; i < gearRatios.size(); ++i)
            {
                std::cout << "  Gear " << i << ": " << gearRatios[i] << '\n';
            }
        }
        std::cout << "RPM Range: " << minRPM << "-" << maxRPM << "\n";
        std::cout << "Wheel Circumference: " << wheelCircumference << " inches\n";
    }
    std::tuple<int, int> revMatcher(int MPH, int rpm)
    {
        int currentGear = getCurrentGear(rpm, MPH);
        if (currentGear < 2)
        {
            if (debug || testMode)
            {
                std::cout << "RPM: " << rpm << ", MPH: " << std::fixed << std::setprecision(1) << MPH << ", Current Gear: " << currentGear << "  --ERROR: Downshift not possible" << '\n';
            };
            return {currentGear, -1};
        }
        else
        {
            int targetRPM = static_cast<int>((TIRE_CONVERSION * MPH * finalDrive * gearRatios[currentGear - 1]) / wheelCircumference);
            if (targetRPM < minRPM || targetRPM > maxRPM)
            {
                if (debug || testMode)
                {
                    std::cout << "RPM: " << rpm << ", MPH: " << std::fixed << std::setprecision(1) << MPH << ", Target Gear: " << currentGear - 1 << ", Target RPM: " << targetRPM << "   --ERROR: Target RPM out of range" << '\n';
                };
                return {currentGear, -1};
            }
            else
            {
                if (debug || testMode)
                {
                    std::cout << "RPM: " << rpm << ", MPH: " << std::fixed << std::setprecision(1) << MPH << ", Target Gear: " << currentGear - 1 << ", Target RPM: " << targetRPM << '\n';
                };
                return {currentGear, targetRPM};
            };
        };
    }

private:
    std::vector<double> gearRatios;
    double finalDrive = -1.0;
    double wheelCircumference = 0.0;
    int minRPM = 0;
    int maxRPM = 0;

    struct GearRatio
    {
        int inputTeeth;
        int outputTeeth;
        double ratio() const
        {
            return static_cast<double>(inputTeeth) / outputTeeth;
        }
    };

    int getCurrentGear(int rpm, int mph)
    {
        double currentRatioValue = ((rpm * wheelCircumference) / (mph * TIRE_CONVERSION)) / finalDrive;
        for (size_t i = 0; i < gearRatios.size() - 1; ++i)
        {
            if (mph == 0)
                return 1;
            double midpoint = (gearRatios[i] + gearRatios[i + 1]) / 2.0;
            if (currentRatioValue > midpoint)
            {
                return (static_cast<int>(i) + 1);
            }
        }
        return static_cast<int>(gearRatios.size());
    };
};

class ELM327Interface : public ELM327Base
{
private:
    asio::io_context io;
    asio::serial_port serial;

public:
    ELM327Interface(const std::string &portName, unsigned int baudRate) : serial(io)
    {
        try
        {
            serial.open(portName);
            serial.set_option(serial_port_base::baud_rate(baudRate));
            serial.set_option(serial_port_base::character_size(8));
            serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
            serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
            serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
        }
        catch (const boost::system::system_error &e)
        {
            std::cerr << "Failed to open port: " << e.what() << '\n';
            throw;
        }
    }

    ~ELM327Interface()
    {
        if (serial.is_open())
        {
            serial.close();
        }
    }

    bool isConnected() const override
    {
        return serial.is_open();
    }

    std::string readUntil(char terminator)
    {
        std::string response;
        char c;
        do
        {
            asio::read(serial, asio::buffer(&c, 1));
            response += c;
        } while (c != terminator);
        return response;
    }

    std::tuple<int, int> getRPMandSpeed() override
    {
        int rpm = message(RPM_CMD);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        int speed = message(SPEED_CMD);
        return {rpm, speed};
    }

    int message(const std::string &cmd)
    {
        asio::write(serial, asio::buffer(cmd + '\r'));
        std::string raw = readUntil('>');
        std::string cleaned = cleanResponse(raw);
        auto tokens = tokenizeResponse(cleaned);
        if (cmd == RPM_CMD)
            return parseRPM(tokens);
        else if (cmd == SPEED_CMD)
            return parseSpeed(tokens);
        return -1;
    }

    int parseHexByte(const std::string &str)
    {
        return std::stoi(str, nullptr, 16);
    }

    std::vector<std::string> tokenizeResponse(const std::string &response)
    {
        std::istringstream ss(response);
        std::string token;
        std::vector<std::string> tokens;
        while (ss >> token)
            tokens.push_back(token);
        return tokens;
    }

    int parseRPM(const std::vector<std::string> &bytes)
    {
        if (bytes.size() >= 4 && bytes[0] == "010C41" && bytes[1] == "0C")
        {
            int A = parseHexByte(bytes[2]);
            int B = parseHexByte(bytes[3]);
            return ((A * 256) + B) / 4;
        }
        return -1;
    }

    int parseSpeed(const std::vector<std::string> &bytes)
    {
        if (bytes.size() >= 3 && bytes[0] == "010D41" && bytes[1] == "0D")
        {
            double speedMph = static_cast<double>(parseHexByte(bytes[2])) * KMH_TO_MPH;
            return static_cast<int>(speedMph);
        }
        return -1;
    }

    std::string cleanResponse(const std::string &raw)
    {
        std::string cleaned;
        for (char c : raw)
        {
            if (std::isxdigit(c) || c == ' ')
            {
                cleaned += c;
            }
        }
        return cleaned;
    }
};

// Dummy ELM327 Interface for testing
class DummyELM327 : public ELM327Base
{
private:
    std::mt19937 gen;
    std::uniform_int_distribution<> rpmDist;
    std::uniform_int_distribution<> speedDist;
    std::vector<std::pair<int, int>> testData;
    size_t dataIndex;
    bool useTestData;

public:
    DummyELM327(bool useRandomData = false) : gen(std::chrono::steady_clock::now().time_since_epoch().count() & 0xFFFFFFFF),
                                              rpmDist(800, 7000),
                                              speedDist(0, 80),
                                              dataIndex(0),
                                              useTestData(!useRandomData)
    {
        if (useTestData)
        {
            loadTestData();
        }
        std::cout << "DummyELM327 initialized " << (useTestData ? "with test data" : "with random data") << "\n";
    }

    bool isConnected() const override
    {
        return true;
    }

    std::tuple<int, int> getRPMandSpeed() override
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Simulate communication delay

        if (useTestData && !testData.empty())
        {
            auto data = testData[dataIndex % testData.size()];
            dataIndex++;
            return {data.first, data.second};
        }
        else
        {
            // Generate realistic correlated data
            int speed = speedDist(gen);
            int baseRpm = speed * 50 + 800;                // Base correlation
            int rpm = baseRpm + rpmDist(gen) % 1000 - 500; // Add some variance
            rpm = std::max(800, std::min(7000, rpm));      // Clamp to realistic range

            return {rpm, speed};
        }
    }

private:
    void loadTestData()
    {
        // Realistic test scenarios
        testData = {
            // City driving scenario
            {1200, 15},
            {1400, 20},
            {1600, 25},
            {2000, 30},
            {2200, 35},
            {2500, 40},
            {2800, 45},
            {3000, 50},

            // Highway acceleration
            {3200, 55},
            {3500, 60},
            {3800, 65},
            {4000, 70},
            {4200, 75},
            {3800, 80},
            {3500, 80},
            {3200, 80},

            // Deceleration for exit
            {3000, 75},
            {2800, 70},
            {2500, 65},
            {2200, 60},
            {2000, 55},
            {1800, 50},
            {1600, 45},
            {1400, 40},

            // Stop and go traffic
            {800, 0},
            {1000, 5},
            {1200, 10},
            {900, 0},
            {1100, 8},
            {1300, 15},
            {1000, 5},
            {800, 0},

            // Sporty driving
            {4500, 40},
            {5000, 50},
            {5500, 60},
            {4000, 65},
            {4500, 70},
            {3800, 75},
            {3500, 80},
            {3200, 80},
        };
    }
};

// Factory function for creating ELM327 interfaces
std::unique_ptr<ELM327Base> createELM327Interface(bool testMode, const std::string &port = "COM9", int baudRate = 9600)
{
    if (testMode)
    {
        return std::make_unique<DummyELM327>(false); // Use realistic test data
    }
    else
    {
        return std::make_unique<ELM327Interface>(port, baudRate);
    }
}

// ========== Main ==========
int main(int argc, char *argv[])
{
    signal(SIGINT, signalHandler);

    // Command line argument parsing

    std::string configPath = "C:\\Users\\Will\\Documents\\rpmrevmatch\\config.txt";
    std::string outputPath = "C:\\Users\\Will\\Documents\\rpmrevmatch\\output.csv";

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--test" || arg == "-t")
        {
            testMode = true;
        }
        else if (arg == "--debug" || arg == "-d")
        {
            debug = true;
        }
        else if (arg == "--config" && i + 1 < argc)
        {
            configPath = argv[++i];
        }
        else if (arg == "--output" && i + 1 < argc)
        {
            outputPath = argv[++i];
        }
        else if (arg == "--help" || arg == "-h")
        {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Options:\n";
            std::cout << "  --test, -t          Use dummy ELM327 for testing\n";
            std::cout << "  --debug, -d         Enable debug output\n";
            std::cout << "  --config <file>     Specify config file path\n";
            std::cout << "  --output <file>     Specify output CSV file path\n";
            std::cout << "  --help, -h          Show this help message\n";
            return 0;
        }
    }

    if (testMode)
    {
        std::cout << "Running in TEST MODE with dummy ELM327\n";
    }

    GearBox gearBox(configPath);

    try
    {
        std::ofstream outFile(outputPath);
        if (!outFile)
        {
            std::cerr << "Failed to open output file: " << outputPath << "\n";
            return 1;
        }

        outFile << "RPM,MPH,CurrentGear,RevMatch\n";

        auto elm = createELM327Interface(testMode);

        if (!elm->isConnected())
        {
            std::cerr << "Failed to connect to ELM327 device\n";
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int sampleCount = 0;
        const int maxSamples = testMode ? 100 : -1; // Limit samples in test mode

        while (running && (maxSamples < 0 || sampleCount < maxSamples))
        {
            auto [rpm, speed] = elm->getRPMandSpeed();
            if (rpm > 0 && speed >= 0) // Allow speed of 0
            {
                auto [gear, revs] = gearBox.revMatcher(speed, rpm);
                outFile << rpm << "," << std::fixed << std::setprecision(1) << speed << "," << gear << "," << revs << '\n';
                outFile.flush(); // Ensure data is written immediately

                sampleCount++;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        if (testMode)
        {
            std::cout << "\nTest completed. Generated " << sampleCount << " samples.\n";
            std::cout << "Output saved to: " << outputPath << "\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }

    std::cout << "Program terminated cleanly.\n";
    return 0;
}