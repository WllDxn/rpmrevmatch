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

/**
 * @file main.cpp
 * @brief Implementation of an OBD-II ELM327 interface for calculating rev-matching RPMs based on vehicle speed and gear ratios.
 * @author William Dixon
 * @date 2025-07-22
 *
 * This program interfaces with an ELM327 OBD-II device (or a dummy simulator) to retrieve vehicle RPM and speed data,
 * processes it through a gearbox model to calculate rev-matching RPMs for downshifts, and logs the results to a CSV file.
 * It supports both real hardware and test modes with configurable parameters.
 */

using boost::asio::serial_port_base;
namespace asio = boost::asio;

/** @brief Conversion factor for tire calculations (revolutions per mile). */
constexpr double TIRE_CONVERSION = 1056.0;
/** @brief Conversion factor from km/h to mph. */
constexpr double KMH_TO_MPH = 0.621371;
/** @brief OBD-II command for retrieving RPM. */
constexpr const char *RPM_CMD = "010C";
/** @brief OBD-II command for retrieving vehicle speed. */
constexpr const char *SPEED_CMD = "010D";
/** @brief Flag to enable debug output. */
bool debug = false;
/** @brief Flag to enable test mode with dummy data. */
bool testMode = false;
/** @brief Atomic flag to control program execution during signal handling. */
std::atomic<bool> running{true};

/**
 * @brief Signal handler for SIGINT to gracefully terminate the program.
 * @param signum The signal number (e.g., SIGINT).
 */
void signalHandler(int signum)
{
    std::cout << "\nSIGINT received. Cleaning up...\n";
    running = false;
}

/**
 * @class ELM327Base
 * @brief Abstract base class for ELM327 interfaces.
 *
 * Defines the interface for communicating with an ELM327 device or a dummy simulator.
 * Provides methods to retrieve RPM and speed data and check connection status.
 */
class ELM327Base
{
public:
    /** @brief Virtual destructor to ensure proper cleanup in derived classes. */
    virtual ~ELM327Base() = default;

    /**
     * @brief Retrieves current RPM and speed from the ELM327 device.
     * @return A tuple containing RPM (int) and speed (int) in MPH.
     */
    virtual std::tuple<int, int> getRPMandSpeed() = 0;

    /**
     * @brief Checks if the ELM327 device is connected.
     * @return True if connected, false otherwise.
     */
    virtual bool isConnected() const = 0;
};

/**
 * @class GearBox
 * @brief Manages gear ratios and calculates rev-matching RPMs for downshifts.
 *
 * Reads gear ratio and vehicle configuration from a file and provides methods
 * to calculate the current gear and target RPM for rev-matching during downshifts.
 */
class GearBox
{
public:
    /**
     * @brief Constructs a GearBox object by reading configuration from a file.
     * @param configPath Path to the configuration file (default: "C:\\Users\\Will\\Documents\\rpmrevmatch\\config.txt").
     *
     * The configuration file should contain gear ratios (input,output), final drive ratio,
     * minimum RPM, maximum RPM, and wheel circumference.
     */
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

    /**
     * @brief Calculates the current gear and target RPM for a downshift.
     * @param MPH Vehicle speed in miles per hour.
     * @param rpm Current engine RPM.
     * @return A tuple containing the current gear (int) and target RPM (int) for the next lower gear.
     *         Returns -1 for target RPM if downshift is not possible or out of range.
     */
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
    /** @brief Vector of gear ratios (including final drive as first element). */
    std::vector<double> gearRatios;
    /** @brief Final drive ratio of the vehicle. */
    double finalDrive = -1.0;
    /** @brief Wheel circumference in inches. */
    double wheelCircumference = 0.0;
    /** @brief Minimum allowed RPM for rev-matching. */
    int minRPM = 0;
    /** @brief Maximum allowed RPM for rev-matching. */
    int maxRPM = 0;

    /**
     * @struct GearRatio
     * @brief Represents a gear ratio with input and output teeth counts.
     */
    struct GearRatio
    {
        /** @brief Number of input teeth. */
        int inputTeeth;
        /** @brief Number of output teeth. */
        int outputTeeth;

        /**
         * @brief Calculates the gear ratio.
         * @return The gear ratio as inputTeeth/outputTeeth.
         */
        double ratio() const
        {
            return static_cast<double>(inputTeeth) / outputTeeth;
        }
    };

    /**
     * @brief Determines the current gear based on RPM and speed.
     * @param rpm Current engine RPM.
     * @param mph Current vehicle speed in miles per hour.
     * @return The current gear number (1-based indexing).
     */
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

/**
 * @class ELM327Interface
 * @brief Implementation of ELM327Base for real ELM327 OBD-II device communication.
 *
 * Communicates with an ELM327 device over a serial port to retrieve vehicle data
 * such as RPM and speed using OBD-II commands.
 */
class ELM327Interface : public ELM327Base
{
private:
    /** @brief Boost.Asio I/O context for serial communication. */
    asio::io_context io;
    /** @brief Serial port object for ELM327 communication. */
    asio::serial_port serial;

public:
    /**
     * @brief Constructs an ELM327 interface for a specified serial port.
     * @param portName The name of the serial port (e.g., "COM9").
     * @param baudRate The baud rate for serial communication (e.g., 9600).
     * @throws boost::system::system_error if the port cannot be opened or configured.
     */
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

    /**
     * @brief Destructor to ensure the serial port is closed.
     */
    ~ELM327Interface()
    {
        if (serial.is_open())
        {
            serial.close();
        }
    }

    /**
     * @brief Checks if the ELM327 device is connected.
     * @return True if the serial port is open, false otherwise.
     */
    bool isConnected() const override
    {
        return serial.is_open();
    }

    /**
     * @brief Reads data from the serial port until a terminator character is received.
     * @param terminator The character that indicates the end of the response.
     * @return The response string read from the serial port.
     */
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

    /**
     * @brief Retrieves RPM and speed from the ELM327 device.
     * @return A tuple containing RPM (int) and speed (int) in MPH.
     */
    std::tuple<int, int> getRPMandSpeed() override
    {
        int rpm = message(RPM_CMD);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        int speed = message(SPEED_CMD);
        return {rpm, speed};
    }

    /**
     * @brief Sends an OBD-II command to the ELM327 and processes the response.
     * @param cmd The OBD-II command to send (e.g., "010C" for RPM).
     * @return The parsed value (RPM or speed) or -1 if parsing fails.
     */
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

    /**
     * @brief Parses a hexadecimal byte string to an integer.
     * @param str The hexadecimal string to parse.
     * @return The integer value of the hexadecimal string.
     */
    int parseHexByte(const std::string &str)
    {
        return std::stoi(str, nullptr, 16);
    }

    /**
     * @brief Tokenizes an ELM327 response into individual strings.
     * @param response The cleaned response string.
     * @return A vector of tokenized strings.
     */
    std::vector<std::string> tokenizeResponse(const std::string &response)
    {
        std::istringstream ss(response);
        std::string token;
        std::vector<std::string> tokens;
        while (ss >> token)
            tokens.push_back(token);
        return tokens;
    }

    /**
     * @brief Parses RPM from tokenized ELM327 response.
     * @param bytes The tokenized response from the ELM327 device.
     * @return The calculated RPM value or -1 if parsing fails.
     */
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

    /**
     * @brief Parses speed from tokenized ELM327 response.
     * @param bytes The tokenized response from the ELM327 device.
     * @return The calculated speed in MPH or -1 if parsing fails.
     */
    int parseSpeed(const std::vector<std::string> &bytes)
    {
        if (bytes.size() >= 3 && bytes[0] == "010D41" && bytes[1] == "0D")
        {
            double speedMph = static_cast<double>(parseHexByte(bytes[2])) * KMH_TO_MPH;
            return static_cast<int>(speedMph);
        }
        return -1;
    }

    /**
     * @brief Cleans the raw ELM327 response by keeping only hexadecimal and space characters.
     * @param raw The raw response string from the ELM327 device.
     * @return The cleaned response string.
     */
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

/**
 * @class DummyELM327
 * @brief A mock ELM327 implementation for testing purposes.
 *
 * Simulates ELM327 data by generating random or predefined RPM and speed values,
 * useful for testing the GearBox class without a real ELM327 device.
 */
class DummyELM327 : public ELM327Base
{
private:
    /** @brief Random number generator for simulating data. */
    std::mt19937 gen;
    /** @brief Distribution for generating random RPM values. */
    std::uniform_int_distribution<> rpmDist;
    /** @brief Distribution for generating random speed values. */
    std::uniform_int_distribution<> speedDist;
    /** @brief Predefined test data for realistic scenarios. */
    std::vector<std::pair<int, int>> testData;
    /** @brief Current index in the test data vector. */
    size_t dataIndex;
    /** @brief Flag to use predefined test data instead of random data. */
    bool useTestData;

public:
    /**
     * @brief Constructs a DummyELM327 object.
     * @param useRandomData If true, generates random data; if false, uses predefined test data.
     */
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

    /**
     * @brief Checks if the dummy ELM327 is connected (always true).
     * @return Always returns true.
     */
    bool isConnected() const override
    {
        return true;
    }

    /**
     * @brief Simulates retrieving RPM and speed data.
     * @return A tuple containing simulated RPM (int) and speed (int) in MPH.
     */
    std::tuple<int, int> getRPMandSpeed() override
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

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
            int baseRpm = speed * 50 + 800;                
            int rpm = baseRpm + rpmDist(gen) % 1000 - 500; 
            rpm = std::max(800, std::min(7000, rpm));      

            return {rpm, speed};
        }
    }

private:
    /**
     * @brief Loads predefined test data for realistic driving scenarios.
     */
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

/**
 * @brief Creates an ELM327 interface based on the operation mode.
 * @param testMode If true, creates a DummyELM327; otherwise, creates an ELM327Interface.
 * @param port The serial port name for real ELM327 (default: "COM9").
 * @param baudRate The baud rate for real ELM327 (default: 9600).
 * @return A unique_ptr to an ELM327Base object.
 */
std::unique_ptr<ELM327Base> createELM327Interface(bool testMode, const std::string &port = "COM9", int baudRate = 9600)
{
    if (testMode)
    {
        return std::make_unique<DummyELM327>(false); 
    }
    else
    {
        return std::make_unique<ELM327Interface>(port, baudRate);
    }
}

/**
 * @brief Main function to run the rev-matching application.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit status (0 for success, 1 for error).
 *
 * Processes command-line arguments, initializes the gearbox and ELM327 interface,
 * and continuously logs RPM, speed, gear, and rev-matching data to a CSV file.
 */
int main(int argc, char *argv[])
{
    signal(SIGINT, signalHandler);

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
        const int maxSamples = testMode ? 100 : -1; 

        while (running && (maxSamples < 0 || sampleCount < maxSamples))
        {
            auto [rpm, speed] = elm->getRPMandSpeed();
            if (rpm > 0 && speed >= 0) 
            {
                auto [gear, revs] = gearBox.revMatcher(speed, rpm);
                outFile << rpm << "," << std::fixed << std::setprecision(1) << speed << "," << gear << "," << revs << '\n';
                outFile.flush();

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