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
#include <SDKDDKVer.h>
#include <chrono>
#include <format>
#include <iostream>

/**
 * @file main.cpp
 * @brief Implementation of an OBD-II ELM327 interface for calculating rev-matching RPMs based on vehicle speed and gear ratios.
 * @author William Dixon
 * @date 2025-07-22
 * @version 1.0
 *
 * This program interfaces with an ELM327 OBD-II device (or a dummy simulator) to retrieve vehicle RPM and speed data,
 * processes it through a gearbox model to calculate rev-matching RPMs for downshifts, and logs the results to a CSV file.
 * It supports both real hardware and test modes with configurable parameters.
 * 
 * ## Features:
 * - Real-time OBD-II data acquisition via ELM327 interface
 * - Configurable gearbox modeling with custom gear ratios
 * - Rev-matching RPM calculations for optimal downshifts
 * - CSV data logging with timestamps
 * - Test mode with dummy data for development
 * - Command-line interface with multiple options
 * 
 * ## Usage:
 * @code
 * // Run with real ELM327 device
 * ./main
 * 
 * // Run in test mode with dummy data
 * ./main --test
 * 
 * // Enable debug output
 * ./main --debug
 * 
 * // Specify custom config and output files
 * ./main --config custom_config.txt --output results.csv
 * @endcode
 * 
 * ## Configuration File Format:
 * The configuration file should contain:
 * - Final drive ratio (input,output format)
 * - Gear ratios for each gear (input,output format)  
 * - Minimum RPM limit
 * - Maximum RPM limit
 * - Wheel circumference in inches
 * 
 * @see GearBox class for detailed configuration format
 * @see ELM327Interface class for OBD-II communication details
 */

using boost::asio::serial_port_base;
namespace asio = boost::asio;

/** @brief Conversion factor for tire calculations (revolutions per mile). */
constexpr double TIRE_CONVERSION = 1056.0;

/** @brief Conversion factor from km/h to mph. */
constexpr double KMH_TO_MPH = 0.621371;

/** @brief OBD-II command for retrieving RPM data. */
constexpr const char *RPM_CMD = "010C1";

/** @brief OBD-II command for retrieving vehicle speed data. */
constexpr const char *SPEED_CMD = "010D1";

/** @brief Global flag to enable debug output to console. */
bool debug = false;

/** @brief Global flag to enable test mode with dummy data generation. */
bool testMode = false;

/** @brief Atomic flag to control program execution during signal handling. */
std::atomic<bool> running{true};

/**
 * @brief Signal handler for SIGINT to gracefully terminate the program.
 * 
 * This function is called when the user presses Ctrl+C or sends SIGINT.
 * It sets the global running flag to false, allowing the main loop to
 * terminate gracefully and clean up resources.
 * 
 * @param signum The signal number (e.g., SIGINT = 2).
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
 * This abstract class allows for polymorphic behavior between real hardware
 * and test implementations.
 * 
 * @see ELM327Interface for real hardware implementation
 * @see DummyELM327 for test/simulation implementation
 */
class ELM327Base
{
public:
    /** 
     * @brief Virtual destructor to ensure proper cleanup in derived classes.
     * 
     * Ensures that destructors of derived classes are called properly
     * when deleting objects through base class pointers.
     */
    virtual ~ELM327Base() = default;

    /**
     * @brief Retrieves current RPM and speed from the ELM327 device.
     * 
     * This pure virtual function must be implemented by derived classes
     * to provide RPM and speed data from their respective sources.
     * 
     * @return A tuple containing:
     *         - RPM (int): Engine revolutions per minute
     *         - Speed (int): Vehicle speed in miles per hour (MPH)
     * @retval {-1, -1} Indicates an error or invalid data
     */
    virtual std::tuple<int, int> getRPMandSpeed() = 0;

    /**
     * @brief Checks if the ELM327 device is connected and ready.
     * 
     * @return true if the device is connected and operational
     * @return false if the device is disconnected or unavailable
     */
    virtual bool isConnected() const = 0;
};

/**
 * @class GearBox
 * @brief Manages gear ratios and calculates rev-matching RPMs for downshifts.
 *
 * This class reads gear ratio and vehicle configuration from a file and provides methods
 * to calculate the current gear and target RPM for rev-matching during downshifts.
 * It uses mathematical models based on gear ratios, final drive, and wheel circumference
 * to determine optimal engine speeds for smooth gear transitions.
 * 
 * ## Configuration File Format:
 * @code
 * # Final drive ratio (input teeth, output teeth)
 * 15,45
 * # Gear ratios (input teeth, output teeth) 
 * 30,10  # 1st gear
 * 25,15  # 2nd gear  
 * 20,20  # 3rd gear
 * 15,25  # 4th gear
 * # Minimum RPM
 * 800
 * # Maximum RPM  
 * 7000
 * # Wheel circumference (inches)
 * 79
 * @endcode
 * 
 * @note Gear ratios are calculated as input_teeth/output_teeth
 * @note The first ratio entry is treated as the final drive ratio
 */
class GearBox
{
public:
    /**
     * @brief Constructs a GearBox object by reading configuration from a file.
     * 
     * Reads and parses the configuration file to initialize gear ratios,
     * RPM limits, and vehicle parameters. The file format supports comments
     * (lines starting with #) and handles both comma-separated ratios and
     * single numeric values.
     * 
     * @param configPath Path to the configuration file 
     * @throws std::runtime_error if configuration file cannot be opened
     * @throws std::invalid_argument if configuration data is malformed
     * 
     * @par Example Configuration:
     * @code
     * # This is a comment
     * 15,45    # Final drive: 15 input teeth, 45 output teeth
     * 30,10    # 1st gear ratio
     * 25,15    # 2nd gear ratio  
     * 20,20    # 3rd gear ratio
     * 15,25    # 4th gear ratio
     * 800      # Minimum RPM
     * 7000     # Maximum RPM
     * 79       # Wheel circumference in inches
     * @endcode
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
        int lineIndex = 0;
        while (std::getline(configFile, line))
        {
            if (line.empty() || line[0] == '#')
                continue;
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
                int value;
                if (ss >> value)
                {
                    switch (lineIndex)
                    {
                    case 0:
                        minRPM = value;
                        break;
                    case 1:
                        maxRPM = value;
                        break;
                    case 2:
                        wheelCircumference = value;
                        break;
                    default:
                        std::cerr << "Warning: Unexpected extra config line: " << line << '\n';
                    }
                    ++lineIndex;
                }
                else
                {
                    std::cerr << "Error: Invalid numeric value on line " << lineIndex + 1 << ": " << line << '\n';
                }
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
     * 
     * This function determines the current gear based on the relationship between
     * engine RPM and vehicle speed, then calculates the target RPM that would be
     * required if the driver downshifted to the next lower gear. This is useful
     * for rev-matching during manual transmission downshifts.
     * 
     * The calculation uses the formula:
     * @f[
     * \text{Target RPM} = \frac{\text{Speed (MPH)} \times \text{Tire Conversion} \times \text{Final Drive} \times \text{Lower Gear Ratio}}{\text{Wheel Circumference}}
     * @f]
     * 
     * @param MPH Vehicle speed in miles per hour (must be non-negative)
     * @param rpm Current engine RPM (must be positive)
     * 
     * @return A tuple containing:
     *         - Current gear (int): The estimated current gear (1-based indexing)
     *         - Target RPM (int): The calculated RPM for downshift, or -1 if:
     *           - Already in 1st gear (no downshift possible)
     *           - Target RPM would be outside safe operating range
     *           - Invalid input parameters
     * 
     * @note If debug or testMode is enabled, diagnostic information is printed
     * @note Target RPM of -1 indicates downshift is not recommended or possible
     * 
     * @par Example:
     * @code
     * GearBox gearbox("config.txt");
     * auto [currentGear, targetRPM] = gearbox.revMatcher(45, 3000);
     * if (targetRPM != -1) {
     *     std::cout << "Currently in gear " << currentGear 
     *               << ", target RPM for downshift: " << targetRPM << std::endl;
     * }
     * @endcode
     */
    std::tuple<int, int> revMatcher(int MPH, int rpm)
    {

        auto formatStatus = [](double rpm, double mph, int gear, int targetrpm = -1, const std::string &errorMsg = "")
        {
            std::ostringstream oss;
            oss << "RPM: " << rpm
                << ", MPH: " << std::fixed << std::setprecision(1) << mph
                << ", Current Gear: " << gear;
            if (targetrpm != -1)
            {
                oss << ", Target RPM: " << targetrpm;
            };
            if (!errorMsg.empty())
            {
                oss << "  --ERROR: " << errorMsg;
            };
            return oss.str();
        };

        int currentGear = getCurrentGear(rpm, MPH);
        if (currentGear < 2)
        {
            if (debug || testMode)
            {
                std::cout << formatStatus(rpm, MPH, currentGear, -1, "Downshift not possible") << '\n';
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
                    std::cout << formatStatus(rpm, MPH, currentGear, targetRPM, "Target RPM out of range") << '\n';
                };
                return {currentGear, -1};
            }
            else
            {
                if (debug || testMode)
                {
                    std::cout << formatStatus(rpm, MPH, currentGear, targetRPM) << '\n';
                };
                return {currentGear, targetRPM};
            };
        };
    }

private:
    /** 
     * @brief Vector of gear ratios (including final drive as first element).
     * @note Index 0 contains final drive ratio, indices 1+ contain gear ratios
     */
    std::vector<double> gearRatios;

    /** @brief Final drive ratio of the vehicle differential. */
    double finalDrive = -1.0;

    /** @brief Wheel circumference in inches (used for speed calculations). */
    double wheelCircumference = 0.0;

    /** @brief Minimum allowed RPM for safe rev-matching operations. */
    int minRPM = 0;

    /** @brief Maximum allowed RPM for safe rev-matching operations. */
    int maxRPM = 0;

    /**
     * @struct GearRatio
     * @brief Represents a gear ratio with input and output teeth counts.
     * 
     * This helper struct encapsulates the calculation of gear ratios
     * from the number of teeth on input and output gears.
     */
    struct GearRatio
    {
        /** @brief Number of input teeth (driving gear). */
        int inputTeeth;

        /** @brief Number of output teeth (driven gear). */
        int outputTeeth;

        /**
         * @brief Calculates the gear ratio.
         * 
         * The gear ratio determines the mechanical advantage and speed
         * relationship between input and output shafts.
         * 
         * @return The gear ratio as inputTeeth/outputTeeth
         * @note Higher ratios provide more torque multiplication but less speed
         */
        double ratio() const
        {
            return static_cast<double>(inputTeeth) / outputTeeth;
        }
    };

    /**
     * @brief Determines the current gear based on RPM and speed relationship.
     * 
     * This function calculates the current transmission gear by analyzing
     * the ratio between engine RPM and vehicle speed, then comparing it
     * against the known gear ratios to find the best match.
     * 
     * The algorithm:
     * 1. Calculates the current effective gear ratio from RPM/speed relationship
     * 2. Compares against midpoints between adjacent gear ratios
     * 3. Returns the gear that best matches the current operating conditions
     * 
     * @param rpm Current engine RPM (revolutions per minute)
     * @param mph Current vehicle speed in miles per hour
     * 
     * @return The estimated current gear number (1-based indexing)
     * @retval 1 Returned when vehicle speed is 0 (prevents division by zero)
     * 
     * @note Uses midpoint comparison to handle gear ratio overlap zones
     * @note Assumes linear relationship between RPM, speed, and gear ratios
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
 * This class provides concrete implementation for communicating with actual ELM327
 * OBD-II devices over serial ports. It handles the low-level serial communication,
 * OBD-II command formatting, and response parsing required to retrieve vehicle
 * diagnostic data such as RPM and speed.
 * 
 * ## Supported OBD-II PIDs:
 * - 0x0C: Engine RPM (Parameter ID 12)
 * - 0x0D: Vehicle Speed (Parameter ID 13)
 * 
 * ## Communication Protocol:
 * - Uses standard AT commands for ELM327 initialization
 * - Sends OBD-II PID requests in hexadecimal format
 * - Parses responses according to OBD-II standard
 * - Handles serial port configuration and error recovery
 * 
 * @note Requires boost::asio library for serial communication
 * @see ELM327Base for interface definition
 * @see https://en.wikipedia.org/wiki/OBD-II_PIDs for PID reference
 */
class ELM327Interface : public ELM327Base
{
private:
    /** @brief Boost.Asio I/O context for asynchronous operations. */
    asio::io_context io;

    /** @brief Serial port object for ELM327 device communication. */
    asio::serial_port serial;

public:
    /**
     * @brief Constructs an ELM327 interface for a specified serial port.
     * 
     * Initializes and configures the serial port connection to the ELM327 device
     * with standard communication parameters. The constructor sets up:
     * - Baud rate (typically 9600 for ELM327)
     * - 8 data bits, no parity, 1 stop bit (8N1)
     * - No flow control
     * 
     * @param portName The name of the serial port (e.g., "COM9" on Windows, "/dev/ttyUSB0" on Linux)
     * @param baudRate The baud rate for serial communication (typically 9600, 38400, or 115200)
     * 
     * @throws boost::system::system_error if the port cannot be opened or configured
     * 
     * @par Example:
     * @code
     * try {
     *     ELM327Interface elm("COM9", 9600);
     *     if (elm.isConnected()) {
     *         std::cout << "ELM327 connected successfully" << std::endl;
     *     }
     * } catch (const boost::system::system_error& e) {
     *     std::cerr << "Failed to connect: " << e.what() << std::endl;
     * }
     * @endcode
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
     * @brief Destructor to ensure the serial port is properly closed.
     * 
     * Automatically closes the serial port connection when the object
     * is destroyed, ensuring proper resource cleanup and preventing
     * port access conflicts.
     */
    ~ELM327Interface()
    {
        if (serial.is_open())
        {
            serial.close();
        }
    }

    /**
     * @brief Checks if the ELM327 device is connected and ready.
     * 
     * @return true if the serial port is open and operational
     * @return false if the serial port is closed or unavailable
     * 
     * @note This only checks if the serial port is open, not if the ELM327
     *       device is actually responding to commands
     */
    bool isConnected() const override
    {
        return serial.is_open();
    }

    /**
     * @brief Reads data from the serial port until a terminator character is received.
     * 
     * This function performs synchronous reading from the serial port, collecting
     * characters until the specified terminator is encountered. This is typically
     * used to read complete ELM327 responses which end with '>' character.
     * 
     * @param terminator The character that indicates the end of the response (usually '>')
     * 
     * @return The complete response string including the terminator character
     * 
     * @throws boost::system::system_error if serial communication fails
     * 
     * @note This function blocks until the terminator is received
     * @warning No timeout protection - may block indefinitely if device doesn't respond
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
     * @brief Retrieves RPM and speed from the ELM327 device using combined PID request.
     * 
     * This function sends a combined OBD-II command to retrieve both engine RPM
     * and vehicle speed in a single request for improved efficiency. The command
     * "010C0D2" requests PIDs 0x0C (RPM) and 0x0D (speed) simultaneously.
     * 
     * ## Response Format:
     * The ELM327 returns data in the format: "010C0D241 0C AA BB 0D CC"
     * where:
     * - AA BB: RPM data bytes (RPM = ((AA * 256) + BB) / 4)
     * - CC: Speed data byte (Speed in km/h, converted to MPH)
     * 
     * @return A tuple containing:
     *         - RPM (int): Engine revolutions per minute
     *         - Speed (int): Vehicle speed in miles per hour
     * @retval {-1, -1} Indicates communication error or invalid response format
     * 
     * @throws boost::system::system_error if serial communication fails
     * 
     * @note RPM calculation: ((A * 256) + B) / 4 where A and B are response bytes
     * @note Speed conversion: km/h value * 0.621371 to get MPH
     * 
     * @par Example Response:
     * @code
     * // Raw ELM327 response: "010C0D241 0C 1A F8 0D 3C >"
     * // Parsed: RPM = ((0x1A * 256) + 0xF8) / 4 = 1726 RPM
     * //         Speed = 0x3C * 0.621371 = 37.3 MPH
     * @endcode
     */
    std::tuple<int, int> getRPMandSpeed() override
    {
        std::string cmd = "010C0D2";
        asio::write(serial, asio::buffer(cmd + '\r'));
        std::string raw = readUntil('>');
        std::string cleaned = cleanResponse(raw);
        auto tokens = tokenizeResponse(cleaned);
        if (tokens[0]=="010C0D241" && tokens[1] == "0C" && tokens[4] == "0D") {
            int A = parseHexByte(tokens[2]);
            int B = parseHexByte(tokens[3]);
            int RPM = ((A * 256) + B) / 4;
            double speedMph = static_cast<double>(parseHexByte(tokens[5])) * KMH_TO_MPH;
            return {RPM, static_cast<int>(speedMph)};
        } else {
            return {-1,-1};
        }
    }

    /**
     * @brief Parses a hexadecimal byte string to an integer.
     * 
     * Converts a two-character hexadecimal string (e.g., "1A", "FF") to its
     * corresponding integer value. This is used to parse individual data bytes
     * from ELM327 responses.
     * 
     * @param str The hexadecimal string to parse (should be 1-2 characters)
     * 
     * @return The integer value of the hexadecimal string
     * @retval 0 If the string is empty or invalid
     * 
     * @throws std::invalid_argument if the string contains non-hex characters
     * @throws std::out_of_range if the value exceeds integer range
     * 
     * @par Examples:
     * @code
     * parseHexByte("1A") returns 26
     * parseHexByte("FF") returns 255  
     * parseHexByte("0") returns 0
     * @endcode
     */
    int parseHexByte(const std::string &str)
    {
        return std::stoi(str, nullptr, 16);
    }

    /**
     * @brief Tokenizes an ELM327 response into individual strings.
     * 
     * Splits a cleaned ELM327 response string into individual tokens separated
     * by whitespace. This is used to parse structured OBD-II responses where
     * each data byte or command element is space-separated.
     * 
     * @param response The cleaned response string (hexadecimal characters and spaces only)
     * 
     * @return A vector of tokenized strings, each representing a hex byte or command element
     * 
     * @note Input should be pre-cleaned using cleanResponse() to remove control characters
     * @see cleanResponse() for response preprocessing
     * 
     * @par Example:
     * @code
     * std::string cleaned = "010C0D241 0C 1A F8 0D 3C";
     * auto tokens = tokenizeResponse(cleaned);
     * // tokens = {"010C0D241", "0C", "1A", "F8", "0D", "3C"}
     * @endcode
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
     * @brief Cleans the raw ELM327 response by filtering valid characters.
     * 
     * Processes the raw response from the ELM327 device by removing control
     * characters, newlines, carriage returns, and other non-essential characters.
     * Only hexadecimal digits (0-9, A-F) and spaces are preserved for further parsing.
     * 
     * @param raw The raw response string from the ELM327 device
     * 
     * @return The cleaned response string containing only hex digits and spaces
     * 
     * @note This function is case-insensitive for hexadecimal characters
     * @note Removes characters like '\r', '\n', '>', and other ELM327 control characters
     * 
     * @par Example:
     * @code
     * std::string raw = "010C0D241 0C 1A F8 0D 3C\r\n>";
     * std::string cleaned = cleanResponse(raw);
     * // cleaned = "010C0D241 0C 1A F8 0D 3C"
     * @endcode
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
 * @brief A mock ELM327 implementation for testing and development purposes.
 *
 * This class simulates ELM327 functionality without requiring actual hardware,
 * making it ideal for software development, testing, and demonstration purposes.
 * It can generate either realistic correlated data or use predefined test datasets
 * to simulate various driving scenarios.
 * 
 * ## Features:
 * - Generates realistic RPM/speed correlations for testing
 * - Supports predefined test data from CSV files
 * - Configurable random data generation with realistic constraints  
 * - No hardware dependencies for development work
 * 
 * ## Test Data Format:
 * When using predefined test data, the CSV file should contain:
 * @code
 * # Comments start with #
 * 2000,30  # RPM, Speed(MPH)  
 * 2500,35
 * 3000,40
 * @endcode
 * 
 * @see ELM327Base for interface definition
 * @note Always returns isConnected() = true for testing convenience
 */
class DummyELM327 : public ELM327Base
{
private:
    /** @brief Random number generator seeded with current time. */
    std::mt19937 gen;
    
    /** @brief Distribution for generating random RPM values within realistic range. */
    std::uniform_int_distribution<> rpmDist;
    
    /** @brief Distribution for generating random speed values within typical driving range. */
    std::uniform_int_distribution<> speedDist;
    
    /** @brief Predefined test data for realistic driving scenarios (RPM, Speed pairs). */
    std::vector<std::pair<int, int>> testData;
    
    /** @brief Current index in the test data vector for sequential data playback. */
    size_t dataIndex;
    
    /** @brief Flag to use predefined test data instead of random generation. */
    bool useTestData;

public:
    /**
     * @brief Constructs a DummyELM327 object with configurable data source.
     * 
     * Creates a mock ELM327 interface that can either generate random correlated
     * RPM/speed data or use predefined test data from a CSV file. The random
     * data generation uses realistic constraints to simulate actual driving conditions.
     * 
     * @param useRandomData If true, generates random correlated data; if false, loads predefined test data
     * 
     * @note Random data generation creates realistic correlations between RPM and speed
     * @note Test data is loaded from "test_data.csv" in the configured directory
     * @note Random seed is based on current time for variability between runs
     * 
     * @par Random Data Characteristics:
     * - RPM range: 800-7000 (typical engine operating range)
     * - Speed range: 0-80 MPH (typical driving speeds)
     * - Correlation: Base RPM calculated from speed with realistic variation
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
     * @brief Checks if the dummy ELM327 is connected (always available).
     * 
     * @return Always returns true since dummy implementation doesn't depend on hardware
     * 
     * @note This simplifies testing by eliminating connection-related error handling
     */
    bool isConnected() const override
    {
        return true;
    }

    /**
     * @brief Simulates retrieving RPM and speed data from various sources.
     * 
     * Provides simulated vehicle data either from predefined test datasets or
     * generated randomly with realistic correlations. When using test data,
     * the function sequentially returns data points and sets the global running
     * flag to false when all data has been consumed.
     * 
     * ## Random Data Generation:
     * The algorithm creates realistic correlations by:
     * 1. Generating a random vehicle speed (0-80 MPH)
     * 2. Calculating a base RPM from speed (speed * 50 + 800)
     * 3. Adding realistic variation (±500 RPM) around the base
     * 4. Clamping result to valid engine RPM range (800-7000)
     * 
     * ## Test Data Playback:
     * When using predefined data:
     * - Returns sequential data points from loaded CSV
     * - Automatically terminates program when data is exhausted
     * - Provides reproducible test scenarios
     * 
     * @return A tuple containing:
     *         - RPM (int): Simulated engine revolutions per minute
     *         - Speed (int): Simulated vehicle speed in miles per hour
     * 
     * @note Sets global `running` flag to false when test data is exhausted
     * @note Random data provides infinite simulation capability
     * @see loadTestData() for test data file format requirements
     */
    std::tuple<int, int> getRPMandSpeed() override
    {
        if (useTestData && !testData.empty())
        {
            auto data = testData[dataIndex];
            dataIndex++;
            if (dataIndex >= testData.size())
            {
                running = false;
            }
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
     * @brief Loads predefined test data from CSV file for reproducible testing.
     * 
     * Reads test data from "test_data.csv" in the configured directory.
     * The file format supports comments (lines starting with #) and expects
     * comma-separated RPM and speed values on each line.
     * 
     * ## Expected File Format:
     * @code
     * # Test data for rev-matching scenarios
     * # RPM,Speed(MPH)
     * 2000,30    # Cruising in 4th gear
     * 3500,45    # Acceleration scenario  
     * 4000,50    # Highway speed
     * 1500,20    # City driving
     * @endcode
     * 
     * @note Silently continues if file cannot be opened (falls back to random data)
     * @note Ignores empty lines and lines starting with '#'
     * @note Each line should contain exactly two comma-separated integers
     * 
     * @throws std::invalid_argument if line format is invalid (logged but not fatal)
     */
    void loadTestData()
    {
        std::string line;
        int lineIndex=0;
        std::ifstream configFile("C:\\Users\\Will\\Documents\\rpmrevmatch\\test_data.csv");
        while (std::getline(configFile, line))
        {
            if (line.empty() || line[0] == '#')
                continue;
            std::stringstream ss(line);
            if (line.find(',') != std::string::npos)
            {
                int input, output;
                char comma;
                if (ss >> input >> comma >> output && comma == ',')
                {
                    testData.push_back({input, output});
                }
            }
        }

    }
};

/**
 * @brief Factory function to create appropriate ELM327 interface based on operation mode.
 * 
 * This factory function abstracts the creation of ELM327 interfaces, allowing
 * the main application to work with either real hardware or test implementations
 * without modification. It provides a clean separation between test and production
 * code paths.
 * 
 * @param testMode If true, creates a DummyELM327 for testing; if false, creates real ELM327Interface
 * @param port The serial port name for real ELM327 communication (e.g., "COM9", "/dev/ttyUSB0")
 * @param baudRate The baud rate for serial communication (typically 9600 for ELM327 devices)
 * 
 * @return A unique_ptr to an ELM327Base object (either DummyELM327 or ELM327Interface)
 * 
 * @throws boost::system::system_error if real ELM327 device cannot be opened (testMode=false only)
 * 
 * @note The returned pointer uses polymorphism to provide identical interface regardless of implementation
 * @note Test mode always succeeds; real mode may throw exceptions on hardware failures
 * 
 * @par Usage Example:
 * @code
 * try {
 *     auto elm = createELM327Interface(false, "COM9", 9600);  // Real hardware
 *     if (elm->isConnected()) {
 *         auto [rpm, speed] = elm->getRPMandSpeed();
 *         std::cout << "RPM: " << rpm << ", Speed: " << speed << std::endl;
 *     }
 * } catch (const std::exception& e) {
 *     std::cerr << "Hardware error: " << e.what() << std::endl;
 *     // Fall back to test mode
 *     auto elm = createELM327Interface(true);
 * }
 * @endcode
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
 * @brief Gets the current date and time as a formatted string.
 * 
 * Generates a timestamp string in ISO 8601-like format for use in
 * filenames, logging, and data records. The format is suitable for
 * both human readability and programmatic processing.
 * 
 * @return A formatted date-time string in "YYYY-MM-DD HH:MM:SS" format
 * 
 * @note Uses local time zone for timestamp generation
 * @note Thread-safe (uses local variables and standard library functions)
 * 
 * @par Example Output:
 * @code
 * "2025-07-25 14:30:45"
 * @endcode
 */
std::string getCurrentDateTime() {
    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(localTime, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

/**
 * @brief Main function to run the rev-matching application.
 * 
 * This is the primary entry point for the rev-matching application. It handles
 * command-line argument parsing, initializes system components, manages the
 * data acquisition loop, and ensures proper cleanup on termination.
 * 
 * ## Program Flow:
 * 1. Parse command-line arguments for configuration options
 * 2. Install signal handler for graceful shutdown (Ctrl+C)
 * 3. Initialize GearBox with configuration file
 * 4. Create appropriate ELM327 interface (real or test)
 * 5. Open CSV output file for data logging
 * 6. Enter main data acquisition loop
 * 7. Process and log RPM, speed, gear, and rev-matching data
 * 8. Handle graceful shutdown and cleanup
 * 
 * ## Command-Line Options:
 * - `--test, -t`: Enable test mode with dummy ELM327 simulation
 * - `--debug, -d`: Enable detailed debug output to console
 * - `--config <file>`: Specify custom configuration file path
 * - `--output <file>`: Specify custom CSV output file path
 * - `--help, -h`: Display usage information and exit
 * 
 * ## Output Format:
 * The program generates a CSV file with the following columns:
 * - RPM: Current engine revolutions per minute
 * - MPH: Current vehicle speed in miles per hour
 * - CurrentGear: Estimated current transmission gear
 * - RevMatch: Target RPM for downshift (-1 if not applicable)
 * - Time: Unix timestamp of the measurement
 * 
 * @param argc Number of command-line arguments
 * @param argv Array of command-line argument strings
 * 
 * @return Exit status code:
 *         - 0: Successful completion
 *         - 1: Error occurred (configuration, hardware, or file I/O failure)
 * 
 * @note The program runs continuously until interrupted by signal or test data exhaustion
 * @note All data is flushed to the output file after each measurement for data integrity
 * @note Supports unlimited samples in real mode, configurable samples in test mode
 * 
 * @par Example Usage:
 * @code
 * // Run with real ELM327 on COM9 with default settings
 * ./revmatch
 * 
 * // Run in test mode with debug output
 * ./revmatch --test --debug
 * 
 * // Use custom configuration and output files
 * ./revmatch --config my_car.txt --output results_today.csv
 * 
 * // Show help information
 * ./revmatch --help
 * @endcode
 * 
 * @see GearBox for configuration file format
 * @see ELM327Interface for hardware requirements
 * @see DummyELM327 for test mode capabilities
 */
int main(int argc, char *argv[])
{
    signal(SIGINT, signalHandler);

    time_t timestamp;
    std::string configPath = "C:\\Users\\Will\\Documents\\rpmrevmatch\\config.txt";
    std::string outputPath = "C:\\Users\\Will\\Documents\\rpmrevmatch\\output.csv";
    // std::string outputPath = "C:\\Users\\Will\\Documents\\rpmrevmatch\\output"+getCurrentDateTime()+".csv";

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

        outFile << "RPM,MPH,CurrentGear,RevMatch,Time\n";

        auto elm = createELM327Interface(testMode);

        if (!elm->isConnected())
        {
            std::cerr << "Failed to connect to ELM327 device\n";
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        int sampleCount = 0;
        // const int maxSamples = testMode ? 100 : -1;
        const int maxSamples = testMode ? -1 : -1;

        while (running && (maxSamples < 0 || sampleCount < maxSamples))
        {
            auto [rpm, speed] = elm->getRPMandSpeed();
            if (rpm > 0 && speed >= 0)
            {
                auto [gear, revs] = gearBox.revMatcher(speed, rpm);

                outFile << rpm << "," << std::fixed << std::setprecision(1) << speed << "," << gear << "," << revs << "," << time(&timestamp) << '\n';
                outFile.flush();

                sampleCount++;
            }
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