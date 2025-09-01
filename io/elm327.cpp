#include <algorithm>
#include <chrono>
#include "elm327.hpp"

ELM327Interface::ELM327Interface(const std::string &portName, unsigned int baudRate)
    : serial(io)
{
    try
    {
        serial.open(portName);
        serial.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    }
    catch (const boost::system::system_error &e)
    {
        std::cerr << "Failed to open port: " << e.what() << '\n';
        throw;
    }

    messageReadOBD("ATZ\r", true);
    messageReadOBD("ATE0\r", true);
    messageReadOBD("ATL0\r", true);
    messageReadOBD("ATSP6\r", true);
    messageReadOBD("ATH0\r", true);
    messageReadOBD("ATAL\r", true);
}

ELM327Interface::~ELM327Interface()
{
    if (serial.is_open())
    {
        serial.close();
    }
}

bool ELM327Interface::isConnected() const
{
    return serial.is_open();
}

std::string ELM327Interface::messageReadOBD(const std::string_view cmd,
                                            const bool printResponse,
                                            char terminator,
                                            const std::chrono::milliseconds &timeout)
{
    boost::asio::write(serial, boost::asio::buffer(cmd.data(), cmd.size()));
    io.restart();

    boost::asio::streambuf response_buf;
    std::string response;

    boost::asio::steady_timer timer(io);
    timer.expires_after(timeout);
    bool timed_out = false;

    timer.async_wait([&](const boost::system::error_code &ec)
                     {
        if (!ec)
        {
            timed_out = true;
            serial.cancel();
        } });

    boost::asio::async_read_until(serial, response_buf, terminator,
                                  [&](const boost::system::error_code &ec, std::size_t /*bytes_transferred*/)
                                  {
                                      timer.cancel();
                                  });

    io.run();

    if (timed_out)
    {
        throw std::runtime_error("Serial read timed out");
    }

    std::string input;
    std::istream is(&response_buf);
    while (std::getline(is, input))
    {
        input.erase(std::remove(input.begin(), input.end(), ' '), input.end());
        input.erase(std::remove(input.begin(), input.end(), '\r'), input.end());
        response += input;
    }

    if (printResponse)
    {
        std::cout << response << '\n';
    }

    return response;
}

Result<std::tuple<int, int, int, int, long long>> ELM327Interface::getEngineData(const SignalHandler &handler, std::string_view cmd = "01 0C 0D 04 11 05 4\r")
{
    std::string response = messageReadOBD("01 0C 0D 04 11 05 4\r", true);

    size_t pos = response.find("0C");
    int rpm = -1, speedMph = -1, load = -1, throttle = -1;

    auto hexByte = [](std::string_view sv) -> std::optional<int>
    {
        if (sv.size() < 2)
            return std::nullopt;
        int value = 0;
        auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + 2, value, 16);
        return (ec == std::errc{}) ? std::optional<int>(value) : std::nullopt;
    };

    auto findByteAfterPID = [&](std::string_view pid, size_t &start) -> std::optional<int>
    {
        start = response.find(pid, start);
        if (start == std::string::npos || start + 4 > response.size())
            return std::nullopt;
        start += 2;
        return hexByte(response.substr(start, 2));
    };

    try
    {
        if (pos == std::string::npos)
            throw std::runtime_error("0C not found");

        pos += 2;
        if (pos + 4 > response.size())
            throw std::runtime_error("0C found but not enough space for valid hex bytes");

        auto A = hexByte(response.substr(pos, 2));
        auto B = hexByte(response.substr(pos + 2, 2));
        if (!A || !B)
            throw std::runtime_error("Invalid RPM hex bytes");

        rpm = ((*A) * 256 + *B) / 4;
        pos += 4;

        auto speedOpt = findByteAfterPID("0D", pos);
        if (!speedOpt)
            throw std::runtime_error("Speed (0D) not found");
        speedMph = static_cast<int>(*speedOpt * 0.621371);
        pos += 2;

        auto loadOpt = findByteAfterPID("04", pos);
        if (!loadOpt)
            throw std::runtime_error("Load (04) not found");
        load = static_cast<int>((*loadOpt * 100.0) / 255.0);
        pos += 2;

        auto throttleOpt = findByteAfterPID("11", pos);
        if (!throttleOpt)
            throw std::runtime_error("Throttle (11) not found");
        throttle = static_cast<int>((*throttleOpt * 100.0) / 255.0);

        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        return std::make_tuple(rpm, speedMph, load, throttle, timestamp);
    }
    catch (const std::runtime_error &e)
    {
        return std::unexpected(std::format("Parse error: {}", e.what()));
    }
}

DummyELM327::DummyELM327(bool useTestData)
    : gen(std::chrono::steady_clock::now().time_since_epoch().count() & 0xFFFFFFFF),
      rpmDist(800, 7000),
      speedDist(0, 80),
      dataIndex(0),
      useTestData(useTestData)
{
    if (useTestData)
    {
        loadTestData();
    }
    std::cout << "DummyELM327 initialized " << (useTestData ? "with test data" : "with random data") << "\n";
}

bool DummyELM327::isConnected() const
{
    return true;
}

Result<std::tuple<int, int, int, int, long long>> DummyELM327::getEngineData(const SignalHandler &handler, std::string_view cmd = "01 0C 0D 04 11 05 4\r")
{
    if (useTestData && !testData.empty())
    {
        auto data = testData[dataIndex++];
        if (dataIndex >= testData.size())
        {
            handler.stop();
        }
        return std::make_tuple(std::get<0>(data), std::get<1>(data), -1, -1, std::get<2>(data));
    }
    else
    {
        int speed = speedDist(gen);
        int baseRpm = speed * 50 + 800;
        int rpm = baseRpm + rpmDist(gen) % 1000 - 500;
        rpm = std::clamp(rpm, 800, 7000);
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        return std::make_tuple(rpm, speed, -1, -1, timestamp);
    }
}

void DummyELM327::loadTestData()
{
    std::ifstream configFile("logs/2025-08-18_16-13-04.csv");
    if (!configFile)
    {
        std::cerr << "Warning: Could not open test_data.csv, using random data\n";
        useTestData = false;
        return;
    }

    std::string line;
    while (std::getline(configFile, line))
    {
        if (line.empty() || line[0] == '#')
            continue;

        std::stringstream ss(line);
        std::string token;
        std::vector<int> intValues;
        long long timestamp = 0;
        intValues.reserve(6); 

        int tokenIndex = 0;
        while (std::getline(ss, token, ','))
        {
            try
            {
                if (tokenIndex < 6)
                {
                    intValues.push_back(std::stoi(token));
                }
                else if (tokenIndex == 6)
                {
                    timestamp = std::stoll(token);
                }
                tokenIndex++;
            }
            catch (const std::exception &)
            {
                break;
            }
        }

        if (intValues.size() == 6 && tokenIndex == 7)
        {
            testData.emplace_back(intValues[0], intValues[1], timestamp);
        }
    }

    if (testData.empty())
    {
        std::cerr << "Warning: No valid test data found, using random data\n";
        useTestData = false;
    }
}
