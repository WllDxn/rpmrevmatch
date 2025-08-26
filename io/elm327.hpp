#pragma once

#include <random>
#include <vector>
#include <string_view>
#include <tuple>
#include <fstream>
#include <sstream>
#include <iostream>
#include "../utils/error_handling.hpp"
#include "../utils/signal_handler.hpp"
#include <boost/asio.hpp>
#include <string>
#include <string_view>
#include <chrono>
#include <tuple>
#include <iostream>
#include <optional>
#include <algorithm>
#include <charconv>
#include <format>

class ELM327Base
{
public:
    virtual ~ELM327Base() = default;

    virtual Result<std::tuple<int, int, int, int>> getEngineData(const SignalHandler& handler, std::string_view cmd) = 0;

    virtual bool isConnected() const = 0;
};

class ELM327Interface : public ELM327Base
{
private:
    boost::asio::io_context io;
    boost::asio::serial_port serial;

public:
    ELM327Interface(const std::string& portName, unsigned int baudRate);
    ~ELM327Interface();
    bool isConnected() const override;
    Result<std::tuple<int, int, int, int>> getEngineData(const SignalHandler& handler, std::string_view cmd) override;

    std::string messageReadOBD(const std::string_view cmd,
                               const bool printResponse = false,
                               char terminator = '>',
                               const std::chrono::milliseconds& timeout = std::chrono::milliseconds(1000));
};

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
    explicit DummyELM327(bool useTestData = true);
    bool isConnected() const override;
    Result<std::tuple<int, int, int, int>> getEngineData(const SignalHandler& handler,std::string_view cmd) override;

private:
    void loadTestData();
};
