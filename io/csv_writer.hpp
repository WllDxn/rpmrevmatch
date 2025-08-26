#pragma once

#include <fstream>
#include <sstream>
#include <chrono>
#include <string>
#include <filesystem>
#include <iostream>
#include <format>
#include <cstdint>
#include <iomanip>

class BufferedCSVWriterBase
{
public:
    virtual ~BufferedCSVWriterBase() = default;

    virtual void writeRow(int rpm, double speed, int gear, int revMatch,
                          int load, int throttle, int64_t timestamp) = 0;

    virtual void flush() = 0;
};

class DummyCSVWriter : public BufferedCSVWriterBase
{
public:
    int writeCount = 0;

    DummyCSVWriter();

    void writeRow(int rpm, double speed, int gear, int revMatch,
                  int load, int throttle, int64_t timestamp) override;

    void flush() override;
};

class BufferedCSVWriter : public BufferedCSVWriterBase
{
private:
    std::ofstream file;
    std::ostringstream buffer;
    size_t bufferSize;
    size_t maxBufferSize;
    std::chrono::steady_clock::time_point lastFlush;
    std::chrono::milliseconds flushInterval;
    bool debugMode;

public:
    int writeCount = 0;

    BufferedCSVWriter(const std::filesystem::path& filename,
                      bool debug = false,
                      size_t maxBuffer = 8192,
                      std::chrono::milliseconds interval = std::chrono::seconds(5));

    void writeRow(int rpm, double speed, int gear, int revMatch,
                  int load, int throttle, int64_t timestamp) override;

    void flush() override;

    ~BufferedCSVWriter() override;
};
