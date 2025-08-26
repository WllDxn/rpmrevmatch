#include "csv_writer.hpp"
#include <format>

DummyCSVWriter::DummyCSVWriter() = default;

void DummyCSVWriter::writeRow(int rpm, double speed, int gear, int revMatch,
                              int load, int throttle, int64_t timestamp)
{
    std::cout << std::format("RPM: {}, MPH: {:.1f}, Current Gear: {}, Target RPM: {}, Time: {}\n",
                             rpm, speed, gear, revMatch, timestamp);
}

void DummyCSVWriter::flush()
{
    // No-op
}

BufferedCSVWriter::BufferedCSVWriter(const std::filesystem::path& filename,
                                     bool debug,
                                     size_t maxBuffer,
                                     std::chrono::milliseconds interval)
    : file(filename),
      maxBufferSize(maxBuffer),
      flushInterval(interval),
      debugMode(debug)
{
    if (!file.is_open())
    {
        std::cerr << "FATAL ERROR: Could not open or create file: " << filename.generic_string() << std::endl;
        if (file.fail())
        {
            std::cerr << "Reason: I/O error on open." << std::endl;
        }
        throw std::runtime_error("Failed to open file: " + filename.generic_string());
    }

    std::cout << "Successfully opened " << filename.generic_string() << " for writing." << std::endl;

    lastFlush = std::chrono::steady_clock::now();
    buffer << "RPM,MPH,CurrentGear,RevMatch,Load,Throttle,Time\n";
}

void BufferedCSVWriter::writeRow(int rpm, double speed, int gear, int revMatch,
                                 int load, int throttle, int64_t timestamp)
{
    if (debugMode)
    {
        std::cout << std::format("RPM: {}, MPH: {:.1f}, Current Gear: {}, Target RPM: {}, Time: {}\n",
                                 rpm, speed, gear, revMatch, timestamp);
    }

    buffer << rpm << "," << std::fixed << std::setprecision(1) << speed
           << "," << gear << "," << revMatch << "," << load
           << "," << throttle << "," << timestamp << '\n';

    auto now = std::chrono::steady_clock::now();

    if (static_cast<size_t>(buffer.tellp()) >= maxBufferSize ||
        (now - lastFlush) >= flushInterval)
    {
        flush();
    }
}

void BufferedCSVWriter::flush()
{
    if (!buffer.str().empty())
    {
        file << buffer.str();
        file.flush();
        buffer.str("");
        buffer.clear();
        lastFlush = std::chrono::steady_clock::now();
        writeCount++;
    }
}

BufferedCSVWriter::~BufferedCSVWriter()
{
    if (buffer.tellp() > 0)
    {
        std::cout << "Flushing remaining CSV data on exit. Total writes: " << writeCount << "..." << std::endl;
        flush();
    }
}
