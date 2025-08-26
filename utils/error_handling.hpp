#pragma once
#include <stdexcept>
#include <expected>
template<typename T>
using Result = std::expected<T, std::string>;

class MyParseError : public std::runtime_error {
public:
    explicit MyParseError(const std::string& msg) : std::runtime_error(msg) {}
};