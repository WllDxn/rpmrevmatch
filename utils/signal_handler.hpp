#pragma once
#include <atomic>
#include <csignal>

class SignalHandler {
public:
    SignalHandler();
    static bool isRunning();
    static void stop();

private:
    static std::atomic<bool> running;
    static void handleSignal(int signal);
};
