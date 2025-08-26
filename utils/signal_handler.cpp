#include "signal_handler.hpp"

std::atomic<bool> SignalHandler::running{true};

SignalHandler::SignalHandler() {
    std::signal(SIGINT, SignalHandler::handleSignal);
    std::signal(SIGTERM, SignalHandler::handleSignal);
}

bool SignalHandler::isRunning() {
    return running.load();
}

void SignalHandler::stop() {
    running.store(false);
}

void SignalHandler::handleSignal(int /*signal*/) {
    running.store(false);
}
