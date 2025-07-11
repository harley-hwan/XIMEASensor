#include "pch.h"
#include "Logger.h"
#include <cstdarg>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <windows.h>  // for OutputDebugString

#include "CameraController.h"

std::ofstream Logger::logFile;
std::mutex Logger::logMutex;
bool Logger::initialized = false;

void Logger::openLogFile(const std::string& filePath) {
    // Open log file (truncate if exists)
    logFile.open(filePath, std::ios::out | std::ios::trunc);
    initialized = logFile.is_open();
}

void Logger::Initialize(const std::string& filePath) {
    std::lock_guard<std::mutex> lock(logMutex);
    if (!initialized) {
        openLogFile(filePath);
        if (initialized) {
            // Write initial timestamp
            auto t = std::time(nullptr);
            std::tm tm;
            localtime_s(&tm, &t);
            char timeBuf[64];
            std::strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", &tm);
            logFile << "===== Log started at " << timeBuf << " =====" << std::endl;
            logFile.flush();
        }
    }
}

void Logger::Close() {
    std::lock_guard<std::mutex> lock(logMutex);
    if (initialized) {
        // Write closing timestamp
        auto t = std::time(nullptr);
        std::tm tm;
        localtime_s(&tm, &t);
        char timeBuf[64];
        std::strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", &tm);
        logFile << "===== Log closed at " << timeBuf << " =====" << std::endl;
        logFile.flush();
        logFile.close();
        initialized = false;
    }
}

void Logger::LogInternal(const char* level, const char* format, va_list args) {
    std::lock_guard<std::mutex> lock(logMutex);
    if (!initialized) {
        openLogFile("XIMEASensor.log");
        if (!initialized) {
            // If failed to open log file, just return
            return;
        }
    }
    // Timestamp (HH:MM:SS)
    auto now = std::time(nullptr);
    std::tm tm;
    localtime_s(&tm, &now);
    char timeBuf[20];
    std::strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", &tm);
    // Format message
    char msgBuf[512];
    vsnprintf_s(msgBuf, sizeof(msgBuf), _TRUNCATE, format, args);
    // Write to log file
    logFile << "[" << timeBuf << "] " << level << ": " << msgBuf << std::endl;
    logFile.flush();
    // Output to Visual Studio debug console
    std::ostringstream oss;
    oss << level << ": " << msgBuf << "\n";
    OutputDebugStringA(oss.str().c_str());
    // Invoke log callback if set (for external handling of logs)
    CameraController::InvokeLogCallback(oss.str().c_str());
}

void Logger::LogInfo(const char* format, ...) {
    va_list args;
    va_start(args, format);
    LogInternal("INFO", format, args);
    va_end(args);
}

void Logger::LogError(const char* format, ...) {
    va_list args;
    va_start(args, format);
    LogInternal("ERROR", format, args);
    va_end(args);
}
