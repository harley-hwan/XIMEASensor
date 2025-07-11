#pragma once
#include <fstream>
#include <mutex>
#include <string>

class Logger {
public:
    // Initialize logger (optional to call, will be auto-called on first log)
    static void Initialize(const std::string& filePath = "XIMEASensor.log");
    // Close logger (flush and close file)
    static void Close();
    // Logging functions
    static void LogInfo(const char* format, ...);
    static void LogError(const char* format, ...);

private:
    static void LogInternal(const char* level, const char* format, va_list args);
    static std::ofstream logFile;
    static std::mutex logMutex;
    static bool initialized;
    static void openLogFile(const std::string& filePath);
};
