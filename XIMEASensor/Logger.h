#pragma once
#include <string>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <memory>

enum class LogLevel {
    Debug = 0,
    Info = 1,
    Warning = 2,
    Error = 3,
    Critical = 4
};

class Logger {
private:
    static std::unique_ptr<Logger> instance;
    static std::mutex instanceMutex;

    std::ofstream logFile;
    std::mutex logMutex;
    std::queue<std::string> logQueue;
    std::thread logThread;
    std::condition_variable logCondition;
    bool isRunning;
    LogLevel currentLogLevel;

    std::string logFilePath;
    size_t maxFileSize;
    int maxBackupFiles;

    Logger();
    void ProcessLogs();
    void RotateLogFile();
    std::string GetTimestamp();
    std::string LogLevelToString(LogLevel level);

public:
    ~Logger();

    static Logger& GetInstance();
    static void Destroy();

    void Initialize(const std::string& filePath = "XIMEASensor.log",
        LogLevel level = LogLevel::Info,
        size_t maxSize = 10 * 1024 * 1024,  // 10MB
        int maxBackups = 5);

    void SetLogLevel(LogLevel level);
    void Log(LogLevel level, const std::string& message, const std::string& function = "", int line = 0);
    void Flush();
    void Shutdown();

    void LogDebug(const std::string& message, const std::string& function, int line);
    void LogInfo(const std::string& message, const std::string& function, int line);
    void LogWarning(const std::string& message, const std::string& function, int line);
    void LogError(const std::string& message, const std::string& function, int line);
    void LogCritical(const std::string& message, const std::string& function, int line);
};

#define LOG_DEBUG(msg) Logger::GetInstance().LogDebug(msg, __FUNCTION__, __LINE__)
#define LOG_INFO(msg) Logger::GetInstance().LogInfo(msg, __FUNCTION__, __LINE__)
#define LOG_WARNING(msg) Logger::GetInstance().LogWarning(msg, __FUNCTION__, __LINE__)
#define LOG_ERROR(msg) Logger::GetInstance().LogError(msg, __FUNCTION__, __LINE__)
#define LOG_CRITICAL(msg) Logger::GetInstance().LogCritical(msg, __FUNCTION__, __LINE__)