#include "pch.h"
#include "Logger.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>

std::unique_ptr<Logger> Logger::instance = nullptr;
std::mutex Logger::instanceMutex;

Logger::Logger()
    : isRunning(false),
    currentLogLevel(LogLevel::Info),
    maxFileSize(10 * 1024 * 1024),
    maxBackupFiles(5) {
}

Logger::~Logger() {
    Shutdown();
}

Logger& Logger::GetInstance() {
    std::lock_guard<std::mutex> lock(instanceMutex);
    if (!instance) {
        instance = std::unique_ptr<Logger>(new Logger());
    }
    return *instance;
}

void Logger::Destroy() {
    std::lock_guard<std::mutex> lock(instanceMutex);
    if (instance) {
        instance->Shutdown();
        instance.reset();
    }
}

void Logger::Initialize(const std::string& filePath, LogLevel level, size_t maxSize, int maxBackups) {
    std::unique_lock<std::mutex> lock(logMutex);
    if (isRunning) {
        return;
    }

    logFilePath = filePath;
    currentLogLevel = level;
    maxFileSize = maxSize;
    maxBackupFiles = maxBackups;

    std::filesystem::path logDir = std::filesystem::path(filePath).parent_path();
    if (!logDir.empty() && !std::filesystem::exists(logDir)) {
        std::filesystem::create_directories(logDir);
    }

    logFile.open(filePath, std::ios::app);
    if (!logFile.is_open()) {
        throw std::runtime_error("Failed to open log file: " + filePath);
    }

    isRunning = true;
    lock.unlock();      // collision occurs if not unlock

    // background log thread
    logThread = std::thread(&Logger::ProcessLogs, this);

    LOG_INFO(std::string("Logger initialized. File: ") + filePath);
}

void Logger::Shutdown() {
    if (!isRunning) return;

    {
        std::lock_guard<std::mutex> lock(logMutex);
        isRunning = false;
    }

    logCondition.notify_all();

    if (logThread.joinable()) {
        logThread.join();
    }

    if (logFile.is_open()) {
        logFile.close();
    }
}

void Logger::ProcessLogs() {
    while (true) {
        std::unique_lock<std::mutex> lock(logMutex);

        logCondition.wait(lock, [this] {
            return !logQueue.empty() || !isRunning;
            });

        if (!isRunning && logQueue.empty()) {
            break;
        }

        while (!logQueue.empty()) {
            std::string logEntry = logQueue.front();
            logQueue.pop();

            lock.unlock();

            if (logFile.is_open()) {
                logFile << logEntry << std::endl;
                logFile.flush();

                // check the file size and rotate
                if (logFile.tellp() >= static_cast<std::streampos>(maxFileSize)) {
                    RotateLogFile();
                }
            }

            lock.lock();
        }
    }
}

void Logger::RotateLogFile() {
    logFile.close();

    for (int i = maxBackupFiles - 1; i > 0; --i) {
        std::string oldName = logFilePath + "." + std::to_string(i);
        std::string newName = logFilePath + "." + std::to_string(i + 1);

        if (std::filesystem::exists(oldName)) {
            if (i == maxBackupFiles - 1) {
                std::filesystem::remove(oldName);
            }
            else {
                std::filesystem::rename(oldName, newName);
            }
        }
    }

    if (std::filesystem::exists(logFilePath)) {
        std::filesystem::rename(logFilePath, logFilePath + ".1");
    }

	// reopen log
    logFile.open(logFilePath, std::ios::app);
}

std::string Logger::GetTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    struct tm localTime;
    localtime_s(&localTime, &time_t);

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();

    return ss.str();
}

std::string Logger::LogLevelToString(LogLevel level) {
    switch (level) {
    case LogLevel::Debug:    return "DEBUG";
    case LogLevel::Info:     return "INFO";
    case LogLevel::Warning:  return "WARN";
    case LogLevel::Error:    return "ERROR";
    case LogLevel::Critical: return "CRIT";
    default:                 return "UNKNOWN";
    }
}

void Logger::SetLogLevel(LogLevel level) {
    std::lock_guard<std::mutex> lock(logMutex);
    currentLogLevel = level;
}


void Logger::Log(LogLevel level, const std::string& message, const std::string& function, int line) {
    if (level < currentLogLevel) {
        return;
    }

    std::stringstream ss;
    ss << "[" << GetTimestamp() << "] ";
    ss << "[" << LogLevelToString(level) << "] ";

    if (!function.empty()) {
        ss << "[" << function;
        if (line > 0) {
            ss << ":" << line;
        }
        ss << "] ";
    }

    ss << message;

    {
        std::lock_guard<std::mutex> lock(logMutex);
        if (isRunning) {
            logQueue.push(ss.str());
            logCondition.notify_one();
        }
    }

#ifdef _DEBUG
    std::cout << ss.str() << std::endl;
#endif
}

void Logger::Flush() {
    std::unique_lock<std::mutex> lock(logMutex);
    logCondition.wait(lock, [this] { return logQueue.empty(); });

    if (logFile.is_open()) {
        logFile.flush();
    }
}

void Logger::LogDebug(const std::string& msg, const std::string& func, int line) {
    Log(LogLevel::Debug, msg, func, line);
}
void Logger::LogInfo(const std::string& msg, const std::string& func, int line) {
    Log(LogLevel::Info, msg, func, line);
}
void Logger::LogWarning(const std::string& msg, const std::string& func, int line) {
    Log(LogLevel::Warning, msg, func, line);
}
void Logger::LogError(const std::string& msg, const std::string& func, int line) {
    Log(LogLevel::Error, msg, func, line);
}
void Logger::LogCritical(const std::string& msg, const std::string& func, int line) {
    Log(LogLevel::Critical, msg, func, line);
}