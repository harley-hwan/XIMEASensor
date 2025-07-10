#include "pch.h"
#include "Logger.h"
#include <iostream>
#include <iomanip>
#include <windows.h>

namespace XimeaSensor {

    std::unique_ptr<Logger> Logger::s_instance = nullptr;
    std::mutex Logger::s_mutex;

    Logger::Logger()
        : m_minLevel(LogLevel::Info)
        , m_callback(nullptr)
        , m_consoleOutput(true)
        , m_fileOutput(false) {
    }

    Logger::~Logger() {
        if (m_fileStream.is_open()) {
            m_fileStream.close();
        }
    }

    Logger& Logger::GetInstance() {
        std::lock_guard<std::mutex> lock(s_mutex);
        if (!s_instance) {
            s_instance.reset(new Logger());
        }
        return *s_instance;
    }

    void Logger::Initialize(const std::string& logFilePath, LogLevel minLevel) {
        std::lock_guard<std::mutex> lock(m_logMutex);

        m_minLevel = minLevel;

        if (!logFilePath.empty()) {
            if (m_fileStream.is_open()) {
                m_fileStream.close();
            }

            m_fileStream.open(logFilePath, std::ios::out | std::ios::app);
            if (m_fileStream.is_open()) {
                m_fileOutput = true;
                m_fileStream << "\n=== Log Session Started at " << GetTimestamp() << " ===" << std::endl;
            }
        }
    }

    std::string Logger::GetTimestamp() const {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;

        struct tm timeinfo;
        localtime_s(&timeinfo, &time_t);

        std::stringstream ss;
        ss << std::put_time(&timeinfo, "%Y-%m-%d %H:%M:%S");
        ss << '.' << std::setfill('0') << std::setw(3) << ms.count();

        return ss.str();
    }

    std::string Logger::LogLevelToString(LogLevel level) const {
        switch (level) {
        case LogLevel::Trace:    return "TRACE";
        case LogLevel::Debug:    return "DEBUG";
        case LogLevel::Info:     return "INFO ";
        case LogLevel::Warning:  return "WARN ";
        case LogLevel::Error:    return "ERROR";
        case LogLevel::Critical: return "CRIT ";
        default:                 return "UNKN ";
        }
    }

    void Logger::WriteLog(LogLevel level, const std::string& message, const char* file, int line) {
        std::stringstream ss;
        ss << "[" << GetTimestamp() << "] "
            << "[" << LogLevelToString(level) << "] ";

        // 파일명만 추출 (전체 경로 제외)
        if (file) {
            const char* filename = strrchr(file, '\\');
            if (!filename) filename = strrchr(file, '/');
            if (filename) filename++;
            else filename = file;

            ss << "[" << filename << ":" << line << "] ";
        }

        ss << message;

        std::string logLine = ss.str();

        // 콘솔 출력
        if (m_consoleOutput) {
            // 레벨에 따라 색상 설정
            HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
            WORD color = FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE;

            switch (level) {
            case LogLevel::Trace:
            case LogLevel::Debug:
                color = FOREGROUND_GREEN | FOREGROUND_BLUE; // Cyan
                break;
            case LogLevel::Info:
                color = FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE; // White
                break;
            case LogLevel::Warning:
                color = FOREGROUND_RED | FOREGROUND_GREEN; // Yellow
                break;
            case LogLevel::Error:
                color = FOREGROUND_RED | FOREGROUND_INTENSITY; // Bright Red
                break;
            case LogLevel::Critical:
                color = FOREGROUND_RED | FOREGROUND_BLUE | FOREGROUND_INTENSITY; // Bright Magenta
                break;
            }

            SetConsoleTextAttribute(hConsole, color);
            std::cout << logLine << std::endl;
            SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
        }

        // 파일 출력
        if (m_fileOutput && m_fileStream.is_open()) {
            m_fileStream << logLine << std::endl;
            m_fileStream.flush();
        }

        // 콜백 호출
        if (m_callback) {
            m_callback->OnLog(level, message.c_str(), file, line);
        }
    }

    void Logger::Log(LogLevel level, const std::string& message, const char* file, int line) {
        if (level < m_minLevel) return;

        std::lock_guard<std::mutex> lock(m_logMutex);
        WriteLog(level, message, file, line);
    }

    void Logger::Trace(const std::string& message, const char* file, int line) {
        Log(LogLevel::Trace, message, file, line);
    }

    void Logger::Debug(const std::string& message, const char* file, int line) {
        Log(LogLevel::Debug, message, file, line);
    }

    void Logger::Info(const std::string& message, const char* file, int line) {
        Log(LogLevel::Info, message, file, line);
    }

    void Logger::Warning(const std::string& message, const char* file, int line) {
        Log(LogLevel::Warning, message, file, line);
    }

    void Logger::Error(const std::string& message, const char* file, int line) {
        Log(LogLevel::Error, message, file, line);
    }

    void Logger::Critical(const std::string& message, const char* file, int line) {
        Log(LogLevel::Critical, message, file, line);
    }

} // namespace XimeaSensor