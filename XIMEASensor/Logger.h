#pragma once

#include "XIMEASensorCommon.h"
#include <string>
#include <mutex>
#include <fstream>
#include <memory>
#include <chrono>
#include <sstream>

namespace XimeaSensor {

    class Logger {
    private:
        static std::unique_ptr<Logger> s_instance;
        static std::mutex s_mutex;

        std::ofstream m_fileStream;
        std::mutex m_logMutex;
        LogLevel m_minLevel;
        ILogCallback* m_callback;
        bool m_consoleOutput;
        bool m_fileOutput;

        Logger();

        std::string GetTimestamp() const;
        std::string LogLevelToString(LogLevel level) const;
        void WriteLog(LogLevel level, const std::string& message, const char* file, int line);

    public:
        ~Logger();

        // 싱글톤 인스턴스 획득
        static Logger& GetInstance();

        // 초기화
        void Initialize(const std::string& logFilePath = "", LogLevel minLevel = LogLevel::Info);

        // 설정
        void SetLogLevel(LogLevel level) { m_minLevel = level; }
        void SetCallback(ILogCallback* callback) { m_callback = callback; }
        void EnableConsoleOutput(bool enable) { m_consoleOutput = enable; }
        void EnableFileOutput(bool enable) { m_fileOutput = enable; }

        // 로그 출력
        void Log(LogLevel level, const std::string& message, const char* file, int line);

        // 헬퍼 메서드
        void Trace(const std::string& message, const char* file, int line);
        void Debug(const std::string& message, const char* file, int line);
        void Info(const std::string& message, const char* file, int line);
        void Warning(const std::string& message, const char* file, int line);
        void Error(const std::string& message, const char* file, int line);
        void Critical(const std::string& message, const char* file, int line);
    };

    // 매크로 정의
#define LOG_TRACE(msg)    XimeaSensor::Logger::GetInstance().Trace(msg, __FILE__, __LINE__)
#define LOG_DEBUG(msg)    XimeaSensor::Logger::GetInstance().Debug(msg, __FILE__, __LINE__)
#define LOG_INFO(msg)     XimeaSensor::Logger::GetInstance().Info(msg, __FILE__, __LINE__)
#define LOG_WARNING(msg)  XimeaSensor::Logger::GetInstance().Warning(msg, __FILE__, __LINE__)
#define LOG_ERROR(msg)    XimeaSensor::Logger::GetInstance().Error(msg, __FILE__, __LINE__)
#define LOG_CRITICAL(msg) XimeaSensor::Logger::GetInstance().Critical(msg, __FILE__, __LINE__)

} // namespace XimeaSensor