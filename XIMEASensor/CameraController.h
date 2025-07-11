#pragma once
#include <xiApi.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <memory>
#include <functional>
#include <queue>
#include <string>
#include <vector>
#include <algorithm>

// Windows ERROR 매크로랑 충돌 방지
#ifdef ERROR
#undef ERROR
#endif

// 로깅 레벨
enum class LogLevel {
    LOG_DEBUG = 0,
    LOG_INFO = 1,
    LOG_WARNING = 2,
    LOG_ERROR = 3,      // ERROR 대신 LOG_ERROR 사용
    LOG_CRITICAL = 4
};

// 카메라 상태
enum class CameraState {
    UNINITIALIZED,
    INITIALIZED,
    STREAMING,
    ERROR_STATE
};

// 트리거 모드
enum class TriggerMode {
    FREE_RUN,
    SOFTWARE_TRIGGER,
    HARDWARE_TRIGGER
};

// 프레임 정보 구조체
struct FrameInfo {
    unsigned char* data;
    int width;
    int height;
    int stride;
    uint64_t frameNumber;
    double timestamp;
    float exposureTime;
    float gain;
    int format;
};

// 카메라 파라미터 구조체
struct CameraParameters {
    int width = 1280;
    int height = 1024;
    float fps = 210.0f;
    int exposureUs = 4000;
    float gain = 0.0f;
    TriggerMode triggerMode = TriggerMode::FREE_RUN;
    int bufferCount = 10;
    bool autoExposure = false;
    bool autoGain = false;
    int binningX = 1;
    int binningY = 1;
    int decimationX = 1;
    int decimationY = 1;
};

// 콜백 타입 정의
using FrameCallback = std::function<void(const FrameInfo&)>;
using ErrorCallback = std::function<void(int errorCode, const std::string& errorMsg)>;
using LogCallback = std::function<void(LogLevel level, const std::string& message)>;

class CameraController
{
private:
    // 싱글톤 인스턴스
    CameraController();
    static std::unique_ptr<CameraController> instance;
    static std::mutex instanceMutex;

    // XIMEA 핸들
    HANDLE xiH;

    // 상태 관리
    std::atomic<CameraState> state;
    std::atomic<bool> isCapturing;

    // 스레드 관리
    std::unique_ptr<std::thread> captureThread;
    std::unique_ptr<std::thread> callbackThread;

    // 동기화 객체
    mutable std::mutex paramMutex;
    mutable std::mutex callbackMutex;
    std::condition_variable frameCV;

    // 버퍼 관리
    struct FrameBuffer {
        std::vector<unsigned char> data;
        FrameInfo info;
        bool ready;
    };
    std::vector<FrameBuffer> frameBuffers;
    std::queue<int> availableBuffers;
    std::queue<int> readyBuffers;
    mutable std::mutex bufferMutex;

    // 파라미터
    CameraParameters params;
    CameraParameters targetParams;
    std::atomic<bool> paramsChanged;

    // 콜백
    FrameCallback frameCallback;
    ErrorCallback errorCallback;
    LogCallback logCallback;

    // 성능 모니터링
    struct PerformanceStats {
        std::atomic<uint64_t> framesCaptures;
        std::atomic<uint64_t> framesDropped;
        std::atomic<float> currentFps;
        std::chrono::steady_clock::time_point startTime;
        std::chrono::steady_clock::time_point lastFrameTime;
    } perfStats;

    // 로깅
    std::string logFilePath;
    std::mutex logMutex;
    LogLevel minLogLevel = LogLevel::LOG_INFO;
    void Log(LogLevel level, const std::string& message);

    // 내부 함수들
    void CaptureLoop();
    void CallbackLoop();
    bool ApplyParameters();
    void InitializeBuffers();
    void CleanupBuffers();
    std::string GetXiErrorString(XI_RETURN error);
    void UpdatePerformanceStats();

public:
    ~CameraController();

    // 싱글톤 접근
    static CameraController& GetInstance();
    static void DestroyInstance();

    // 카메라 제어
    bool Initialize(int deviceIndex = 0);
    bool StartCapture();
    void StopCapture();
    void Shutdown();

    // 파라미터 설정
    bool SetParameter(const std::string& name, int value);
    bool SetParameter(const std::string& name, float value);
    bool SetParameter(const std::string& name, bool value);
    bool GetParameter(const std::string& name, int& value) const;
    bool GetParameter(const std::string& name, float& value) const;
    bool GetParameter(const std::string& name, bool& value) const;

    // 편의 함수들
    bool SetExposure(int microsec);
    bool SetGain(float gain);
    bool SetFPS(float fps);
    bool SetROI(int offsetX, int offsetY, int width, int height);
    bool SetBinning(int horizontal, int vertical);
    bool SetDecimation(int horizontal, int vertical);
    bool SetTriggerMode(TriggerMode mode);
    bool SoftwareTrigger();

    // 자동 모드
    bool SetAutoExposure(bool enable, float targetLevel = 0.5f);
    bool SetAutoGain(bool enable);
    bool SetAutoWhiteBalance(bool enable);

    // 이미지 포맷
    bool SetImageFormat(int format);
    std::vector<int> GetSupportedFormats() const;

    // 카메라 정보
    std::string GetDeviceName() const;
    std::string GetSerialNumber() const;
    bool GetSensorSize(int& width, int& height) const;
    bool GetTemperature(float& temp) const;

    // 프레임 획득 (폴링 방식)
    bool GetLatestFrame(unsigned char* buffer, int bufferSize,
        int& width, int& height, uint64_t& frameNumber);

    // 콜백 설정
    void SetFrameCallback(FrameCallback callback);
    void SetErrorCallback(ErrorCallback callback);
    void SetLogCallback(LogCallback callback);

    // 로깅 설정
    void SetLogFile(const std::string& filepath);
    void SetLogLevel(LogLevel level);

    // 성능 통계
    float GetCurrentFPS() const { return perfStats.currentFps; }
    uint64_t GetFramesCaptured() const { return perfStats.framesCaptures; }
    uint64_t GetFramesDropped() const { return perfStats.framesDropped; }

    // 상태 확인
    CameraState GetState() const { return state; }
    bool IsCapturing() const { return isCapturing; }
    CameraParameters GetCurrentParameters() const;

    // 고급 기능
    bool SaveParametersToFile(const std::string& filepath) const;
    bool LoadParametersFromFile(const std::string& filepath);
    bool CaptureImage(const std::string& filepath, int format = XI_RAW8);

    bool SaveBMP(const std::string& filepath, const unsigned char* data, int width, int height);
};