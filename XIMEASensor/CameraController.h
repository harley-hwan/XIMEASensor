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

// Windows ERROR ��ũ�ζ� �浹 ����
#ifdef ERROR
#undef ERROR
#endif

// �α� ����
enum class LogLevel {
    LOG_DEBUG = 0,
    LOG_INFO = 1,
    LOG_WARNING = 2,
    LOG_ERROR = 3,      // ERROR ��� LOG_ERROR ���
    LOG_CRITICAL = 4
};

// ī�޶� ����
enum class CameraState {
    UNINITIALIZED,
    INITIALIZED,
    STREAMING,
    ERROR_STATE
};

// Ʈ���� ���
enum class TriggerMode {
    FREE_RUN,
    SOFTWARE_TRIGGER,
    HARDWARE_TRIGGER
};

// ������ ���� ����ü
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

// ī�޶� �Ķ���� ����ü
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

// �ݹ� Ÿ�� ����
using FrameCallback = std::function<void(const FrameInfo&)>;
using ErrorCallback = std::function<void(int errorCode, const std::string& errorMsg)>;
using LogCallback = std::function<void(LogLevel level, const std::string& message)>;

class CameraController
{
private:
    // �̱��� �ν��Ͻ�
    CameraController();
    static std::unique_ptr<CameraController> instance;
    static std::mutex instanceMutex;

    // XIMEA �ڵ�
    HANDLE xiH;

    // ���� ����
    std::atomic<CameraState> state;
    std::atomic<bool> isCapturing;

    // ������ ����
    std::unique_ptr<std::thread> captureThread;
    std::unique_ptr<std::thread> callbackThread;

    // ����ȭ ��ü
    mutable std::mutex paramMutex;
    mutable std::mutex callbackMutex;
    std::condition_variable frameCV;

    // ���� ����
    struct FrameBuffer {
        std::vector<unsigned char> data;
        FrameInfo info;
        bool ready;
    };
    std::vector<FrameBuffer> frameBuffers;
    std::queue<int> availableBuffers;
    std::queue<int> readyBuffers;
    mutable std::mutex bufferMutex;

    // �Ķ����
    CameraParameters params;
    CameraParameters targetParams;
    std::atomic<bool> paramsChanged;

    // �ݹ�
    FrameCallback frameCallback;
    ErrorCallback errorCallback;
    LogCallback logCallback;

    // ���� ����͸�
    struct PerformanceStats {
        std::atomic<uint64_t> framesCaptures;
        std::atomic<uint64_t> framesDropped;
        std::atomic<float> currentFps;
        std::chrono::steady_clock::time_point startTime;
        std::chrono::steady_clock::time_point lastFrameTime;
    } perfStats;

    // �α�
    std::string logFilePath;
    std::mutex logMutex;
    LogLevel minLogLevel = LogLevel::LOG_INFO;
    void Log(LogLevel level, const std::string& message);

    // ���� �Լ���
    void CaptureLoop();
    void CallbackLoop();
    bool ApplyParameters();
    void InitializeBuffers();
    void CleanupBuffers();
    std::string GetXiErrorString(XI_RETURN error);
    void UpdatePerformanceStats();

public:
    ~CameraController();

    // �̱��� ����
    static CameraController& GetInstance();
    static void DestroyInstance();

    // ī�޶� ����
    bool Initialize(int deviceIndex = 0);
    bool StartCapture();
    void StopCapture();
    void Shutdown();

    // �Ķ���� ����
    bool SetParameter(const std::string& name, int value);
    bool SetParameter(const std::string& name, float value);
    bool SetParameter(const std::string& name, bool value);
    bool GetParameter(const std::string& name, int& value) const;
    bool GetParameter(const std::string& name, float& value) const;
    bool GetParameter(const std::string& name, bool& value) const;

    // ���� �Լ���
    bool SetExposure(int microsec);
    bool SetGain(float gain);
    bool SetFPS(float fps);
    bool SetROI(int offsetX, int offsetY, int width, int height);
    bool SetBinning(int horizontal, int vertical);
    bool SetDecimation(int horizontal, int vertical);
    bool SetTriggerMode(TriggerMode mode);
    bool SoftwareTrigger();

    // �ڵ� ���
    bool SetAutoExposure(bool enable, float targetLevel = 0.5f);
    bool SetAutoGain(bool enable);
    bool SetAutoWhiteBalance(bool enable);

    // �̹��� ����
    bool SetImageFormat(int format);
    std::vector<int> GetSupportedFormats() const;

    // ī�޶� ����
    std::string GetDeviceName() const;
    std::string GetSerialNumber() const;
    bool GetSensorSize(int& width, int& height) const;
    bool GetTemperature(float& temp) const;

    // ������ ȹ�� (���� ���)
    bool GetLatestFrame(unsigned char* buffer, int bufferSize,
        int& width, int& height, uint64_t& frameNumber);

    // �ݹ� ����
    void SetFrameCallback(FrameCallback callback);
    void SetErrorCallback(ErrorCallback callback);
    void SetLogCallback(LogCallback callback);

    // �α� ����
    void SetLogFile(const std::string& filepath);
    void SetLogLevel(LogLevel level);

    // ���� ���
    float GetCurrentFPS() const { return perfStats.currentFps; }
    uint64_t GetFramesCaptured() const { return perfStats.framesCaptures; }
    uint64_t GetFramesDropped() const { return perfStats.framesDropped; }

    // ���� Ȯ��
    CameraState GetState() const { return state; }
    bool IsCapturing() const { return isCapturing; }
    CameraParameters GetCurrentParameters() const;

    // ��� ���
    bool SaveParametersToFile(const std::string& filepath) const;
    bool LoadParametersFromFile(const std::string& filepath);
    bool CaptureImage(const std::string& filepath, int format = XI_RAW8);

    bool SaveBMP(const std::string& filepath, const unsigned char* data, int width, int height);
};