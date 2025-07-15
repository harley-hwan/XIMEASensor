#pragma once
#include <xiApi.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include "XIMEASensor.h" 
#include "IXIMEACallback.h"
#include "Logger.h"
#include "ContinuousCaptureManager.h"

#define PYTHON1300_WIDTH    1280
#define PYTHON1300_HEIGHT   1024
#define PYTHON1300_MAX_FPS  210

struct CameraStatistics {
    unsigned long totalFrames;
    unsigned long droppedFrames;
    double averageFPS;
    double minFPS;
    double maxFPS;
    std::chrono::steady_clock::time_point startTime;

    void Reset() {
        totalFrames = 0;
        droppedFrames = 0;
        averageFPS = 0.0;
        minFPS = 9999.0;
        maxFPS = 0.0;
        startTime = std::chrono::steady_clock::now();
    }
};

class CameraController {
private:
    // Singleton instance
    static std::unique_ptr<CameraController> instance;
    static std::mutex instanceMutex;

    // XIMEA handle
    HANDLE xiH;

    // Thread control
    std::thread captureThread;
    std::atomic<bool> isRunning;
    std::atomic<bool> isPaused;

    // Frame buffer
    std::mutex frameMutex;
    unsigned char* frameBuffer;
    unsigned char* workingBuffer;

    // Camera parameters
    int width;
    int height;
    int currentExposure;
    float currentGain;
    float currentFrameRate;  // Current framerate setting

    // Callbacks
    std::mutex callbackMutex;
    std::vector<IXIMEACallback*> callbacks;

    // State management
    std::atomic<CameraState> currentState;
    CameraStatistics stats;
    std::chrono::steady_clock::time_point lastFrameTime;
    std::unique_ptr<ContinuousCaptureManager> m_continuousCapture;

    // Error handling
    std::atomic<int> deviceNotReadyCount;
    static const int MAX_DEVICE_NOT_READY_ERRORS = 5;

    CameraController();

    void CaptureLoop();

    void NotifyFrameReceived(const FrameInfo& info);
    void NotifyStateChanged(CameraState newState);
    void NotifyError(CameraError error, const std::string& message);
    void NotifyPropertyChanged(const std::string& property, const std::string& value);

    void UpdateStatistics(bool frameReceived);
    std::string GetXiApiErrorString(XI_RETURN error);

public:
    ~CameraController();

    // Singleton
    static CameraController& GetInstance();
    static void Destroy();

    // Camera control
    bool OpenCamera(int deviceIndex);
    void CloseCamera();
    bool StartCapture();
    void StopCapture();
    void PauseCapture(bool pause);

    bool GetFrame(unsigned char* buffer, int bufferSize, int& outWidth, int& outHeight);

    // Camera settings
    bool SetExposure(int microsec);
    bool SetGain(float gain);
    bool SetROI(int offsetX, int offsetY, int width, int height);
    bool SetFrameRate(float fps);
    bool SetTriggerMode(bool enabled);

    // Getters
    int GetExposure() const { return currentExposure; }
    float GetGain() const { return currentGain; }
    int GetWidth() const { return width; }
    int GetHeight() const { return height; }
    float GetFrameRate();
    CameraState GetState() const { return currentState.load(); }
    CameraStatistics GetStatistics() const { return stats; }
    void ResetStatistics() { stats.Reset(); }
    ContinuousCaptureManager* GetContinuousCaptureManager() { return m_continuousCapture.get(); }

    // Callback management
    void RegisterCallback(IXIMEACallback* callback);
    void UnregisterCallback(IXIMEACallback* callback);
    void ClearCallbacks();

    // Device enumeration
    int GetConnectedDeviceCount();
    bool GetDeviceInfo(int index, std::string& name, std::string& serial);
};