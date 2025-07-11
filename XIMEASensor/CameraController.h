// CameraController.h
#pragma once
#include <xiApi.h>
#include <thread>
#include <mutex>
#include <chrono>

class CameraController
{
private:
    HANDLE xiH;
    std::thread captureThread;
    std::mutex frameMutex;
    bool running;

    unsigned char* frameBuffer;
    int width;
    int height;

    // 디버깅용 추가 변수
    int actualWidth;
    int actualHeight;
    unsigned long frameCounter;

    CameraController();

    void CaptureLoop();

public:
    ~CameraController();

    static CameraController& GetInstance();

    bool OpenCamera(int deviceIndex);
    void CloseCamera();

    bool StartCapture();
    void StopCapture();

    bool GetFrame(unsigned char* buffer, int bufferSize, int& outWidth, int& outHeight);

    bool SetExposure(int microsec);
    bool SetROI(int offsetX, int offsetY, int width, int height);
    bool SetGain(float gain);

    // MQ013MG-ON 전용 추가 함수
    float GetFramerate();
    unsigned long GetFrameCounter();
};