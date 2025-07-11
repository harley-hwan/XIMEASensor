// CameraController.h�� ������ Callback Ÿ�� ���� �߰�
#pragma once
#include <xiApi.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <string>
#include "Logger.h"

// Callback type definitions
using CameraFrameCallback = void(*)(const unsigned char*, int, int);
using CameraErrorCallback = void(*)(int, const char*);
using CameraLogCallback = void(*)(const char*);

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

    // ������ �߰� ����
    int actualWidth;
    int actualHeight;
    unsigned long frameCounter;

    // Static callback function pointers
    static CameraFrameCallback s_frameCallback;
    static CameraErrorCallback s_errorCallback;
    static CameraLogCallback s_logCallback;

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

    float GetFramerate();
    unsigned long GetFrameCounter();

    // Callback registration (static)
    static void SetFrameCallback(CameraFrameCallback cb);
    static void SetErrorCallback(CameraErrorCallback cb);
    static void SetLogCallback(CameraLogCallback cb);
    // Direct invocation of callbacks (internal use)
    static void InvokeFrameCallback(const unsigned char* data, int width, int height);
    static void InvokeErrorCallback(int errorCode, const char* msg);
    static void InvokeLogCallback(const char* msg);
};
