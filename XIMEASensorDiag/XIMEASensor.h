#pragma once

#ifdef XIMEASENSOR_EXPORTS
#define XIMEASENSOR_API __declspec(dllexport)
#else
#define XIMEASENSOR_API __declspec(dllimport)
#endif
//
#include "IXIMEACallback.h"

extern "C" {
    XIMEASENSOR_API bool Camera_Initialize(const char* logPath = nullptr, int logLevel = 1);
    XIMEASENSOR_API void Camera_Shutdown();

    XIMEASENSOR_API int Camera_GetDeviceCount();
    XIMEASENSOR_API bool Camera_GetDeviceInfo(int index, char* name, int nameSize, char* serial, int serialSize);

    XIMEASENSOR_API bool Camera_Open(int deviceIndex);
    XIMEASENSOR_API void Camera_Close();
    XIMEASENSOR_API bool Camera_Start();
    XIMEASENSOR_API void Camera_Stop();
    XIMEASENSOR_API bool Camera_Pause(bool pause);

    XIMEASENSOR_API bool Camera_GetFrame(unsigned char* buffer, int bufferSize,int* width, int* height);

    XIMEASENSOR_API bool Camera_SetExposure(int microsec);
    XIMEASENSOR_API bool Camera_SetGain(float gain);
    XIMEASENSOR_API bool Camera_SetROI(int offsetX, int offsetY, int width, int height);
    XIMEASENSOR_API bool Camera_SetFrameRate(float fps);
    XIMEASENSOR_API bool Camera_SetTriggerMode(bool enabled);

    XIMEASENSOR_API int Camera_GetExposure();
    XIMEASENSOR_API float Camera_GetGain();
    XIMEASENSOR_API bool Camera_GetROI(int* offsetX, int* offsetY, int* width, int* height);
    XIMEASENSOR_API float Camera_GetFrameRate();
    XIMEASENSOR_API int Camera_GetState();

    XIMEASENSOR_API bool Camera_GetStatistics(unsigned long* totalFrames, unsigned long* droppedFrames, double* averageFPS, double* minFPS, double* maxFPS);
    XIMEASENSOR_API void Camera_ResetStatistics();

    XIMEASENSOR_API bool Camera_RegisterCallback(IXIMEACallback* callback);
    XIMEASENSOR_API bool Camera_UnregisterCallback(IXIMEACallback* callback);
    XIMEASENSOR_API void Camera_ClearCallbacks();

    XIMEASENSOR_API void Camera_SetLogLevel(int level);
    XIMEASENSOR_API void Camera_FlushLog();
}