#pragma once

#ifdef SLSENSOR_EXPORTS
#define SLSENSOR_API __declspec(dllexport)
#else
#define SLSENSOR_API __declspec(dllimport)
#endif

// Callback function pointer types for events
typedef void (*CameraFrameCallback)(const unsigned char* frame, int width, int height);
typedef void (*CameraErrorCallback)(int errorCode, const char* errorMsg);
typedef void (*CameraLogCallback)(const char* logMsg);

extern "C" {

    SLSENSOR_API bool Camera_Open(int deviceIndex);
    SLSENSOR_API void Camera_Close();

    SLSENSOR_API bool Camera_Start();
    SLSENSOR_API void Camera_Stop();

    SLSENSOR_API bool Camera_GetFrame(unsigned char* buffer, int bufferSize, int* width, int* height);

    SLSENSOR_API bool Camera_SetExposure(int microsec);
    SLSENSOR_API bool Camera_SetROI(int offsetX, int offsetY, int width, int height);
    SLSENSOR_API bool Camera_SetGain(float gain);

    // Register callback functions for frame, error, and log events
    SLSENSOR_API void Camera_SetFrameCallback(CameraFrameCallback cb);
    SLSENSOR_API void Camera_SetErrorCallback(CameraErrorCallback cb);
    SLSENSOR_API void Camera_SetLogCallback(CameraLogCallback cb);
}
