#pragma once

#ifdef XIMEASENSOR_EXPORTS
#define XIMEASENSOR_API __declspec(dllexport)
#else
#define XIMEASENSOR_API __declspec(dllimport)
#endif

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    // ���� �ڵ� (C �������̽���)
    typedef enum {
        XSENSOR_SUCCESS = 0,
        XSENSOR_ERROR_CAMERA_NOT_FOUND = -1000,
        XSENSOR_ERROR_CAMERA_OPEN_FAILED = -1001,
        XSENSOR_ERROR_CAMERA_ALREADY_OPEN = -1002,
        XSENSOR_ERROR_CAMERA_NOT_OPEN = -1003,
        XSENSOR_ERROR_ACQUISITION_START_FAILED = -1004,
        XSENSOR_ERROR_ACQUISITION_STOP_FAILED = -1005,
        XSENSOR_ERROR_FRAME_CAPTURE_FAILED = -1006,
        XSENSOR_ERROR_INVALID_PARAMETER = -1007,
        XSENSOR_ERROR_BUFFER_TOO_SMALL = -1008,
        XSENSOR_ERROR_MEMORY_ALLOCATION_FAILED = -1009,
        XSENSOR_ERROR_THREAD_CREATION_FAILED = -1010,
        XSENSOR_ERROR_CALLBACK_NOT_SET = -1011,
        XSENSOR_ERROR_NETWORK_ERROR = -1012,
        XSENSOR_ERROR_UNKNOWN = -9999
    } XSensorError;

    // �α� ����
    typedef enum {
        XSENSOR_LOG_TRACE = 0,
        XSENSOR_LOG_DEBUG = 1,
        XSENSOR_LOG_INFO = 2,
        XSENSOR_LOG_WARNING = 3,
        XSENSOR_LOG_ERROR = 4,
        XSENSOR_LOG_CRITICAL = 5
    } XSensorLogLevel;

    // ī�޶� ���� ����ü
    typedef struct {
        char serialNumber[32];
        char modelName[64];
        uint32_t maxWidth;
        uint32_t maxHeight;
        uint32_t maxFps;
        int isColorCamera;
    } XSensorCameraInfo;

    // ������ ���� ����ü
    typedef struct {
        uint64_t frameNumber;
        uint64_t timestamp;
        uint32_t width;
        uint32_t height;
        uint32_t stride;
        uint8_t bytesPerPixel;
        float exposureTime;
        float gain;
    } XSensorFrameInfo;

    // ROI ����ü
    typedef struct {
        uint32_t offsetX;
        uint32_t offsetY;
        uint32_t width;
        uint32_t height;
    } XSensorROI;

    // ī�޶� ���� ����ü
    typedef struct {
        float exposureTime;
        float gain;
        uint32_t targetFps;
        XSensorROI roi;
        int autoExposure;
        int autoGain;
    } XSensorCameraSettings;

    // ���� ��� ����ü
    typedef struct {
        uint64_t totalFrames;
        uint64_t droppedFrames;
        uint64_t errorCount;
        float averageFps;
    } XSensorPerformanceStats;

    // �ݹ� �Լ� Ÿ��
    typedef void (*XSensorFrameCallback)(const uint8_t* frameData, const XSensorFrameInfo* info, void* userData);
    typedef void (*XSensorErrorCallback)(XSensorError error, const char* message, void* userData);
    typedef void (*XSensorDisconnectCallback)(void* userData);
    typedef void (*XSensorLogCallback)(XSensorLogLevel level, const char* message, const char* file, int line, void* userData);

    // ============================================================================
    // �ý��� �ʱ�ȭ/����
    // ============================================================================
    XIMEASENSOR_API XSensorError XSensor_Initialize(const char* logFilePath, XSensorLogLevel minLogLevel);
    XIMEASENSOR_API void XSensor_Shutdown();
    XIMEASENSOR_API const char* XSensor_GetVersion();
    XIMEASENSOR_API const char* XSensor_GetErrorString(XSensorError error);

    // ============================================================================
    // �α� �ý���
    // ============================================================================
    XIMEASENSOR_API void XSensor_SetLogLevel(XSensorLogLevel level);
    XIMEASENSOR_API void XSensor_SetLogCallback(XSensorLogCallback callback, void* userData);
    XIMEASENSOR_API void XSensor_EnableConsoleLog(int enable);
    XIMEASENSOR_API void XSensor_EnableFileLog(int enable);

    // ============================================================================
    // ī�޶� ����
    // ============================================================================
    XIMEASENSOR_API XSensorError XSensor_OpenCamera(int deviceIndex);
    XIMEASENSOR_API XSensorError XSensor_CloseCamera();
    XIMEASENSOR_API int XSensor_IsCameraOpen();

    XIMEASENSOR_API XSensorError XSensor_StartCapture();
    XIMEASENSOR_API XSensorError XSensor_StopCapture();
    XIMEASENSOR_API int XSensor_IsCapturing();

    // ============================================================================
    // ī�޶� ����
    // ============================================================================
    XIMEASENSOR_API XSensorError XSensor_SetExposureTime(float microseconds);
    XIMEASENSOR_API XSensorError XSensor_SetGain(float gainDb);
    XIMEASENSOR_API XSensorError XSensor_SetROI(const XSensorROI* roi);
    XIMEASENSOR_API XSensorError XSensor_SetTargetFps(uint32_t fps);
    XIMEASENSOR_API XSensorError XSensor_SetAutoExposure(int enable);
    XIMEASENSOR_API XSensorError XSensor_SetAutoGain(int enable);
    XIMEASENSOR_API XSensorError XSensor_ApplySettings(const XSensorCameraSettings* settings);

    // ============================================================================
    // ī�޶� ��ȸ
    // ============================================================================
    XIMEASENSOR_API XSensorError XSensor_GetCameraInfo(XSensorCameraInfo* info);
    XIMEASENSOR_API XSensorError XSensor_GetCurrentSettings(XSensorCameraSettings* settings);
    XIMEASENSOR_API XSensorError XSensor_GetExposureRange(float* minUs, float* maxUs);
    XIMEASENSOR_API XSensorError XSensor_GetGainRange(float* minDb, float* maxDb);
    XIMEASENSOR_API float XSensor_GetCurrentFps();
    XIMEASENSOR_API XSensorError XSensor_GetPerformanceStats(XSensorPerformanceStats* stats);
    XIMEASENSOR_API void XSensor_ResetPerformanceStats();

    // ============================================================================
    // �ݹ� ����
    // ============================================================================
    XIMEASENSOR_API void XSensor_SetFrameCallback(XSensorFrameCallback callback, void* userData);
    XIMEASENSOR_API void XSensor_SetErrorCallback(XSensorErrorCallback callback, void* userData);
    XIMEASENSOR_API void XSensor_SetDisconnectCallback(XSensorDisconnectCallback callback, void* userData);

    // ============================================================================
    // ����� ĸó
    // ============================================================================
    XIMEASENSOR_API XSensorError XSensor_CaptureSnapshot(uint8_t* buffer, size_t bufferSize,
        XSensorFrameInfo* info, uint32_t timeoutMs);

    // ============================================================================
    // ī�޶� ����
    // ============================================================================
    XIMEASENSOR_API XSensorError XSensor_EnumerateCameras(XSensorCameraInfo* cameras,
        int* count, int maxCount);

    // ============================================================================
    // ��Ʈ��ũ ����
    // ============================================================================
    XIMEASENSOR_API XSensorError XSensor_StartNetworkServer(uint16_t port);
    XIMEASENSOR_API XSensorError XSensor_StopNetworkServer();
    XIMEASENSOR_API int XSensor_IsNetworkServerRunning();
    XIMEASENSOR_API int XSensor_GetNetworkClientCount();
    XIMEASENSOR_API void XSensor_SetFrameCompression(int enable, int jpegQuality);

    // ============================================================================
    // ���Ž� ȣȯ�� (���� ���� API)
    // ============================================================================
    XIMEASENSOR_API int Camera_Open(int deviceIndex);
    XIMEASENSOR_API void Camera_Close();
    XIMEASENSOR_API int Camera_Start();
    XIMEASENSOR_API void Camera_Stop();
    XIMEASENSOR_API int Camera_GetFrame(unsigned char* buffer, int bufferSize, int* width, int* height);
    XIMEASENSOR_API int Camera_SetExposure(int microsec);
    XIMEASENSOR_API int Camera_SetROI(int offsetX, int offsetY, int width, int height);
    XIMEASENSOR_API int Camera_SetGain(float gain);

#ifdef __cplusplus
}
#endif