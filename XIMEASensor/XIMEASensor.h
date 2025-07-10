#pragma once

#ifdef XIMEASENSOR_EXPORTS
#define XIMEASENSOR_API __declspec(dllexport)
#else
#define XIMEASENSOR_API __declspec(dllimport)
#endif

#ifdef __cplusplus
extern "C" {
#endif

    // �ݹ� Ÿ�� ����
    typedef void (*FRAME_CALLBACK)(const unsigned char* data, int width, int height,
        unsigned long frameNumber, double timestamp, void* userContext);
    typedef void (*ERROR_CALLBACK)(int errorCode, const char* errorMessage, void* userContext);
    typedef void (*LOG_CALLBACK)(int logLevel, const char* message, void* userContext);

    // �α� ����
    enum XIMEA_LOG_LEVEL {
        XIMEA_LOG_DEBUG = 0,
        XIMEA_LOG_INFO = 1,
        XIMEA_LOG_WARNING = 2,
        XIMEA_LOG_ERROR = 3,
        XIMEA_LOG_CRITICAL = 4
    };

    // ī�޶� ����
    enum XIMEA_CAMERA_STATE {
        XIMEA_STATE_UNINITIALIZED = 0,
        XIMEA_STATE_INITIALIZED = 1,
        XIMEA_STATE_STREAMING = 2,
        XIMEA_STATE_ERROR = 3
    };

    // Ʈ���� ���
    enum XIMEA_TRIGGER_MODE {
        XIMEA_TRIGGER_FREE_RUN = 0,
        XIMEA_TRIGGER_SOFTWARE = 1,
        XIMEA_TRIGGER_HARDWARE = 2
    };

    // �̹��� ����
    enum XIMEA_IMAGE_FORMAT {
        XIMEA_FORMAT_MONO8 = 0,
        XIMEA_FORMAT_MONO16 = 1,
        XIMEA_FORMAT_RGB24 = 2,
        XIMEA_FORMAT_RGB32 = 3,
        XIMEA_FORMAT_RAW8 = 4,
        XIMEA_FORMAT_RAW16 = 5
    };

    // ī�޶� ���� ����ü
    typedef struct {
        char deviceName[256];
        char serialNumber[256];
        int sensorWidth;
        int sensorHeight;
        float pixelSize;
        int maxFPS;
        float temperature;
        int apiVersion;
        int driverVersion;
    } XIMEA_CAMERA_INFO;

    // ���� ��� ����ü
    typedef struct {
        float currentFPS;
        unsigned long framesCaptured;
        unsigned long framesDropped;
        double captureTime;
        float cpuUsage;
        float bandwidthMbps;
    } XIMEA_PERFORMANCE_STATS;

    // �⺻ ī�޶� ����
    XIMEASENSOR_API bool Camera_Initialize(int deviceIndex);
    XIMEASENSOR_API bool Camera_Shutdown();
    XIMEASENSOR_API bool Camera_StartCapture();
    XIMEASENSOR_API bool Camera_StopCapture();
    XIMEASENSOR_API int Camera_GetState();

    // ������ ȹ�� (���� ���)
    XIMEASENSOR_API bool Camera_GetLatestFrame(unsigned char* buffer, int bufferSize,
        int* width, int* height,
        unsigned long* frameNumber);

    // �ݹ� ����
    XIMEASENSOR_API void Camera_SetFrameCallback(FRAME_CALLBACK callback, void* userContext);
    XIMEASENSOR_API void Camera_SetErrorCallback(ERROR_CALLBACK callback, void* userContext);
    XIMEASENSOR_API void Camera_SetLogCallback(LOG_CALLBACK callback, void* userContext);

    // �Ķ���� ����
    XIMEASENSOR_API bool Camera_SetExposure(int microsec);
    XIMEASENSOR_API bool Camera_SetGain(float gain);
    XIMEASENSOR_API bool Camera_SetFPS(float fps);
    XIMEASENSOR_API bool Camera_SetROI(int offsetX, int offsetY, int width, int height);
    XIMEASENSOR_API bool Camera_SetBinning(int horizontal, int vertical);
    XIMEASENSOR_API bool Camera_SetDecimation(int horizontal, int vertical);

    // �Ķ���� �б�
    XIMEASENSOR_API int Camera_GetExposure();
    XIMEASENSOR_API float Camera_GetGain();
    XIMEASENSOR_API float Camera_GetFPS();
    XIMEASENSOR_API bool Camera_GetROI(int* offsetX, int* offsetY, int* width, int* height);

    // �ڵ� ���
    XIMEASENSOR_API bool Camera_SetAutoExposure(bool enable, float targetLevel);
    XIMEASENSOR_API bool Camera_SetAutoGain(bool enable);
    XIMEASENSOR_API bool Camera_SetAutoWhiteBalance(bool enable);

    // Ʈ���� ����
    XIMEASENSOR_API bool Camera_SetTriggerMode(int mode);
    XIMEASENSOR_API bool Camera_SoftwareTrigger();

    // �̹��� ����
    XIMEASENSOR_API bool Camera_SetImageFormat(int format);
    XIMEASENSOR_API int Camera_GetImageFormat();

    // ī�޶� ����
    XIMEASENSOR_API bool Camera_GetInfo(XIMEA_CAMERA_INFO* info);
    XIMEASENSOR_API bool Camera_GetPerformanceStats(XIMEA_PERFORMANCE_STATS* stats);

    // �α�
    XIMEASENSOR_API void Camera_SetLogFile(const char* filepath);
    XIMEASENSOR_API void Camera_SetLogLevel(int level);

    // ���� ����
    XIMEASENSOR_API bool Camera_SaveParameters(const char* filepath);
    XIMEASENSOR_API bool Camera_LoadParameters(const char* filepath);

    // �̹��� ĸó
    XIMEASENSOR_API bool Camera_CaptureImage(const char* filepath, int format);

    // ��� ����
    XIMEASENSOR_API bool Camera_SetParameter(const char* name, int value);
    XIMEASENSOR_API bool Camera_SetParameterFloat(const char* name, float value);
    XIMEASENSOR_API bool Camera_GetParameter(const char* name, int* value);
    XIMEASENSOR_API bool Camera_GetParameterFloat(const char* name, float* value);

    // ��ƿ��Ƽ
    XIMEASENSOR_API const char* Camera_GetLastError();
    XIMEASENSOR_API int Camera_GetVersion();

#ifdef __cplusplus
}
#endif