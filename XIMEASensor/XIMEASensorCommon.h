#pragma once

#ifdef XIMEASENSOR_EXPORTS
#define XIMEASENSOR_API __declspec(dllexport)
#else
#define XIMEASENSOR_API __declspec(dllimport)
#endif

#include <cstdint>

namespace XimeaSensor {

    // ���� �ڵ� ����
    enum class ErrorCode : int32_t {
        Success = 0,
        CameraNotFound = -1000,
        CameraOpenFailed = -1001,
        CameraAlreadyOpen = -1002,
        CameraNotOpen = -1003,
        AcquisitionStartFailed = -1004,
        AcquisitionStopFailed = -1005,
        FrameCaptureFailed = -1006,
        InvalidParameter = -1007,
        BufferTooSmall = -1008,
        MemoryAllocationFailed = -1009,
        ThreadCreationFailed = -1010,
        CallbackNotSet = -1011,
        NetworkError = -1012,
        UnknownError = -9999
    };

    // �α� ����
    enum class LogLevel : uint8_t {
        Trace = 0,
        Debug = 1,
        Info = 2,
        Warning = 3,
        Error = 4,
        Critical = 5
    };

    // ī�޶� ���� ����ü
    struct CameraInfo {
        char serialNumber[32];
        char modelName[64];
        uint32_t maxWidth;
        uint32_t maxHeight;
        uint32_t maxFps;
        bool isColorCamera;
    };

    // ������ ���� ����ü
    struct FrameInfo {
        uint64_t frameNumber;
        uint64_t timestamp;  // microseconds since epoch
        uint32_t width;
        uint32_t height;
        uint32_t stride;
        uint8_t bytesPerPixel;
        float exposureTime;  // milliseconds
        float gain;         // dB
    };

    // ROI (Region of Interest) ����ü
    struct ROI {
        uint32_t offsetX;
        uint32_t offsetY;
        uint32_t width;
        uint32_t height;
    };

    // ī�޶� ���� ����ü
    struct CameraSettings {
        float exposureTime;     // microseconds
        float gain;            // dB
        uint32_t targetFps;
        ROI roi;
        bool autoExposure;
        bool autoGain;
    };

    // ������ �ݹ� �������̽�
    class IFrameCallback {
    public:
        virtual ~IFrameCallback() = default;

        // �� �������� �غ�Ǿ��� �� ȣ��
        virtual void OnFrameReady(const uint8_t* frameData, const FrameInfo& info) = 0;

        // ���� �߻� �� ȣ��
        virtual void OnError(ErrorCode error, const char* message) = 0;

        // ī�޶� ������ �������� �� ȣ��
        virtual void OnCameraDisconnected() = 0;
    };

    // �α� �ݹ� �������̽�
    class ILogCallback {
    public:
        virtual ~ILogCallback() = default;
        virtual void OnLog(LogLevel level, const char* message, const char* file, int line) = 0;
    };

} // namespace XimeaSensor