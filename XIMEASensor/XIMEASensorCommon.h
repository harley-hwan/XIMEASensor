#pragma once

#ifdef XIMEASENSOR_EXPORTS
#define XIMEASENSOR_API __declspec(dllexport)
#else
#define XIMEASENSOR_API __declspec(dllimport)
#endif

#include <cstdint>

namespace XimeaSensor {

    // 에러 코드 정의
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

    // 로그 레벨
    enum class LogLevel : uint8_t {
        Trace = 0,
        Debug = 1,
        Info = 2,
        Warning = 3,
        Error = 4,
        Critical = 5
    };

    // 카메라 정보 구조체
    struct CameraInfo {
        char serialNumber[32];
        char modelName[64];
        uint32_t maxWidth;
        uint32_t maxHeight;
        uint32_t maxFps;
        bool isColorCamera;
    };

    // 프레임 정보 구조체
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

    // ROI (Region of Interest) 구조체
    struct ROI {
        uint32_t offsetX;
        uint32_t offsetY;
        uint32_t width;
        uint32_t height;
    };

    // 카메라 설정 구조체
    struct CameraSettings {
        float exposureTime;     // microseconds
        float gain;            // dB
        uint32_t targetFps;
        ROI roi;
        bool autoExposure;
        bool autoGain;
    };

    // 프레임 콜백 인터페이스
    class IFrameCallback {
    public:
        virtual ~IFrameCallback() = default;

        // 새 프레임이 준비되었을 때 호출
        virtual void OnFrameReady(const uint8_t* frameData, const FrameInfo& info) = 0;

        // 에러 발생 시 호출
        virtual void OnError(ErrorCode error, const char* message) = 0;

        // 카메라 연결이 끊어졌을 때 호출
        virtual void OnCameraDisconnected() = 0;
    };

    // 로그 콜백 인터페이스
    class ILogCallback {
    public:
        virtual ~ILogCallback() = default;
        virtual void OnLog(LogLevel level, const char* message, const char* file, int line) = 0;
    };

} // namespace XimeaSensor