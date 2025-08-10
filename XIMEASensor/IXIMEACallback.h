#pragma once
#include <string>
#include "CameraTypes.h"

// FrameInfo - platform independent
struct FrameInfo {
    unsigned char* data;          // 프레임 데이터
    int width;                    // 프레임 너비
    int height;                   // 프레임 높이
    unsigned long frameNumber;    // 프레임 번호
    unsigned long acqFrameNumber; // 획득 프레임 번호
    double timestamp;             // 타임스탬프
    float currentFPS;             // 현재 FPS (계산값)
    unsigned int blackLevel;      // 블랙 레벨
    unsigned int GPI_level;       // GPI 레벨
    float exposureTime_ms;        // 노출 시간 (ms)
    float gain_db;                // 게인 (dB)
    Camera::ImageFormat format;   // 이미지 포맷
};

enum class CameraState {
    DISCONNECTED = 0,
    CONNECTED,
    CAPTURING,
    kERROR
};

// Camera Error Code - platform independent
enum class CameraError {
    NONE = 0,
    DEVICE_NOT_FOUND,
    OPEN_FAILED,
    START_FAILED,
    PARAMETER_ERROR,
    FRAME_GRAB_ERROR,
    TIMEOUT,
    MEMORY_ERROR,
    DEVICE_NOT_READY,
    UNKNOWN = -1
};

// callback interface
class IXIMEACallback {
public:
    virtual ~IXIMEACallback() = default;
    virtual void OnFrameReceived(const FrameInfo& frameInfo) = 0;
    virtual void OnCameraStateChanged(CameraState newState, CameraState oldState) = 0;
    virtual void OnError(CameraError error, const std::string& errorMessage) = 0;
    virtual void OnPropertyChanged(const std::string& propertyName, const std::string& value) {}
};