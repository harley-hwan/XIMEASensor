#pragma once
#include <xiApi.h>
#include <string>

// FrameInfo - from XI_IMG struct
struct FrameInfo {
    unsigned char* data;          // 프레임 데이터 (XI_IMG.bp)
    int width;                    // 프레임 너비 (XI_IMG.width)
    int height;                   // 프레임 높이 (XI_IMG.height)
    unsigned long frameNumber;    // 프레임 번호 (XI_IMG.nframe)
    unsigned long acqFrameNumber; // 획득 프레임 번호 (XI_IMG.acq_nframe)
    double timestamp;             // 타임스탬프 (XI_IMG.tsSec + tsUSec/1000000.0)
    float currentFPS;             // 현재 FPS (계산값)
    unsigned int blackLevel;      // 블랙 레벨 (XI_IMG.black_level)
    unsigned int GPI_level;       // GPI 레벨 (XI_IMG.GPI_level)
    float exposureTime_ms;        // 노출 시간 (XI_IMG.exposure_time_us / 1000.0)
    float gain_db;                // 게인 (XI_IMG.gain_db)
    XI_IMG_FORMAT format;         // 이미지 포맷 (XI_IMG.frm)
};

enum class CameraState {
    DISCONNECTED = 0,
    CONNECTED,
    CAPTURING,
    kERROR
};

// Camera Error Code - from XI_RETURN of xiApi
enum class CameraError {
    NONE = XI_OK,
    DEVICE_NOT_FOUND = XI_NO_DEVICES_FOUND,
    OPEN_FAILED = XI_INVALID_HANDLE,
    START_FAILED = XI_ACQUISITION_STOPED,
    PARAMETER_ERROR = XI_WRONG_PARAM_VALUE,
    FRAME_GRAB_ERROR = XI_TIMEOUT,
    TIMEOUT = XI_TIMEOUT,
    MEMORY_ERROR = XI_MEMORY_ALLOCATION,
    UNKNOWN = -1
};

// callback interface
class IXIMEACallback {
public:
    virtual ~IXIMEACallback() = default;

    // 새 프레임이 캡처되었을 때
    virtual void OnFrameReceived(const FrameInfo& frameInfo) = 0;

    // 카메라 상태가 변경되었을 때
    virtual void OnCameraStateChanged(CameraState newState, CameraState oldState) = 0;

    // 에러가 발생했을 때
    virtual void OnError(CameraError error, const std::string& errorMessage) = 0;

    // 카메라 속성이 변경되었을 때
    virtual void OnPropertyChanged(const std::string& propertyName, const std::string& value) {
        
    }
};