#pragma once
#include <xiApi.h>
#include <string>

// FrameInfo - from XI_IMG struct
struct FrameInfo {
    unsigned char* data;          // ������ ������ (XI_IMG.bp)
    int width;                    // ������ �ʺ� (XI_IMG.width)
    int height;                   // ������ ���� (XI_IMG.height)
    unsigned long frameNumber;    // ������ ��ȣ (XI_IMG.nframe)
    unsigned long acqFrameNumber; // ȹ�� ������ ��ȣ (XI_IMG.acq_nframe)
    double timestamp;             // Ÿ�ӽ����� (XI_IMG.tsSec + tsUSec/1000000.0)
    float currentFPS;             // ���� FPS (��갪)
    unsigned int blackLevel;      // �� ���� (XI_IMG.black_level)
    unsigned int GPI_level;       // GPI ���� (XI_IMG.GPI_level)
    float exposureTime_ms;        // ���� �ð� (XI_IMG.exposure_time_us / 1000.0)
    float gain_db;                // ���� (XI_IMG.gain_db)
    XI_IMG_FORMAT format;         // �̹��� ���� (XI_IMG.frm)
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

    // �� �������� ĸó�Ǿ��� ��
    virtual void OnFrameReceived(const FrameInfo& frameInfo) = 0;

    // ī�޶� ���°� ����Ǿ��� ��
    virtual void OnCameraStateChanged(CameraState newState, CameraState oldState) = 0;

    // ������ �߻����� ��
    virtual void OnError(CameraError error, const std::string& errorMessage) = 0;

    // ī�޶� �Ӽ��� ����Ǿ��� ��
    virtual void OnPropertyChanged(const std::string& propertyName, const std::string& value) {
        
    }
};