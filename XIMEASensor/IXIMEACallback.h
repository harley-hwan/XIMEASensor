#pragma once
#include <string>
#include "CameraTypes.h"

// FrameInfo - platform independent
struct FrameInfo {
    unsigned char* data;          // ������ ������
    int width;                    // ������ �ʺ�
    int height;                   // ������ ����
    unsigned long frameNumber;    // ������ ��ȣ
    unsigned long acqFrameNumber; // ȹ�� ������ ��ȣ
    double timestamp;             // Ÿ�ӽ�����
    float currentFPS;             // ���� FPS (��갪)
    unsigned int blackLevel;      // �� ����
    unsigned int GPI_level;       // GPI ����
    float exposureTime_ms;        // ���� �ð� (ms)
    float gain_db;                // ���� (dB)
    Camera::ImageFormat format;   // �̹��� ����
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