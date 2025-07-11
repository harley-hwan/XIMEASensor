#include "pch.h"
#include "XIMEASensor.h"
#include "CameraController.h"
#include "Logger.h"

bool Camera_Open(int deviceIndex)
{
    bool result = CameraController::GetInstance().OpenCamera(deviceIndex);
    if (!result) {
        // If open fails, log error and trigger error callback
        std::string errMsg = "Camera_Open: Failed to open device index " + std::to_string(deviceIndex);
        Logger::LogError("%s (deviceIndex=%d)", "Failed to open camera", deviceIndex);
        CameraController::InvokeErrorCallback(-1, errMsg.c_str());
    }
    else {
        Logger::LogInfo("Camera_Open: Successfully opened device index %d", deviceIndex);
    }
    return result;
}

void Camera_Close()
{
    CameraController::GetInstance().CloseCamera();
    Logger::LogInfo("Camera_Close: Camera closed");
}

bool Camera_Start()
{
    bool result = CameraController::GetInstance().StartCapture();
    if (!result) {
        // Log and callback on error (if camera handle exists but failed to start)
        Logger::LogError("%s", "Camera_Start: Failed to start acquisition");
        CameraController::InvokeErrorCallback(-2, "Failed to start camera acquisition");
    }
    else {
        Logger::LogInfo("%s", "Camera_Start: Acquisition started");
    }
    return result;
}

void Camera_Stop()
{
    CameraController::GetInstance().StopCapture();
    Logger::LogInfo("%s", "Camera_Stop: Acquisition stopped");
}

bool Camera_GetFrame(unsigned char* buffer, int bufferSize, int* width, int* height)
{
    bool result = CameraController::GetInstance().GetFrame(buffer, bufferSize, *width, *height);
    if (!result) {
        // If get frame fails (likely buffer too small), log error
        Logger::LogError("%s", "Camera_GetFrame: Failed to get frame (buffer too small or not available)");
    }
    return result;
}

bool Camera_SetExposure(int microsec)
{
    bool result = CameraController::GetInstance().SetExposure(microsec);
    if (!result) {
        Logger::LogError("Camera_SetExposure: Failed to set exposure to %d µs", microsec);
        CameraController::InvokeErrorCallback(-3, "Failed to set camera exposure");
    }
    else {
        Logger::LogInfo("Camera_SetExposure: Exposure set to %d µs", microsec);
    }
    return result;
}

bool Camera_SetROI(int offsetX, int offsetY, int width, int height)
{
    bool result = CameraController::GetInstance().SetROI(offsetX, offsetY, width, height);
    if (!result) {
        Logger::LogError("Camera_SetROI: Failed to set ROI to (%d, %d, %d, %d)", offsetX, offsetY, width, height);
        CameraController::InvokeErrorCallback(-4, "Failed to set camera ROI");
    }
    else {
        Logger::LogInfo("Camera_SetROI: ROI set to (x=%d, y=%d, w=%d, h=%d)", offsetX, offsetY, width, height);
    }
    return result;
}

bool Camera_SetGain(float gain)
{
    bool result = CameraController::GetInstance().SetGain(gain);
    if (!result) {
        Logger::LogError("Camera_SetGain: Failed to set gain to %.2f dB", gain);
        CameraController::InvokeErrorCallback(-5, "Failed to set camera gain");
    }
    else {
        Logger::LogInfo("Camera_SetGain: Gain set to %.2f dB", gain);
    }
    return result;
}

// Register callback functions
void Camera_SetFrameCallback(CameraFrameCallback cb) {
    CameraController::SetFrameCallback(cb);
    Logger::LogInfo("%s", "Camera_SetFrameCallback: Frame callback registered");
}

void Camera_SetErrorCallback(CameraErrorCallback cb) {
    CameraController::SetErrorCallback(cb);
    Logger::LogInfo("%s", "Camera_SetErrorCallback: Error callback registered");
}

void Camera_SetLogCallback(CameraLogCallback cb) {
    CameraController::SetLogCallback(cb);
    Logger::LogInfo("%s", "Camera_SetLogCallback: Log callback registered");
}
