#include "pch.h"
#include "XIMEASensor.h"
#include "CameraController.h"

bool Camera_Open(int deviceIndex)
{
    return CameraController::GetInstance().OpenCamera(deviceIndex);
}

void Camera_Close()
{
    CameraController::GetInstance().CloseCamera();
}

bool Camera_Start()
{
    return CameraController::GetInstance().StartCapture();
}

void Camera_Stop()
{
    CameraController::GetInstance().StopCapture();
}

bool Camera_GetFrame(unsigned char* buffer, int bufferSize, int* width, int* height)
{
    return CameraController::GetInstance().GetFrame(buffer, bufferSize, *width, *height);
}

bool Camera_SetExposure(int microsec)
{
    return CameraController::GetInstance().SetExposure(microsec);
}

bool Camera_SetROI(int offsetX, int offsetY, int width, int height)
{
    return CameraController::GetInstance().SetROI(offsetX, offsetY, width, height);
}

bool Camera_SetGain(float gain)
{
    return CameraController::GetInstance().SetGain(gain);
}
