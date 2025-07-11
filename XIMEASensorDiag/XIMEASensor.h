#pragma once

#ifdef XIMEASENSOR_EXPORTS
#define XIMEASENSOR_API __declspec(dllexport)
#else
#define XIMEASENSOR_API __declspec(dllimport)
#endif

extern "C" {
	XIMEASENSOR_API bool Camera_Open(int deviceIndex);
	XIMEASENSOR_API void Camera_Close();
	XIMEASENSOR_API bool Camera_Start();
	XIMEASENSOR_API void Camera_Stop();
	XIMEASENSOR_API bool Camera_GetFrame(unsigned char* buffer, int bufferSize, int* width, int* height);
	XIMEASENSOR_API bool Camera_SetExposure(int microsec);
	XIMEASENSOR_API bool Camera_SetROI(int offsetX, int offsetY, int width, int height);
	XIMEASENSOR_API bool Camera_SetGain(float gain);
}
