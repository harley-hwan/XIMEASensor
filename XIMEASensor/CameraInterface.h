#pragma once
#include "CameraTypes.h"
#include <string>
#include <memory>

namespace Camera {

    // Abstract camera interface
    class ICameraInterface {
    public:
        virtual ~ICameraInterface() = default;

        // Device management
        virtual ReturnCode OpenDevice(int deviceIndex, void** handle) = 0;
        virtual ReturnCode CloseDevice(void* handle) = 0;
        virtual ReturnCode GetNumberDevices(uint32_t* count) = 0;
        virtual ReturnCode GetDeviceInfoString(int deviceIndex, ParamType param, char* value, size_t size) = 0;

        // Acquisition control
        virtual ReturnCode StartAcquisition(void* handle) = 0;
        virtual ReturnCode StopAcquisition(void* handle) = 0;
        virtual ReturnCode GetImage(void* handle, uint32_t timeout_ms, ImageData* image) = 0;

        // Parameter set/get
        virtual ReturnCode SetParamInt(void* handle, ParamType param, int value) = 0;
        virtual ReturnCode GetParamInt(void* handle, ParamType param, int* value) = 0;
        virtual ReturnCode SetParamFloat(void* handle, ParamType param, float value) = 0;
        virtual ReturnCode GetParamFloat(void* handle, ParamType param, float* value) = 0;
        virtual ReturnCode SetParamString(void* handle, ParamType param, const char* value) = 0;
        virtual ReturnCode GetParamString(void* handle, ParamType param, char* value, size_t size) = 0;

        // Error handling
        virtual std::string GetErrorString(ReturnCode error) = 0;
    };

    // Factory for creating camera implementations
    class CameraFactory {
    public:
        enum class CameraType {
            XIMEA,
           
        };

        static std::unique_ptr<ICameraInterface> CreateCamera(CameraType type);
    };

} // namespace Camera