#pragma once
#include "CameraInterface.h"
#include <map>

namespace Camera {

    // Forward declaration to hide xiApi dependency
    class XIMEACameraPrivate;

    class XIMEACameraImpl : public ICameraInterface {
    public:
        XIMEACameraImpl();
        ~XIMEACameraImpl() override;

        // ICameraInterface implementation
        ReturnCode OpenDevice(int deviceIndex, void** handle) override;
        ReturnCode CloseDevice(void* handle) override;
        ReturnCode GetNumberDevices(uint32_t* count) override;
        ReturnCode GetDeviceInfoString(int deviceIndex, ParamType param, char* value, size_t size) override;

        ReturnCode StartAcquisition(void* handle) override;
        ReturnCode StopAcquisition(void* handle) override;
        ReturnCode GetImage(void* handle, uint32_t timeout_ms, ImageData* image) override;

        ReturnCode SetParamInt(void* handle, ParamType param, int value) override;
        ReturnCode GetParamInt(void* handle, ParamType param, int* value) override;
        ReturnCode SetParamFloat(void* handle, ParamType param, float value) override;
        ReturnCode GetParamFloat(void* handle, ParamType param, float* value) override;
        ReturnCode SetParamString(void* handle, ParamType param, const char* value) override;
        ReturnCode GetParamString(void* handle, ParamType param, char* value, size_t size) override;

        std::string GetErrorString(ReturnCode error) override;

    private:
        // Pimpl idiom to hide xiApi dependency
        std::unique_ptr<XIMEACameraPrivate> d;
    };

} // namespace Camera