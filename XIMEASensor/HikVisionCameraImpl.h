#pragma once
#include "CameraInterface.h"
#include "MvCameraControl.h"
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>

namespace Camera {

    class HikVisionCameraImpl : public ICameraInterface {
    public:
        HikVisionCameraImpl();
        ~HikVisionCameraImpl() override;

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
        // HikVision specific structures
        struct HikHandle {
            void* deviceHandle;
            bool isAcquiring;
            std::queue<std::unique_ptr<ImageData>> imageQueue;
            std::mutex queueMutex;
            std::condition_variable queueCV;
            MV_CC_DEVICE_INFO deviceInfo;
            std::atomic<uint64_t> lastFrameNumber;
            std::chrono::steady_clock::time_point lastFrameTime;
        };

        // Callback for image reception
        static void __stdcall ImageCallbackEx(unsigned char* pData,
            MV_FRAME_OUT_INFO_EX* pFrameInfo,
            void* pUser);
        void ProcessImageCallback(HikHandle* handle, unsigned char* pData,
            MV_FRAME_OUT_INFO_EX* pFrameInfo);

        // Helper functions
        ReturnCode ConvertMvError(int mvError);
        MvGvspPixelType ConvertToMvPixelFormat(ImageFormat format);
        ImageFormat ConvertFromMvPixelFormat(MvGvspPixelType mvFormat);
        std::string ConvertParamName(ParamType param);

        // Device list cache
        MV_CC_DEVICE_INFO_LIST m_deviceList;
        std::mutex m_deviceListMutex;
        std::chrono::steady_clock::time_point m_lastEnumTime;
        static constexpr int ENUM_CACHE_MS = 5000; // 5 seconds cache
    };

} // namespace Camera