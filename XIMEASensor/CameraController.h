#pragma once

#include "XIMEASensorCommon.h"
#include <xiApi.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <vector>
#include <chrono>

namespace XimeaSensor {

    class CameraController {
    private:
        // �̱��� �ν��Ͻ�
        static std::unique_ptr<CameraController> s_instance;
        static std::mutex s_instanceMutex;

        // ī�޶� �ڵ�
        HANDLE m_xiHandle;

        // ī�޶� ����
        CameraInfo m_cameraInfo;
        CameraSettings m_settings;

        // ������ ����
        std::unique_ptr<std::thread> m_captureThread;
        std::atomic<bool> m_isCapturing;
        std::atomic<bool> m_shouldStop;

        // �ݹ�
        IFrameCallback* m_frameCallback;
        std::mutex m_callbackMutex;

        // ������ ���� (Ʈ���� ���۸�)
        struct FrameBuffer {
            std::unique_ptr<uint8_t[]> data;
            FrameInfo info;
            std::atomic<bool> ready;
            std::mutex mutex;
        };

        static constexpr size_t BUFFER_COUNT = 3;
        FrameBuffer m_frameBuffers[BUFFER_COUNT];
        std::atomic<size_t> m_writeIndex;
        std::atomic<size_t> m_readIndex;

        // ���� ���
        struct PerformanceStats {
            std::atomic<uint64_t> totalFrames;
            std::atomic<uint64_t> droppedFrames;
            std::atomic<uint64_t> errorCount;
            std::chrono::steady_clock::time_point startTime;
            std::mutex mutex;
        } m_stats;

        // Private ������ (�̱���)
        CameraController();

        // ���� �޼���
        ErrorCode InitializeCamera();
        ErrorCode ConfigureCamera();
        ErrorCode ValidateROI(const ROI& roi);
        void CaptureThreadFunc();
        void ResetBuffers();
        std::string GetXiErrorString(XI_RETURN status);

    public:
        ~CameraController();

        // �̱��� �ν��Ͻ� ȹ��
        static CameraController& GetInstance();

        // ī�޶� ����
        ErrorCode OpenCamera(int deviceIndex = 0);
        ErrorCode CloseCamera();
        bool IsOpen() const { return m_xiHandle != nullptr; }

        // ĸó ����
        ErrorCode StartCapture();
        ErrorCode StopCapture();
        bool IsCapturing() const { return m_isCapturing; }

        // ����
        ErrorCode SetExposureTime(float microseconds);
        ErrorCode SetGain(float gainDb);
        ErrorCode SetROI(const ROI& roi);
        ErrorCode SetTargetFps(uint32_t fps);
        ErrorCode SetAutoExposure(bool enable);
        ErrorCode SetAutoGain(bool enable);
        ErrorCode ApplySettings(const CameraSettings& settings);

        // ��ȸ
        ErrorCode GetCameraInfo(CameraInfo& info) const;
        ErrorCode GetCurrentSettings(CameraSettings& settings) const;
        ErrorCode GetExposureRange(float& minUs, float& maxUs) const;
        ErrorCode GetGainRange(float& minDb, float& maxDb) const;
        float GetCurrentFps() const;

        // �ݹ� ����
        void SetFrameCallback(IFrameCallback* callback);

        // ���� ���
        void GetPerformanceStats(uint64_t& totalFrames, uint64_t& droppedFrames, uint64_t& errorCount, float& avgFps) const;
        void ResetPerformanceStats();

        // ī�޶� ������ (�����)
        ErrorCode CaptureSnapshot(uint8_t* buffer, size_t bufferSize, FrameInfo& info, uint32_t timeoutMs = 1000);

        // ī�޶� ����
        static ErrorCode EnumerateCameras(std::vector<CameraInfo>& cameras);
    };

} // namespace XimeaSensor