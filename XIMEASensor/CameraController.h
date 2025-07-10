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
        // 싱글톤 인스턴스
        static std::unique_ptr<CameraController> s_instance;
        static std::mutex s_instanceMutex;

        // 카메라 핸들
        HANDLE m_xiHandle;

        // 카메라 정보
        CameraInfo m_cameraInfo;
        CameraSettings m_settings;

        // 스레드 관련
        std::unique_ptr<std::thread> m_captureThread;
        std::atomic<bool> m_isCapturing;
        std::atomic<bool> m_shouldStop;

        // 콜백
        IFrameCallback* m_frameCallback;
        std::mutex m_callbackMutex;

        // 프레임 버퍼 (트리플 버퍼링)
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

        // 성능 통계
        struct PerformanceStats {
            std::atomic<uint64_t> totalFrames;
            std::atomic<uint64_t> droppedFrames;
            std::atomic<uint64_t> errorCount;
            std::chrono::steady_clock::time_point startTime;
            std::mutex mutex;
        } m_stats;

        // Private 생성자 (싱글톤)
        CameraController();

        // 내부 메서드
        ErrorCode InitializeCamera();
        ErrorCode ConfigureCamera();
        ErrorCode ValidateROI(const ROI& roi);
        void CaptureThreadFunc();
        void ResetBuffers();
        std::string GetXiErrorString(XI_RETURN status);

    public:
        ~CameraController();

        // 싱글톤 인스턴스 획득
        static CameraController& GetInstance();

        // 카메라 제어
        ErrorCode OpenCamera(int deviceIndex = 0);
        ErrorCode CloseCamera();
        bool IsOpen() const { return m_xiHandle != nullptr; }

        // 캡처 제어
        ErrorCode StartCapture();
        ErrorCode StopCapture();
        bool IsCapturing() const { return m_isCapturing; }

        // 설정
        ErrorCode SetExposureTime(float microseconds);
        ErrorCode SetGain(float gainDb);
        ErrorCode SetROI(const ROI& roi);
        ErrorCode SetTargetFps(uint32_t fps);
        ErrorCode SetAutoExposure(bool enable);
        ErrorCode SetAutoGain(bool enable);
        ErrorCode ApplySettings(const CameraSettings& settings);

        // 조회
        ErrorCode GetCameraInfo(CameraInfo& info) const;
        ErrorCode GetCurrentSettings(CameraSettings& settings) const;
        ErrorCode GetExposureRange(float& minUs, float& maxUs) const;
        ErrorCode GetGainRange(float& minDb, float& maxDb) const;
        float GetCurrentFps() const;

        // 콜백 설정
        void SetFrameCallback(IFrameCallback* callback);

        // 성능 통계
        void GetPerformanceStats(uint64_t& totalFrames, uint64_t& droppedFrames, uint64_t& errorCount, float& avgFps) const;
        void ResetPerformanceStats();

        // 카메라 스냅샷 (동기식)
        ErrorCode CaptureSnapshot(uint8_t* buffer, size_t bufferSize, FrameInfo& info, uint32_t timeoutMs = 1000);

        // 카메라 열거
        static ErrorCode EnumerateCameras(std::vector<CameraInfo>& cameras);
    };

} // namespace XimeaSensor