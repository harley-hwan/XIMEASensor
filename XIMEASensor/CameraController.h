#pragma once
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <queue>
#include <condition_variable>
#include <array>
#include <deque>
#include "CameraInterface.h"
#include "Logger.h"
#include "BallDetector.h"
#ifdef ENABLE_CONTINUOUS_CAPTURE
#include "ContinuousCaptureManager.h"
#endif

#define PYTHON1300_WIDTH    1280
#define PYTHON1300_HEIGHT   1024
#define PYTHON1300_MAX_FPS  210

struct CameraStatistics {
    int totalFrames;
    int droppedFrames;
    float averageFPS;
    float minFPS;
    float maxFPS;
    std::chrono::steady_clock::time_point startTime;

    void Reset() {
        totalFrames = 0;
        droppedFrames = 0;
        averageFPS = 0.0f;
        minFPS = 9999.0f;
        maxFPS = 0.0f;
        startTime = std::chrono::steady_clock::now();
    }
};

// Ball trajectory point structure
struct TrajectoryPoint {
    cv::Point2f position;
    float radius;
    float confidence;
    double timestamp;
    int frameNumber;
    BallState state;

    TrajectoryPoint() : radius(0.0f), confidence(0.0f), timestamp(0.0), frameNumber(0), state(BallState::NOT_DETECTED) {}

    TrajectoryPoint(const cv::Point2f& pos, float r, float conf, double t, int frame, BallState s)
        : position(pos), radius(r), confidence(conf), timestamp(t), frameNumber(frame), state(s) {
    }
};

// Shot trajectory data structure
struct ShotTrajectoryData {
    std::vector<TrajectoryPoint> fullTrajectory;     // All points from READY to STOPPED
    std::vector<TrajectoryPoint> movingTrajectory;   // Only MOVING state points
    cv::Point2f startPosition;                        // Position when READY -> MOVING
    cv::Point2f endPosition;                          // Position when STOPPED
    double totalDistance;                             // Total distance traveled
    double maxVelocity;                               // Maximum velocity during shot
    double averageVelocity;                           // Average velocity
    double shotDuration;                              // Time from MOVING to STOPPED
    std::chrono::steady_clock::time_point shotStartTime;
    std::chrono::steady_clock::time_point shotEndTime;

    void Clear() {
        fullTrajectory.clear();
        movingTrajectory.clear();
        startPosition = cv::Point2f(0, 0);
        endPosition = cv::Point2f(0, 0);
        totalDistance = 0.0;
        maxVelocity = 0.0;
        averageVelocity = 0.0;
        shotDuration = 0.0;
    }

    void CalculateMetrics() {
        if (movingTrajectory.size() < 2) return;

        totalDistance = 0.0;
        maxVelocity = 0.0;

        for (size_t i = 1; i < movingTrajectory.size(); ++i) {
            float dist = static_cast<float>(cv::norm(movingTrajectory[i].position - movingTrajectory[i - 1].position));
            double timeDiff = movingTrajectory[i].timestamp - movingTrajectory[i - 1].timestamp;

            totalDistance += dist;

            if (timeDiff > 0) {
                double velocity = dist / timeDiff;
                maxVelocity = std::max(maxVelocity, velocity);
            }
        }

        shotDuration = std::chrono::duration<double>(shotEndTime - shotStartTime).count();
        if (shotDuration > 0) {
            averageVelocity = totalDistance / shotDuration;
        }
    }
};

// Internal shot completed callback for CameraController
typedef void(*InternalShotCompletedCallback)(const ShotTrajectoryData* trajectoryData, void* userContext);

class CameraController {
private:
    // singleton
    static std::unique_ptr<CameraController> instance;
    static std::mutex instanceMutex;

    // Camera interface and handle
    std::unique_ptr<Camera::ICameraInterface> cameraInterface;
    void* cameraHandle;

    std::thread captureThread;
    std::atomic<bool> isRunning;
    std::atomic<bool> isPaused;

    // ���� ���۸��� ���� ����ü
    struct ImageBuffer {
        std::unique_ptr<unsigned char[]> data;
        int width = 0;
        int height = 0;
        std::atomic<bool> ready{ false };
    };

    std::array<ImageBuffer, 2> buffers;
    std::atomic<int> writeIndex{ 0 };
    std::atomic<int> readIndex{ 1 };
    std::mutex bufferSwapMutex;

    // ���� ���۴� ȣȯ���� ���� ����
    std::mutex frameMutex;
    unsigned char* frameBuffer;
    unsigned char* workingBuffer;

    // camera param
    int width;
    int height;
    int currentExposure;
    float currentGain;
    float currentFrameRate;

    // callbacks
    std::mutex callbackMutex;
    std::vector<IXIMEACallback*> callbacks;

    std::atomic<CameraState> currentState;
    CameraStatistics stats;
    std::chrono::steady_clock::time_point lastFrameTime;

#ifdef ENABLE_CONTINUOUS_CAPTURE
    std::unique_ptr<ContinuousCaptureManager> m_continuousCapture;
#endif

    std::atomic<int> deviceNotReadyCount;
    static const int MAX_DEVICE_NOT_READY_ERRORS = 5;

    // Real-time ball detection
    std::atomic<bool> m_realtimeDetectionEnabled;
    std::unique_ptr<BallDetector> m_realtimeBallDetector;
    RealtimeDetectionCallback m_realtimeCallback;
    void* m_realtimeCallbackContext;

    // Detection queue item with ROI info
    struct DetectionQueueItem {
        std::vector<unsigned char> frameData;
        FrameInfo frameInfo;
        bool isROI;                // ROI ����
        cv::Rect roiRect;          // ROI ���� ����

        DetectionQueueItem() : isROI(false), roiRect(0, 0, 0, 0) {}
    };

    std::mutex m_detectionQueueMutex;
    std::queue<DetectionQueueItem> m_detectionQueue;  // Ÿ�� ����
    std::thread m_detectionThread;
    std::atomic<bool> m_detectionThreadRunning;
    std::condition_variable m_detectionCV;

    // Real-time detection statistics
    std::atomic<int> m_realtimeProcessedFrames;
    double m_realtimeTotalProcessingTime;
    std::chrono::steady_clock::time_point m_realtimeStartTime;
    RealtimeDetectionResult m_lastDetectionResult;
    std::mutex m_realtimeStatsMutex;
    std::mutex m_lastResultMutex;

    // Ball state tracking - ENHANCED
    struct BallTrackingData {
        BallState currentState;
        BallState previousState;
        float lastPositionX;
        float lastPositionY;
        std::chrono::steady_clock::time_point lastDetectionTime;
        std::chrono::steady_clock::time_point stableStartTime;
        std::chrono::steady_clock::time_point lastStateChangeTime;
        std::chrono::steady_clock::time_point movingStartTime;
        int consecutiveDetections;
        bool isTracking;
        int missedDetectionCount;
        std::chrono::steady_clock::time_point lastMissedTime;

        // Trajectory recording
        std::deque<TrajectoryPoint> trajectoryBuffer;
        bool recordingTrajectory;
        cv::Point2f readyPosition;

        // ============ ������ ������ ���� ���� ============

        // ��ġ �����丮 (�ֱ� N������)
        static constexpr size_t POSITION_HISTORY_SIZE = 30;  // 0.5�� @ 60fps
        std::deque<cv::Point2f> positionHistory;

        // ������ �� ��ȭ�� ����
        std::deque<float> frameDeltas;  // ������ �� �ȼ� �Ÿ�
        static constexpr size_t DELTA_HISTORY_SIZE = 20;

        // ���� �̵� ����
        float recentAccumulatedMovement;    // �ֱ� N������ ������ ���� �̵�
        float totalAccumulatedMovement;     // READY ���� �� ���� �̵�
        cv::Point2f movementStartPosition;  // ������ ���� ��ġ

        // ������ �ϰ��� ����
        int consecutiveMovingFrames;        // ���� ������ ������ ��
        int consecutiveStableFrames;        // ���� ���� ������ ��
        int microMovementCount;              // �̼� ������ ī����

        // ������ ���� �м�
        float averageFrameDelta;            // ��� ������ �� �̵� �Ÿ�
        float maxRecentDelta;               // �ֱ� �ִ� �̵� �Ÿ�
        float movementNoiseLevel;           // ���� ������ ����

        // ���� ��ȯ ī����
        int movingStateFrameCount;          // MOVING ���� ���� ������ ��
        int requiredStableFramesForStop;    // ���� �Ǵܿ� �ʿ��� ���� ���� ������

        // ���� �Ӱ谪
        float currentMovementThreshold;     // ���� ������ �Ӱ谪
        float currentStabilityThreshold;    // ���� ������ �Ӱ谪

        BallTrackingData()
            : currentState(BallState::NOT_DETECTED)
            , previousState(BallState::NOT_DETECTED)
            , lastPositionX(0.0f)
            , lastPositionY(0.0f)
            , consecutiveDetections(0)
            , isTracking(false)
            , missedDetectionCount(0)
            , recordingTrajectory(false)
            , recentAccumulatedMovement(0.0f)
            , totalAccumulatedMovement(0.0f)
            , consecutiveMovingFrames(0)
            , consecutiveStableFrames(0)
            , microMovementCount(0)
            , averageFrameDelta(0.0f)
            , maxRecentDelta(0.0f)
            , movementNoiseLevel(1.0f)
            , movingStateFrameCount(0)
            , requiredStableFramesForStop(15)  // 0.25�� @ 60fps
            , currentMovementThreshold(3.0f)    // �⺻ 3�ȼ�
            , currentStabilityThreshold(1.5f) { // �⺻ 1.5�ȼ�
        }

        // ��ġ �����丮 ������Ʈ
        void UpdatePositionHistory(const cv::Point2f& newPos) {
            positionHistory.push_back(newPos);
            if (positionHistory.size() > POSITION_HISTORY_SIZE) {
                positionHistory.pop_front();
            }
        }

        // ������ ��Ÿ ������Ʈ �� ��� ���
        void UpdateFrameDeltas(float delta) {
            frameDeltas.push_back(delta);
            if (frameDeltas.size() > DELTA_HISTORY_SIZE) {
                frameDeltas.pop_front();
            }

            // ��� ���
            if (frameDeltas.size() >= 5) {
                float sum = 0.0f;
                maxRecentDelta = 0.0f;

                for (float d : frameDeltas) {
                    sum += d;
                    maxRecentDelta = std::max(maxRecentDelta, d);
                }

                averageFrameDelta = sum / frameDeltas.size();

                // ������ ���� ���� (ǥ������ ���)
                float variance = 0.0f;
                for (float d : frameDeltas) {
                    float diff = d - averageFrameDelta;
                    variance += diff * diff;
                }
                variance /= frameDeltas.size();
                movementNoiseLevel = std::sqrt(variance) + 0.5f;  // �⺻ 0.5 �ȼ� ������
            }
        }

        // �ֱ� ���� ������ ���
        void CalculateRecentAccumulatedMovement() {
            if (positionHistory.size() < 2) {
                recentAccumulatedMovement = 0.0f;
                return;
            }

            // �ֱ� 10�������� ���� �̵� ���
            size_t windowSize = std::min(size_t(10), positionHistory.size() - 1);
            float accumulated = 0.0f;

            for (size_t i = positionHistory.size() - windowSize - 1;
                i < positionHistory.size() - 1; ++i) {
                float dist = static_cast<float>(cv::norm(positionHistory[i + 1] - positionHistory[i]));
                accumulated += dist;
            }

            recentAccumulatedMovement = accumulated;
        }

        // ������ ���� �м�
        bool HasConsistentMovement() const {
            if (frameDeltas.size() < 5) return false;

            // �ֱ� 5������ �� 3������ �̻��� ������ ���� �̻� ������
            int movingFrames = 0;
            size_t startIdx = frameDeltas.size() >= 5 ? frameDeltas.size() - 5 : 0;

            for (size_t i = startIdx; i < frameDeltas.size(); ++i) {
                if (frameDeltas[i] > movementNoiseLevel) {
                    movingFrames++;
                }
            }

            return movingFrames >= 3;
        }

        // ���� ���� �Ǵ�
        bool IsDefinitelyStable() const {
            if (frameDeltas.empty()) return false;

            // �ֱ� �����ӵ��� ��� ������ �Ӱ谪 ����
            size_t checkFrames = std::min(size_t(10), frameDeltas.size());
            for (size_t i = frameDeltas.size() - checkFrames; i < frameDeltas.size(); ++i) {
                if (frameDeltas[i] > currentStabilityThreshold) {
                    return false;
                }
            }

            return true;
        }

        // ���� �Ӱ谪 ������Ʈ
        void UpdateDynamicThresholds() {
            // ������ ������ ���� �Ӱ谪 ����
            currentMovementThreshold = std::max(2.0f, movementNoiseLevel * 2.0f);
            currentStabilityThreshold = std::max(1.0f, movementNoiseLevel * 1.5f);

            // MOVING ���¿����� �� �ΰ��ϰ�
            if (currentState == BallState::MOVING) {
                currentStabilityThreshold *= 0.8f;
            }
        }

        // ���� ������ ����
        bool DetectSlowMovement() const {
            // ���� �������� �Ӱ谪 �ʰ�
            if (recentAccumulatedMovement > currentMovementThreshold * 5) {
                return true;
            }

            // �ϰ��� �̼� ������
            if (microMovementCount >= 5 && averageFrameDelta > currentMovementThreshold * 0.5f) {
                return true;
            }

            return false;
        }

        void Reset() {
            positionHistory.clear();
            frameDeltas.clear();
            recentAccumulatedMovement = 0.0f;
            totalAccumulatedMovement = 0.0f;
            consecutiveMovingFrames = 0;
            consecutiveStableFrames = 0;
            microMovementCount = 0;
            averageFrameDelta = 0.0f;
            maxRecentDelta = 0.0f;
            movementNoiseLevel = 1.0f;
            movingStateFrameCount = 0;
        }
    };

    // Helper methods
    void ProcessMovementData(BallTrackingData& tracking, float currentX, float currentY, float pixelDelta);
    void HandleBallNotDetected(std::chrono::steady_clock::time_point now);
    void StartNewTracking(const cv::Point2f& currentPos, std::chrono::steady_clock::time_point now);
    void HandleNotDetectedState(float pixelDelta, const cv::Point2f& currentPos, std::chrono::steady_clock::time_point now);
    void HandleReadyState(float pixelDelta, const cv::Point2f& currentPos, std::chrono::steady_clock::time_point now);
    void HandleMovingState(float pixelDelta, std::chrono::steady_clock::time_point now);
    void HandleStabilizingState(float pixelDelta, std::chrono::steady_clock::time_point now);
    void HandleStoppedState(const cv::Point2f& currentPos, std::chrono::steady_clock::time_point now);
    bool ShouldStartMoving(const BallTrackingData& tracking, float pixelDelta);
    bool ShouldStopMoving(const BallTrackingData& tracking);

    // Ball state tracking members
    std::atomic<bool> m_ballStateTrackingEnabled;
    BallTrackingData m_ballTracking;
    BallStateConfig m_ballStateConfig;
    BallStateChangeCallback m_ballStateCallback;
    void* m_ballStateCallbackContext;
    mutable std::mutex m_ballStateMutex;

    // Shot trajectory data
    ShotTrajectoryData m_currentShotData;
    InternalShotCompletedCallback m_shotCompletedCallback;
    void* m_shotCompletedCallbackContext;
    mutable std::mutex m_shotDataMutex;
    static const size_t MAX_TRAJECTORY_BUFFER_SIZE = 1000;

    // Dynamic ROI members - ENHANCED
    std::atomic<bool> m_dynamicROIEnabled;
    DynamicROIConfig m_dynamicROIConfig;
    DynamicROIInfo m_dynamicROIInfo;
    mutable std::mutex m_dynamicROIMutex;
    std::chrono::steady_clock::time_point m_lastROIUpdateTime;
    cv::Rect m_currentROI;
    bool m_usingDynamicROI;

    // ROI state based on ball state
    enum class ROIMode {
        FULL_FRAME,      // No ROI, full frame processing
        READY_ROI,       // Small ROI for READY state
        MOVING_ROI,      // Larger ROI for MOVING state
        TRACKING_ROI     // Dynamic tracking ROI
    };
    ROIMode m_currentROIMode;

    CameraController();

    void CaptureLoop();

    void NotifyFrameReceived(const FrameInfo& info);
    void NotifyStateChanged(CameraState newState);
    void NotifyError(CameraError error, const std::string& message);
    void NotifyPropertyChanged(const std::string& property, const std::string& value);

    void UpdateStatistics(bool frameReceived);
    std::string GetCameraErrorString(Camera::ReturnCode error);

    void RealtimeDetectionWorker();
    void ProcessRealtimeDetection(const unsigned char* data, int width, int height, int frameIndex);
    void ProcessRealtimeDetectionWithROI(const unsigned char* data, int width, int height,
        int frameIndex, bool isROI, const cv::Rect& roiRect);

    // Ball state tracking methods - ENHANCED
    void UpdateBallState(const RealtimeDetectionResult* result);
    float CalculateDistance(float x1, float y1, float x2, float y2);
    void NotifyBallStateChanged(BallState newState, BallState oldState, const BallStateInfo& info);
    void ResetBallTracking();
    void RecordTrajectoryPoint(float x, float y, float radius, float confidence, int frameNumber, BallState state);
    void StartTrajectoryRecording();
    void StopTrajectoryRecording();
    void ProcessShotCompleted();
    void SaveTrajectoryDataAutomatic();
    void NotifyShotCompleted();

    // Dynamic ROI methods - ENHANCED
    void UpdateDynamicROI(const RealtimeDetectionResult* result);
    cv::Rect CalculateDynamicROI(float centerX, float centerY, float radius, BallState state);
    void ApplyDynamicROI(const cv::Rect& roi);
    void ClearDynamicROI();
    void SetROIMode(ROIMode mode);
    bool ShouldUpdateROI(const cv::Rect& newROI);

    // Convert between Camera::ReturnCode and CameraError
    CameraError ConvertReturnCodeToError(Camera::ReturnCode code);

public:
    ~CameraController();

    // singleton
    static CameraController& GetInstance();
    static void Destroy();

    // Set camera type (must be called before OpenCamera)
    void SetCameraType(Camera::CameraFactory::CameraType type);

    // camera control
    bool OpenCamera(int deviceIndex);
    void CloseCamera();
    bool StartCapture();
    void StopCapture();
    void PauseCapture(bool pause);

    bool GetFrame(unsigned char* buffer, int bufferSize, int& outWidth, int& outHeight);

    // camera settings
    bool SetExposure(int microsec);
    bool SetGain(float gain);
    bool SetROI(int offsetX, int offsetY, int width, int height);
    bool SetFrameRate(float fps);
    bool SetTriggerMode(bool enabled);

    // getters
    int GetExposure() const { return currentExposure; }
    float GetGain() const { return currentGain; }
    int GetWidth() const { return width; }
    int GetHeight() const { return height; }
    float GetFrameRate();
    CameraState GetState() const { return currentState.load(); }
    CameraStatistics GetStatistics() const { return stats; }
    void ResetStatistics() { stats.Reset(); }

#ifdef ENABE_CONTINUOUS_CAPTURE
    ContinuousCaptureManager* GetContinuousCaptureManager() {
        return m_continuousCapture.get();
    }
#else
    void* GetContinuousCaptureManager() { return nullptr; }
#endif

    // callback
    void RegisterCallback(IXIMEACallback* callback);
    void UnregisterCallback(IXIMEACallback* callback);
    void ClearCallbacks();

    int GetConnectedDeviceCount();
    bool GetDeviceInfo(int index, std::string& name, std::string& serial);

    // Real-time ball detection
    bool EnableRealtimeDetection(bool enable);
    bool IsRealtimeDetectionEnabled() const { return m_realtimeDetectionEnabled.load(); }
    void SetRealtimeDetectionCallback(RealtimeDetectionCallback callback, void* context);
    bool GetLastDetectionResult(RealtimeDetectionResult* result);

    bool SetRealtimeDetectionROI(float roiScale);
    bool SetRealtimeDetectionDownscale(int factor);
    bool SetRealtimeDetectionMaxCandidates(int maxCandidates);
    void GetRealtimeDetectionStats(int* processedFrames, double* avgProcessingTimeMs, double* detectionFPS);

    // Ball state tracking
    bool EnableBallStateTracking(bool enable);
    bool IsBallStateTrackingEnabled() const { return m_ballStateTrackingEnabled.load(); }
    BallState GetBallState() const;
    bool GetBallStateInfo(BallStateInfo* info) const;
    void GetBallStateInfoInternal(BallStateInfo* info) const;
    bool SetBallStateConfig(const BallStateConfig& config);
    BallStateConfig GetBallStateConfig() const;
    void SetBallStateChangeCallback(BallStateChangeCallback callback, void* context);
    void ResetBallStateTracking();
    int GetTimeInCurrentState() const;
    bool IsBallStable() const;

    // Shot trajectory methods - Use Internal callback type
    void SetShotCompletedCallback(InternalShotCompletedCallback callback, void* context);
    bool GetLastShotTrajectory(ShotTrajectoryData* data) const;
    void ClearShotTrajectory();
    bool SaveTrajectoryData(const std::string& filename) const;

    bool SaveTrajectoryDataCSV(const std::string& filename) const;

    // Dynamic ROI
    bool EnableDynamicROI(bool enable);
    bool IsDynamicROIEnabled() const { return m_dynamicROIEnabled.load(); }
    bool SetDynamicROIConfig(const DynamicROIConfig& config);
    DynamicROIConfig GetDynamicROIConfig() const;
    bool GetDynamicROIInfo(DynamicROIInfo* info) const;
    void ResetDynamicROI();
    cv::Rect GetCurrentROI() const {
        std::lock_guard<std::mutex> lock(m_dynamicROIMutex);
        return m_currentROI;
    }
};