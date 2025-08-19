#pragma once

#ifdef XIMEASENSOR_EXPORTS
#define XIMEASENSOR_API __declspec(dllexport)
#else
#define XIMEASENSOR_API __declspec(dllimport)
#endif

#include <string>
#include <functional>

// ============================================================================
// CAMERA::IMAGEFORMAT DEFINITION FOR FRAMEINFO
// ============================================================================

namespace Camera {
    // Image format enum (replaces XI_IMG_FORMAT)
    enum class ImageFormat {
        MONO8 = 0,
        MONO16 = 1,
        RGB24 = 2,
        RGB32 = 3,
        RAW8 = 4,
        RAW16 = 5,
        FRM_TRANSPORT_DATA = 6
    };
}

// FrameInfo - platform independent
struct FrameInfo {
    unsigned char* data;          // 프레임 데이터
    int width;                    // 프레임 너비
    int height;                   // 프레임 높이
    unsigned long frameNumber;    // 프레임 번호
    unsigned long acqFrameNumber; // 획득 프레임 번호
    double timestamp;             // 타임스탬프
    float currentFPS;             // 현재 FPS (계산값)
    unsigned int blackLevel;      // 블랙 레벨
    unsigned int GPI_level;       // GPI 레벨
    float exposureTime_ms;        // 노출 시간 (ms)
    float gain_db;                // 게인 (dB)
    Camera::ImageFormat format;   // 이미지 포맷
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

// ============================================================================
// CAMERA CALLBACK IMPLEMENTATION EXAMPLE (from CameraCallback.h)
// ============================================================================

class CameraCallback : public IXIMEACallback {
private:
    std::function<void(const FrameInfo&)> onFrameReceived;
    std::function<void(CameraState, CameraState)> onStateChanged;
    std::function<void(CameraError, const std::string&)> onError;
    std::function<void(const std::string&, const std::string&)> onPropertyChanged;

public:
    void SetFrameCallback(std::function<void(const FrameInfo&)> callback) {
        onFrameReceived = callback;
    }

    void SetStateCallback(std::function<void(CameraState, CameraState)> callback) {
        onStateChanged = callback;
    }

    void SetErrorCallback(std::function<void(CameraError, const std::string&)> callback) {
        onError = callback;
    }

    void SetPropertyCallback(std::function<void(const std::string&, const std::string&)> callback) {
        onPropertyChanged = callback;
    }

    virtual void OnFrameReceived(const FrameInfo& frameInfo) override {
        if (onFrameReceived) {
            onFrameReceived(frameInfo);
        }
    }

    virtual void OnCameraStateChanged(CameraState newState, CameraState oldState) override {
        if (onStateChanged) {
            onStateChanged(newState, oldState);
        }
    }

    virtual void OnError(CameraError error, const std::string& errorMessage) override {
        if (onError) {
            onError(error, errorMessage);
        }
    }

    virtual void OnPropertyChanged(const std::string& propertyName, const std::string& value) override {
        if (onPropertyChanged) {
            onPropertyChanged(propertyName, value);
        }
    }
};

// ============================================================================
// BALL STATE TRACKING ENUMS AND STRUCTURES: 2025-07-30 Added
// ============================================================================

// Ball state enumeration
enum class XIMEASENSOR_API BallState {
    NOT_DETECTED = 0,   // No ball detected
    MOVING = 1,         // Ball is moving
    STABILIZING = 2,    // Ball is stabilizing (slowing down)
    READY = 3,          // Ball has been stable for required duration
    STOPPED = 4         // Ball is stopped (after READY state)
};

// Ball state tracking configuration
struct XIMEASENSOR_API BallStateConfig {
    float positionTolerance;          // 정지 판단 픽셀 허용치 (default: 1.5)
    float movementThreshold;          // 움직임 시작 픽셀 임계값 (default: 3.0)
    int stableTimeMs;                // READY 전환 시간 (default: 4000ms)
    int stabilizingTimeMs;           // STABILIZING 최소 시간 (default: 2000ms)
    int minConsecutiveDetections;    // 최소 연속 감지 (default: 5)
    bool enableStateCallback;        // 콜백 활성화 (default: true)
    int maxMissedDetections;         // READY 상태 최대 실패 (default: 2)
    int missedDetectionTimeoutMs;    // 실패 복구 타임아웃 (default: 500ms)

    // 추가 파라미터
    int minMovingFrames;             // MOVING 최소 유지 프레임 (default: 15)
    int requiredStableFrames;        // 정지 판단 연속 프레임 (default: 15)
    float noiseFloor;                // 기본 노이즈 레벨 (default: 0.5px)
    float microMovementThreshold;    // 미세 움직임 임계값 (default: 2.0px)

    BallStateConfig()
        : positionTolerance(1.5f)      // 더 엄격하게
        , movementThreshold(3.0f)      // 더 민감하게
        , stableTimeMs(4000)
        , stabilizingTimeMs(2000)      // 증가
        , minConsecutiveDetections(5)
        , enableStateCallback(true)
        , maxMissedDetections(2)
        , missedDetectionTimeoutMs(500)
        , minMovingFrames(15)          // 0.25초 @ 60fps
        , requiredStableFrames(15)     // 0.25초 @ 60fps
        , noiseFloor(0.5f)
        , microMovementThreshold(2.0f) {
    }
};

// Ball state information
struct XIMEASENSOR_API BallStateInfo {
    BallState currentState;          // Current ball state
    BallState previousState;         // Previous ball state
    float lastPositionX;            // Last detected X position
    float lastPositionY;            // Last detected Y position
    int stableDurationMs;           // Duration in stable position (ms)
    int consecutiveDetections;      // Number of consecutive detections
    bool isTracking;                // Currently tracking a ball
    double lastStateChangeTime;     // Timestamp of last state change
};

// Dynamic ROI Configuration
struct XIMEASENSOR_API DynamicROIConfig {
    bool enabled;                    // Enable dynamic ROI (default: false)
    float roiSizeMultiplier;        // ROI size as multiplier of ball radius (default: 4.0)
    float minROISize;               // Minimum ROI size in pixels (default: 100)
    float maxROISize;               // Maximum ROI size in pixels (default: 400)
    bool showROIOverlay;            // Show ROI boundary in UI (default: true)

    DynamicROIConfig()
        : enabled(false)
        , roiSizeMultiplier(10.0f)    // 기본값 10.0
        , minROISize(100.0f)
        , maxROISize(600.0f)
        , showROIOverlay(true) {
    }
};

// Dynamic ROI Information
struct XIMEASENSOR_API DynamicROIInfo {
    bool active;                     // ROI is currently active
    int centerX;                     // ROI center X
    int centerY;                     // ROI center Y
    int width;                       // ROI width
    int height;                      // ROI height
    float processingTimeReduction;   // Processing time reduction percentage

    DynamicROIInfo()
        : active(false)
        , centerX(0)
        , centerY(0)
        , width(0)
        , height(0)
        , processingTimeReduction(0.0f) {
    }
};

// Callback for ball state changes
typedef void(*BallStateChangeCallback)(BallState newState, BallState oldState, const BallStateInfo* info, void* userContext);

// --------------------------------------------------------

// Enable/Disable continuous capture functionality
#ifdef ENABLE_CONTINUOUS_CAPTURE

// Configuration structure for continuous capture operations
struct XIMEASENSOR_API ContinuousCaptureConfig {
    double durationSeconds;           // Capture duration in seconds
    int imageFormat;                  // Image format: 0=PNG, 1=JPG
    int jpgQuality;                   // JPEG compression quality (1-100)
    bool createMetadata;              // Create metadata file after capture
    bool useAsyncSave;                // Use asynchronous image saving
    //std::string baseFolder;         // Base folder for captured images

    // Ball detection specific settings
    bool enableBallDetection;         // Enable ball detection during capture
    bool saveOriginalImages;          // Save original captured images
    bool saveDetectionImages;         // Save images with detection overlays
    bool saveBallDetectorDebugImages; // Save intermediate processing images
    //std::string debugImagePath;     // Custom path for debug images

private:
    char m_baseFolder[260];           // for std::string baseFolder;
    char m_debugImagePath[260];       // for std::string debugImagePath;

public:
    // Default constructor with standard values
    ContinuousCaptureConfig()
        : durationSeconds(1.0)
        , imageFormat(0)
        , jpgQuality(90)
        , createMetadata(true)
        , useAsyncSave(true)
        , enableBallDetection(true)
        , saveOriginalImages(true)
        , saveDetectionImages(true)
        , saveBallDetectorDebugImages(true)
    {
        strcpy_s(m_baseFolder, ".");
        strcpy_s(m_debugImagePath, "");
    }

    // Getter/Setter for string members
    const char* getBaseFolder() const { return m_baseFolder; }
    void setBaseFolder(const char* folder) {
        if (folder) {
            strcpy_s(m_baseFolder, folder);
        }
    }

    const char* getDebugImagePath() const { return m_debugImagePath; }
    void setDebugImagePath(const char* path) {
        if (path) {
            strcpy_s(m_debugImagePath, path);
        }
    }

    std::string baseFolderStr() const { return std::string(m_baseFolder); }
    std::string debugImagePathStr() const { return std::string(m_debugImagePath); }

    // Parameterized constructor for custom configuration
    ContinuousCaptureConfig(double duration, int format, int quality, bool metadata,
        bool asyncSave, const char* folder,
        bool ballDetection, bool saveOriginal,
        bool saveDetection, bool saveDebug,
        const char* debugPath = "")
        : durationSeconds(duration)
        , imageFormat(format)
        , jpgQuality(quality)
        , createMetadata(metadata)
        , useAsyncSave(asyncSave)
        , enableBallDetection(ballDetection)
        , saveOriginalImages(saveOriginal)
        , saveDetectionImages(saveDetection)
        , saveBallDetectorDebugImages(saveDebug)
    {
        setBaseFolder(folder);
        setDebugImagePath(debugPath);
    }

    // Factory method to create default configuration
    static ContinuousCaptureConfig CreateDefault() {
        return ContinuousCaptureConfig(1.0, 0, 90, true, true, ".", true, true, true, true, "");
    }
};
#endif // ENABLE_CONTINUOUS_CAPTURE


// ============================================================================
// CAMERA DEFAULT SETTINGS
// ============================================================================

// Namespace containing default camera parameters and limits
namespace CameraDefaults {
    // Default operational values
    XIMEASENSOR_API extern const int EXPOSURE_US;        // Default exposure time in microseconds
    XIMEASENSOR_API extern const float GAIN_DB;          // Default gain in decibels
    XIMEASENSOR_API extern const float FRAMERATE_FPS;    // Default frame rate in FPS

    // Hardware limits
    XIMEASENSOR_API extern const int MIN_EXPOSURE_US;    // Minimum exposure time
    XIMEASENSOR_API extern const int MAX_EXPOSURE_US;    // Maximum exposure time
    XIMEASENSOR_API extern const float MIN_GAIN_DB;      // Minimum gain value
    XIMEASENSOR_API extern const float MAX_GAIN_DB;      // Maximum gain value
    XIMEASENSOR_API extern const float MIN_FPS;          // Minimum frame rate
    XIMEASENSOR_API extern const float MAX_FPS;          // Maximum frame rate (PYTHON1300 limit)

    // Snapshot defaults
    XIMEASENSOR_API extern const int SNAPSHOT_FORMAT;    // Default snapshot format (0=PNG)
    XIMEASENSOR_API extern const int SNAPSHOT_QUALITY;   // Default JPEG quality (90%)
}


// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Configuration for single snapshot operations
struct SnapshotDefaults {
    int format;     // Image format: 0=PNG, 1=JPG
    int quality;    // JPEG quality: 1-100
};

// Information about a single detected ball
struct XIMEASENSOR_API RealtimeBallInfo {
    float centerX;      // Ball center X coordinate in pixels
    float centerY;      // Ball center Y coordinate in pixels
    float radius;       // Ball radius in pixels
    float confidence;   // Detection confidence (0.0-1.0)
    int frameIndex;     // Frame number where ball was detected
};


// Real-time ball detection result containing multiple detections
struct XIMEASENSOR_API RealtimeDetectionResult {
    bool ballFound;                 // True if at least one ball was found
    int ballCount;                  // Number of balls detected (max 5)
    RealtimeBallInfo balls[5];      // Array of detected balls
    double detectionTimeMs;         // Processing time in milliseconds
};


// Callback function type for real-time detection notifications
typedef void(*RealtimeDetectionCallback)(const RealtimeDetectionResult* result, void* userContext);


// Shot completed notification structure
struct XIMEASENSOR_API ShotCompletedInfo {
    float startX, startY;           // Starting position
    float endX, endY;               // Ending position
    float totalDistance;            // Total distance traveled
    float avgVelocity;              // Average velocity
    float maxVelocity;              // Maximum velocity
    double shotDuration;            // Duration in seconds
    int trajectoryPointCount;       // Number of trajectory points
    bool dataAvailable;             // Whether full trajectory data is available
};

// Shot completed callback
typedef void(*ShotCompletedCallback)(const ShotCompletedInfo* info, void* userContext);


// ============================================================================
// C API FUNCTIONS
// ============================================================================

extern "C" {
    // ------------------------------------------------------------------------
    // System Initialization
    // ------------------------------------------------------------------------

    // Initialize the camera system and logging
    XIMEASENSOR_API bool Camera_Initialize(const char* logPath = nullptr, int logLevel = 1);

    // Shutdown camera system and clean up resources
    XIMEASENSOR_API void Camera_Shutdown();


    // ------------------------------------------------------------------------
    // Device Management
    // ------------------------------------------------------------------------

    // Get number of connected XIMEA cameras
    XIMEASENSOR_API int Camera_GetDeviceCount();

    // Get information about a specific camera device
    XIMEASENSOR_API bool Camera_GetDeviceInfo(int index, char* name, int nameSize, char* serial, int serialSize);


    // ------------------------------------------------------------------------
    // Camera Control
    // ------------------------------------------------------------------------

    // Open camera device
    XIMEASENSOR_API bool Camera_Open(int deviceIndex);

    // Close the currently open camera
    XIMEASENSOR_API void Camera_Close();

    // Start image acquisition
    XIMEASENSOR_API bool Camera_Start();

    // Stop image acquisition
    XIMEASENSOR_API void Camera_Stop();

    // Pause/resume image acquisition
    XIMEASENSOR_API bool Camera_Pause(bool pause);


    // ------------------------------------------------------------------------
    // Image Acquisition
    // ------------------------------------------------------------------------

    // Get the latest captured frame
    XIMEASENSOR_API bool Camera_GetFrame(unsigned char* buffer, int bufferSize, int* width, int* height);


    // ------------------------------------------------------------------------
    // Camera Parameters
    // ------------------------------------------------------------------------

    // Set exposure time
    XIMEASENSOR_API bool Camera_SetExposure(int microsec);

    // Set camera gain
    XIMEASENSOR_API bool Camera_SetGain(float gain);

    // Set region of interest (ROI)
    XIMEASENSOR_API bool Camera_SetROI(int offsetX, int offsetY, int width, int height);

    // Set frame rate
    XIMEASENSOR_API bool Camera_SetFrameRate(float fps);

    // Enable/disable trigger mode
    XIMEASENSOR_API bool Camera_SetTriggerMode(bool enabled);

    // Get current exposure time
    XIMEASENSOR_API int Camera_GetExposure();

    // Get current gain value
    XIMEASENSOR_API float Camera_GetGain();

    // Get current ROI settings
    XIMEASENSOR_API bool Camera_GetROI(int* offsetX, int* offsetY, int* width, int* height);

    // Get current frame rate
    XIMEASENSOR_API float Camera_GetFrameRate();

    // Get current camera state
    XIMEASENSOR_API int Camera_GetState();


    // ------------------------------------------------------------------------
    // Statistics and Monitoring
    // ------------------------------------------------------------------------

    // Get capture statistics
    XIMEASENSOR_API bool Camera_GetStatistics(unsigned long* totalFrames, unsigned long* droppedFrames, double* averageFPS, double* minFPS,
        double* maxFPS);

    // Reset capture statistics
    XIMEASENSOR_API void Camera_ResetStatistics();


    // ------------------------------------------------------------------------
    // Callback Management
    // ------------------------------------------------------------------------

    // Register a callback for camera events
    XIMEASENSOR_API bool Camera_RegisterCallback(IXIMEACallback* callback);

    // Unregister a callback
    XIMEASENSOR_API bool Camera_UnregisterCallback(IXIMEACallback* callback);

    // Clear all registered callbacks
    XIMEASENSOR_API void Camera_ClearCallbacks();


    // ------------------------------------------------------------------------
    // Logging Control
    // ------------------------------------------------------------------------

    // Set logging level
    XIMEASENSOR_API void Camera_SetLogLevel(int level);

    // Flush log buffer to file
    XIMEASENSOR_API void Camera_FlushLog();


    // ------------------------------------------------------------------------
    // Image Saving
    // ------------------------------------------------------------------------

    // Save a snapshot from current frame
    XIMEASENSOR_API bool Camera_SaveSnapshot(const char* filename, int format = 0, int quality = 90);

    // Save provided frame data to file
    XIMEASENSOR_API bool Camera_SaveCurrentFrame(unsigned char* buffer, int bufferSize, int* width, int* height, const char* filename, int format = 0, int quality = 90);


#ifdef ENABLE_CONTINUOUS_CAPTURE
    // ------------------------------------------------------------------------
    // Continuous Capture Functions (Conditional Compilation)
    // ------------------------------------------------------------------------

    // Configure continuous capture parameters
    XIMEASENSOR_API bool Camera_SetContinuousCaptureConfig(const ContinuousCaptureConfig* config);

    // Get current continuous capture configuration
    XIMEASENSOR_API bool Camera_GetContinuousCaptureConfig(ContinuousCaptureConfig* config);

    // Start continuous capture session
    XIMEASENSOR_API bool Camera_StartContinuousCapture();

    // Stop ongoing continuous capture
    XIMEASENSOR_API void Camera_StopContinuousCapture();

    // Check if continuous capture is active
    XIMEASENSOR_API bool Camera_IsContinuousCapturing();

    // Get continuous capture state
    XIMEASENSOR_API int Camera_GetContinuousCaptureState();

    // Get results from last continuous capture session
    XIMEASENSOR_API bool Camera_GetContinuousCaptureResult(int* totalFrames, int* savedFrames, int* droppedFrames, double* duration, char* folderPath, int pathSize);

    // Set callback for continuous capture progress updates
    XIMEASENSOR_API void Camera_SetContinuousCaptureProgressCallback(void(*callback)(int currentFrame, double elapsedSeconds, int state));

    // Get default continuous capture configuration
    XIMEASENSOR_API void Camera_GetContinuousCaptureDefaults(ContinuousCaptureConfig* config);

    // Set default continuous capture configuration
    XIMEASENSOR_API void Camera_SetContinuousCaptureDefaults(const ContinuousCaptureConfig* config);

    // Start continuous capture with default settings
    XIMEASENSOR_API bool Camera_StartContinuousCaptureWithDefaults();

    // Get ball detection results from continuous capture
    XIMEASENSOR_API bool Camera_GetContinuousCaptureDetectionResult(int* framesWithBalls, int* totalBallsDetected, float* averageConfidence, char* detectionFolder,
        int folderSize);
#else
    // Stub functions when continuous capture is disabled
    inline bool Camera_SetContinuousCaptureConfig(const void*) { return false; }
    inline bool Camera_GetContinuousCaptureConfig(void*) { return false; }
    inline bool Camera_StartContinuousCapture() { return false; }
    inline void Camera_StopContinuousCapture() {}
    inline bool Camera_IsContinuousCapturing() { return false; }
    inline int Camera_GetContinuousCaptureState() { return 0; }
    inline bool Camera_GetContinuousCaptureResult(int*, int*, int*, double*, char*, int) { return false; }
    inline void Camera_SetContinuousCaptureProgressCallback(void(*)(int, double, int)) {}
    inline void Camera_GetContinuousCaptureDefaults(void*) {}
    inline void Camera_SetContinuousCaptureDefaults(const void*) {}
    inline bool Camera_StartContinuousCaptureWithDefaults() { return false; }
    inline bool Camera_GetContinuousCaptureDetectionResult(int*, int*, float*, char*, int) { return false; }
#endif

    // ------------------------------------------------------------------------
    // Default Settings Management
    // ------------------------------------------------------------------------

    // Get default camera settings
    XIMEASENSOR_API void Camera_GetDefaultSettings(int* exposureUs, float* gainDb, float* fps);

    // Get default snapshot settings
    XIMEASENSOR_API void Camera_GetSnapshotDefaults(SnapshotDefaults* defaults);

    // Set default snapshot settings
    XIMEASENSOR_API void Camera_SetSnapshotDefaults(const SnapshotDefaults* defaults);

    // Save snapshot using default settings
    XIMEASENSOR_API bool Camera_SaveSnapshotWithDefaults(const char* filename);

    // ------------------------------------------------------------------------
    // Debug Functions
    // ------------------------------------------------------------------------

    // Enable/disable ball detector debug image saving
    XIMEASENSOR_API bool Camera_SetBallDetectorDebugImages(bool enable);

    // Check if debug image saving is enabled
    XIMEASENSOR_API bool Camera_GetBallDetectorDebugImages();


    // ------------------------------------------------------------------------
    // Real-time Ball Detection
    // ------------------------------------------------------------------------

    // Enable/disable real-time ball detection
    XIMEASENSOR_API bool Camera_EnableRealtimeDetection(bool enable);

    // Check if real-time detection is enabled
    XIMEASENSOR_API bool Camera_IsRealtimeDetectionEnabled();

    // Set callback for real-time detection results
    XIMEASENSOR_API void Camera_SetRealtimeDetectionCallback(RealtimeDetectionCallback callback, void* userContext);

    // Get the most recent detection result
    XIMEASENSOR_API bool Camera_GetLastDetectionResult(RealtimeDetectionResult* result);

    // Set ROI scale for real-time detection
    XIMEASENSOR_API bool Camera_SetRealtimeDetectionROI(float roiScale);

    // Set downscale factor for real-time detection
    XIMEASENSOR_API bool Camera_SetRealtimeDetectionDownscale(int factor);

    // Set maximum candidates for real-time detection
    XIMEASENSOR_API bool Camera_SetRealtimeDetectionMaxCandidates(int maxCandidates);

    // Get real-time detection statistics
    XIMEASENSOR_API void Camera_GetRealtimeDetectionStats(int* processedFrames, double* avgProcessingTimeMs, double* detectionFPS);


    // ------------------------------------------------------------------------
    // Ball State Tracking Functions: 2025-07-30
    // ------------------------------------------------------------------------

    // Enable/disable ball state tracking
    XIMEASENSOR_API bool Camera_EnableBallStateTracking(bool enable);

    // Check if ball state tracking is enabled
    XIMEASENSOR_API bool Camera_IsBallStateTrackingEnabled();

    // Get current ball state
    XIMEASENSOR_API BallState Camera_GetBallState();

    // Get detailed ball state information
    XIMEASENSOR_API bool Camera_GetBallStateInfo(BallStateInfo* info);

    // Set ball state tracking configuration
    XIMEASENSOR_API bool Camera_SetBallStateConfig(const BallStateConfig* config);

    // Get current ball state tracking configuration
    XIMEASENSOR_API bool Camera_GetBallStateConfig(BallStateConfig* config);

    // Set callback for ball state changes
    XIMEASENSOR_API void Camera_SetBallStateChangeCallback(BallStateChangeCallback callback, void* userContext);

    // Reset ball state tracking
    XIMEASENSOR_API void Camera_ResetBallStateTracking();

    // Get ball state as string
    XIMEASENSOR_API const char* Camera_GetBallStateString(BallState state);

    // Get time in current state (milliseconds)
    XIMEASENSOR_API int Camera_GetTimeInCurrentState();

    // Check if ball is in stable state (READY or STOPPED)
    XIMEASENSOR_API bool Camera_IsBallStable();

    // ------------------------------------------------------------------------
    // Dynamic ROI Functions
    // ------------------------------------------------------------------------

    // Enable/disable dynamic ROI for READY state
    XIMEASENSOR_API bool Camera_EnableDynamicROI(bool enable);

    // Check if dynamic ROI is enabled
    XIMEASENSOR_API bool Camera_IsDynamicROIEnabled();

    // Set dynamic ROI configuration
    XIMEASENSOR_API bool Camera_SetDynamicROIConfig(const DynamicROIConfig* config);

    // Get dynamic ROI configuration
    XIMEASENSOR_API bool Camera_GetDynamicROIConfig(DynamicROIConfig* config);

    // Get current dynamic ROI information
    XIMEASENSOR_API bool Camera_GetDynamicROIInfo(DynamicROIInfo* info);

    // Reset dynamic ROI to full frame
    XIMEASENSOR_API void Camera_ResetDynamicROI();

    // Set callback for shot completion notifications
    XIMEASENSOR_API void Camera_SetShotCompletedCallback(ShotCompletedCallback callback, void* userContext);

    // Get last shot trajectory data
    XIMEASENSOR_API bool Camera_GetLastShotTrajectory(ShotCompletedInfo* info);

    // Save trajectory data to file
    XIMEASENSOR_API bool Camera_SaveTrajectoryToFile(const char* filename);

    // Clear shot trajectory data
    XIMEASENSOR_API void Camera_ClearShotTrajectory();

    // Show shot completed dialog (blocking)
    XIMEASENSOR_API void Camera_ShowShotCompletedDialog();

}