#pragma once

#include "XIMEASensor.h"
#include <memory>
#include <chrono>
#include <atomic>
#include <mutex>
#include <array>
#include <deque>
#include <opencv2/core/types.hpp>
#include <gdiplus.h>

// ============================================================================
// Forward Declarations
// ============================================================================
class CameraCallback;

// ============================================================================
// Constants
// ============================================================================
namespace DialogConstants {
    // Window Messages
    constexpr UINT WM_UPDATE_FRAME = WM_USER + 100;
    constexpr UINT WM_UPDATE_STATUS = WM_USER + 101;
    constexpr UINT WM_UPDATE_ERROR = WM_USER + 102;
    constexpr UINT WM_UPDATE_FPS = WM_USER + 103;
    constexpr UINT WM_CONTINUOUS_CAPTURE_COMPLETE = WM_USER + 104;
    constexpr UINT WM_UPDATE_BALL_DETECTION = WM_USER + 105;
    constexpr UINT WM_UPDATE_BALL_STATE = WM_USER + 106;
    constexpr UINT WM_UPDATE_DYNAMIC_ROI = WM_USER + 107;
    constexpr UINT WM_UPDATE_SHOT_COMPLETED = WM_USER + 108;

    // Timer IDs
    constexpr UINT_PTR TIMER_UPDATE_STATISTICS = 1001;
    constexpr UINT_PTR TIMER_FRAME_UPDATE = 1002;
    constexpr UINT_PTR TIMER_BALL_STATE_UPDATE = 1003;
    constexpr UINT_PTR TIMER_DYNAMIC_ROI_UPDATE = 1004;
    constexpr UINT_PTR TIMER_TRAJECTORY_FADE = 1005;
    constexpr UINT_PTR TIMER_CAMERA_RESTART = 3000;  // USB error recovery timer

    // Performance Settings
    constexpr int MAX_PENDING_FRAMES = 2;
    constexpr int MIN_FRAME_INTERVAL_MS = 4;
    constexpr int STATISTICS_UPDATE_INTERVAL_MS = 1000;
    constexpr int UI_UPDATE_INTERVAL_MS = 100;
    constexpr int FRAME_UPDATE_INTERVAL_MS = 33;  // ~30 FPS

    // USB Error Handling
    constexpr int MAX_USB_ERRORS = 3;
    constexpr int USB_ERROR_RESET_TIME_MS = 5000;
    constexpr int CAMERA_RESTART_DELAY_MS = 2000;

    // Trajectory Visualization
    constexpr int FADE_DURATION_MS = 3000;
    constexpr int FADE_STEPS = 30;
    constexpr int MAX_TRAJECTORY_POINTS = 1000;
    constexpr int TRAJECTORY_POINT_INTERVAL = 20;  // Show every Nth point
}

class CXIMEASensorDiagDlg : public CDialogEx
{
    // Construction
public:
    CXIMEASensorDiagDlg(CWnd* pParent = nullptr);
    virtual ~CXIMEASensorDiagDlg();

    enum { IDD = IDD_XIMEASENSORDIAG_DIALOG };

    // MFC Framework
protected:
    virtual void DoDataExchange(CDataExchange* pDX) override;
    virtual BOOL OnInitDialog() override;
    DECLARE_MESSAGE_MAP()

private:
    // Frame buffer for triple buffering
    struct FrameBuffer {
        std::unique_ptr<unsigned char[]> data;
        int width = 0;
        int height = 0;
        std::atomic<bool> ready{ false };

        void allocate(size_t size);
        void reallocate(size_t newSize);
        size_t getCurrentSize() const;
    };

    // Trajectory point for visualization
    struct TrajectoryPoint {
        cv::Point2f position;
        DWORD timestamp;
        float confidence;

        TrajectoryPoint(const cv::Point2f& pos, DWORD time, float conf)
            : position(pos), timestamp(time), confidence(conf) {
        }
    };

    // Trajectory rendering style
    struct TrajectoryStyle {
        COLORREF lineColor = RGB(0, 255, 0);
        int lineWidth = 3;
        bool showPoints = true;
        int pointSize = 4;
    };

    // ============================================================================
    // UI Control Management
    // ============================================================================
private:
    struct UIControls {
        // Main controls
        CStatic* pictureCtrl = nullptr;
        CButton* btnStart = nullptr;
        CButton* btnStop = nullptr;
        CButton* btnSnapshot = nullptr;
        CButton* btnRefresh = nullptr;
        CButton* btnSettings = nullptr;
        CComboBox* comboDevices = nullptr;

        // Status displays
        CStatic* status = nullptr;
        CStatic* fps = nullptr;
        CStatic* ballPosition = nullptr;
        CStatic* ballInfo = nullptr;
        CStatic* detectionFPS = nullptr;

        // Parameter controls
        CSliderCtrl* sliderExposure = nullptr;
        CSliderCtrl* sliderGain = nullptr;
        CSliderCtrl* sliderFramerate = nullptr;
        CEdit* editExposure = nullptr;
        CEdit* editGain = nullptr;
        CEdit* editFramerate = nullptr;

        // Ball detection features
        CButton* checkRealtimeDetection = nullptr;
        CButton* checkContinuous = nullptr;

        // Ball state tracking
        CStatic* ballState = nullptr;
        CStatic* stateTime = nullptr;
        CStatic* stableTime = nullptr;
        CButton* btnResetTracking = nullptr;
        CButton* btnConfigureTracking = nullptr;

        // Dynamic ROI features
        CButton* checkEnableDynamicROI = nullptr;
        CButton* checkShowROIOverlay = nullptr;
        CStatic* roiStatus = nullptr;
        CStatic* roiSize = nullptr;
        CStatic* roiReduction = nullptr;
        CSliderCtrl* sliderROIMultiplier = nullptr;
        CEdit* editROIMultiplier = nullptr;
        CButton* btnResetROI = nullptr;
    } m_ui;

    // ============================================================================
    // Camera State Management
    // ============================================================================
private:
    // Camera connection
    std::unique_ptr<CameraCallback> m_cameraCallback;
    std::atomic<bool> m_isStreaming{ false };

    // Default settings
    struct DefaultSettings {
        int exposureUs = 4000;
        float gainDb = 0.0f;
        float fps = 60.0f;
    } m_defaultSettings;

    // ============================================================================
    // Frame Buffer Management
    // ============================================================================
private:
    // Triple buffering
    std::array<FrameBuffer, 3> m_frameBuffers;
    std::atomic<int> m_writeBufferIndex{ 0 };
    std::atomic<int> m_readBufferIndex{ 1 };
    std::atomic<int> m_displayBufferIndex{ 2 };

    // Frame timing
    std::atomic<int> m_pendingFrameUpdates{ 0 };
    std::chrono::steady_clock::time_point m_lastFrameDrawTime;
    std::chrono::steady_clock::time_point m_lastRenderTime;

    // FPS calculation
    std::chrono::steady_clock::time_point m_lastFPSUpdate;
    int m_frameCount = 0;
    double m_currentFPS = 0.0;

    // ============================================================================
    // Ball Detection State
    // ============================================================================
private:
    // Detection results
    RealtimeDetectionResult m_lastDetectionResult{};
    std::mutex m_detectionMutex;
    std::chrono::steady_clock::time_point m_lastDetectionStatsUpdate;

    // Dynamic ROI
    DynamicROIInfo m_lastROIInfo{};
    std::mutex m_roiMutex;

    // 2025-08-21: Ball state tracking
    struct BallStateTracking {
        BallState previousState = BallState::NOT_DETECTED;
        BallState currentState = BallState::NOT_DETECTED;
        bool shotInProgress = false;
        bool waitingForStop = false;
        std::chrono::steady_clock::time_point movingStartTime;
        std::chrono::steady_clock::time_point lastStateChangeTime;
    } m_ballStateTracking;

    // ============================================================================
    // Trajectory Visualization
    // ============================================================================
private:
    // Trajectory data
    std::deque<TrajectoryPoint> m_trajectoryPoints;
    std::mutex m_trajectoryMutex;

    // Trajectory state
    std::atomic<bool> m_isRecordingTrajectory{ false };
    std::atomic<bool> m_showTrajectory{ false };
    std::atomic<int> m_trajectoryAlpha{ 255 };

    // Style settings
    TrajectoryStyle m_trajectoryStyle;

    // Coordinate transformation cache
    float m_lastScaleX = 1.0f;
    float m_lastScaleY = 1.0f;

    // ============================================================================
    // Graphics Resources
    // ============================================================================
private:
    // GDI+ initialization
    ULONG_PTR m_gdiplusToken = 0;
    Gdiplus::GdiplusStartupInput m_gdiplusStartupInput;

    // Memory DC for flicker-free drawing
    CDC m_memDC;
    CBitmap m_memBitmap;
    CBitmap* m_pOldBitmap = nullptr;
    bool m_bMemDCReady = false;
    CRect m_lastPictureRect;

    // ============================================================================
    // Error Handling
    // ============================================================================
private:
    // USB error recovery
    std::atomic<int> m_usbErrorCount{ 0 };
    std::chrono::steady_clock::time_point m_lastUSBError;

    // Camera restart state
    bool m_pendingCameraRestart = false;
    int m_lastDeviceIndex = -1;


private:
    static CXIMEASensorDiagDlg* s_pThis;

protected:
    // System messages
    afx_msg void OnPaint();
    afx_msg HCURSOR OnQueryDragIcon();
    afx_msg void OnDestroy();
    afx_msg void OnTimer(UINT_PTR nIDEvent);
    afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);

    // Control events
    afx_msg void OnBnClickedButtonStart();
    afx_msg void OnBnClickedButtonStop();
    afx_msg void OnBnClickedButtonRefresh();
    afx_msg void OnBnClickedButtonSnapshot();
    afx_msg void OnBnClickedButtonSettings();
    afx_msg void OnCbnSelchangeComboDevices();

    // Feature toggles
    afx_msg void OnBnClickedCheckRealtimeDetection();
    afx_msg void OnBnClickedButtonResetTracking();
    afx_msg void OnBnClickedButtonConfigureTracking();
    afx_msg void OnBnClickedCheckEnableDynamicROI();
    afx_msg void OnBnClickedCheckShowROIOverlay();
    afx_msg void OnBnClickedButtonResetROI();

    // Edit control changes
    afx_msg void OnEnChangeEditExposure();
    afx_msg void OnEnChangeEditGain();
    afx_msg void OnEnChangeEditFramerate();
    afx_msg void OnEnChangeEditROIMultiplier();

    // Custom messages
    afx_msg LRESULT OnUpdateFrame(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateStatus(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateError(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateFPS(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateBallDetection(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateBallState(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateDynamicROI(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateShotCompleted(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnContinuousCaptureComplete(WPARAM wParam, LPARAM lParam);

    // ============================================================================
    // Initialization
    // ============================================================================
private:
    bool InitializeControls();
    bool InitializeCamera();
    void InitializeCallbacks();
    void InitializeTimers();
    void InitializeFrameBuffers();
    void InitializeMemoryDC();
    void LoadDefaultSettings();

    // ============================================================================
    // Cleanup
    // ============================================================================
private:
    void CleanupCamera();
    void CleanupGraphics();
    void CleanupTimers();
    void CleanupMemoryDC();

    // ============================================================================
    // UI Update
    // ============================================================================
private:
    void UpdateDeviceList();
    void UpdateUI(bool isStreaming);
    void UpdateBallStateDisplay();
    void UpdateDynamicROIDisplay();
    void SyncSlidersWithCamera();

    // ============================================================================
    // Camera Control
    // ============================================================================
private:
    bool StartCamera(int deviceIndex);
    void StopCamera();
    bool ApplyCameraSettings();
    bool SetCameraParameter(int exposureUs, float gainDb, float fps);

    // ============================================================================
    // Frame Processing
    // ============================================================================
private:
    void ProcessFrame(const FrameInfo& frameInfo);
    bool ShouldSkipFrame() const;
    void SwapBuffers();
    void UpdateFPSCalculation();

    // ============================================================================
    // Rendering
    // ============================================================================
private:
    void DrawFrame();
    void DrawCameraImage(CDC& dc, const CRect& rect);
    void DrawOverlays(CDC& dc, const CRect& rect);
    void DrawDetectionOverlay(CDC& dc, const CRect& rect);
    void DrawDynamicROIOverlay(CDC& dc, const CRect& rect);
    void DrawTrajectory(CDC& dc, const CRect& rect);

    // ============================================================================
    // Trajectory
    // ============================================================================
private:
    void StartTrajectoryRecording();
    void StopTrajectoryRecording();
    void ClearTrajectory();
    void AddTrajectoryPoint(float x, float y, float confidence);
    void StartTrajectoryFadeOut();
    void UpdateTrajectoryFade();

    // Trajectory rendering helpers
    void DrawTrajectoryLine(CDC& dc, const std::vector<CPoint>& points);
    void DrawTrajectoryPoints(CDC& dc, const std::vector<CPoint>& points);
    void DrawTrajectoryWithAlpha(Gdiplus::Graphics& graphics, const std::vector<CPoint>& points, int alpha);
    CPoint ConvertToScreenCoordinates(const cv::Point2f& point, const CRect& rect);

    // ============================================================================
    // Ball State
    // ============================================================================
private:
    CString GetBallStateDisplayString(BallState state) const;
    COLORREF GetBallStateColor(BallState state) const;
    void HandleBallStateChange(BallState newState, BallState oldState);
    // 2025-08-21
    bool IsValidShotSequence(BallState newState, BallState oldState) const;
    void HandleIncompleteShotSequence();

    //123123
    void HandleShotStarted();
    void StartIncompleteShotFadeOut();

    void CheckForStuckStates();

    // ============================================================================
    // Error Handling
    // ============================================================================
private:
    void HandleUSBError();
    void ResetUSBErrorCount();
    void ShowError(const CString& message);
    bool HandleCameraError(CameraError error, const CString& message);
    void ScheduleCameraRestart();
    void ExecuteCameraRestart();

    // ============================================================================
    // Callback Handlers
    // ============================================================================
private:
    // Camera callbacks
    void OnFrameReceivedCallback(const FrameInfo& frameInfo);
    void OnStateChangedCallback(CameraState newState, CameraState oldState);
    void OnErrorCallback(CameraError error, const std::string& errorMessage);
    void OnPropertyChangedCallback(const std::string& propertyName, const std::string& value);

    // Feature callbacks
    void OnRealtimeDetectionResult(const RealtimeDetectionResult* result);
    void OnBallStateChanged(BallState newState, BallState oldState, const BallStateInfo* info);
    void OnShotCompleted(const ShotCompletedInfo* info);
    void OnContinuousCaptureProgress(int currentFrame, double elapsedSeconds, int state);

    // ============================================================================
    // Static Callback
    // ============================================================================
private:
    static void RealtimeDetectionCallback(const RealtimeDetectionResult* result, void* userContext);
    static void BallStateChangeCallback(BallState newState, BallState oldState, const BallStateInfo* info, void* userContext);
    static void ShotCompletedCallback(const ShotCompletedInfo* info, void* userContext);
    static void ContinuousCaptureProgressCallback(int currentFrame, double elapsedSeconds, int state);

    // ============================================================================
    // Helper
    // ============================================================================
private:
    // Timer update handlers
    void UpdateStatistics();

    // Slider handlers
    void HandleROIMultiplierSlider();
    void UpdateParameterEditsFromSliders(CSliderCtrl* pSlider);
    void HandleExposureSlider();
    void HandleGainSlider();
    void HandleFramerateSlider();
    void HandleFrameRateError(float requestedFPS);

    // Detection feature handlers
    void EnableRealtimeDetection();
    void DisableRealtimeDetection();
    void ResetDetectionDisplays();

    // Dynamic ROI handlers
    void EnableDynamicROI();
    void DisableDynamicROI();

    // Snapshot handlers
    void HandleSingleSnapshot();
#ifdef ENABLE_CONTINUOUS_CAPTURE
    void HandleContinuousCapture();
    void HandleContinuousCaptureComplete();
    void AppendBallDetectionResults(CString& msg);
#endif
};