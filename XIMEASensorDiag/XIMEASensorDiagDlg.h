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
using namespace Gdiplus;
#pragma comment(lib, "gdiplus.lib")

// 메시지 정의
#define WM_UPDATE_FRAME     (WM_USER + 100)
#define WM_UPDATE_STATUS    (WM_USER + 101)
#define WM_UPDATE_ERROR     (WM_USER + 102)
#define WM_UPDATE_FPS       (WM_USER + 103)
#define WM_CONTINUOUS_CAPTURE_COMPLETE (WM_USER + 104)
#define WM_UPDATE_BALL_DETECTION (WM_USER + 105)
#define WM_UPDATE_BALL_STATE (WM_USER + 106)
#define WM_UPDATE_DYNAMIC_ROI (WM_USER + 107)
#define WM_UPDATE_SHOT_COMPLETED (WM_USER + 108)

class CXIMEASensorDiagDlg : public CDialogEx
{
public:
    CXIMEASensorDiagDlg(CWnd* pParent = nullptr);
    virtual ~CXIMEASensorDiagDlg();

    enum { IDD = IDD_XIMEASENSORDIAG_DIALOG };

protected:
    virtual void DoDataExchange(CDataExchange* pDX);
    virtual BOOL OnInitDialog();
    afx_msg void OnPaint();
    afx_msg HCURSOR OnQueryDragIcon();
    DECLARE_MESSAGE_MAP()

private:
    // UI Controls
    CStatic* m_pictureCtrl = nullptr;
    CButton* m_btnStart = nullptr;
    CButton* m_btnStop = nullptr;
    CButton* m_btnSnapshot = nullptr;
    CButton* m_checkContinuous = nullptr;
    CButton* m_checkRealtimeDetection = nullptr;
    CStatic* m_staticStatus = nullptr;
    CStatic* m_staticFPS = nullptr;
    CStatic* m_staticBallPosition = nullptr;
    CStatic* m_staticBallInfo = nullptr;
    CStatic* m_staticDetectionFPS = nullptr;

    // 볼 상태 표시 컨트롤들
    CStatic* m_staticBallState = nullptr;
    CStatic* m_staticStateTime = nullptr;
    CStatic* m_staticStableTime = nullptr;
    CButton* m_btnResetTracking = nullptr;
    CButton* m_btnConfigureTracking = nullptr;

    // Dynamic ROI 컨트롤들
    CButton* m_checkEnableDynamicROI = nullptr;
    CButton* m_checkShowROIOverlay = nullptr;
    CStatic* m_staticROIStatus = nullptr;
    CStatic* m_staticROISize = nullptr;
    CStatic* m_staticROIReduction = nullptr;
    CSliderCtrl* m_sliderROIMultiplier = nullptr;
    CEdit* m_editROIMultiplier = nullptr;
    CButton* m_btnResetROI = nullptr;

    CSliderCtrl* m_sliderExposure = nullptr;
    CSliderCtrl* m_sliderGain = nullptr;
    CSliderCtrl* m_sliderFramerate = nullptr;
    CComboBox* m_comboDevices = nullptr;
    CEdit* m_editExposure = nullptr;
    CEdit* m_editGain = nullptr;
    CEdit* m_editFramerate = nullptr;

    // Camera 관련
    std::unique_ptr<CameraCallback> m_cameraCallback;
    std::atomic<bool> m_isStreaming;

    // Triple buffering for smooth display - 개선된 버전
    struct FrameBuffer {
        std::unique_ptr<unsigned char[]> data;
        int width;
        int height;
        std::atomic<bool> ready;

        FrameBuffer() : width(0), height(0), ready(false) {}

        void allocate(size_t size) {
            data = std::make_unique<unsigned char[]>(size);
            memset(data.get(), 0, size);
        }

        void reallocate(size_t newSize) {
            if (!data || getCurrentSize() < newSize) {
                data = std::make_unique<unsigned char[]>(newSize);
                memset(data.get(), 0, newSize);
            }
        }

        size_t getCurrentSize() const {
            return data ? (2048 * 2048) : 0;
        }
    };

    std::array<FrameBuffer, 3> m_frameBuffers;
    std::atomic<int> m_writeBufferIndex;
    std::atomic<int> m_readBufferIndex;
    std::atomic<int> m_displayBufferIndex;

    // Frame skip logic
    std::atomic<int> m_pendingFrameUpdates;
    std::chrono::steady_clock::time_point m_lastFrameDrawTime;
    static constexpr int MAX_PENDING_FRAMES = 2;
    static constexpr int MIN_FRAME_INTERVAL_MS = 4;

    // FPS 계산
    std::chrono::steady_clock::time_point m_lastFPSUpdate;
    int m_frameCount;
    double m_currentFPS;

    // 기본 설정값
    int m_defaultExposureUs;
    float m_defaultGainDb;
    float m_defaultFps;

    // 실시간 검출 관련 - CRITICAL_SECTION 대신 mutex 사용
    RealtimeDetectionResult m_lastDetectionResult;
    std::mutex m_detectionMutex;
    std::chrono::steady_clock::time_point m_lastDetectionStatsUpdate;

    // Dynamic ROI 관련
    DynamicROIInfo m_lastROIInfo;
    std::mutex m_roiMutex;
    std::chrono::steady_clock::time_point m_lastROIUpdate;

    // 볼 상태 업데이트 타이머
    static constexpr UINT_PTR TIMER_BALL_STATE_UPDATE = 1003;
    static constexpr UINT_PTR TIMER_DYNAMIC_ROI_UPDATE = 1004;

    // USB 상태 모니터링
    std::atomic<int> m_usbErrorCount;
    std::chrono::steady_clock::time_point m_lastUSBError;
    static constexpr int MAX_USB_ERRORS = 3;
    static constexpr int USB_ERROR_RESET_TIME_MS = 5000;

    // ============================================================================
    // 궤적 시각화 관련 멤버 변수
    // ============================================================================

    // 궤적 포인트 구조체
    struct TrajectoryPoint {
        cv::Point2f position;       // 공의 위치
        DWORD timestamp;            // 시간 정보
        float confidence;           // 검출 신뢰도

        TrajectoryPoint(const cv::Point2f& pos, DWORD time, float conf)
            : position(pos), timestamp(time), confidence(conf) {
        }
    };

    // 궤적 관리
    std::deque<TrajectoryPoint> m_trajectoryPoints;     // 현재 샷의 궤적 포인트들
    std::mutex m_trajectoryMutex;                       // 궤적 데이터 보호
    bool m_isRecordingTrajectory;                       // 궤적 기록 중 여부

    // 페이드 아웃 관련
    std::atomic<bool> m_showTrajectory;                 // 궤적 표시 여부
    std::atomic<int> m_trajectoryAlpha;                 // 궤적 투명도 (255 = 불투명, 0 = 투명)
    static constexpr UINT_PTR TIMER_TRAJECTORY_FADE = 1005;
    static constexpr int FADE_DURATION_MS = 3000;       // 3초간 페이드 아웃
    static constexpr int FADE_STEPS = 30;               // 페이드 단계

    // GDI+ 관련 추가: 2025-08-20
    ULONG_PTR m_gdiplusToken;
    Gdiplus::GdiplusStartupInput m_gdiplusStartupInput;

    // 궤적 스타일 설정
    struct TrajectoryStyle {
        COLORREF lineColor;         // 선 색상
        int lineWidth;              // 선 두께
        bool showPoints;            // 개별 포인트 표시 여부
        int pointSize;              // 포인트 크기

        TrajectoryStyle()
            : lineColor(RGB(0, 255, 0))      // 초록색 시작
            , lineWidth(3)
            , showPoints(true)
            , pointSize(4) {
        }
    } m_trajectoryStyle;

    // 화면 좌표 변환을 위한 캐시
    float m_lastScaleX;
    float m_lastScaleY;

    // 메모리 DC와 비트맵 (m_pictureCtrl 전용)
    CDC m_memDC;
    CBitmap m_memBitmap;
    CBitmap* m_pOldBitmap;
    bool m_bMemDCReady;

    // 궤적 오버레이 전용 (옵션)
    CDC m_overlayDC;
    CBitmap m_overlayBitmap;
    CBitmap* m_pOldOverlayBitmap;
    bool m_bOverlayNeedsUpdate;

    // 렌더링 최적화
    std::chrono::steady_clock::time_point m_lastRenderTime;
    CRect m_lastPictureRect;

    // 간소화된 메서드
    void CleanupMemoryDC();
    void DrawFrameToMemDC();


    // 정적 콜백 함수
    static void RealtimeDetectionCallback(const RealtimeDetectionResult* result, void* userContext);
    static void ContinuousCaptureProgressCallback(int currentFrame, double elapsedSeconds, int state);
    static void BallStateChangeCallback(BallState newState, BallState oldState, const BallStateInfo* info, void* userContext);
    static void ShotCompletedCallback(const ShotCompletedInfo* info, void* userContext);
    static CXIMEASensorDiagDlg* s_pThis;

    // Helper functions
    void UpdateDeviceList();
    void UpdateUI(bool isStreaming);
    void DrawFrame();
    void DrawDetectionOverlay(CDC& dc, const CRect& rect);
    void DrawDynamicROIOverlay(CDC& dc, const CRect& rect);
    void ShowError(const CString& message);
    void SyncSlidersWithCamera();
    void LoadDefaultSettings();
    void InitializeMemoryDC();
    void InitializeFrameBuffers();
    void SwapBuffers();
    bool ShouldSkipFrame();
    void HandleUSBError();
    void ResetUSBErrorCount();
    void UpdateBallStateDisplay();
    void UpdateDynamicROIDisplay();
    CString GetBallStateDisplayString(BallState state);
    COLORREF GetBallStateColor(BallState state);

    // ============================================================================
    // 퍼팅 궤적 시각화 관련 메서드
    // ============================================================================

    // 궤적 관리
    void StartTrajectoryRecording();
    void StopTrajectoryRecording();
    void ClearTrajectory();
    void AddTrajectoryPoint(float x, float y, float confidence);

    // 궤적 그리기
    void DrawTrajectory(CDC& dc, const CRect& rect);
    void DrawTrajectoryLine(CDC& dc, const std::vector<CPoint>& screenPoints);
    void DrawTrajectoryPoints(CDC& dc, const std::vector<CPoint>& screenPoints);

    void DrawTrajectoryPointsGDIPlus(Gdiplus::Graphics& graphics, const std::vector<CPoint>& screenPoints, int alpha);

    // 페이드 아웃
    void StartTrajectoryFadeOut();
    void StopTrajectoryFadeOut();

    // 좌표 변환
    CPoint ConvertToScreenCoordinates(const cv::Point2f& point, const CRect& displayRect);

    // Shot completed 처리
    void OnShotCompleted(const ShotCompletedInfo* info);

    // Callback handlers
    void OnFrameReceivedCallback(const FrameInfo& frameInfo);
    void OnStateChangedCallback(CameraState newState, CameraState oldState);
    void OnErrorCallback(CameraError error, const std::string& errorMessage);
    void OnPropertyChangedCallback(const std::string& propertyName, const std::string& value);
    void OnContinuousCaptureProgress(int currentFrame, double elapsedSeconds, int state);
    void OnRealtimeDetectionResult(const RealtimeDetectionResult* result);
    void OnBallStateChanged(BallState newState, BallState oldState, const BallStateInfo* info);

public:
    // Message handlers
    afx_msg void OnBnClickedButtonStart();
    afx_msg void OnBnClickedButtonStop();
    afx_msg void OnBnClickedButtonRefresh();
    afx_msg void OnBnClickedButtonSnapshot();
    afx_msg void OnBnClickedButtonSettings();
    afx_msg void OnBnClickedCheckRealtimeDetection();
    afx_msg void OnBnClickedButtonResetTracking();
    afx_msg void OnBnClickedButtonConfigureTracking();
    afx_msg void OnBnClickedCheckEnableDynamicROI();
    afx_msg void OnBnClickedCheckShowROIOverlay();
    afx_msg void OnBnClickedButtonResetROI();
    afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
    afx_msg void OnCbnSelchangeComboDevices();
    afx_msg void OnEnChangeEditROIMultiplier();

    afx_msg LRESULT OnUpdateFrame(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateStatus(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateError(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateFPS(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateBallDetection(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateBallState(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateDynamicROI(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateShotCompleted(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnContinuousCaptureComplete(WPARAM wParam, LPARAM lParam);

    afx_msg void OnDestroy();
    afx_msg void OnTimer(UINT_PTR nIDEvent);
    afx_msg void OnEnChangeEditExposure();
    afx_msg void OnEnChangeEditGain();
    afx_msg void OnEnChangeEditFramerate();
};