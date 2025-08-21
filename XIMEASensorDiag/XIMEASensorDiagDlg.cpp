#include "pch.h"
#include "framework.h"
#include "XIMEASensorDiag.h"
#include "XIMEASensorDiagDlg.h"
#include "afxdialogex.h"

#include <sstream>
#include <iomanip>
#include <algorithm>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace DialogConstants;
using namespace Gdiplus;

#pragma comment(lib, "gdiplus.lib")

CXIMEASensorDiagDlg* CXIMEASensorDiagDlg::s_pThis = nullptr;

CXIMEASensorDiagDlg::CXIMEASensorDiagDlg(CWnd* pParent)
    : CDialogEx(IDD_XIMEASENSORDIAG_DIALOG, pParent)
{
    s_pThis = this;
    m_cameraCallback = std::make_unique<CameraCallback>();
}

CXIMEASensorDiagDlg::~CXIMEASensorDiagDlg()
{
    s_pThis = nullptr;
}

void CXIMEASensorDiagDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CXIMEASensorDiagDlg, CDialogEx)
    ON_WM_PAINT()
    ON_WM_QUERYDRAGICON()
    ON_WM_DESTROY()
    ON_WM_TIMER()
    ON_WM_HSCROLL()

    // Button clicks
    ON_BN_CLICKED(IDC_BUTTON_START, &CXIMEASensorDiagDlg::OnBnClickedButtonStart)
    ON_BN_CLICKED(IDC_BUTTON_STOP, &CXIMEASensorDiagDlg::OnBnClickedButtonStop)
    ON_BN_CLICKED(IDC_BUTTON_REFRESH, &CXIMEASensorDiagDlg::OnBnClickedButtonRefresh)
    ON_BN_CLICKED(IDC_BUTTON_SNAPSHOT, &CXIMEASensorDiagDlg::OnBnClickedButtonSnapshot)
    ON_BN_CLICKED(IDC_BUTTON_SETTINGS, &CXIMEASensorDiagDlg::OnBnClickedButtonSettings)

    // Feature controls
    ON_BN_CLICKED(IDC_CHECK_REALTIME_DETECTION, &CXIMEASensorDiagDlg::OnBnClickedCheckRealtimeDetection)
    ON_BN_CLICKED(IDC_BUTTON_RESET_TRACKING, &CXIMEASensorDiagDlg::OnBnClickedButtonResetTracking)
    ON_BN_CLICKED(IDC_BUTTON_CONFIGURE_TRACKING, &CXIMEASensorDiagDlg::OnBnClickedButtonConfigureTracking)
    ON_BN_CLICKED(IDC_CHECK_ENABLE_DYNAMIC_ROI, &CXIMEASensorDiagDlg::OnBnClickedCheckEnableDynamicROI)
    ON_BN_CLICKED(IDC_CHECK_SHOW_ROI_OVERLAY, &CXIMEASensorDiagDlg::OnBnClickedCheckShowROIOverlay)
    ON_BN_CLICKED(IDC_BUTTON_RESET_ROI, &CXIMEASensorDiagDlg::OnBnClickedButtonResetROI)

    // Selection changes
    ON_CBN_SELCHANGE(IDC_COMBO_DEVICES, &CXIMEASensorDiagDlg::OnCbnSelchangeComboDevices)

    // Edit changes
    ON_EN_CHANGE(IDC_EDIT_EXPOSURE, &CXIMEASensorDiagDlg::OnEnChangeEditExposure)
    ON_EN_CHANGE(IDC_EDIT_GAIN, &CXIMEASensorDiagDlg::OnEnChangeEditGain)
    ON_EN_CHANGE(IDC_EDIT_FRAMERATE, &CXIMEASensorDiagDlg::OnEnChangeEditFramerate)
    ON_EN_CHANGE(IDC_EDIT_ROI_MULTIPLIER, &CXIMEASensorDiagDlg::OnEnChangeEditROIMultiplier)

    // Custom messages
    ON_MESSAGE(WM_UPDATE_FRAME, &CXIMEASensorDiagDlg::OnUpdateFrame)
    ON_MESSAGE(WM_UPDATE_STATUS, &CXIMEASensorDiagDlg::OnUpdateStatus)
    ON_MESSAGE(WM_UPDATE_ERROR, &CXIMEASensorDiagDlg::OnUpdateError)
    ON_MESSAGE(WM_UPDATE_FPS, &CXIMEASensorDiagDlg::OnUpdateFPS)
    ON_MESSAGE(WM_CONTINUOUS_CAPTURE_COMPLETE, &CXIMEASensorDiagDlg::OnContinuousCaptureComplete)
    ON_MESSAGE(WM_UPDATE_BALL_DETECTION, &CXIMEASensorDiagDlg::OnUpdateBallDetection)
    ON_MESSAGE(WM_UPDATE_BALL_STATE, &CXIMEASensorDiagDlg::OnUpdateBallState)
    ON_MESSAGE(WM_UPDATE_DYNAMIC_ROI, &CXIMEASensorDiagDlg::OnUpdateDynamicROI)
    ON_MESSAGE(WM_UPDATE_SHOT_COMPLETED, &CXIMEASensorDiagDlg::OnUpdateShotCompleted)
END_MESSAGE_MAP()


BOOL CXIMEASensorDiagDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    // Initialize GDI+
    GdiplusStartup(&m_gdiplusToken, &m_gdiplusStartupInput, NULL);

    // Initialize console for debugging
#ifdef _DEBUG
    if (AllocConsole()) {
        FILE* fp;
        freopen_s(&fp, "CONOUT$", "w", stdout);
        freopen_s(&fp, "CONOUT$", "w", stderr);
        freopen_s(&fp, "CONIN$", "r", stdin);
    }
#endif

    // Initialize all components
    if (!InitializeControls()) {
        AfxMessageBox(_T("Failed to initialize controls!"));
        return FALSE;
    }

    if (!InitializeCamera()) {
        AfxMessageBox(_T("Failed to initialize camera system!"));
        return FALSE;
    }

    InitializeCallbacks();
    InitializeTimers();
    InitializeFrameBuffers();

    // Load settings and update UI
    LoadDefaultSettings();
    UpdateDeviceList();
    UpdateUI(false);

    return TRUE;
}

bool CXIMEASensorDiagDlg::InitializeControls()
{
    try {
        // Main controls
        m_ui.pictureCtrl = (CStatic*)GetDlgItem(IDC_STATIC_VIDEO);
        m_ui.btnStart = (CButton*)GetDlgItem(IDC_BUTTON_START);
        m_ui.btnStop = (CButton*)GetDlgItem(IDC_BUTTON_STOP);
        m_ui.btnSnapshot = (CButton*)GetDlgItem(IDC_BUTTON_SNAPSHOT);
        m_ui.btnRefresh = (CButton*)GetDlgItem(IDC_BUTTON_REFRESH);
        m_ui.btnSettings = (CButton*)GetDlgItem(IDC_BUTTON_SETTINGS);
        m_ui.comboDevices = (CComboBox*)GetDlgItem(IDC_COMBO_DEVICES);

        // Status controls
        m_ui.status = (CStatic*)GetDlgItem(IDC_STATIC_STATUS);
        m_ui.fps = (CStatic*)GetDlgItem(IDC_STATIC_FPS);
        m_ui.ballPosition = (CStatic*)GetDlgItem(IDC_STATIC_BALL_POSITION);
        m_ui.ballInfo = (CStatic*)GetDlgItem(IDC_STATIC_BALL_INFO);
        m_ui.detectionFPS = (CStatic*)GetDlgItem(IDC_STATIC_DETECTION_FPS);

        // Parameter controls
        m_ui.sliderExposure = (CSliderCtrl*)GetDlgItem(IDC_SLIDER_EXPOSURE);
        m_ui.sliderGain = (CSliderCtrl*)GetDlgItem(IDC_SLIDER_GAIN);
        m_ui.sliderFramerate = (CSliderCtrl*)GetDlgItem(IDC_SLIDER_FRAMERATE);
        m_ui.editExposure = (CEdit*)GetDlgItem(IDC_EDIT_EXPOSURE);
        m_ui.editGain = (CEdit*)GetDlgItem(IDC_EDIT_GAIN);
        m_ui.editFramerate = (CEdit*)GetDlgItem(IDC_EDIT_FRAMERATE);

        // Feature controls - Ball detection
        m_ui.checkRealtimeDetection = (CButton*)GetDlgItem(IDC_CHECK_REALTIME_DETECTION);
        m_ui.checkContinuous = (CButton*)GetDlgItem(IDC_CHECK_CONTINUOUS);

        // Feature controls - Ball state tracking
        m_ui.ballState = (CStatic*)GetDlgItem(IDC_STATIC_BALL_STATE);
        m_ui.stateTime = (CStatic*)GetDlgItem(IDC_STATIC_STATE_TIME);
        m_ui.stableTime = (CStatic*)GetDlgItem(IDC_STATIC_STABLE_TIME);
        m_ui.btnResetTracking = (CButton*)GetDlgItem(IDC_BUTTON_RESET_TRACKING);
        m_ui.btnConfigureTracking = (CButton*)GetDlgItem(IDC_BUTTON_CONFIGURE_TRACKING);

        // Feature controls - Dynamic ROI
        m_ui.checkEnableDynamicROI = (CButton*)GetDlgItem(IDC_CHECK_ENABLE_DYNAMIC_ROI);
        m_ui.checkShowROIOverlay = (CButton*)GetDlgItem(IDC_CHECK_SHOW_ROI_OVERLAY);
        m_ui.roiStatus = (CStatic*)GetDlgItem(IDC_STATIC_ROI_STATUS);
        m_ui.roiSize = (CStatic*)GetDlgItem(IDC_STATIC_ROI_SIZE);
        m_ui.roiReduction = (CStatic*)GetDlgItem(IDC_STATIC_ROI_REDUCTION);
        m_ui.sliderROIMultiplier = (CSliderCtrl*)GetDlgItem(IDC_SLIDER_ROI_MULTIPLIER);
        m_ui.editROIMultiplier = (CEdit*)GetDlgItem(IDC_EDIT_ROI_MULTIPLIER);
        m_ui.btnResetROI = (CButton*)GetDlgItem(IDC_BUTTON_RESET_ROI);

        // Initialize control states
        if (m_ui.checkRealtimeDetection) {
            m_ui.checkRealtimeDetection->SetCheck(BST_UNCHECKED);
        }

        if (m_ui.checkContinuous) {
            m_ui.checkContinuous->SetCheck(BST_UNCHECKED);
        }

        if (m_ui.checkEnableDynamicROI) {
            m_ui.checkEnableDynamicROI->SetCheck(BST_UNCHECKED);
        }

        if (m_ui.checkShowROIOverlay) {
            m_ui.checkShowROIOverlay->SetCheck(BST_CHECKED);
        }

        // Initialize sliders
        if (m_ui.sliderExposure) {
            m_ui.sliderExposure->SetRange(CameraDefaults::MIN_EXPOSURE_US,
                CameraDefaults::MAX_EXPOSURE_US);
            m_ui.sliderExposure->SetTicFreq(10000);
        }

        if (m_ui.sliderGain) {
            m_ui.sliderGain->SetRange(static_cast<int>(CameraDefaults::MIN_GAIN_DB * 10),
                static_cast<int>(CameraDefaults::MAX_GAIN_DB * 10));
            m_ui.sliderGain->SetTicFreq(30);
        }

        if (m_ui.sliderFramerate) {
            m_ui.sliderFramerate->SetRange(static_cast<int>(CameraDefaults::MIN_FPS * 10),
                static_cast<int>(CameraDefaults::MAX_FPS * 10));
            m_ui.sliderFramerate->SetTicFreq(100);
        }

        if (m_ui.sliderROIMultiplier) {
            m_ui.sliderROIMultiplier->SetRange(60, 160); // 6.0 ~ 16.0
            m_ui.sliderROIMultiplier->SetPos(100); // 10.0 default
            m_ui.sliderROIMultiplier->SetTicFreq(5);
        }

        // Set initial text
        if (m_ui.ballState) {
            m_ui.ballState->SetWindowText(_T("NOT DETECTED"));
        }

        if (m_ui.editROIMultiplier) {
            m_ui.editROIMultiplier->SetWindowText(_T("10.0"));
        }

        return true;
    }
    catch (...) {
        return false;
    }
}

bool CXIMEASensorDiagDlg::InitializeCamera()
{
    return Camera_Initialize("./logs/XIMEASensor.log", 1);
}

void CXIMEASensorDiagDlg::InitializeCallbacks()
{
    // Frame callback
    m_cameraCallback->SetFrameCallback(
        [this](const FrameInfo& info) {
            OnFrameReceivedCallback(info);
        });

    // State callback
    m_cameraCallback->SetStateCallback(
        [this](CameraState newState, CameraState oldState) {
            OnStateChangedCallback(newState, oldState);
        });

    // Error callback
    m_cameraCallback->SetErrorCallback(
        [this](CameraError error, const std::string& msg) {
            OnErrorCallback(error, msg);
        });

    // Property callback
    m_cameraCallback->SetPropertyCallback(
        [this](const std::string& prop, const std::string& value) {
            OnPropertyChangedCallback(prop, value);
        });

    // Register camera callback
    Camera_RegisterCallback(m_cameraCallback.get());

    // Register shot completed callback
    Camera_SetShotCompletedCallback(ShotCompletedCallback, this);
}

void CXIMEASensorDiagDlg::InitializeTimers()
{
    SetTimer(TIMER_UPDATE_STATISTICS, STATISTICS_UPDATE_INTERVAL_MS, nullptr);
    SetTimer(TIMER_FRAME_UPDATE, FRAME_UPDATE_INTERVAL_MS, nullptr);
}

// ============================================================================
// Cleanup
// ============================================================================
void CXIMEASensorDiagDlg::OnDestroy()
{
    CDialogEx::OnDestroy();

    // Clear any remaining trajectory
    ClearTrajectory();

    // Reset ball state tracking
    m_ballStateTracking = BallStateTracking();

    // Continue with normal cleanup...
    CleanupCamera();
    CleanupTimers();
    CleanupGraphics();
}

void CXIMEASensorDiagDlg::CleanupCamera()
{
    // Unregister callbacks
    Camera_SetShotCompletedCallback(nullptr, nullptr);
    Camera_SetBallStateChangeCallback(nullptr, nullptr);
    Camera_SetRealtimeDetectionCallback(nullptr, nullptr);
    Camera_SetContinuousCaptureProgressCallback(nullptr);

    // Disable features
    if (Camera_IsBallStateTrackingEnabled()) {
        Camera_EnableBallStateTracking(false);
    }

    if (Camera_IsRealtimeDetectionEnabled()) {
        Camera_EnableRealtimeDetection(false);
    }

    // Stop and close camera
    if (m_isStreaming) {
        Camera_Stop();
    }
    Camera_Close();

    // Unregister main callback
    Camera_UnregisterCallback(m_cameraCallback.get());

    // Shutdown camera system
    Camera_Shutdown();
}

void CXIMEASensorDiagDlg::CleanupTimers()
{
    KillTimer(TIMER_UPDATE_STATISTICS);
    KillTimer(TIMER_FRAME_UPDATE);
    KillTimer(TIMER_BALL_STATE_UPDATE);
    KillTimer(TIMER_DYNAMIC_ROI_UPDATE);
    KillTimer(TIMER_TRAJECTORY_FADE);
}

void CXIMEASensorDiagDlg::CleanupGraphics()
{
    CleanupMemoryDC();
    GdiplusShutdown(m_gdiplusToken);
}

// ============================================================================
// Frame Buffer Management
// ============================================================================
void CXIMEASensorDiagDlg::FrameBuffer::allocate(size_t size)
{
    data = std::make_unique<unsigned char[]>(size);
    if (data) {
        memset(data.get(), 0, size);
    }
}

void CXIMEASensorDiagDlg::FrameBuffer::reallocate(size_t newSize)
{
    if (!data || getCurrentSize() < newSize) {
        allocate(newSize);
    }
}

size_t CXIMEASensorDiagDlg::FrameBuffer::getCurrentSize() const
{
    return data ? (2048 * 2048) : 0;  // Fixed max size
}

void CXIMEASensorDiagDlg::InitializeFrameBuffers()
{
    size_t maxBufferSize = 2048 * 2048;
    for (auto& buffer : m_frameBuffers) {
        buffer.allocate(maxBufferSize);
        buffer.ready = false;
    }
}

// ============================================================================
// UI Updates
// ============================================================================
void CXIMEASensorDiagDlg::UpdateUI(bool isStreaming)
{
    m_ui.btnStart->EnableWindow(!isStreaming);
    m_ui.btnStop->EnableWindow(isStreaming);
    m_ui.comboDevices->EnableWindow(!isStreaming);

    m_ui.sliderExposure->EnableWindow(isStreaming);
    m_ui.sliderGain->EnableWindow(isStreaming);
    m_ui.sliderFramerate->EnableWindow(isStreaming);
}

void CXIMEASensorDiagDlg::UpdateDeviceList()
{
    if (!m_ui.comboDevices) return;

    m_ui.comboDevices->ResetContent();

    int deviceCount = Camera_GetDeviceCount();
    if (deviceCount == 0) {
        m_ui.comboDevices->AddString(_T("No devices found"));
        m_ui.comboDevices->EnableWindow(FALSE);
        return;
    }

    for (int i = 0; i < deviceCount; i++) {
        char name[256] = { 0 };
        char serial[256] = { 0 };

        if (Camera_GetDeviceInfo(i, name, sizeof(name), serial, sizeof(serial))) {
            CString deviceStr;
            deviceStr.Format(_T("%d: %s (S/N: %s)"), i,
                CString(name), CString(serial));
            m_ui.comboDevices->AddString(deviceStr);
        }
    }

    m_ui.comboDevices->SetCurSel(0);
    m_ui.comboDevices->EnableWindow(TRUE);
}

// ============================================================================
// Camera Operations
// ============================================================================
void CXIMEASensorDiagDlg::OnBnClickedButtonStart()
{
    int deviceIndex = m_ui.comboDevices->GetCurSel();
    if (deviceIndex < 0) {
        AfxMessageBox(_T("Please select a device!"));
        return;
    }

    if (!StartCamera(deviceIndex)) {
        AfxMessageBox(_T("Failed to start camera!"));
    }
}

bool CXIMEASensorDiagDlg::StartCamera(int deviceIndex)
{
    // Open camera
    if (!Camera_Open(deviceIndex)) {
        return false;
    }

    // Apply default settings
    if (!ApplyCameraSettings()) {
        Camera_Close();
        return false;
    }

    // Sync UI with camera
    SyncSlidersWithCamera();

    // Start streaming
    if (!Camera_Start()) {
        Camera_Close();
        return false;
    }

    // Update state
    m_isStreaming = true;
    UpdateUI(true);

    // Reset counters
    m_frameCount = 0;
    m_lastFPSUpdate = std::chrono::steady_clock::now();
    m_lastFrameDrawTime = std::chrono::steady_clock::now();
    ResetUSBErrorCount();

    return true;
}

void CXIMEASensorDiagDlg::OnBnClickedButtonStop()
{
    StopCamera();
}

void CXIMEASensorDiagDlg::StopCamera()
{
    // Disable features
    if (Camera_IsRealtimeDetectionEnabled()) {
        Camera_EnableRealtimeDetection(false);
        Camera_SetRealtimeDetectionCallback(nullptr, nullptr);

        if (m_ui.checkRealtimeDetection) {
            m_ui.checkRealtimeDetection->SetCheck(BST_UNCHECKED);
        }
    }

    if (Camera_IsContinuousCapturing()) {
        Camera_StopContinuousCapture();

        if (m_ui.btnSnapshot) {
            m_ui.btnSnapshot->EnableWindow(TRUE);
        }
        if (m_ui.checkContinuous) {
            m_ui.checkContinuous->EnableWindow(TRUE);
        }
    }

    // Stop camera
    Camera_Stop();
    Camera_Close();

    // Update state
    m_isStreaming = false;
    UpdateUI(false);

    // Refresh display
    Invalidate();
}

// ============================================================================
// Camera Parameter Management
// ============================================================================
bool CXIMEASensorDiagDlg::ApplyCameraSettings()
{
    return SetCameraParameter(m_defaultSettings.exposureUs,
        m_defaultSettings.gainDb,
        m_defaultSettings.fps);
}

bool CXIMEASensorDiagDlg::SetCameraParameter(int exposureUs, float gainDb, float fps)
{
    bool success = true;

    success &= Camera_SetExposure(exposureUs);
    success &= Camera_SetGain(gainDb);
    success &= Camera_SetFrameRate(fps);

    return success;
}

void CXIMEASensorDiagDlg::LoadDefaultSettings()
{
    Camera_GetDefaultSettings(&m_defaultSettings.exposureUs,
        &m_defaultSettings.gainDb,
        &m_defaultSettings.fps);
}

void CXIMEASensorDiagDlg::SyncSlidersWithCamera()
{
    int currentExposure = Camera_GetExposure();
    float currentGain = Camera_GetGain();
    float currentFramerate = Camera_GetFrameRate();

    // Update exposure
    if (m_ui.sliderExposure && currentExposure > 0) {
        m_ui.sliderExposure->SetPos(currentExposure);
        if (m_ui.editExposure) {
            CString strExposure;
            strExposure.Format(_T("%d"), currentExposure);
            m_ui.editExposure->SetWindowText(strExposure);
        }
    }

    // Update gain
    if (m_ui.sliderGain) {
        m_ui.sliderGain->SetPos(static_cast<int>(currentGain * 10));
        if (m_ui.editGain) {
            CString strGain;
            strGain.Format(_T("%.1f"), currentGain);
            m_ui.editGain->SetWindowText(strGain);
        }
    }

    // Update framerate
    if (m_ui.sliderFramerate && currentFramerate > 0) {
        m_ui.sliderFramerate->SetPos(static_cast<int>(currentFramerate * 10));
        if (m_ui.editFramerate) {
            CString strFPS;
            strFPS.Format(_T("%.1f"), currentFramerate);
            m_ui.editFramerate->SetWindowText(strFPS);
        }
    }
}

// ============================================================================
// Frame Processing
// ============================================================================
void CXIMEASensorDiagDlg::OnFrameReceivedCallback(const FrameInfo& frameInfo)
{
    ProcessFrame(frameInfo);
    UpdateFPSCalculation();
}

void CXIMEASensorDiagDlg::ProcessFrame(const FrameInfo& frameInfo)
{
    // Skip frame if necessary
    if (ShouldSkipFrame()) {
        return;
    }

    // Get write buffer
    int writeIdx = m_writeBufferIndex.load();
    FrameBuffer& writeBuffer = m_frameBuffers[writeIdx];

    // Only update if buffer is available
    if (!writeBuffer.ready.exchange(false)) {
        int requiredSize = frameInfo.width * frameInfo.height;

        // Reallocate if needed
        writeBuffer.reallocate(requiredSize);

        // Copy frame data
        if (frameInfo.data && requiredSize > 0) {
            memcpy(writeBuffer.data.get(), frameInfo.data, requiredSize);
            writeBuffer.width = frameInfo.width;
            writeBuffer.height = frameInfo.height;
        }

        writeBuffer.ready = true;

        // Swap buffers
        SwapBuffers();

        // Post update message
        if (m_pendingFrameUpdates.fetch_add(1) < MAX_PENDING_FRAMES) {
            PostMessage(WM_UPDATE_FRAME);
        }
        else {
            m_pendingFrameUpdates.fetch_sub(1);
        }
    }
}

bool CXIMEASensorDiagDlg::ShouldSkipFrame() const
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_lastFrameDrawTime).count();

    return (m_pendingFrameUpdates > MAX_PENDING_FRAMES ||
        elapsed < MIN_FRAME_INTERVAL_MS);
}

void CXIMEASensorDiagDlg::SwapBuffers()
{
    int oldWrite = m_writeBufferIndex.load();
    int oldRead = m_readBufferIndex.load();
    int oldDisplay = m_displayBufferIndex.load();

    m_displayBufferIndex = oldRead;
    m_readBufferIndex = oldWrite;
    m_writeBufferIndex = oldDisplay;
}

void CXIMEASensorDiagDlg::UpdateFPSCalculation()
{
    m_frameCount++;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_lastFPSUpdate).count();

    if (elapsed >= 500) {  // Update every 500ms
        m_currentFPS = (m_frameCount * 1000.0) / elapsed;
        m_frameCount = 0;
        m_lastFPSUpdate = now;
        PostMessage(WM_UPDATE_FPS, 0, static_cast<LPARAM>(m_currentFPS * 10));
    }
}

// ============================================================================
// Rendering
// ============================================================================
void CXIMEASensorDiagDlg::OnPaint()
{
    if (IsIconic()) {
        CPaintDC dc(this);
        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);
    }
    else {
        CDialogEx::OnPaint();

        if (m_isStreaming) {
            DrawFrame();
        }
    }
}

void CXIMEASensorDiagDlg::DrawFrame()
{
    if (!m_ui.pictureCtrl || !m_ui.pictureCtrl->GetSafeHwnd()) {
        return;
    }

    // Initialize memory DC if needed
    if (!m_bMemDCReady) {
        InitializeMemoryDC();
        if (!m_bMemDCReady) return;
    }

    CRect rect;
    m_ui.pictureCtrl->GetClientRect(&rect);

    // Recreate memory DC if size changed
    if (rect != m_lastPictureRect) {
        CleanupMemoryDC();
        InitializeMemoryDC();
    }

    // Check frame rate limiting
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_lastRenderTime).count();

    if (elapsed < 16) {  // 60 FPS limit
        return;
    }
    m_lastRenderTime = now;

    // Draw to memory DC
    DrawCameraImage(m_memDC, rect);
    DrawOverlays(m_memDC, rect);

    // Copy to screen
    CClientDC dc(m_ui.pictureCtrl);
    dc.BitBlt(0, 0, rect.Width(), rect.Height(), &m_memDC, 0, 0, SRCCOPY);

    m_lastFrameDrawTime = std::chrono::steady_clock::now();
}

void CXIMEASensorDiagDlg::DrawCameraImage(CDC& dc, const CRect& rect)
{
    // Get display buffer
    int displayIdx = m_displayBufferIndex.load();
    FrameBuffer& displayBuffer = m_frameBuffers[displayIdx];

    if (!displayBuffer.ready || !displayBuffer.data ||
        displayBuffer.width <= 0 || displayBuffer.height <= 0) {
        return;
    }

    // Create grayscale bitmap info
    constexpr int PALETTE_SIZE = 256;
    const size_t bmpInfoSize = sizeof(BITMAPINFOHEADER) + PALETTE_SIZE * sizeof(RGBQUAD);

    std::unique_ptr<uint8_t[]> bmpInfoBuffer;
    try {
        bmpInfoBuffer.reset(new uint8_t[bmpInfoSize]);
    }
    catch (const std::bad_alloc&) {
        TRACE(_T("Failed to allocate memory for BITMAPINFO\n"));
        return;
    }

    BITMAPINFO* pBmpInfo = reinterpret_cast<BITMAPINFO*>(bmpInfoBuffer.get());
    memset(pBmpInfo, 0, bmpInfoSize);

    // Setup bitmap header
    BITMAPINFOHEADER& header = pBmpInfo->bmiHeader;
    header.biSize = sizeof(BITMAPINFOHEADER);
    header.biWidth = displayBuffer.width;
    header.biHeight = -displayBuffer.height;  // Top-down
    header.biPlanes = 1;
    header.biBitCount = 8;
    header.biCompression = BI_RGB;
    header.biSizeImage = displayBuffer.width * displayBuffer.height;
    header.biClrUsed = PALETTE_SIZE;
    header.biClrImportant = PALETTE_SIZE;

    // Setup grayscale palette
    RGBQUAD* pPalette = pBmpInfo->bmiColors;
    for (int i = 0; i < PALETTE_SIZE; i++) {
        pPalette[i].rgbBlue = static_cast<BYTE>(i);
        pPalette[i].rgbGreen = static_cast<BYTE>(i);
        pPalette[i].rgbRed = static_cast<BYTE>(i);
        pPalette[i].rgbReserved = 0;
    }

    // Draw image
    int oldStretchMode = SetStretchBltMode(dc.GetSafeHdc(), HALFTONE);
    SetBrushOrgEx(dc.GetSafeHdc(), 0, 0, NULL);

    StretchDIBits(
        dc.GetSafeHdc(),
        0, 0, rect.Width(), rect.Height(),
        0, 0, displayBuffer.width, displayBuffer.height,
        displayBuffer.data.get(),
        pBmpInfo,
        DIB_RGB_COLORS,
        SRCCOPY
    );

    SetStretchBltMode(dc.GetSafeHdc(), oldStretchMode);
}

void CXIMEASensorDiagDlg::DrawOverlays(CDC& dc, const CRect& rect)
{
    // Draw trajectory if visible
    if (m_showTrajectory && m_trajectoryAlpha > 0) {
        DrawTrajectory(dc, rect);
    }

    // Draw detection overlay
    if (Camera_IsRealtimeDetectionEnabled()) {
        DrawDetectionOverlay(dc, rect);
    }

    // Draw ROI overlay
    if (Camera_IsDynamicROIEnabled()) {
        DrawDynamicROIOverlay(dc, rect);
    }
}

void CXIMEASensorDiagDlg::InitializeMemoryDC()
{
    if (!m_ui.pictureCtrl || !m_ui.pictureCtrl->GetSafeHwnd()) {
        return;
    }

    CClientDC dc(m_ui.pictureCtrl);
    CRect rect;
    m_ui.pictureCtrl->GetClientRect(&rect);

    // Create memory DC
    if (!m_memDC.GetSafeHdc()) {
        m_memDC.CreateCompatibleDC(&dc);
        m_memBitmap.CreateCompatibleBitmap(&dc, rect.Width(), rect.Height());
        m_pOldBitmap = m_memDC.SelectObject(&m_memBitmap);

        // Fill with gray background
        m_memDC.FillSolidRect(rect, RGB(128, 128, 128));
    }

    m_lastPictureRect = rect;
    m_bMemDCReady = true;
}

void CXIMEASensorDiagDlg::CleanupMemoryDC()
{
    if (m_memDC.GetSafeHdc()) {
        m_memDC.SelectObject(m_pOldBitmap);
        m_memBitmap.DeleteObject();
        m_memDC.DeleteDC();
    }

    m_bMemDCReady = false;
}

// ============================================================================
// Ball Detection
// ============================================================================
void CXIMEASensorDiagDlg::OnBnClickedCheckRealtimeDetection()
{
    if (!m_ui.checkRealtimeDetection || !m_isStreaming) {
        return;
    }

    BOOL isChecked = (m_ui.checkRealtimeDetection->GetCheck() == BST_CHECKED);

    if (isChecked) {
        EnableRealtimeDetection();
    }
    else {
        DisableRealtimeDetection();
    }
}

void CXIMEASensorDiagDlg::EnableRealtimeDetection()
{
    // Set detection callback
    Camera_SetRealtimeDetectionCallback(RealtimeDetectionCallback, this);

    if (Camera_EnableRealtimeDetection(true)) {
        m_lastDetectionStatsUpdate = std::chrono::steady_clock::now();

        // Enable ball state tracking
        Camera_SetBallStateChangeCallback(BallStateChangeCallback, this);
        Camera_EnableBallStateTracking(true);

        // Start update timer
        SetTimer(TIMER_BALL_STATE_UPDATE, UI_UPDATE_INTERVAL_MS, nullptr);

        // Update UI
        if (m_ui.btnResetTracking) {
            m_ui.btnResetTracking->EnableWindow(TRUE);
        }

        if (m_ui.ballPosition) {
            m_ui.ballPosition->SetWindowText(_T("Detecting..."));
        }

        if (m_ui.ballState) {
            m_ui.ballState->SetWindowText(_T("NOT DETECTED"));
        }

        TRACE(_T("Real-time detection and state tracking enabled\n"));
    }
    else {
        AfxMessageBox(_T("Failed to enable real-time detection!"));
        m_ui.checkRealtimeDetection->SetCheck(BST_UNCHECKED);
    }
}

void CXIMEASensorDiagDlg::DisableRealtimeDetection()
{
    // Disable tracking
    Camera_EnableBallStateTracking(false);
    Camera_SetBallStateChangeCallback(nullptr, nullptr);

    Camera_EnableRealtimeDetection(false);
    Camera_SetRealtimeDetectionCallback(nullptr, nullptr);

    // Stop timer
    KillTimer(TIMER_BALL_STATE_UPDATE);

    // Update UI
    if (m_ui.btnResetTracking) {
        m_ui.btnResetTracking->EnableWindow(FALSE);
    }

    // Reset displays
    ResetDetectionDisplays();

    TRACE(_T("Real-time detection and state tracking disabled\n"));
}

void CXIMEASensorDiagDlg::ResetDetectionDisplays()
{
    if (m_ui.ballPosition) {
        m_ui.ballPosition->SetWindowText(_T("Not detected"));
    }
    if (m_ui.ballInfo) {
        m_ui.ballInfo->SetWindowText(_T("-"));
    }
    if (m_ui.detectionFPS) {
        m_ui.detectionFPS->SetWindowText(_T("0.0"));
    }
    if (m_ui.ballState) {
        m_ui.ballState->SetWindowText(_T("NOT DETECTED"));
    }
    if (m_ui.stateTime) {
        m_ui.stateTime->SetWindowText(_T("0 ms"));
    }
    if (m_ui.stableTime) {
        m_ui.stableTime->SetWindowText(_T("0 ms"));
    }
}

void CXIMEASensorDiagDlg::DrawDetectionOverlay(CDC& dc, const CRect& rect)
{
    RealtimeDetectionResult result;
    {
        std::lock_guard<std::mutex> lock(m_detectionMutex);
        result = m_lastDetectionResult;
    }

    if (!result.ballFound || result.ballCount == 0) {
        return;
    }

    // Get current ball state for color
    BallState currentState = Camera_GetBallState();

    // Get display buffer dimensions
    int displayIdx = m_displayBufferIndex.load();
    FrameBuffer& displayBuffer = m_frameBuffers[displayIdx];

    if (displayBuffer.width == 0 || displayBuffer.height == 0) {
        return;
    }

    float scaleX = static_cast<float>(rect.Width()) / displayBuffer.width;
    float scaleY = static_cast<float>(rect.Height()) / displayBuffer.height;

    // Set pen based on state
    COLORREF penColor = GetBallStateColor(currentState);
    int penWidth = (currentState == BallState::READY) ? 4 : 3;

    CPen pen(PS_SOLID, penWidth, penColor);
    CPen* pOldPen = dc.SelectObject(&pen);
    CBrush* pOldBrush = static_cast<CBrush*>(dc.SelectStockObject(NULL_BRUSH));

    // Draw each detected ball
    for (int i = 0; i < result.ballCount; i++) {
        int x = static_cast<int>(result.balls[i].centerX * scaleX);
        int y = static_cast<int>(result.balls[i].centerY * scaleY);
        int radius = static_cast<int>(result.balls[i].radius * scaleX);

        // Draw circle
        dc.Ellipse(x - radius, y - radius, x + radius, y + radius);

        // Draw crosshair
        CPen crosshairPen(PS_SOLID, 2, RGB(255, 200, 100));
        dc.SelectObject(&crosshairPen);
        dc.MoveTo(x - 5, y);
        dc.LineTo(x + 5, y);
        dc.MoveTo(x, y - 5);
        dc.LineTo(x, y + 5);
        dc.SelectObject(&pen);

        // Draw confidence for high confidence detections
        if (result.balls[i].confidence > 0.75f) {
            CString confStr;
            confStr.Format(_T("%.0f%%"), result.balls[i].confidence * 100);
            dc.SetBkMode(TRANSPARENT);
            dc.SetTextColor(RGB(0, 255, 0));
            dc.TextOut(x + radius + 5, y - 10, confStr);
        }
    }

    // Draw READY indicator if applicable
    if (currentState == BallState::READY) {
        dc.SetTextColor(RGB(0, 255, 0));
        dc.SetBkMode(TRANSPARENT);

        CFont font;
        font.CreateFont(16, 0, 0, 0, FW_BOLD, FALSE, FALSE, 0,
            ANSI_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
            DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, _T("Arial"));

        CFont* pOldFont = dc.SelectObject(&font);

        int x = static_cast<int>(result.balls[0].centerX * scaleX);
        int y = static_cast<int>(result.balls[0].centerY * scaleY);
        int radius = static_cast<int>(result.balls[0].radius * scaleX);

        dc.TextOut(x - 20, y + radius + 10, _T("READY"));
        dc.SelectObject(pOldFont);
    }

    dc.SelectObject(pOldPen);
    dc.SelectObject(pOldBrush);
}

// ============================================================================
// Ball State Management
// ============================================================================
void CXIMEASensorDiagDlg::UpdateBallStateDisplay()
{
    if (!Camera_IsBallStateTrackingEnabled()) {
        return;
    }

    BallStateInfo info;
    if (!Camera_GetBallStateInfo(&info)) {
        return;
    }

    // Update state text
    if (m_ui.ballState) {
        CString stateStr = GetBallStateDisplayString(info.currentState);
        m_ui.ballState->SetWindowText(stateStr);
    }

    // Update time in state
    int timeInState = Camera_GetTimeInCurrentState();
    if (m_ui.stateTime) {
        CString timeStr;
        if (timeInState < 1000) {
            timeStr.Format(_T("%d ms"), timeInState);
        }
        else {
            timeStr.Format(_T("%.1f s"), timeInState / 1000.0f);
        }
        m_ui.stateTime->SetWindowText(timeStr);
    }

    // Update stable time
    if (m_ui.stableTime) {
        if (info.currentState == BallState::STABILIZING ||
            info.currentState == BallState::READY ||
            info.currentState == BallState::STOPPED) {
            CString stableStr;
            if (info.stableDurationMs < 1000) {
                stableStr.Format(_T("%d ms"), info.stableDurationMs);
            }
            else {
                stableStr.Format(_T("%.1f s"), info.stableDurationMs / 1000.0f);
            }
            m_ui.stableTime->SetWindowText(stableStr);
        }
        else {
            m_ui.stableTime->SetWindowText(_T("0 ms"));
        }
    }
}

CString CXIMEASensorDiagDlg::GetBallStateDisplayString(BallState state) const
{
    switch (state) {
    case BallState::NOT_DETECTED:
        return _T("NOT DETECTED");
    case BallState::MOVING:
        return _T("MOVING");
    case BallState::STABILIZING:
        return _T("STABILIZING...");
    case BallState::READY:
        return _T("READY");
    case BallState::STOPPED:
        return _T("STOPPED");
    default:
        return _T("UNKNOWN");
    }
}

COLORREF CXIMEASensorDiagDlg::GetBallStateColor(BallState state) const
{
    switch (state) {
    case BallState::NOT_DETECTED:
        return RGB(128, 128, 128);  // Gray
    case BallState::MOVING:
        return RGB(255, 165, 0);    // Orange
    case BallState::STABILIZING:
        return RGB(255, 255, 0);    // Yellow
    case BallState::READY:
        return RGB(0, 255, 0);      // Green
    case BallState::STOPPED:
        return RGB(0, 128, 255);    // Blue
    default:
        return RGB(255, 0, 0);      // Red
    }
}

void CXIMEASensorDiagDlg::OnBnClickedButtonResetTracking()
{
    Camera_ResetBallStateTracking();
    UpdateBallStateDisplay();

    CString msg = _T("Ball state tracking has been reset.");
    SetDlgItemText(IDC_STATIC_STATUS, msg);

    TRACE(_T("Ball state tracking reset\n"));
}

void CXIMEASensorDiagDlg::OnBnClickedButtonConfigureTracking()
{
    BallStateConfig config;
    Camera_GetBallStateConfig(&config);

    CString msg;
    msg.Format(_T("Ball State Tracking Configuration:\n\n")
        _T("Position Tolerance: %.1f pixels\n")
        _T("Movement Threshold: %.1f pixels\n")
        _T("Stable Time: %d ms\n")
        _T("Min Consecutive Detections: %d\n\n")
        _T("Default values are optimized for most cases."),
        config.positionTolerance,
        config.movementThreshold,
        config.stableTimeMs,
        config.minConsecutiveDetections);

    AfxMessageBox(msg, MB_OK | MB_ICONINFORMATION);
}


void CXIMEASensorDiagDlg::CheckForStuckStates()
{
    if (!m_ballStateTracking.shotInProgress) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    auto timeSinceMovingStart = std::chrono::duration_cast<std::chrono::seconds>(
        now - m_ballStateTracking.movingStartTime).count();

    // If shot has been in progress for more than 30 seconds, assume it's stuck
    const int MAX_SHOT_DURATION_SECONDS = 30;

    if (timeSinceMovingStart > MAX_SHOT_DURATION_SECONDS) {
        TRACE(_T("Shot timeout - clearing stuck trajectory\n"));

        // Force clear the trajectory
        StopTrajectoryRecording();
        ClearTrajectory();

        // Reset tracking
        m_ballStateTracking.shotInProgress = false;
        m_ballStateTracking.waitingForStop = false;

        // Update status
        if (m_ui.status) {
            m_ui.status->SetWindowText(_T("Shot timeout - ready for new shot"));
        }
    }
}


// ============================================================================
// Trajectory Visualization
// ============================================================================
void CXIMEASensorDiagDlg::StartTrajectoryRecording()
{
    std::lock_guard<std::mutex> lock(m_trajectoryMutex);

    m_trajectoryPoints.clear();
    m_isRecordingTrajectory = true;
    m_showTrajectory = true;
    m_trajectoryAlpha = 255;

    // Stop fade timer if running
    KillTimer(TIMER_TRAJECTORY_FADE);

    TRACE(_T("Started trajectory recording\n"));
}

void CXIMEASensorDiagDlg::StopTrajectoryRecording()
{
    std::lock_guard<std::mutex> lock(m_trajectoryMutex);

    m_isRecordingTrajectory = false;

    TRACE(_T("Stopped trajectory recording - %zu points recorded\n"),
        m_trajectoryPoints.size());
}

void CXIMEASensorDiagDlg::ClearTrajectory()
{
    std::lock_guard<std::mutex> lock(m_trajectoryMutex);

    m_trajectoryPoints.clear();
    m_isRecordingTrajectory = false;
    m_showTrajectory = false;
    m_trajectoryAlpha = 255;

    // Reset ball state tracking
    m_ballStateTracking.shotInProgress = false;
    m_ballStateTracking.waitingForStop = false;

    KillTimer(DialogConstants::TIMER_TRAJECTORY_FADE);

    // Force redraw to clear any remaining trajectory
    if (m_ui.pictureCtrl) {
        CRect rect;
        m_ui.pictureCtrl->GetClientRect(&rect);
        InvalidateRect(&rect, FALSE);
    }
}

void CXIMEASensorDiagDlg::AddTrajectoryPoint(float x, float y, float confidence)
{
    if (!m_isRecordingTrajectory) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_trajectoryMutex);

    // Limit maximum points
    if (m_trajectoryPoints.size() >= MAX_TRAJECTORY_POINTS) {
        m_trajectoryPoints.pop_front();
    }

    m_trajectoryPoints.emplace_back(cv::Point2f(x, y), GetTickCount(), confidence);
}

void CXIMEASensorDiagDlg::StartTrajectoryFadeOut()
{
    m_trajectoryAlpha = 255;
    SetTimer(TIMER_TRAJECTORY_FADE, FADE_DURATION_MS / FADE_STEPS, nullptr);

    TRACE(_T("Started trajectory fade out\n"));
}

void CXIMEASensorDiagDlg::UpdateTrajectoryFade()
{
    int currentAlpha = m_trajectoryAlpha.load();
    int fadeStep = 255 / FADE_STEPS;

    currentAlpha -= fadeStep;

    if (currentAlpha <= 0) {
        m_trajectoryAlpha = 0;
        m_showTrajectory = false;
        ClearTrajectory();
        KillTimer(TIMER_TRAJECTORY_FADE);

        TRACE(_T("Trajectory fade out completed\n"));
    }
    else {
        m_trajectoryAlpha = currentAlpha;
    }

    // Refresh display
    Invalidate(FALSE);
}

void CXIMEASensorDiagDlg::DrawTrajectory(CDC& dc, const CRect& rect)
{
    if (!m_showTrajectory || m_trajectoryAlpha == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_trajectoryMutex);

    if (m_trajectoryPoints.size() < 2) {
        return;
    }

    HDC hdc = dc.GetSafeHdc();
    if (!hdc) {
        return;
    }

    // Convert to screen coordinates
    std::vector<CPoint> screenPoints;
    screenPoints.reserve(m_trajectoryPoints.size());

    for (const auto& point : m_trajectoryPoints) {
        screenPoints.push_back(ConvertToScreenCoordinates(point.position, rect));
    }

    // Draw based on alpha value
    int alpha = m_trajectoryAlpha.load();

    if (alpha == 255) {
        // Full opacity - use GDI
        DrawTrajectoryLine(dc, screenPoints);

        if (m_trajectoryStyle.showPoints) {
            DrawTrajectoryPoints(dc, screenPoints);
        }
    }
    else if (alpha > 0) {
        // Fading - use GDI+
        Graphics graphics(hdc);
        graphics.SetSmoothingMode(SmoothingModeAntiAlias);
        DrawTrajectoryWithAlpha(graphics, screenPoints, alpha);
    }

    // Draw start and end markers
    if (screenPoints.size() > 0) {
        CBrush startBrush(RGB(0, 255, 0));
        CBrush* pOldBrush = dc.SelectObject(&startBrush);
        dc.Ellipse(screenPoints.front().x - 8, screenPoints.front().y - 8,
            screenPoints.front().x + 8, screenPoints.front().y + 8);

        CBrush endBrush(RGB(255, 0, 0));
        dc.SelectObject(&endBrush);
        dc.Ellipse(screenPoints.back().x - 8, screenPoints.back().y - 8,
            screenPoints.back().x + 8, screenPoints.back().y + 8);

        dc.SelectObject(pOldBrush);
    }
}

void CXIMEASensorDiagDlg::DrawTrajectoryLine(CDC& dc, const std::vector<CPoint>& points)
{
    if (points.size() < 2) {
        return;
    }

    CPen pen(PS_SOLID, m_trajectoryStyle.lineWidth, m_trajectoryStyle.lineColor);
    CPen* pOldPen = dc.SelectObject(&pen);

    dc.MoveTo(points[0]);
    for (size_t i = 1; i < points.size(); ++i) {
        dc.LineTo(points[i]);
    }

    dc.SelectObject(pOldPen);
}

void CXIMEASensorDiagDlg::DrawTrajectoryPoints(CDC& dc, const std::vector<CPoint>& points)
{
    const int POINT_INTERVAL = std::max(1, static_cast<int>(points.size() / 20));

    CBrush brush(m_trajectoryStyle.lineColor);
    CBrush* pOldBrush = dc.SelectObject(&brush);

    for (size_t i = 0; i < points.size(); i += POINT_INTERVAL) {
        int size = m_trajectoryStyle.pointSize;
        dc.Ellipse(points[i].x - size, points[i].y - size,
            points[i].x + size, points[i].y + size);
    }

    dc.SelectObject(pOldBrush);
}

void CXIMEASensorDiagDlg::DrawTrajectoryWithAlpha(Gdiplus::Graphics& graphics,
    const std::vector<CPoint>& points,
    int alpha)
{
    if (points.size() < 2) {
        return;
    }

    Color lineColor(alpha,
        GetRValue(m_trajectoryStyle.lineColor),
        GetGValue(m_trajectoryStyle.lineColor),
        GetBValue(m_trajectoryStyle.lineColor));

    Pen pen(lineColor, static_cast<REAL>(m_trajectoryStyle.lineWidth));

    // Convert to GDI+ points
    std::vector<Point> gdiPoints;
    gdiPoints.reserve(points.size());

    for (const auto& pt : points) {
        gdiPoints.emplace_back(pt.x, pt.y);
    }

    graphics.DrawLines(&pen, gdiPoints.data(), static_cast<INT>(gdiPoints.size()));
}

CPoint CXIMEASensorDiagDlg::ConvertToScreenCoordinates(const cv::Point2f& point,
    const CRect& displayRect)
{
    // Get current display buffer dimensions
    int displayIdx = m_displayBufferIndex.load();
    FrameBuffer& displayBuffer = m_frameBuffers[displayIdx];

    if (displayBuffer.width == 0 || displayBuffer.height == 0 ||
        displayRect.Width() == 0 || displayRect.Height() == 0) {
        return CPoint(0, 0);
    }

    float scaleX = static_cast<float>(displayRect.Width()) / displayBuffer.width;
    float scaleY = static_cast<float>(displayRect.Height()) / displayBuffer.height;

    int x = static_cast<int>(point.x * scaleX);
    int y = static_cast<int>(point.y * scaleY);

    // Boundary check
    x = std::max(0, std::min(x, displayRect.Width() - 1));
    y = std::max(0, std::min(y, displayRect.Height() - 1));

    return CPoint(x, y);
}

// ============================================================================
// Dynamic ROI
// ============================================================================
void CXIMEASensorDiagDlg::OnBnClickedCheckEnableDynamicROI()
{
    if (!m_ui.checkEnableDynamicROI || !m_isStreaming) {
        return;
    }

    BOOL isChecked = (m_ui.checkEnableDynamicROI->GetCheck() == BST_CHECKED);

    if (isChecked) {
        EnableDynamicROI();
    }
    else {
        DisableDynamicROI();
    }
}

void CXIMEASensorDiagDlg::EnableDynamicROI()
{
    // Get current configuration
    DynamicROIConfig config;
    Camera_GetDynamicROIConfig(&config);

    // Update from UI
    config.enabled = true;
    config.showROIOverlay = (m_ui.checkShowROIOverlay &&
        m_ui.checkShowROIOverlay->GetCheck() == BST_CHECKED);

    if (m_ui.sliderROIMultiplier) {
        config.roiSizeMultiplier = m_ui.sliderROIMultiplier->GetPos() / 10.0f;
    }

    // Apply configuration
    Camera_SetDynamicROIConfig(&config);
    Camera_EnableDynamicROI(true);

    // Start update timer
    SetTimer(TIMER_DYNAMIC_ROI_UPDATE, UI_UPDATE_INTERVAL_MS, nullptr);

    TRACE(_T("Dynamic ROI enabled\n"));
}

void CXIMEASensorDiagDlg::DisableDynamicROI()
{
    Camera_EnableDynamicROI(false);
    KillTimer(TIMER_DYNAMIC_ROI_UPDATE);

    UpdateDynamicROIDisplay();

    TRACE(_T("Dynamic ROI disabled\n"));
}

void CXIMEASensorDiagDlg::UpdateDynamicROIDisplay()
{
    DynamicROIInfo info;
    if (!Camera_GetDynamicROIInfo(&info)) {
        return;
    }

    if (m_ui.roiStatus) {
        m_ui.roiStatus->SetWindowText(info.active ? _T("Active") : _T("Inactive"));
    }

    if (m_ui.roiSize) {
        if (info.active) {
            CString sizeStr;
            sizeStr.Format(_T("%dx%d"), info.width, info.height);
            m_ui.roiSize->SetWindowText(sizeStr);
        }
        else {
            m_ui.roiSize->SetWindowText(_T("-"));
        }
    }

    if (m_ui.roiReduction) {
        if (info.active) {
            CString reductionStr;
            reductionStr.Format(_T("%.1f%%"), info.processingTimeReduction);
            m_ui.roiReduction->SetWindowText(reductionStr);
        }
        else {
            m_ui.roiReduction->SetWindowText(_T("0%"));
        }
    }
}

void CXIMEASensorDiagDlg::DrawDynamicROIOverlay(CDC& dc, const CRect& rect)
{
    DynamicROIConfig config;
    DynamicROIInfo info;

    if (!Camera_GetDynamicROIConfig(&config) || !config.showROIOverlay) {
        return;
    }

    if (!Camera_GetDynamicROIInfo(&info) || !info.active) {
        return;
    }

    // Get display buffer dimensions
    int displayIdx = m_displayBufferIndex.load();
    FrameBuffer& displayBuffer = m_frameBuffers[displayIdx];

    if (displayBuffer.width == 0 || displayBuffer.height == 0) {
        return;
    }

    float scaleX = static_cast<float>(rect.Width()) / displayBuffer.width;
    float scaleY = static_cast<float>(rect.Height()) / displayBuffer.height;

    // Calculate ROI rectangle
    int x = static_cast<int>((info.centerX - info.width / 2) * scaleX);
    int y = static_cast<int>((info.centerY - info.height / 2) * scaleY);
    int w = static_cast<int>(info.width * scaleX);
    int h = static_cast<int>(info.height * scaleY);

    // Draw ROI boundary
    CPen pen(PS_DASH, 2, RGB(255, 128, 0)); // Orange dashed line
    CPen* pOldPen = dc.SelectObject(&pen);
    CBrush* pOldBrush = static_cast<CBrush*>(dc.SelectStockObject(NULL_BRUSH));

    dc.Rectangle(x, y, x + w, y + h);

    // Draw ROI info
    dc.SetBkMode(TRANSPARENT);
    dc.SetTextColor(RGB(255, 128, 0));

    CString roiText;
    roiText.Format(_T("ROI: %dx%d (%.1f%% reduction)"),
        info.width, info.height, info.processingTimeReduction);
    dc.TextOut(x + 2, y + 2, roiText);

    dc.SelectObject(pOldPen);
    dc.SelectObject(pOldBrush);
}

void CXIMEASensorDiagDlg::OnBnClickedCheckShowROIOverlay()
{
    if (!m_ui.checkShowROIOverlay) {
        return;
    }

    DynamicROIConfig config;
    if (Camera_GetDynamicROIConfig(&config)) {
        config.showROIOverlay = (m_ui.checkShowROIOverlay->GetCheck() == BST_CHECKED);
        Camera_SetDynamicROIConfig(&config);
    }
}

void CXIMEASensorDiagDlg::OnBnClickedButtonResetROI()
{
    Camera_ResetDynamicROI();
    UpdateDynamicROIDisplay();

    TRACE(_T("Dynamic ROI reset\n"));
}

// ============================================================================
// Timer Handlers
// ============================================================================
void CXIMEASensorDiagDlg::OnTimer(UINT_PTR nIDEvent)
{
    switch (nIDEvent) {
    case DialogConstants::TIMER_UPDATE_STATISTICS:
        if (m_isStreaming) {
            UpdateStatistics();
        }
        break;

    case DialogConstants::TIMER_FRAME_UPDATE:
        if (m_pendingFrameUpdates > 0 && m_ui.pictureCtrl) {
            CRect rect;
            m_ui.pictureCtrl->GetWindowRect(&rect);
            ScreenToClient(&rect);
            InvalidateRect(&rect, FALSE);
        }
        break;

    case DialogConstants::TIMER_BALL_STATE_UPDATE:
        UpdateBallStateDisplay();

        // Check for stuck states
        CheckForStuckStates();
        break;

    case DialogConstants::TIMER_DYNAMIC_ROI_UPDATE:
        UpdateDynamicROIDisplay();
        break;

    case DialogConstants::TIMER_TRAJECTORY_FADE:
        UpdateTrajectoryFade();
        break;

    case DialogConstants::TIMER_CAMERA_RESTART:
        KillTimer(DialogConstants::TIMER_CAMERA_RESTART);
        ExecuteCameraRestart();
        break;
    }

    CDialogEx::OnTimer(nIDEvent);
}

void CXIMEASensorDiagDlg::UpdateStatistics()
{
    unsigned long totalFrames, droppedFrames;
    double avgFPS, minFPS, maxFPS;

    if (Camera_GetStatistics(&totalFrames, &droppedFrames, &avgFPS, &minFPS, &maxFPS)) {
        CString str;
        str.Format(_T("Total: %lu, Dropped: %lu, Avg FPS: %.1f"),
            totalFrames, droppedFrames, avgFPS);

        if (m_ui.status) {
            m_ui.status->SetWindowText(str);
        }
    }
}

// ============================================================================
// Error Handling
// ============================================================================
void CXIMEASensorDiagDlg::HandleUSBError()
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_lastUSBError).count();

    // Reset error count if enough time has passed
    if (elapsed > USB_ERROR_RESET_TIME_MS) {
        m_usbErrorCount = 0;
    }

    m_lastUSBError = now;
    m_usbErrorCount++;

    // If too many errors, attempt recovery
    if (m_usbErrorCount >= MAX_USB_ERRORS) {
        TRACE(_T("Too many USB errors. Attempting to restart camera...\n"));

        // Stop and restart
        PostMessage(WM_COMMAND, MAKEWPARAM(IDC_BUTTON_STOP, BN_CLICKED), 0);

        // Set timer to restart after delay
        SetTimer(3000, 2000, nullptr); // Timer ID 3000, 2 second delay
    }
}

void CXIMEASensorDiagDlg::ResetUSBErrorCount()
{
    m_usbErrorCount = 0;
    m_lastUSBError = std::chrono::steady_clock::now();
}

void CXIMEASensorDiagDlg::ShowError(const CString& message)
{
    // Avoid showing message box for recoverable errors
    if (message.Find(_T("Device not ready")) != -1 ||
        message.Find(_T("USB")) != -1) {
        TRACE(_T("Recoverable error: %s\n"), message.GetString());
        return;
    }

    MessageBox(message, _T("Camera Error"), MB_OK | MB_ICONERROR);
}


bool CXIMEASensorDiagDlg::HandleCameraError(CameraError error, const CString& message)
{
    bool handled = false;

    switch (error) {
    case CameraError::DEVICE_NOT_FOUND:
    {
        // Device disconnected - stop streaming
        PostMessage(WM_COMMAND, MAKEWPARAM(IDC_BUTTON_STOP, BN_CLICKED), 0);
        MessageBox(_T("Camera device was disconnected. Please reconnect and try again."),
            _T("Device Error"), MB_OK | MB_ICONWARNING);
        handled = true;
    }
    break;

    case CameraError::DEVICE_NOT_READY:
    case CameraError::TIMEOUT:
    {
        // USB/Timeout errors - attempt recovery
        HandleUSBError();
        handled = true;
    }
    break;

    case CameraError::FRAME_GRAB_ERROR:
    {
        // Frame grab error - log but continue
        TRACE(_T("Frame grab error: %s\n"), message.GetString());
        handled = true;
    }
    break;

    case CameraError::PARAMETER_ERROR:
    {
        // Parameter error - sync UI with camera
        SyncSlidersWithCamera();
        ShowError(_T("Invalid parameter. Settings have been reset to current values."));
        handled = true;
    }
    break;

    case CameraError::MEMORY_ERROR:
    {
        // Memory error - critical
        StopCamera();
        MessageBox(_T("Memory allocation error. Camera has been stopped."),
            _T("Critical Error"), MB_OK | MB_ICONERROR);
        handled = true;
    }
    break;

    case CameraError::OPEN_FAILED:
    case CameraError::START_FAILED:
    {
        // Start/Open failed - already handled by caller
        handled = false;
    }
    break;

    default:
    {
        // Unknown error
        TRACE(_T("Unhandled camera error: %d - %s\n"),
            static_cast<int>(error), message.GetString());
        handled = false;
    }
    break;
    }

    return handled;
}

void CXIMEASensorDiagDlg::ScheduleCameraRestart()
{
    m_pendingCameraRestart = true;
    m_lastDeviceIndex = m_ui.comboDevices->GetCurSel();

    // Schedule restart after delay
    SetTimer(DialogConstants::TIMER_CAMERA_RESTART,
        DialogConstants::CAMERA_RESTART_DELAY_MS, nullptr);
}

void CXIMEASensorDiagDlg::ExecuteCameraRestart()
{
    if (!m_pendingCameraRestart || m_lastDeviceIndex < 0) {
        return;
    }

    m_pendingCameraRestart = false;

    // Attempt to restart camera
    if (StartCamera(m_lastDeviceIndex)) {
        TRACE(_T("Camera successfully restarted after USB error\n"));
        ResetUSBErrorCount();
    }
    else {
        MessageBox(_T("Failed to restart camera. Please check the connection."),
            _T("Camera Error"), MB_OK | MB_ICONERROR);
    }
}

// ============================================================================
// Snapshot Handling
// ============================================================================
void CXIMEASensorDiagDlg::OnBnClickedButtonSnapshot()
{
    if (!m_isStreaming) {
        AfxMessageBox(_T("Camera is not running!"));
        return;
    }

#ifdef ENABLE_CONTINUOUS_CAPTURE
    if (Camera_IsContinuousCapturing()) {
        AfxMessageBox(_T("Already in continuous capture mode!"));
        return;
    }

    BOOL isContinuous = (m_ui.checkContinuous &&
        m_ui.checkContinuous->GetCheck() == BST_CHECKED);

    if (isContinuous) {
        HandleContinuousCapture();
    }
    else
#endif
    {
        HandleSingleSnapshot();
    }
}

void CXIMEASensorDiagDlg::HandleSingleSnapshot()
{
    SnapshotDefaults defaults;
    Camera_GetSnapshotDefaults(&defaults);

    // Ask for format
    int result = MessageBox(_T("Select image format:\n\nPNG (Yes) - Lossless format\nJPG (No) - Compressed format"),
        _T("Image Format"), MB_YESNOCANCEL | MB_ICONQUESTION);

    if (result == IDCANCEL) {
        return;
    }

    defaults.format = (result == IDYES) ? 0 : 1;
    Camera_SetSnapshotDefaults(&defaults);

    // Generate filename
    SYSTEMTIME st;
    GetLocalTime(&st);
    CString filename;
    const char* extension = (defaults.format == 0) ? "png" : "jpg";
    filename.Format(_T("snapshot_%04d%02d%02d_%02d%02d%02d.%s"),
        st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond,
        CString(extension));

    // Save snapshot
    CStringA filenameA(filename);
    bool saveSuccess = Camera_SaveSnapshotWithDefaults(filenameA.GetString());

    if (saveSuccess) {
        int width = 0, height = 0;
        Camera_GetROI(nullptr, nullptr, &width, &height);

        CString msg;
        msg.Format(_T("Snapshot saved: %s\nSize: %dx%d\nFormat: %s\nQuality: %d%%"),
            filename.GetString(), width, height,
            (defaults.format == 0) ? _T("PNG") : _T("JPG"),
            defaults.quality);
        AfxMessageBox(msg);
    }
    else {
        AfxMessageBox(_T("Failed to save image!"));
    }
}

#ifdef ENABLE_CONTINUOUS_CAPTURE
void CXIMEASensorDiagDlg::HandleContinuousCapture()
{
    ContinuousCaptureConfig config;
    Camera_GetContinuousCaptureDefaults(&config);

    // Ask for format
    int result = MessageBox(_T("Select image format:\n\nPNG (Yes) - Lossless, larger files\nJPG (No) - Compressed, smaller files"),
        _T("Image Format"), MB_YESNOCANCEL | MB_ICONQUESTION);

    if (result == IDCANCEL) {
        return;
    }

    config.imageFormat = (result == IDYES) ? 0 : 1;
    Camera_SetContinuousCaptureDefaults(&config);

    // Set progress callback
    Camera_SetContinuousCaptureProgressCallback(ContinuousCaptureProgressCallback);

    TRACE(_T("Starting continuous capture for %.1f seconds with ball detection\n"),
        config.durationSeconds);

    if (Camera_StartContinuousCaptureWithDefaults()) {
        // Disable controls during capture
        if (m_ui.btnSnapshot) {
            m_ui.btnSnapshot->EnableWindow(FALSE);
        }
        if (m_ui.checkContinuous) {
            m_ui.checkContinuous->EnableWindow(FALSE);
        }

        // Update status
        if (m_ui.status) {
            CString status;
            status.Format(_T("Continuous capture (%.1fs) with ball detection in progress..."),
                config.durationSeconds);
            m_ui.status->SetWindowText(status);
        }

        TRACE(_T("Continuous capture started successfully\n"));
    }
    else {
        AfxMessageBox(_T("Failed to start continuous capture!"));
        TRACE(_T("Failed to start continuous capture\n"));
    }
}
#endif

// ============================================================================
// Message Handlers
// ============================================================================
LRESULT CXIMEASensorDiagDlg::OnUpdateFrame(WPARAM wParam, LPARAM lParam)
{
    m_pendingFrameUpdates.fetch_sub(1);

    if (!IsIconic() && m_ui.pictureCtrl && m_ui.pictureCtrl->GetSafeHwnd()) {
        CRect rect;
        m_ui.pictureCtrl->GetWindowRect(&rect);
        ScreenToClient(&rect);
        InvalidateRect(&rect, FALSE);
    }

    return 0;
}

LRESULT CXIMEASensorDiagDlg::OnUpdateStatus(WPARAM wParam, LPARAM lParam)
{
    CString* pMsg = reinterpret_cast<CString*>(lParam);
    if (pMsg && m_ui.status) {
        m_ui.status->SetWindowText(*pMsg);
        delete pMsg;
    }
    return 0;
}

LRESULT CXIMEASensorDiagDlg::OnUpdateError(WPARAM wParam, LPARAM lParam)
{
    CString* pMsg = reinterpret_cast<CString*>(lParam);
    if (pMsg) {
        ShowError(*pMsg);
        delete pMsg;
    }
    return 0;
}

LRESULT CXIMEASensorDiagDlg::OnUpdateFPS(WPARAM wParam, LPARAM lParam)
{
    double fps = lParam / 10.0;
    CString str;
    str.Format(_T("%.1f"), fps);

    if (m_ui.fps) {
        m_ui.fps->SetWindowText(str);
    }
    return 0;
}

LRESULT CXIMEASensorDiagDlg::OnUpdateBallDetection(WPARAM wParam, LPARAM lParam)
{
    RealtimeDetectionResult result;
    {
        std::lock_guard<std::mutex> lock(m_detectionMutex);
        result = m_lastDetectionResult;
    }

    // Update position display
    if (result.ballFound && result.ballCount > 0) {
        CString posStr;
        posStr.Format(_T("X: %d, Y: %d"),
            static_cast<int>(result.balls[0].centerX),
            static_cast<int>(result.balls[0].centerY));

        if (m_ui.ballPosition) {
            m_ui.ballPosition->SetWindowText(posStr);
        }

        CString infoStr;
        infoStr.Format(_T("Radius: %d px, Confidence: %.1f%%, Time: %.1f ms"),
            static_cast<int>(result.balls[0].radius),
            result.balls[0].confidence * 100.0f,
            result.detectionTimeMs);

        if (m_ui.ballInfo) {
            m_ui.ballInfo->SetWindowText(infoStr);
        }
    }
    else {
        if (m_ui.ballPosition) {
            m_ui.ballPosition->SetWindowText(_T("Not detected"));
        }
        if (m_ui.ballInfo) {
            m_ui.ballInfo->SetWindowText(_T("-"));
        }
    }

    // Update detection FPS periodically
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_lastDetectionStatsUpdate).count();

    if (elapsed >= 1000) {
        int processedFrames = 0;
        double avgProcessingTime = 0.0;
        double detectionFPS = 0.0;

        Camera_GetRealtimeDetectionStats(&processedFrames, &avgProcessingTime, &detectionFPS);

        if (m_ui.detectionFPS) {
            CString fpsStr;
            fpsStr.Format(_T("%.1f"), detectionFPS);
            m_ui.detectionFPS->SetWindowText(fpsStr);
        }

        m_lastDetectionStatsUpdate = now;
    }

    return 0;
}

LRESULT CXIMEASensorDiagDlg::OnUpdateBallState(WPARAM wParam, LPARAM lParam)
{
    BallState newState = static_cast<BallState>(wParam);
    BallState oldState = m_ballStateTracking.currentState;

    // Handle state change
    HandleBallStateChange(newState, oldState);

    // Update display
    UpdateBallStateDisplay();

    return 0;
}

LRESULT CXIMEASensorDiagDlg::OnUpdateDynamicROI(WPARAM wParam, LPARAM lParam)
{
    UpdateDynamicROIDisplay();
    return 0;
}

LRESULT CXIMEASensorDiagDlg::OnUpdateShotCompleted(WPARAM wParam, LPARAM lParam)
{
    const ShotCompletedInfo* info = reinterpret_cast<const ShotCompletedInfo*>(lParam);
    if (info) {
        OnShotCompleted(info);
    }
    return 0;
}

LRESULT CXIMEASensorDiagDlg::OnContinuousCaptureComplete(WPARAM wParam, LPARAM lParam)
{
#ifdef ENABLE_CONTINUOUS_CAPTURE
    HandleContinuousCaptureComplete();
#endif
    return 0;
}

#ifdef ENABLE_CONTINUOUS_CAPTURE
void CXIMEASensorDiagDlg::HandleContinuousCaptureComplete()
{
    int totalFrames = 0, savedFrames = 0, droppedFrames = 0;
    double duration = 0.0;
    char folderPath[256] = { 0 };

    bool success = Camera_GetContinuousCaptureResult(&totalFrames, &savedFrames,
        &droppedFrames, &duration,
        folderPath, sizeof(folderPath));

    // Re-enable controls
    if (m_ui.btnSnapshot) {
        m_ui.btnSnapshot->EnableWindow(TRUE);
    }
    if (m_ui.checkContinuous) {
        m_ui.checkContinuous->EnableWindow(TRUE);
    }

    // Build result message
    CString msg;
    if (success) {
        msg.Format(_T("Continuous capture completed!\n\n")
            _T("Total frames: %d\n")
            _T("Saved frames: %d\n")
            _T("Dropped frames: %d\n")
            _T("Duration: %.3f sec\n")
            _T("Average FPS: %.1f\n")
            _T("Folder: %s"),
            totalFrames, savedFrames, droppedFrames,
            duration, totalFrames / duration,
            CString(folderPath).GetString());

        // Add ball detection results if continuous mode
        BOOL isContinuous = (m_ui.checkContinuous &&
            m_ui.checkContinuous->GetCheck() == BST_CHECKED);

        if (isContinuous) {
            AppendBallDetectionResults(msg);
        }
    }
    else {
        msg = _T("Continuous capture failed!");
    }

    AfxMessageBox(msg);

    // Reset status
    if (m_ui.status) {
        m_ui.status->SetWindowText(_T("Capturing..."));
    }
}

void CXIMEASensorDiagDlg::AppendBallDetectionResults(CString& msg)
{
    int framesWithBalls = 0;
    int totalBallsDetected = 0;
    float averageConfidence = 0.0f;
    char detectionFolder[256] = { 0 };

    if (Camera_GetContinuousCaptureDetectionResult(&framesWithBalls, &totalBallsDetected,
        &averageConfidence, detectionFolder,
        sizeof(detectionFolder))) {
        CString detectionMsg;
        detectionMsg.Format(_T("\n\nBall Detection Results:\n")
            _T("Frames with balls: %d\n")
            _T("Total balls detected: %d\n")
            _T("Average confidence: %.1f%%\n")
            _T("Detection images: %s"),
            framesWithBalls, totalBallsDetected,
            averageConfidence * 100.0f,
            CString(detectionFolder).GetString());
        msg += detectionMsg;
    }
}
#endif

// ============================================================================
// Callback Implementations
// ============================================================================
void CXIMEASensorDiagDlg::OnStateChangedCallback(CameraState newState, CameraState oldState)
{
    CString* pMsg = new CString();

    switch (newState) {
    case CameraState::DISCONNECTED:
        *pMsg = _T("Camera disconnected");
        break;
    case CameraState::CONNECTED:
        *pMsg = _T("Camera connected");
        break;
    case CameraState::CAPTURING:
        *pMsg = _T("Capturing...");
        break;
    case CameraState::kERROR:
        *pMsg = _T("Camera error");
        break;
    }

    PostMessage(WM_UPDATE_STATUS, 0, reinterpret_cast<LPARAM>(pMsg));
}

void CXIMEASensorDiagDlg::OnErrorCallback(CameraError error, const std::string& errorMessage)
{
    // Handle USB-related errors
    if (error == CameraError::DEVICE_NOT_READY ||
        error == CameraError::TIMEOUT ||
        errorMessage.find("USB") != std::string::npos ||
        errorMessage.find("Device not ready") != std::string::npos) {
        HandleUSBError();
    }

    // Handle disconnection
    if (errorMessage.find("Camera disconnected") != std::string::npos) {
        PostMessage(WM_COMMAND, MAKEWPARAM(IDC_BUTTON_STOP, BN_CLICKED), 0);
    }

    CString* pMsg = new CString(errorMessage.c_str());
    PostMessage(WM_UPDATE_ERROR, static_cast<WPARAM>(error), reinterpret_cast<LPARAM>(pMsg));
}

void CXIMEASensorDiagDlg::OnPropertyChangedCallback(const std::string& propertyName,
    const std::string& value)
{
    TRACE(_T("Property changed: %s = %s\n"),
        CString(propertyName.c_str()), CString(value.c_str()));
}

void CXIMEASensorDiagDlg::OnRealtimeDetectionResult(const RealtimeDetectionResult* result)
{
    {
        std::lock_guard<std::mutex> lock(m_detectionMutex);
        m_lastDetectionResult = *result;
    }

    // Add to trajectory if recording and ball is in appropriate state
    if (m_isRecordingTrajectory && result && result->ballFound && result->ballCount > 0) {
        BallState currentState = Camera_GetBallState();

        // Only record trajectory during MOVING and STABILIZING states
        if (currentState == BallState::MOVING || currentState == BallState::STABILIZING) {
            AddTrajectoryPoint(result->balls[0].centerX,
                result->balls[0].centerY,
                result->balls[0].confidence);
        }
    }

    PostMessage(WM_UPDATE_BALL_DETECTION);
}

void CXIMEASensorDiagDlg::OnBallStateChanged(BallState newState, BallState oldState,
    const BallStateInfo* info)
{
    // Validate state transition
    if (!IsValidShotSequence(newState, oldState)) {
        TRACE(_T("Warning: Unexpected state transition %s -> %s\n"),
            GetBallStateDisplayString(oldState).GetString(),
            GetBallStateDisplayString(newState).GetString());
    }

    // Post message for UI update
    PostMessage(WM_UPDATE_BALL_STATE, static_cast<WPARAM>(newState),
        reinterpret_cast<LPARAM>(info));
}

void CXIMEASensorDiagDlg::HandleBallStateChange(BallState newState, BallState oldState)
{
    // Update tracking structure
    m_ballStateTracking.previousState = m_ballStateTracking.currentState;
    m_ballStateTracking.currentState = newState;
    m_ballStateTracking.lastStateChangeTime = std::chrono::steady_clock::now();

    // Log state transition
    TRACE(_T("Ball State: %s -> %s\n"),
        GetBallStateDisplayString(oldState).GetString(),
        GetBallStateDisplayString(newState).GetString());

    // Handle specific transitions
    switch (oldState) {
    case BallState::READY:
        if (newState == BallState::MOVING) {
            // Shot started
            HandleShotStarted();
        }
        break;

    case BallState::MOVING:
        if (newState == BallState::STABILIZING) {
            // Ball is slowing down
            m_ballStateTracking.waitingForStop = true;
        }
        else if (newState == BallState::READY) {
            // Abnormal transition - ball went back to ready without stopping
            HandleIncompleteShotSequence();
        }
        break;

    case BallState::STABILIZING:
        if (newState == BallState::STOPPED) {
            // Normal shot completion
            m_ballStateTracking.shotInProgress = false;
            m_ballStateTracking.waitingForStop = false;
            // OnShotCompleted will be called by the camera system
        }
        else if (newState == BallState::READY) {
            // Incomplete shot - ball stabilized but didn't stop properly
            HandleIncompleteShotSequence();
        }
        else if (newState == BallState::MOVING) {
            // Ball started moving again
            TRACE(_T("Ball resumed movement during stabilization\n"));
        }
        break;

    case BallState::STOPPED:
        if (newState == BallState::READY) {
            // Normal reset after shot completion
            m_ballStateTracking.shotInProgress = false;
        }
        break;
    }

    // Special handling for READY state
    if (newState == BallState::READY) {
        MessageBeep(MB_OK);
        TRACE(_T("Ball is READY!\n"));
    }
}


void CXIMEASensorDiagDlg::HandleShotStarted()
{
    m_ballStateTracking.shotInProgress = true;
    m_ballStateTracking.waitingForStop = false;
    m_ballStateTracking.movingStartTime = std::chrono::steady_clock::now();

    StartTrajectoryRecording();
    TRACE(_T("Shot started - trajectory recording initiated\n"));
}


void CXIMEASensorDiagDlg::StartIncompleteShotFadeOut()
{
    m_trajectoryAlpha = 255;

    // Use shorter fade duration for incomplete shots (1.5 seconds instead of 3)
    const int INCOMPLETE_FADE_DURATION_MS = 1500;
    const int INCOMPLETE_FADE_STEPS = 30;

    SetTimer(DialogConstants::TIMER_TRAJECTORY_FADE,
        INCOMPLETE_FADE_DURATION_MS / INCOMPLETE_FADE_STEPS, nullptr);

    TRACE(_T("Started incomplete shot trajectory fade out\n"));
}


void CXIMEASensorDiagDlg::HandleIncompleteShotSequence()
{
    TRACE(_T("Incomplete shot detected - clearing trajectory\n"));

    // Stop recording
    StopTrajectoryRecording();

    // Reset shot tracking
    m_ballStateTracking.shotInProgress = false;
    m_ballStateTracking.waitingForStop = false;

    // Fade out trajectory if visible
    if (m_showTrajectory && m_trajectoryAlpha > 0) {
        // Start fade out with shorter duration for incomplete shots
        StartIncompleteShotFadeOut();
    }

    // Optional: Show message to user
    if (m_ui.status) {
        m_ui.status->SetWindowText(_T("Shot incomplete - ball returned to ready position"));
    }
}

bool CXIMEASensorDiagDlg::IsValidShotSequence(BallState newState, BallState oldState) const
{
    // Define valid state transitions
    const std::pair<BallState, BallState> validTransitions[] = {
        {BallState::NOT_DETECTED, BallState::READY},
        {BallState::NOT_DETECTED, BallState::MOVING},
        {BallState::READY, BallState::MOVING},
        {BallState::READY, BallState::NOT_DETECTED},
        {BallState::MOVING, BallState::STABILIZING},
        {BallState::MOVING, BallState::STOPPED},
        {BallState::STABILIZING, BallState::STOPPED},
        {BallState::STABILIZING, BallState::MOVING},  // Ball can resume movement
        {BallState::STOPPED, BallState::READY},
        {BallState::STOPPED, BallState::MOVING},
        {BallState::STOPPED, BallState::NOT_DETECTED}
    };

    for (const auto& transition : validTransitions) {
        if (transition.first == oldState && transition.second == newState) {
            return true;
        }
    }

    return false;
}

void CXIMEASensorDiagDlg::OnShotCompleted(const ShotCompletedInfo* info)
{
    // Stop recording
    StopTrajectoryRecording();

    // Start fade out
    StartTrajectoryFadeOut();

    // Update status
    CString msg;
    msg.Format(_T("Shot Completed! Distance: %.1f px, Avg Speed: %.1f px/s"),
        info->totalDistance, info->avgVelocity);

    if (m_ui.status) {
        m_ui.status->SetWindowText(msg);
    }

    TRACE(_T("Shot completed - Total distance: %.1f pixels\n"), info->totalDistance);
}

void CXIMEASensorDiagDlg::OnContinuousCaptureProgress(int currentFrame, double elapsedSeconds, int state)
{
    if (state == 1) {  // CAPTURING
        CString status;
        status.Format(_T("Continuous capture: %d frames (%.1f sec)"),
            currentFrame, elapsedSeconds);

        if (m_ui.status && ::IsWindow(m_ui.status->GetSafeHwnd())) {
            m_ui.status->SetWindowText(status);
        }
    }
    else if (state == 3) {  // COMPLETED
        PostMessage(WM_CONTINUOUS_CAPTURE_COMPLETE);
    }
}

// ============================================================================
// Static Callback Wrappers
// ============================================================================
void CXIMEASensorDiagDlg::RealtimeDetectionCallback(const RealtimeDetectionResult* result,
    void* userContext)
{
    CXIMEASensorDiagDlg* pDlg = static_cast<CXIMEASensorDiagDlg*>(userContext);
    if (pDlg && result) {
        pDlg->OnRealtimeDetectionResult(result);
    }
}

void CXIMEASensorDiagDlg::BallStateChangeCallback(BallState newState, BallState oldState,
    const BallStateInfo* info, void* userContext)
{
    CXIMEASensorDiagDlg* pDlg = static_cast<CXIMEASensorDiagDlg*>(userContext);
    if (pDlg && info) {
        pDlg->OnBallStateChanged(newState, oldState, info);
    }
}

void CXIMEASensorDiagDlg::ShotCompletedCallback(const ShotCompletedInfo* info, void* userContext)
{
    CXIMEASensorDiagDlg* pDlg = static_cast<CXIMEASensorDiagDlg*>(userContext);
    if (pDlg && info) {
        pDlg->PostMessage(WM_UPDATE_SHOT_COMPLETED, 0, reinterpret_cast<LPARAM>(info));
    }
}

void CXIMEASensorDiagDlg::ContinuousCaptureProgressCallback(int currentFrame,
    double elapsedSeconds, int state)
{
    if (s_pThis) {
        s_pThis->OnContinuousCaptureProgress(currentFrame, elapsedSeconds, state);
    }
}

// ============================================================================
// Misc Handlers
// ============================================================================
HCURSOR CXIMEASensorDiagDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(AfxGetApp()->LoadStandardIcon(IDI_APPLICATION));
}

void CXIMEASensorDiagDlg::OnBnClickedButtonRefresh()
{
    UpdateDeviceList();
}

void CXIMEASensorDiagDlg::OnBnClickedButtonSettings()
{
    AfxMessageBox(_T("Settings dialog not implemented yet."));
}

void CXIMEASensorDiagDlg::OnCbnSelchangeComboDevices()
{
    // Device selection changed - no action needed unless streaming
}

// ============================================================================
// Parameter Edit Controls
// ============================================================================
void CXIMEASensorDiagDlg::OnEnChangeEditExposure()
{
    if (!m_ui.editExposure || !m_ui.sliderExposure) {
        return;
    }

    CString str;
    m_ui.editExposure->GetWindowText(str);
    int exposure = _ttoi(str);

    if (exposure >= CameraDefaults::MIN_EXPOSURE_US &&
        exposure <= CameraDefaults::MAX_EXPOSURE_US) {
        m_ui.sliderExposure->SetPos(exposure);

        if (m_isStreaming) {
            Camera_SetExposure(exposure);
        }
    }
}

void CXIMEASensorDiagDlg::OnEnChangeEditGain()
{
    if (!m_ui.editGain || !m_ui.sliderGain) {
        return;
    }

    CString str;
    m_ui.editGain->GetWindowText(str);
    float gain = static_cast<float>(_ttof(str));

    if (gain >= CameraDefaults::MIN_GAIN_DB &&
        gain <= CameraDefaults::MAX_GAIN_DB) {
        m_ui.sliderGain->SetPos(static_cast<int>(gain * 10));

        if (m_isStreaming) {
            Camera_SetGain(gain);
        }
    }
}

void CXIMEASensorDiagDlg::OnEnChangeEditFramerate()
{
    if (!m_ui.editFramerate || !m_ui.sliderFramerate) {
        return;
    }

    CString str;
    m_ui.editFramerate->GetWindowText(str);
    float fps = static_cast<float>(_ttof(str));

    if (fps >= CameraDefaults::MIN_FPS &&
        fps <= CameraDefaults::MAX_FPS) {
        m_ui.sliderFramerate->SetPos(static_cast<int>(fps * 10));

        if (m_isStreaming) {
            if (!Camera_SetFrameRate(fps)) {
                HandleFrameRateError(fps);
            }
        }
    }
}

void CXIMEASensorDiagDlg::HandleFrameRateError(float requestedFPS)
{
    int currentExposure = Camera_GetExposure();
    float maxPossibleFPS = 1000000.0f / currentExposure;

    CString msg;
    msg.Format(_T("Cannot set %.1f FPS with current exposure time (%d us).\n")
        _T("Maximum possible FPS: %.1f"),
        requestedFPS, currentExposure, maxPossibleFPS);

    MessageBox(msg, _T("FPS Limitation"), MB_OK | MB_ICONWARNING);

    // Reset to current value
    float currentFPS = Camera_GetFrameRate();
    m_ui.sliderFramerate->SetPos(static_cast<int>(currentFPS * 10));

    CString strFPS;
    strFPS.Format(_T("%.1f"), currentFPS);
    m_ui.editFramerate->SetWindowText(strFPS);
}

void CXIMEASensorDiagDlg::OnEnChangeEditROIMultiplier()
{
    if (!m_ui.editROIMultiplier || !m_ui.sliderROIMultiplier) {
        return;
    }

    CString str;
    m_ui.editROIMultiplier->GetWindowText(str);
    float multiplier = static_cast<float>(_ttof(str));

    if (multiplier >= 6.0f && multiplier <= 16.0f) {
        m_ui.sliderROIMultiplier->SetPos(static_cast<int>(multiplier * 10));

        if (Camera_IsDynamicROIEnabled()) {
            DynamicROIConfig config;
            if (Camera_GetDynamicROIConfig(&config)) {
                config.roiSizeMultiplier = multiplier;
                Camera_SetDynamicROIConfig(&config);
            }
        }
    }
}

// ============================================================================
// Slider Control
// ============================================================================
void CXIMEASensorDiagDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
    CSliderCtrl* pSlider = reinterpret_cast<CSliderCtrl*>(pScrollBar);

    // Handle ROI multiplier slider
    if (pSlider == m_ui.sliderROIMultiplier) {
        HandleROIMultiplierSlider();
        CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
        return;
    }

    // Handle parameter sliders only when streaming
    if (!m_isStreaming) {
        UpdateParameterEditsFromSliders(pSlider);
        CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
        return;
    }

    // Handle parameter sliders during streaming
    if (pSlider == m_ui.sliderExposure) {
        HandleExposureSlider();
    }
    else if (pSlider == m_ui.sliderGain) {
        HandleGainSlider();
    }
    else if (pSlider == m_ui.sliderFramerate) {
        HandleFramerateSlider();
    }

    CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
}

void CXIMEASensorDiagDlg::HandleROIMultiplierSlider()
{
    if (!m_ui.sliderROIMultiplier) {
        return;
    }

    float multiplier = m_ui.sliderROIMultiplier->GetPos() / 10.0f;

    // Update edit control
    if (m_ui.editROIMultiplier) {
        CString str;
        str.Format(_T("%.1f"), multiplier);
        m_ui.editROIMultiplier->SetWindowText(str);
    }

    // Apply if Dynamic ROI is enabled
    if (Camera_IsDynamicROIEnabled()) {
        DynamicROIConfig config;
        if (Camera_GetDynamicROIConfig(&config)) {
            config.roiSizeMultiplier = multiplier;
            Camera_SetDynamicROIConfig(&config);
        }
    }
}

void CXIMEASensorDiagDlg::UpdateParameterEditsFromSliders(CSliderCtrl* pSlider)
{
    if (pSlider == m_ui.sliderExposure && m_ui.editExposure) {
        int exposure = m_ui.sliderExposure->GetPos();
        CString str;
        str.Format(_T("%d"), exposure);
        m_ui.editExposure->SetWindowText(str);
    }
    else if (pSlider == m_ui.sliderGain && m_ui.editGain) {
        float gain = m_ui.sliderGain->GetPos() / 10.0f;
        CString str;
        str.Format(_T("%.1f"), gain);
        m_ui.editGain->SetWindowText(str);
    }
    else if (pSlider == m_ui.sliderFramerate && m_ui.editFramerate) {
        float fps = m_ui.sliderFramerate->GetPos() / 10.0f;
        CString str;
        str.Format(_T("%.1f"), fps);
        m_ui.editFramerate->SetWindowText(str);
    }
}

void CXIMEASensorDiagDlg::HandleExposureSlider()
{
    int exposure = m_ui.sliderExposure->GetPos();

    if (Camera_SetExposure(exposure)) {
        if (m_ui.editExposure) {
            CString str;
            str.Format(_T("%d"), exposure);
            m_ui.editExposure->SetWindowText(str);
        }
    }
    else {
        // Revert to current value
        int currentExposure = Camera_GetExposure();
        m_ui.sliderExposure->SetPos(currentExposure);
        if (m_ui.editExposure) {
            CString str;
            str.Format(_T("%d"), currentExposure);
            m_ui.editExposure->SetWindowText(str);
        }
    }
}

void CXIMEASensorDiagDlg::HandleGainSlider()
{
    float gain = m_ui.sliderGain->GetPos() / 10.0f;

    if (Camera_SetGain(gain)) {
        if (m_ui.editGain) {
            CString str;
            str.Format(_T("%.1f"), gain);
            m_ui.editGain->SetWindowText(str);
        }
    }
    else {
        // Revert to current value
        float currentGain = Camera_GetGain();
        m_ui.sliderGain->SetPos(static_cast<int>(currentGain * 10));
        if (m_ui.editGain) {
            CString str;
            str.Format(_T("%.1f"), currentGain);
            m_ui.editGain->SetWindowText(str);
        }
    }
}

void CXIMEASensorDiagDlg::HandleFramerateSlider()
{
    float fps = m_ui.sliderFramerate->GetPos() / 10.0f;

    if (Camera_SetFrameRate(fps)) {
        if (m_ui.editFramerate) {
            CString str;
            str.Format(_T("%.1f"), fps);
            m_ui.editFramerate->SetWindowText(str);
        }
    }
    else {
        HandleFrameRateError(fps);
    }
}