// XIMEASensorDiagDlg.cpp : 구현 파일
//

#include "pch.h"
#include "framework.h"
#include "XIMEASensorDiag.h"
#include "XIMEASensorDiagDlg.h"
#include "afxdialogex.h"
#include <sstream>
#include <iomanip>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define TIMER_UPDATE_STATISTICS 1001
#define TIMER_FRAME_UPDATE 1002

CXIMEASensorDiagDlg* CXIMEASensorDiagDlg::s_pThis = nullptr;

CXIMEASensorDiagDlg::CXIMEASensorDiagDlg(CWnd* pParent)
    : CDialogEx(IDD_XIMEASENSORDIAG_DIALOG, pParent),
    m_isStreaming(false),
    m_frameCount(0),
    m_currentFPS(0.0),
    m_defaultExposureUs(4000),
    m_defaultGainDb(0.0f),
    m_defaultFps(60.0f),
    m_writeBufferIndex(0),
    m_readBufferIndex(1),
    m_displayBufferIndex(2),
    m_pendingFrameUpdates(0),
    m_usbErrorCount(0),
    m_isRecordingTrajectory(false),
    m_showTrajectory(false),
    m_trajectoryAlpha(255),
    m_lastScaleX(1.0f),
    m_lastScaleY(1.0f)
{
    m_cameraCallback = std::make_unique<CameraCallback>();
    memset(&m_lastDetectionResult, 0, sizeof(m_lastDetectionResult));
    s_pThis = this;
}

CXIMEASensorDiagDlg::~CXIMEASensorDiagDlg()
{
    // 소멸자는 이제 자동으로 모든 unique_ptr을 정리
}

void CXIMEASensorDiagDlg::InitializeFrameBuffers()
{
    size_t maxBufferSize = 2048 * 2048;
    for (auto& buffer : m_frameBuffers) {
        buffer.allocate(maxBufferSize);
        buffer.ready = false;
    }
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
    ON_BN_CLICKED(IDC_BUTTON_START, &CXIMEASensorDiagDlg::OnBnClickedButtonStart)
    ON_BN_CLICKED(IDC_BUTTON_STOP, &CXIMEASensorDiagDlg::OnBnClickedButtonStop)
    ON_BN_CLICKED(IDC_BUTTON_REFRESH, &CXIMEASensorDiagDlg::OnBnClickedButtonRefresh)
    ON_BN_CLICKED(IDC_BUTTON_SNAPSHOT, &CXIMEASensorDiagDlg::OnBnClickedButtonSnapshot)
    ON_BN_CLICKED(IDC_BUTTON_SETTINGS, &CXIMEASensorDiagDlg::OnBnClickedButtonSettings)
    ON_BN_CLICKED(IDC_CHECK_REALTIME_DETECTION, &CXIMEASensorDiagDlg::OnBnClickedCheckRealtimeDetection)
    ON_CBN_SELCHANGE(IDC_COMBO_DEVICES, &CXIMEASensorDiagDlg::OnCbnSelchangeComboDevices)
    ON_MESSAGE(WM_UPDATE_FRAME, &CXIMEASensorDiagDlg::OnUpdateFrame)
    ON_MESSAGE(WM_UPDATE_STATUS, &CXIMEASensorDiagDlg::OnUpdateStatus)
    ON_MESSAGE(WM_UPDATE_ERROR, &CXIMEASensorDiagDlg::OnUpdateError)
    ON_MESSAGE(WM_UPDATE_FPS, &CXIMEASensorDiagDlg::OnUpdateFPS)
    ON_MESSAGE(WM_CONTINUOUS_CAPTURE_COMPLETE, &CXIMEASensorDiagDlg::OnContinuousCaptureComplete)
    ON_MESSAGE(WM_UPDATE_BALL_DETECTION, &CXIMEASensorDiagDlg::OnUpdateBallDetection)
    ON_EN_CHANGE(IDC_EDIT_EXPOSURE, &CXIMEASensorDiagDlg::OnEnChangeEditExposure)
    ON_EN_CHANGE(IDC_EDIT_GAIN, &CXIMEASensorDiagDlg::OnEnChangeEditGain)
    ON_EN_CHANGE(IDC_EDIT_FRAMERATE, &CXIMEASensorDiagDlg::OnEnChangeEditFramerate)
    // 2025-07-30
    ON_BN_CLICKED(IDC_BUTTON_RESET_TRACKING, &CXIMEASensorDiagDlg::OnBnClickedButtonResetTracking)
    ON_BN_CLICKED(IDC_BUTTON_CONFIGURE_TRACKING, &CXIMEASensorDiagDlg::OnBnClickedButtonConfigureTracking)
    ON_MESSAGE(WM_UPDATE_BALL_STATE, &CXIMEASensorDiagDlg::OnUpdateBallState)
    // 2025-08-06
    ON_BN_CLICKED(IDC_CHECK_ENABLE_DYNAMIC_ROI, &CXIMEASensorDiagDlg::OnBnClickedCheckEnableDynamicROI)
    ON_BN_CLICKED(IDC_CHECK_SHOW_ROI_OVERLAY, &CXIMEASensorDiagDlg::OnBnClickedCheckShowROIOverlay)
    ON_BN_CLICKED(IDC_BUTTON_RESET_ROI, &CXIMEASensorDiagDlg::OnBnClickedButtonResetROI)
    ON_EN_CHANGE(IDC_EDIT_ROI_MULTIPLIER, &CXIMEASensorDiagDlg::OnEnChangeEditROIMultiplier)
    ON_MESSAGE(WM_UPDATE_DYNAMIC_ROI, &CXIMEASensorDiagDlg::OnUpdateDynamicROI)
    // 궤적 시각화
    ON_MESSAGE(WM_UPDATE_SHOT_COMPLETED, &CXIMEASensorDiagDlg::OnUpdateShotCompleted)
END_MESSAGE_MAP()

void CXIMEASensorDiagDlg::LoadDefaultSettings()
{
    Camera_GetDefaultSettings(&m_defaultExposureUs, &m_defaultGainDb, &m_defaultFps);
}

BOOL CXIMEASensorDiagDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    if (AllocConsole()) {
        FILE* fp;
        freopen_s(&fp, "CONOUT$", "w", stdout);
        freopen_s(&fp, "CONOUT$", "w", stderr);
        freopen_s(&fp, "CONIN$", "r", stdin);
    }

    m_checkContinuous = (CButton*)GetDlgItem(IDC_CHECK_CONTINUOUS);
    if (m_checkContinuous) {
        m_checkContinuous->SetCheck(BST_UNCHECKED);
    }

    m_pictureCtrl = (CStatic*)GetDlgItem(IDC_STATIC_VIDEO);
    m_btnStart = (CButton*)GetDlgItem(IDC_BUTTON_START);
    m_btnStop = (CButton*)GetDlgItem(IDC_BUTTON_STOP);
    m_btnSnapshot = (CButton*)GetDlgItem(IDC_BUTTON_SNAPSHOT);
    m_staticStatus = (CStatic*)GetDlgItem(IDC_STATIC_STATUS);
    m_staticFPS = (CStatic*)GetDlgItem(IDC_STATIC_FPS);
    m_sliderExposure = (CSliderCtrl*)GetDlgItem(IDC_SLIDER_EXPOSURE);
    m_sliderGain = (CSliderCtrl*)GetDlgItem(IDC_SLIDER_GAIN);
    m_sliderFramerate = (CSliderCtrl*)GetDlgItem(IDC_SLIDER_FRAMERATE);
    m_comboDevices = (CComboBox*)GetDlgItem(IDC_COMBO_DEVICES);

    m_editExposure = (CEdit*)GetDlgItem(IDC_EDIT_EXPOSURE);
    m_editGain = (CEdit*)GetDlgItem(IDC_EDIT_GAIN);
    m_editFramerate = (CEdit*)GetDlgItem(IDC_EDIT_FRAMERATE);

    m_checkRealtimeDetection = (CButton*)GetDlgItem(IDC_CHECK_REALTIME_DETECTION);
    if (m_checkRealtimeDetection) {
        m_checkRealtimeDetection->SetCheck(BST_UNCHECKED);
    }

    m_staticBallPosition = (CStatic*)GetDlgItem(IDC_STATIC_BALL_POSITION);
    m_staticBallInfo = (CStatic*)GetDlgItem(IDC_STATIC_BALL_INFO);
    m_staticDetectionFPS = (CStatic*)GetDlgItem(IDC_STATIC_DETECTION_FPS);

    // 2025-07-30: 볼 상태 관련 컨트롤 초기화
    m_staticBallState = (CStatic*)GetDlgItem(IDC_STATIC_BALL_STATE);
    m_staticStateTime = (CStatic*)GetDlgItem(IDC_STATIC_STATE_TIME);
    m_staticStableTime = (CStatic*)GetDlgItem(IDC_STATIC_STABLE_TIME);
    m_btnResetTracking = (CButton*)GetDlgItem(IDC_BUTTON_RESET_TRACKING);
    m_btnConfigureTracking = (CButton*)GetDlgItem(IDC_BUTTON_CONFIGURE_TRACKING);

    // 2025-08-06: Dynamic ROI 컨트롤 초기화
    m_checkEnableDynamicROI = (CButton*)GetDlgItem(IDC_CHECK_ENABLE_DYNAMIC_ROI);
    m_checkShowROIOverlay = (CButton*)GetDlgItem(IDC_CHECK_SHOW_ROI_OVERLAY);
    m_staticROIStatus = (CStatic*)GetDlgItem(IDC_STATIC_ROI_STATUS);
    m_staticROISize = (CStatic*)GetDlgItem(IDC_STATIC_ROI_SIZE);
    m_staticROIReduction = (CStatic*)GetDlgItem(IDC_STATIC_ROI_REDUCTION);
    m_sliderROIMultiplier = (CSliderCtrl*)GetDlgItem(IDC_SLIDER_ROI_MULTIPLIER);
    m_editROIMultiplier = (CEdit*)GetDlgItem(IDC_EDIT_ROI_MULTIPLIER);
    m_btnResetROI = (CButton*)GetDlgItem(IDC_BUTTON_RESET_ROI);

    if (m_staticBallState) { m_staticBallState->SetWindowText(_T("NOT DETECTED")); }
    if (m_btnResetTracking) { m_btnResetTracking->EnableWindow(FALSE); }
    if (m_btnConfigureTracking) { m_btnConfigureTracking->EnableWindow(TRUE); }

    // 2025-08-06
    if (m_checkEnableDynamicROI) {
        m_checkEnableDynamicROI->SetCheck(BST_UNCHECKED);
    }

    if (m_checkShowROIOverlay) {
        m_checkShowROIOverlay->SetCheck(BST_CHECKED);
    }

    if (m_sliderROIMultiplier) {
        m_sliderROIMultiplier->SetRange(60, 100); // 8.0 ~ 10.0으로 변경 (기존: 20, 80)
        m_sliderROIMultiplier->SetPos(80); // 8.0 default
        m_sliderROIMultiplier->SetTicFreq(5);  // 더 세밀한 조정을 위해 변경
    }

    if (m_editROIMultiplier) {
        m_editROIMultiplier->SetWindowText(_T("8.0")); // 기본값 8.0
    }

    if (!Camera_Initialize("./logs/XIMEASensor.log", 1)) {
        AfxMessageBox(_T("Failed to initialize camera system!"));
        return FALSE;
    }

    LoadDefaultSettings();

    if (m_sliderExposure) {
        m_sliderExposure->SetRange(CameraDefaults::MIN_EXPOSURE_US, CameraDefaults::MAX_EXPOSURE_US);
        m_sliderExposure->SetPos(m_defaultExposureUs);
        m_sliderExposure->SetTicFreq(10000);
    }

    if (m_sliderGain) {
        m_sliderGain->SetRange(static_cast<int>(CameraDefaults::MIN_GAIN_DB * 10),
            static_cast<int>(CameraDefaults::MAX_GAIN_DB * 10));
        m_sliderGain->SetPos(static_cast<int>(m_defaultGainDb * 10));
        m_sliderGain->SetTicFreq(30);
    }

    if (m_sliderFramerate) {
        m_sliderFramerate->SetRange(static_cast<int>(CameraDefaults::MIN_FPS * 10),
            static_cast<int>(CameraDefaults::MAX_FPS * 10));
        m_sliderFramerate->SetPos(static_cast<int>(m_defaultFps * 10));
        m_sliderFramerate->SetTicFreq(100);
    }

    if (m_editExposure) {
        CString strExposure;
        strExposure.Format(_T("%d"), m_defaultExposureUs);
        m_editExposure->SetWindowText(strExposure);
    }

    if (m_editGain) {
        CString strGain;
        strGain.Format(_T("%.1f"), m_defaultGainDb);
        m_editGain->SetWindowText(strGain);
    }

    if (m_editFramerate) {
        CString strFPS;
        strFPS.Format(_T("%.1f"), m_defaultFps);
        m_editFramerate->SetWindowText(strFPS);
    }

    m_cameraCallback->SetFrameCallback(
        [this](const FrameInfo& info) { OnFrameReceivedCallback(info); });

    m_cameraCallback->SetStateCallback(
        [this](CameraState newState, CameraState oldState) {
            OnStateChangedCallback(newState, oldState);
        });

    m_cameraCallback->SetErrorCallback(
        [this](CameraError error, const std::string& msg) {
            OnErrorCallback(error, msg);
        });

    m_cameraCallback->SetPropertyCallback(
        [this](const std::string& prop, const std::string& value) {
            OnPropertyChangedCallback(prop, value);
        });

    Camera_RegisterCallback(m_cameraCallback.get());

    // 퍼팅 궤적 시각화 초기화
    m_isRecordingTrajectory = false;
    m_showTrajectory = false;
    m_trajectoryAlpha = 255;
    m_lastScaleX = 1.0f;
    m_lastScaleY = 1.0f;

    // Shot completed 콜백 등록
    Camera_SetShotCompletedCallback(ShotCompletedCallback, this);

    InitializeFrameBuffers();

    UpdateDeviceList();
    UpdateUI(false);
    SetTimer(TIMER_UPDATE_STATISTICS, 1000, nullptr);
    SetTimer(TIMER_FRAME_UPDATE, 33, nullptr); // 30 FPS UI update

    return TRUE;
}

void CXIMEASensorDiagDlg::OnDestroy()
{
    CDialogEx::OnDestroy();

    // Shot completed 콜백 해제
    Camera_SetShotCompletedCallback(nullptr, nullptr);

    // 페이드 아웃 타이머 정지
    KillTimer(TIMER_TRAJECTORY_FADE);

    // 볼 상태 추적 비활성화
    if (Camera_IsBallStateTrackingEnabled()) {
        Camera_EnableBallStateTracking(false);
    }
    Camera_SetBallStateChangeCallback(nullptr, nullptr);

    if (Camera_IsRealtimeDetectionEnabled()) {
        Camera_EnableRealtimeDetection(false);
    }
    Camera_SetRealtimeDetectionCallback(nullptr, nullptr);

    Camera_SetContinuousCaptureProgressCallback(nullptr);
    s_pThis = nullptr;

    KillTimer(TIMER_UPDATE_STATISTICS);
    KillTimer(TIMER_FRAME_UPDATE);

    if (m_isStreaming) {
        Camera_Stop();
    }
    Camera_Close();
    Camera_UnregisterCallback(m_cameraCallback.get());
    Camera_Shutdown();
}

void CXIMEASensorDiagDlg::UpdateDeviceList()
{
    if (!m_comboDevices) return;

    m_comboDevices->ResetContent();

    int deviceCount = Camera_GetDeviceCount();
    if (deviceCount == 0) {
        m_comboDevices->AddString(_T("No devices found"));
        m_comboDevices->EnableWindow(FALSE);
        return;
    }

    for (int i = 0; i < deviceCount; i++) {
        char name[256] = { 0 };
        char serial[256] = { 0 };

        if (Camera_GetDeviceInfo(i, name, sizeof(name), serial, sizeof(serial))) {
            CString deviceStr;
            deviceStr.Format(_T("%d: %s (S/N: %s)"), i,
                CString(name), CString(serial));
            m_comboDevices->AddString(deviceStr);
        }
    }

    m_comboDevices->SetCurSel(0);
    m_comboDevices->EnableWindow(TRUE);
}

void CXIMEASensorDiagDlg::UpdateUI(bool isStreaming)
{
    m_btnStart->EnableWindow(!isStreaming);
    m_btnStop->EnableWindow(isStreaming);
    m_comboDevices->EnableWindow(!isStreaming);

    if (m_sliderExposure) {
        m_sliderExposure->EnableWindow(isStreaming);
    }

    if (m_sliderGain) {
        m_sliderGain->EnableWindow(isStreaming);
    }

    if (m_sliderFramerate) {
        m_sliderFramerate->EnableWindow(isStreaming);
    }
}

void CXIMEASensorDiagDlg::OnBnClickedButtonStart()
{
    int deviceIndex = m_comboDevices->GetCurSel();
    if (deviceIndex < 0) {
        AfxMessageBox(_T("Please select a device!"));
        return;
    }

    if (!Camera_Open(deviceIndex)) {
        AfxMessageBox(_T("Failed to open camera!"));
        return;
    }

    Camera_SetExposure(m_defaultExposureUs);
    Camera_SetGain(m_defaultGainDb);
    Camera_SetFrameRate(m_defaultFps);

    SyncSlidersWithCamera();

    if (!Camera_Start()) {
        AfxMessageBox(_T("Failed to start streaming!"));
        Camera_Close();
        return;
    }

    m_isStreaming = true;
    UpdateUI(true);

    m_frameCount = 0;
    m_lastFPSUpdate = std::chrono::steady_clock::now();
    m_lastFrameDrawTime = std::chrono::steady_clock::now();
    ResetUSBErrorCount();
}

void CXIMEASensorDiagDlg::SyncSlidersWithCamera()
{
    int currentExposure = Camera_GetExposure();
    float currentGain = Camera_GetGain();
    float currentFramerate = Camera_GetFrameRate();

    if (m_sliderExposure && currentExposure > 0) {
        m_sliderExposure->SetPos(currentExposure);
        if (m_editExposure) {
            CString strExposure;
            strExposure.Format(_T("%d"), currentExposure);
            m_editExposure->SetWindowText(strExposure);
        }
    }

    if (m_sliderGain) {
        m_sliderGain->SetPos((int)(currentGain * 10));
        if (m_editGain) {
            CString strGain;
            strGain.Format(_T("%.1f"), currentGain);
            m_editGain->SetWindowText(strGain);
        }
    }

    if (m_sliderFramerate && currentFramerate > 0) {
        m_sliderFramerate->SetPos((int)(currentFramerate * 10));
        if (m_editFramerate) {
            CString strFPS;
            strFPS.Format(_T("%.1f"), currentFramerate);
            m_editFramerate->SetWindowText(strFPS);
        }
    }
}

void CXIMEASensorDiagDlg::OnBnClickedButtonStop()
{
    if (Camera_IsRealtimeDetectionEnabled()) {
        Camera_EnableRealtimeDetection(false);
        Camera_SetRealtimeDetectionCallback(nullptr, nullptr);

        if (m_checkRealtimeDetection) {
            m_checkRealtimeDetection->SetCheck(BST_UNCHECKED);
        }
    }

    if (Camera_IsContinuousCapturing()) {
        Camera_StopContinuousCapture();

        if (m_btnSnapshot) m_btnSnapshot->EnableWindow(TRUE);
        if (m_checkContinuous) m_checkContinuous->EnableWindow(TRUE);
    }

    Camera_Stop();
    Camera_Close();

    m_isStreaming = false;
    UpdateUI(false);

    Invalidate();
}

void CXIMEASensorDiagDlg::OnBnClickedButtonRefresh()
{
    UpdateDeviceList();
}

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

    BOOL isContinuous = (m_checkContinuous && m_checkContinuous->GetCheck() == BST_CHECKED);

    if (isContinuous) {
        ContinuousCaptureConfig config;
        Camera_GetContinuousCaptureDefaults(&config);

        int result = MessageBox(_T("Select image format:\n\nPNG (Yes) - Lossless, larger files\nJPG (No) - Compressed, smaller files"),
            _T("Image Format"), MB_YESNOCANCEL | MB_ICONQUESTION);

        if (result == IDCANCEL) {
            return;
        }

        config.imageFormat = (result == IDYES) ? 0 : 1;
        Camera_SetContinuousCaptureDefaults(&config);

        Camera_SetContinuousCaptureProgressCallback(ContinuousCaptureProgressCallback);

        TRACE(_T("Starting continuous capture for %.1f seconds with ball detection\n"), config.durationSeconds);

        if (Camera_StartContinuousCaptureWithDefaults()) {
            if (m_btnSnapshot) m_btnSnapshot->EnableWindow(FALSE);
            if (m_checkContinuous) m_checkContinuous->EnableWindow(FALSE);

            if (m_staticStatus) {
                CString status;
                status.Format(_T("Continuous capture (%.1fs) with ball detection in progress..."),
                    config.durationSeconds);
                m_staticStatus->SetWindowText(status);
            }

            TRACE(_T("Continuous capture started successfully\n"));
        }
        else {
            AfxMessageBox(_T("Failed to start continuous capture!"));
            TRACE(_T("Failed to start continuous capture\n"));
        }
    }
    else
#endif
    {
        SnapshotDefaults defaults;
        Camera_GetSnapshotDefaults(&defaults);

        int result = MessageBox(_T("Select image format:\n\nPNG (Yes) - Lossless format\nJPG (No) - Compressed format"),
            _T("Image Format"), MB_YESNOCANCEL | MB_ICONQUESTION);

        if (result == IDCANCEL) {
            return;
        }

        defaults.format = (result == IDYES) ? 0 : 1;
        Camera_SetSnapshotDefaults(&defaults);

        SYSTEMTIME st;
        GetLocalTime(&st);
        CString filename;
        const char* extension = (defaults.format == 0) ? "png" : "jpg";
        filename.Format(_T("snapshot_%04d%02d%02d_%02d%02d%02d.%s"),
            st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond,
            CString(extension));

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
}

void CXIMEASensorDiagDlg::OnBnClickedButtonSettings()
{
    AfxMessageBox(_T("Settings dialog not implemented yet."));
}

void CXIMEASensorDiagDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
    CSliderCtrl* pSlider = (CSliderCtrl*)pScrollBar;

    // ROI Multiplier 슬라이더 처리 추가
    if (pSlider == m_sliderROIMultiplier) {
        float multiplier = m_sliderROIMultiplier->GetPos() / 10.0f;

        // Edit 컨트롤 업데이트
        if (m_editROIMultiplier) {
            CString str;
            str.Format(_T("%.1f"), multiplier);
            m_editROIMultiplier->SetWindowText(str);
        }

        // Dynamic ROI가 활성화되어 있으면 설정 적용
        if (Camera_IsDynamicROIEnabled()) {
            DynamicROIConfig config;
            if (Camera_GetDynamicROIConfig(&config)) {
                config.roiSizeMultiplier = multiplier;
                Camera_SetDynamicROIConfig(&config);
            }
        }

        CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
        return;
    }

    if (!m_isStreaming) {
        if (pSlider == m_sliderExposure && m_editExposure) {
            int exposure = m_sliderExposure->GetPos();
            CString str;
            str.Format(_T("%d"), exposure);
            m_editExposure->SetWindowText(str);
        }
        else if (pSlider == m_sliderGain && m_editGain) {
            float gain = m_sliderGain->GetPos() / 10.0f;
            CString str;
            str.Format(_T("%.1f"), gain);
            m_editGain->SetWindowText(str);
        }
        else if (pSlider == m_sliderFramerate && m_editFramerate) {
            float fps = m_sliderFramerate->GetPos() / 10.0f;
            CString str;
            str.Format(_T("%.1f"), fps);
            m_editFramerate->SetWindowText(str);
        }

        CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
        return;
    }

    // 기존 코드 유지 (exposure, gain, framerate 처리)
    if (pSlider == m_sliderExposure) {
        int exposure = m_sliderExposure->GetPos();

        if (Camera_SetExposure(exposure)) {
            if (m_editExposure) {
                CString str;
                str.Format(_T("%d"), exposure);
                m_editExposure->SetWindowText(str);
            }
        }
        else {
            int currentExposure = Camera_GetExposure();
            m_sliderExposure->SetPos(currentExposure);
            if (m_editExposure) {
                CString str;
                str.Format(_T("%d"), currentExposure);
                m_editExposure->SetWindowText(str);
            }
        }
    }
    else if (pSlider == m_sliderGain) {
        float gain = m_sliderGain->GetPos() / 10.0f;

        if (Camera_SetGain(gain)) {
            if (m_editGain) {
                CString str;
                str.Format(_T("%.1f"), gain);
                m_editGain->SetWindowText(str);
            }
        }
        else {
            float currentGain = Camera_GetGain();
            m_sliderGain->SetPos((int)(currentGain * 10));
            if (m_editGain) {
                CString str;
                str.Format(_T("%.1f"), currentGain);
                m_editGain->SetWindowText(str);
            }
        }
    }
    else if (pSlider == m_sliderFramerate) {
        float fps = m_sliderFramerate->GetPos() / 10.0f;

        if (Camera_SetFrameRate(fps)) {
            if (m_editFramerate) {
                CString str;
                str.Format(_T("%.1f"), fps);
                m_editFramerate->SetWindowText(str);
            }
        }
        else {
            int currentExposure = Camera_GetExposure();
            float maxPossibleFPS = 1000000.0f / currentExposure;

            CString msg;
            msg.Format(_T("Cannot set %.1f FPS with current exposure time (%d us).\nMaximum possible FPS: %.1f"),
                fps, currentExposure, maxPossibleFPS);
            MessageBox(msg, _T("FPS Limitation"), MB_OK | MB_ICONWARNING);

            float currentFPS = Camera_GetFrameRate();
            m_sliderFramerate->SetPos((int)(currentFPS * 10));
            if (m_editFramerate) {
                CString str;
                str.Format(_T("%.1f"), currentFPS);
                m_editFramerate->SetWindowText(str);
            }
        }
    }

    CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
}

void CXIMEASensorDiagDlg::OnCbnSelchangeComboDevices()
{
    //
}

void CXIMEASensorDiagDlg::OnTimer(UINT_PTR nIDEvent)
{
    if (nIDEvent == TIMER_UPDATE_STATISTICS && m_isStreaming) {
        unsigned long totalFrames, droppedFrames;
        double avgFPS, minFPS, maxFPS;

        if (Camera_GetStatistics(&totalFrames, &droppedFrames,
            &avgFPS, &minFPS, &maxFPS)) {
            CString str;
            str.Format(_T("Total: %lu, Dropped: %lu, Avg FPS: %.1f"),
                totalFrames, droppedFrames, avgFPS);

            if (m_staticStatus) {
                m_staticStatus->SetWindowText(str);
            }
        }
    }
    else if (nIDEvent == TIMER_FRAME_UPDATE) {
        // Redraw frame if there's a pending update
        if (m_pendingFrameUpdates > 0) {
            DrawFrame();
        }
    }
    else if (nIDEvent == TIMER_BALL_STATE_UPDATE) {
        // 볼 상태 표시 업데이트
        UpdateBallStateDisplay();
    }
    else if (nIDEvent == TIMER_DYNAMIC_ROI_UPDATE) {
        // Dynamic ROI 표시 업데이트
        UpdateDynamicROIDisplay();
    }
    else if (nIDEvent == TIMER_TRAJECTORY_FADE) {
        // 페이드 아웃 처리
        int currentAlpha = m_trajectoryAlpha.load();
        int fadeStep = 255 / FADE_STEPS;
        
        currentAlpha -= fadeStep;
        
        if (currentAlpha <= 0) {
            m_trajectoryAlpha = 0;
            m_showTrajectory = false;
            ClearTrajectory();
            KillTimer(TIMER_TRAJECTORY_FADE);
            
            TRACE(_T("Trajectory fade out completed\n"));
        } else {
            m_trajectoryAlpha = currentAlpha;
        }
        
        // 화면 갱신
        Invalidate(FALSE);
    }

    CDialogEx::OnTimer(nIDEvent);
}

bool CXIMEASensorDiagDlg::ShouldSkipFrame()
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_lastFrameDrawTime).count();

    // Skip frame if too many pending or too soon since last draw
    if (m_pendingFrameUpdates > MAX_PENDING_FRAMES ||
        elapsed < MIN_FRAME_INTERVAL_MS) {
        return true;
    }

    return false;
}

void CXIMEASensorDiagDlg::SwapBuffers()
{
    // Rotate buffer indices for triple buffering
    int oldWrite = m_writeBufferIndex.load();
    int oldRead = m_readBufferIndex.load();
    int oldDisplay = m_displayBufferIndex.load();

    m_displayBufferIndex = oldRead;
    m_readBufferIndex = oldWrite;
    m_writeBufferIndex = oldDisplay;
}

// 궤적 포인트 추가
void CXIMEASensorDiagDlg::AddTrajectoryPoint(float x, float y, float confidence)
{
    if (!m_isRecordingTrajectory) return;

    std::lock_guard<std::mutex> lock(m_trajectoryMutex);

    // 최대 포인트 수 제한 (메모리 관리)
    const size_t MAX_TRAJECTORY_POINTS = 1000;
    if (m_trajectoryPoints.size() >= MAX_TRAJECTORY_POINTS) {
        m_trajectoryPoints.pop_front();
    }

    m_trajectoryPoints.emplace_back(cv::Point2f(x, y), GetTickCount(), confidence);
}


// 궤적 그리기 메인 함수 - 수정 버전
void CXIMEASensorDiagDlg::DrawTrajectory(CDC& dc, const CRect& rect)
{
    if (!m_showTrajectory || m_trajectoryAlpha == 0) return;

    std::lock_guard<std::mutex> lock(m_trajectoryMutex);

    if (m_trajectoryPoints.size() < 2) return;

    // 화면 좌표로 변환
    std::vector<CPoint> screenPoints;
    screenPoints.reserve(m_trajectoryPoints.size());

    for (const auto& point : m_trajectoryPoints) {
        screenPoints.push_back(ConvertToScreenCoordinates(point.position, rect));
    }

    // 페이드 아웃을 위한 알파값 계산
    int alpha = m_trajectoryAlpha.load();

    // 직접 화면 DC에 그리기 (알파 블렌딩 없이)
    if (alpha == 255) {
        // 완전 불투명일 때는 직접 그리기
        if (m_trajectoryStyle.useGradient) {
            DrawTrajectoryWithGradient(dc, screenPoints);
        }
        else {
            DrawTrajectoryLine(dc, screenPoints);
        }

        if (m_trajectoryStyle.showPoints) {
            DrawTrajectoryPoints(dc, screenPoints);
        }
    }
    else {
        // 페이드 아웃 중일 때는 GDI+ 사용하여 반투명 그리기
        Graphics graphics(dc.GetSafeHdc());
        graphics.SetSmoothingMode(SmoothingModeAntiAlias);

        // 알파값이 적용된 펜 생성
        if (m_trajectoryStyle.useGradient) {
            DrawTrajectoryWithGradientGDIPlus(graphics, screenPoints, alpha);
        }
        else {
            Color lineColor(alpha, GetRValue(m_trajectoryStyle.startColor),
                GetGValue(m_trajectoryStyle.startColor),
                GetBValue(m_trajectoryStyle.startColor));
            Pen pen(lineColor, static_cast<REAL>(m_trajectoryStyle.lineWidth));

            // 선 그리기
            if (screenPoints.size() >= 2) {
                std::vector<Point> gdiPoints;
                for (const auto& pt : screenPoints) {
                    gdiPoints.push_back(Point(pt.x, pt.y));
                }
                graphics.DrawLines(&pen, gdiPoints.data(), static_cast<INT>(gdiPoints.size()));
            }
        }

        if (m_trajectoryStyle.showPoints) {
            DrawTrajectoryPointsGDIPlus(graphics, screenPoints, alpha);
        }
    }

    // 시작점과 끝점 표시
    if (screenPoints.size() > 0) {
        // 시작점 - 초록색 원
        CBrush startBrush(RGB(0, 255, 0));
        CBrush* pOldBrush = dc.SelectObject(&startBrush);
        dc.Ellipse(screenPoints.front().x - 8, screenPoints.front().y - 8,
            screenPoints.front().x + 8, screenPoints.front().y + 8);

        // 끝점 - 빨간색 원
        CBrush endBrush(RGB(255, 0, 0));
        dc.SelectObject(&endBrush);
        dc.Ellipse(screenPoints.back().x - 8, screenPoints.back().y - 8,
            screenPoints.back().x + 8, screenPoints.back().y + 8);

        dc.SelectObject(pOldBrush);

        // 궤적 정보 텍스트
        if (m_trajectoryAlpha > 128) {
            dc.SetBkMode(TRANSPARENT);
            dc.SetTextColor(RGB(255, 255, 0));

            CString info;
            info.Format(_T("Points: %zu"), m_trajectoryPoints.size());
            dc.TextOut(10, rect.bottom - 40, info);
        }
    }
}


// GDI+ 그라데이션 그리기
void CXIMEASensorDiagDlg::DrawTrajectoryWithGradientGDIPlus(Graphics& graphics,
    const std::vector<CPoint>& screenPoints,
    int alpha)
{
    if (screenPoints.size() < 2) return;

    for (size_t i = 1; i < screenPoints.size(); ++i) {
        float t = static_cast<float>(i) / (screenPoints.size() - 1);
        COLORREF color = InterpolateColor(m_trajectoryStyle.startColor,
            m_trajectoryStyle.endColor, t);

        Color lineColor(alpha, GetRValue(color), GetGValue(color), GetBValue(color));

        int lineWidth = m_trajectoryStyle.lineWidth;
        if (i > screenPoints.size() * 0.8) {
            lineWidth = std::max(1, lineWidth - 1);
        }

        Pen pen(lineColor, static_cast<REAL>(lineWidth));

        graphics.DrawLine(&pen,
            Point(screenPoints[i - 1].x, screenPoints[i - 1].y),
            Point(screenPoints[i].x, screenPoints[i].y));
    }
}

// GDI+ 포인트 그리기
void CXIMEASensorDiagDlg::DrawTrajectoryPointsGDIPlus(Graphics& graphics,
    const std::vector<CPoint>& screenPoints,
    int alpha)
{
    const int POINT_INTERVAL = std::max(1, static_cast<int>(screenPoints.size() / 20));

    for (size_t i = 0; i < screenPoints.size(); i += POINT_INTERVAL) {
        float t = static_cast<float>(i) / (screenPoints.size() - 1);
        COLORREF color = InterpolateColor(m_trajectoryStyle.startColor,
            m_trajectoryStyle.endColor, t);

        Color fillColor(alpha, GetRValue(color), GetGValue(color), GetBValue(color));
        SolidBrush brush(fillColor);

        int size = m_trajectoryStyle.pointSize;
        graphics.FillEllipse(&brush,
            screenPoints[i].x - size,
            screenPoints[i].y - size,
            size * 2, size * 2);
    }
}


// 단순 선 그리기
void CXIMEASensorDiagDlg::DrawTrajectoryLine(CDC& dc, const std::vector<CPoint>& screenPoints)
{
    if (screenPoints.size() < 2) return;

    CPen pen(PS_SOLID, m_trajectoryStyle.lineWidth, m_trajectoryStyle.startColor);
    CPen* pOldPen = dc.SelectObject(&pen);

    dc.MoveTo(screenPoints[0]);
    for (size_t i = 1; i < screenPoints.size(); ++i) {
        dc.LineTo(screenPoints[i]);
    }

    dc.SelectObject(pOldPen);
}

// 개별 포인트 그리기
void CXIMEASensorDiagDlg::DrawTrajectoryPoints(CDC& dc, const std::vector<CPoint>& screenPoints)
{
    // 일정 간격으로 포인트 표시
    const int POINT_INTERVAL = std::max(1, static_cast<int>(screenPoints.size() / 20));

    for (size_t i = 0; i < screenPoints.size(); i += POINT_INTERVAL) {
        float t = static_cast<float>(i) / (screenPoints.size() - 1);
        COLORREF color = InterpolateColor(m_trajectoryStyle.startColor,
            m_trajectoryStyle.endColor, t);

        CBrush brush(color);
        CBrush* pOldBrush = dc.SelectObject(&brush);

        int size = m_trajectoryStyle.pointSize;
        dc.Ellipse(screenPoints[i].x - size, screenPoints[i].y - size,
            screenPoints[i].x + size, screenPoints[i].y + size);

        dc.SelectObject(pOldBrush);
    }
}

void CXIMEASensorDiagDlg::DrawTrajectoryWithGradient(CDC& dc, const std::vector<CPoint>& screenPoints)
{
    if (screenPoints.size() < 2) return;

    // 각 세그먼트를 다른 색상으로 그리기
    for (size_t i = 1; i < screenPoints.size(); ++i) {
        float t = static_cast<float>(i) / (screenPoints.size() - 1);
        COLORREF color = InterpolateColor(m_trajectoryStyle.startColor,
            m_trajectoryStyle.endColor, t);

        // 선 두께도 변화시킬 수 있음
        int lineWidth = m_trajectoryStyle.lineWidth;
        if (i > screenPoints.size() * 0.8) {
            lineWidth = std::max(1, lineWidth - 1);
        }

        CPen pen(PS_SOLID, lineWidth, color);
        CPen* pOldPen = dc.SelectObject(&pen);

        dc.MoveTo(screenPoints[i - 1]);
        dc.LineTo(screenPoints[i]);

        dc.SelectObject(pOldPen);
    }
}

// 페이드 아웃 시작
void CXIMEASensorDiagDlg::StartTrajectoryFadeOut()
{
    m_trajectoryAlpha = 255;
    SetTimer(TIMER_TRAJECTORY_FADE, FADE_DURATION_MS / FADE_STEPS, nullptr);

    TRACE(_T("Started trajectory fade out\n"));
}

void CXIMEASensorDiagDlg::StopTrajectoryFadeOut()
{
    KillTimer(TIMER_TRAJECTORY_FADE);
    m_trajectoryAlpha = 255;
}

// 좌표 변환
CPoint CXIMEASensorDiagDlg::ConvertToScreenCoordinates(const cv::Point2f& point, const CRect& displayRect)
{
    // 현재 디스플레이 버퍼의 크기 가져오기
    int displayIdx = m_displayBufferIndex.load();
    FrameBuffer& displayBuffer = m_frameBuffers[displayIdx];

    if (displayBuffer.width == 0 || displayBuffer.height == 0) {
        return CPoint(0, 0);
    }

    float scaleX = static_cast<float>(displayRect.Width()) / displayBuffer.width;
    float scaleY = static_cast<float>(displayRect.Height()) / displayBuffer.height;

    int x = static_cast<int>(point.x * scaleX);
    int y = static_cast<int>(point.y * scaleY);

    return CPoint(x, y);
}

// 색상 보간
COLORREF CXIMEASensorDiagDlg::InterpolateColor(COLORREF color1, COLORREF color2, float t)
{
    t = std::max(0.0f, std::min(1.0f, t));

    int r1 = GetRValue(color1);
    int g1 = GetGValue(color1);
    int b1 = GetBValue(color1);

    int r2 = GetRValue(color2);
    int g2 = GetGValue(color2);
    int b2 = GetBValue(color2);

    int r = static_cast<int>(r1 + (r2 - r1) * t);
    int g = static_cast<int>(g1 + (g2 - g1) * t);
    int b = static_cast<int>(b1 + (b2 - b1) * t);

    return RGB(r, g, b);
}

void CXIMEASensorDiagDlg::OnShotCompleted(const ShotCompletedInfo* info)
{
    // 궤적 기록 중지
    StopTrajectoryRecording();

    // 페이드 아웃 시작
    StartTrajectoryFadeOut();

    // 상태 표시
    CString msg;
    msg.Format(_T("Shot Completed! Distance: %.1f px, Avg Speed: %.1f px/s"),
        info->totalDistance, info->avgVelocity);

    if (m_staticStatus) {
        m_staticStatus->SetWindowText(msg);
    }

    TRACE(_T("Shot completed - Total distance: %.1f pixels\n"), info->totalDistance);
}

void CXIMEASensorDiagDlg::OnFrameReceivedCallback(const FrameInfo& frameInfo)
{
    // Skip frame if we're falling behind
    if (ShouldSkipFrame()) {
        return;
    }

    // Get the write buffer
    int writeIdx = m_writeBufferIndex.load();
    FrameBuffer& writeBuffer = m_frameBuffers[writeIdx];

    // Only update if buffer is not being read
    if (!writeBuffer.ready.exchange(false)) {
        int requiredSize = frameInfo.width * frameInfo.height;

        // 재할당이 필요한 경우
        writeBuffer.reallocate(requiredSize);

        if (frameInfo.data && requiredSize > 0) {
            memcpy(writeBuffer.data.get(), frameInfo.data, requiredSize);
            writeBuffer.width = frameInfo.width;
            writeBuffer.height = frameInfo.height;
        }

        writeBuffer.ready = true;

        // Swap buffers
        SwapBuffers();

        // Post update message only if not too many pending
        if (m_pendingFrameUpdates.fetch_add(1) < MAX_PENDING_FRAMES) {
            PostMessage(WM_UPDATE_FRAME);
        }
        else {
            m_pendingFrameUpdates.fetch_sub(1);
        }
    }

    // Update FPS display
    m_frameCount++;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastFPSUpdate).count();

    if (elapsed >= 500) {  // Update every 500ms
        m_currentFPS = (m_frameCount * 1000.0) / elapsed;
        m_frameCount = 0;
        m_lastFPSUpdate = now;
        PostMessage(WM_UPDATE_FPS, 0, (LPARAM)(m_currentFPS * 10));
    }
}

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

    PostMessage(WM_UPDATE_STATUS, 0, (LPARAM)pMsg);
}

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

    // If too many errors, stop and restart
    if (m_usbErrorCount >= MAX_USB_ERRORS) {
        TRACE(_T("Too many USB errors. Attempting to restart camera...\n"));

        // Post stop and restart messages
        PostMessage(WM_COMMAND, MAKEWPARAM(IDC_BUTTON_STOP, BN_CLICKED), 0);

        // Set timer to restart after a delay
        SetTimer(3000, 2000, nullptr); // Timer ID 3000, 2 second delay
    }
}

void CXIMEASensorDiagDlg::ResetUSBErrorCount()
{
    m_usbErrorCount = 0;
    m_lastUSBError = std::chrono::steady_clock::now();
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

    if (errorMessage.find("Camera disconnected") != std::string::npos) {
        PostMessage(WM_COMMAND, MAKEWPARAM(IDC_BUTTON_STOP, BN_CLICKED), 0);
    }

    CString* pMsg = new CString(errorMessage.c_str());
    PostMessage(WM_UPDATE_ERROR, (WPARAM)error, (LPARAM)pMsg);
}

void CXIMEASensorDiagDlg::OnPropertyChangedCallback(const std::string& propertyName,
    const std::string& value)
{
    TRACE(_T("Property changed: %s = %s\n"),
        CString(propertyName.c_str()), CString(value.c_str()));
}

LRESULT CXIMEASensorDiagDlg::OnUpdateFrame(WPARAM wParam, LPARAM lParam)
{
    // Decrement pending count
    m_pendingFrameUpdates.fetch_sub(1);

    // Don't draw if minimized
    if (IsIconic()) {
        return 0;
    }

    DrawFrame();
    return 0;
}

LRESULT CXIMEASensorDiagDlg::OnUpdateStatus(WPARAM wParam, LPARAM lParam)
{
    CString* pMsg = (CString*)lParam;
    if (pMsg && m_staticStatus) {
        m_staticStatus->SetWindowText(*pMsg);
        delete pMsg;
    }
    return 0;
}

LRESULT CXIMEASensorDiagDlg::OnUpdateError(WPARAM wParam, LPARAM lParam)
{
    CString* pMsg = (CString*)lParam;
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

    if (m_staticFPS) {
        m_staticFPS->SetWindowText(str);
    }
    return 0;
}

void CXIMEASensorDiagDlg::DrawFrame()
{
    if (!m_pictureCtrl) return;

    CClientDC dc(m_pictureCtrl);
    CRect rect;
    m_pictureCtrl->GetClientRect(&rect);

    // Get display buffer
    int displayIdx = m_displayBufferIndex.load();
    FrameBuffer& displayBuffer = m_frameBuffers[displayIdx];

    if (!displayBuffer.ready || !displayBuffer.data ||
        displayBuffer.width <= 0 || displayBuffer.height <= 0) {
        return;
    }

    // Mark last draw time
    m_lastFrameDrawTime = std::chrono::steady_clock::now();

    // 그레이스케일 8비트 이미지를 위한 BITMAPINFO 구조체 생성
    constexpr int PALETTE_SIZE = 256;
    const size_t bmpInfoSize = sizeof(BITMAPINFOHEADER) + PALETTE_SIZE * sizeof(RGBQUAD);

    // 일반 new 사용 (예외 발생 가능)
    std::unique_ptr<uint8_t[]> bmpInfoBuffer;
    try {
        bmpInfoBuffer.reset(new uint8_t[bmpInfoSize]);
    }
    catch (const std::bad_alloc&) {
        TRACE(_T("Failed to allocate memory for BITMAPINFO\n"));
        return;
    }

    // BITMAPINFO 포인터로 캐스팅
    BITMAPINFO* pBmpInfo = reinterpret_cast<BITMAPINFO*>(bmpInfoBuffer.get());
    memset(pBmpInfo, 0, bmpInfoSize);

    // BITMAPINFOHEADER 설정
    BITMAPINFOHEADER& header = pBmpInfo->bmiHeader;
    header.biSize = sizeof(BITMAPINFOHEADER);
    header.biWidth = displayBuffer.width;
    header.biHeight = -displayBuffer.height;  // Top-down DIB (음수 = top-down)
    header.biPlanes = 1;
    header.biBitCount = 8;  // 8비트 그레이스케일
    header.biCompression = BI_RGB;
    header.biSizeImage = displayBuffer.width * displayBuffer.height;
    header.biXPelsPerMeter = 0;
    header.biYPelsPerMeter = 0;
    header.biClrUsed = PALETTE_SIZE;
    header.biClrImportant = PALETTE_SIZE;

    // 그레이스케일 팔레트 설정
    RGBQUAD* pPalette = pBmpInfo->bmiColors;
    for (int i = 0; i < PALETTE_SIZE; i++) {
        pPalette[i].rgbBlue = static_cast<BYTE>(i);
        pPalette[i].rgbGreen = static_cast<BYTE>(i);
        pPalette[i].rgbRed = static_cast<BYTE>(i);
        pPalette[i].rgbReserved = 0;
    }

    // 고품질 스트레칭 모드 설정
    HDC hdc = dc.GetSafeHdc();
    int oldStretchMode = SetStretchBltMode(hdc, HALFTONE);
    SetBrushOrgEx(hdc, 0, 0, NULL);

    // 이미지 그리기 - displayBuffer.data.get()으로 실제 포인터 얻기
    int result = StretchDIBits(
        hdc,
        0, 0, rect.Width(), rect.Height(),  // 대상 영역
        0, 0, displayBuffer.width, displayBuffer.height,  // 소스 영역
        displayBuffer.data.get(),  // unique_ptr에서 실제 포인터 얻기
        pBmpInfo,
        DIB_RGB_COLORS,
        SRCCOPY
    );

    if (result == GDI_ERROR) {
        DWORD error = GetLastError();
        TRACE(_T("StretchDIBits failed with error: %d\n"), error);
    }

    // 스트레칭 모드 복원
    SetStretchBltMode(hdc, oldStretchMode);

    // 궤적 그리기 (Detection overlay 전에)
    if (m_showTrajectory && m_trajectoryAlpha > 0) {
        DrawTrajectory(dc, rect);
    }

    // Detection overlay 그리기
    if (Camera_IsRealtimeDetectionEnabled()) {
        DrawDetectionOverlay(dc, rect);
    }

    // Dynamic ROI overlay 그리기
    if (Camera_IsDynamicROIEnabled()) {
        DrawDynamicROIOverlay(dc, rect);
    }
}
void CXIMEASensorDiagDlg::ShowError(const CString& message)
{
    // Avoid showing message box if it's a recoverable USB error
    if (message.Find(_T("Device not ready")) != -1 ||
        message.Find(_T("USB")) != -1) {
        TRACE(_T("Recoverable error: %s\n"), message.GetString());
        return;
    }

    MessageBox(message, _T("Camera Error"), MB_OK | MB_ICONERROR);
}

void CXIMEASensorDiagDlg::OnPaint()
{
    if (IsIconic()) {
        CPaintDC dc(this);
        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);
    }
    else {
        CDialogEx::OnPaint();
        DrawFrame();
    }
}

HCURSOR CXIMEASensorDiagDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(AfxGetApp()->LoadStandardIcon(IDI_APPLICATION));
}

void CXIMEASensorDiagDlg::ContinuousCaptureProgressCallback(int currentFrame, double elapsedSeconds, int state)
{
    if (s_pThis) {
        s_pThis->OnContinuousCaptureProgress(currentFrame, elapsedSeconds, state);
    }
}

void CXIMEASensorDiagDlg::OnContinuousCaptureProgress(int currentFrame, double elapsedSeconds, int state)
{
    if (state == 1) {  // CAPTURING
        CString status;
        status.Format(_T("Continuous capture: %d frames (%.1f sec)"), currentFrame, elapsedSeconds);

        if (m_staticStatus && ::IsWindow(m_staticStatus->GetSafeHwnd())) {
            m_staticStatus->SetWindowText(status);
        }
    }
    else if (state == 3) {  // COMPLETED
        PostMessage(WM_CONTINUOUS_CAPTURE_COMPLETE);
    }
}

LRESULT CXIMEASensorDiagDlg::OnContinuousCaptureComplete(WPARAM wParam, LPARAM lParam)
{
    int totalFrames = 0, savedFrames = 0, droppedFrames = 0;
    double duration = 0.0;
    char folderPath[256] = { 0 };

    bool success = Camera_GetContinuousCaptureResult(&totalFrames, &savedFrames,
        &droppedFrames, &duration,
        folderPath, sizeof(folderPath));

    if (m_btnSnapshot) m_btnSnapshot->EnableWindow(TRUE);
    if (m_checkContinuous) m_checkContinuous->EnableWindow(TRUE);

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

        BOOL isContinuous = (m_checkContinuous && m_checkContinuous->GetCheck() == BST_CHECKED);
        if (isContinuous) {
            int framesWithBalls = 0;
            int totalBallsDetected = 0;
            float averageConfidence = 0.0f;
            char detectionFolder[256] = { 0 };

            if (Camera_GetContinuousCaptureDetectionResult(&framesWithBalls, &totalBallsDetected,
                &averageConfidence, detectionFolder, sizeof(detectionFolder))) {
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
    }
    else {
        msg = _T("Continuous capture failed!");
    }

    AfxMessageBox(msg);

    if (m_staticStatus) {
        m_staticStatus->SetWindowText(_T("Capturing..."));
    }

    return 0;
}

void CXIMEASensorDiagDlg::OnEnChangeEditExposure()
{
    if (!m_editExposure || !m_sliderExposure) return;

    CString str;
    m_editExposure->GetWindowText(str);
    int exposure = _ttoi(str);

    if (exposure >= CameraDefaults::MIN_EXPOSURE_US && exposure <= CameraDefaults::MAX_EXPOSURE_US) {
        m_sliderExposure->SetPos(exposure);

        if (m_isStreaming) {
            Camera_SetExposure(exposure);
        }
    }
}

void CXIMEASensorDiagDlg::OnEnChangeEditGain()
{
    if (!m_editGain || !m_sliderGain) return;

    CString str;
    m_editGain->GetWindowText(str);
    float gain = (float)_ttof(str);

    if (gain >= CameraDefaults::MIN_GAIN_DB && gain <= CameraDefaults::MAX_GAIN_DB) {
        m_sliderGain->SetPos((int)(gain * 10));

        if (m_isStreaming) {
            Camera_SetGain(gain);
        }
    }
}

void CXIMEASensorDiagDlg::OnEnChangeEditFramerate()
{
    if (!m_editFramerate || !m_sliderFramerate) return;

    CString str;
    m_editFramerate->GetWindowText(str);
    float fps = (float)_ttof(str);

    if (fps >= CameraDefaults::MIN_FPS && fps <= CameraDefaults::MAX_FPS) {
        m_sliderFramerate->SetPos((int)(fps * 10));

        if (m_isStreaming) {
            if (!Camera_SetFrameRate(fps)) {
                int currentExposure = Camera_GetExposure();
                float maxPossibleFPS = 1000000.0f / currentExposure;

                CString msg;
                msg.Format(_T("Cannot set %.1f FPS with current exposure time (%d us).\nMaximum possible FPS: %.1f"),
                    fps, currentExposure, maxPossibleFPS);
                MessageBox(msg, _T("FPS Limitation"), MB_OK | MB_ICONWARNING);

                float currentFPS = Camera_GetFrameRate();
                m_sliderFramerate->SetPos((int)(currentFPS * 10));
                CString strFPS;
                strFPS.Format(_T("%.1f"), currentFPS);
                m_editFramerate->SetWindowText(strFPS);
            }
        }
    }
}

void CXIMEASensorDiagDlg::RealtimeDetectionCallback(const RealtimeDetectionResult* result, void* userContext)
{
    CXIMEASensorDiagDlg* pDlg = static_cast<CXIMEASensorDiagDlg*>(userContext);
    if (pDlg && result) {
        pDlg->OnRealtimeDetectionResult(result);
    }
}

void CXIMEASensorDiagDlg::OnRealtimeDetectionResult(const RealtimeDetectionResult* result)
{
    {
        std::lock_guard<std::mutex> lock(m_detectionMutex);
        m_lastDetectionResult = *result;
    }

    // 공이 움직이는 중이고 검출되었다면 궤적에 추가
    if (m_isRecordingTrajectory && result && result->ballFound && result->ballCount > 0) {
        BallState currentState = Camera_GetBallState();
        if (currentState == BallState::MOVING || currentState == BallState::STABILIZING) {
            AddTrajectoryPoint(result->balls[0].centerX,
                result->balls[0].centerY,
                result->balls[0].confidence);
        }
    }

    PostMessage(WM_UPDATE_BALL_DETECTION);
}

LRESULT CXIMEASensorDiagDlg::OnUpdateBallDetection(WPARAM wParam, LPARAM lParam)
{
    RealtimeDetectionResult result;
    {
        std::lock_guard<std::mutex> lock(m_detectionMutex);
        result = m_lastDetectionResult;
    }

    if (result.ballFound && result.ballCount > 0) {
        CString posStr;
        posStr.Format(_T("X: %d, Y: %d"),
            static_cast<int>(result.balls[0].centerX),
            static_cast<int>(result.balls[0].centerY));

        if (m_staticBallPosition) {
            m_staticBallPosition->SetWindowText(posStr);
        }

        CString infoStr;
        infoStr.Format(_T("Radius: %d px, Confidence: %.1f%%, Time: %.1f ms"),
            static_cast<int>(result.balls[0].radius),
            result.balls[0].confidence * 100.0f,
            result.detectionTimeMs);

        if (m_staticBallInfo) {
            m_staticBallInfo->SetWindowText(infoStr);
        }
    }
    else {
        if (m_staticBallPosition) {
            m_staticBallPosition->SetWindowText(_T("Not detected"));
        }
        if (m_staticBallInfo) {
            m_staticBallInfo->SetWindowText(_T("-"));
        }
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_lastDetectionStatsUpdate).count();

    if (elapsed >= 1000) {
        int processedFrames = 0;
        double avgProcessingTime = 0.0;
        double detectionFPS = 0.0;

        Camera_GetRealtimeDetectionStats(&processedFrames, &avgProcessingTime, &detectionFPS);

        if (m_staticDetectionFPS) {
            CString fpsStr;
            fpsStr.Format(_T("%.1f"), detectionFPS);
            m_staticDetectionFPS->SetWindowText(fpsStr);
        }

        m_lastDetectionStatsUpdate = now;
    }

    return 0;
}


void CXIMEASensorDiagDlg::DrawDetectionOverlay(CDC& dc, const CRect& rect)
{
    RealtimeDetectionResult result;
    {
        std::lock_guard<std::mutex> lock(m_detectionMutex);
        result = m_lastDetectionResult;
    }

    // 현재 볼 상태 가져오기
    BallState currentState = Camera_GetBallState();

    if (result.ballFound && result.ballCount > 0) {
        // Get display buffer dimensions for proper scaling
        int displayIdx = m_displayBufferIndex.load();
        FrameBuffer& displayBuffer = m_frameBuffers[displayIdx];

        if (displayBuffer.width == 0 || displayBuffer.height == 0) {
            return;
        }

        float scaleX = (float)rect.Width() / displayBuffer.width;
        float scaleY = (float)rect.Height() / displayBuffer.height;

        // 상태별 색상과 두께 설정
        COLORREF penColor = GetBallStateColor(currentState);
        int penWidth = (currentState == BallState::READY) ? 4 : 3;

        CPen pen(PS_SOLID, penWidth, penColor);
        CPen* pOldPen = dc.SelectObject(&pen);
        CBrush* pOldBrush = (CBrush*)dc.SelectStockObject(NULL_BRUSH);

        int x, y, radius;
        for (int i = 0; i < result.ballCount; i++) {
            x = (int)(result.balls[i].centerX * scaleX);
            y = (int)(result.balls[i].centerY * scaleY);
            radius = (int)(result.balls[i].radius * scaleX);

            // 원만 그리고 십자가는 제거
            dc.Ellipse(x - radius, y - radius, x + radius, y + radius);

            // 연주황색 십자가 표시 제거 (주석 처리)
            CPen yellowPen(PS_SOLID, 2, RGB(255, 200, 100));
            dc.SelectObject(&yellowPen);
            dc.MoveTo(x - 5, y);
            dc.LineTo(x + 5, y);
            dc.MoveTo(x, y - 5);
            dc.LineTo(x, y + 5);
            dc.SelectObject(&pen);


            // 신뢰도가 높을 때만 표시
            if (result.balls[i].confidence > 0.8f) {
                CString confStr;
                confStr.Format(_T("%.0f%%"), result.balls[i].confidence * 100);
                dc.SetBkMode(TRANSPARENT);
                dc.SetTextColor(RGB(0, 255, 0));
                dc.TextOut(x + radius + 5, y - 10, confStr);
            }
        }

        // READY 상태면 추가 표시
        if (currentState == BallState::READY) {
            dc.SetTextColor(RGB(0, 255, 0));
            dc.SetBkMode(TRANSPARENT);

            CFont font;
            font.CreateFont(16, 0, 0, 0, FW_BOLD, FALSE, FALSE, 0,
                ANSI_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, _T("Arial"));

            CFont* pOldFont = dc.SelectObject(&font);
            dc.TextOut(x - 20, y + radius + 10, _T("READY"));
            dc.SelectObject(pOldFont);
        }

        dc.SelectObject(pOldPen);
        dc.SelectObject(pOldBrush);
    }
}

// 볼 상태 변경 콜백 (static)
void CXIMEASensorDiagDlg::BallStateChangeCallback(BallState newState, BallState oldState,
    const BallStateInfo* info, void* userContext)
{
    CXIMEASensorDiagDlg* pDlg = static_cast<CXIMEASensorDiagDlg*>(userContext);
    if (pDlg && info) {
        pDlg->OnBallStateChanged(newState, oldState, info);
    }
}

// Shot completed 콜백
void CXIMEASensorDiagDlg::ShotCompletedCallback(const ShotCompletedInfo* info, void* userContext)
{
    CXIMEASensorDiagDlg* pDlg = static_cast<CXIMEASensorDiagDlg*>(userContext);
    if (pDlg && info) {
        pDlg->PostMessage(WM_UPDATE_SHOT_COMPLETED, 0, reinterpret_cast<LPARAM>(info));
    }
}

// 볼 상태 변경 핸들러
void CXIMEASensorDiagDlg::OnBallStateChanged(BallState newState, BallState oldState,
    const BallStateInfo* info)
{
    // UI 업데이트를 위해 메시지 포스트
    PostMessage(WM_UPDATE_BALL_STATE, static_cast<WPARAM>(newState),
        reinterpret_cast<LPARAM>(info));

    // READY 상태 전환 시 알림
    if (newState == BallState::READY && oldState != BallState::READY) {
        MessageBeep(MB_OK);
        TRACE(_T("Ball is READY!\n"));
    }
}

// 실시간 탐지 활성화 수정
void CXIMEASensorDiagDlg::OnBnClickedCheckRealtimeDetection()
{
    if (!m_checkRealtimeDetection || !m_isStreaming) return;

    BOOL isChecked = (m_checkRealtimeDetection->GetCheck() == BST_CHECKED);

    if (isChecked) {
        // 실시간 탐지 콜백 설정
        Camera_SetRealtimeDetectionCallback(RealtimeDetectionCallback, this);

        if (Camera_EnableRealtimeDetection(true)) {
            m_lastDetectionStatsUpdate = std::chrono::steady_clock::now();

            // 볼 상태 추적 활성화
            Camera_SetBallStateChangeCallback(BallStateChangeCallback, this);
            Camera_EnableBallStateTracking(true);

            // 볼 상태 업데이트 타이머 시작
            SetTimer(TIMER_BALL_STATE_UPDATE, 100, nullptr);  // 100ms 간격

            if (m_btnResetTracking) {
                m_btnResetTracking->EnableWindow(TRUE);
            }

            if (m_staticBallPosition) {
                m_staticBallPosition->SetWindowText(_T("Detecting..."));
            }
            if (m_staticBallState) {
                m_staticBallState->SetWindowText(_T("NOT DETECTED"));
            }

            TRACE(_T("Real-time ball detection and state tracking enabled\n"));
        }
        else {
            AfxMessageBox(_T("Failed to enable real-time detection!"));
            m_checkRealtimeDetection->SetCheck(BST_UNCHECKED);
        }
    }
    else {
        // 볼 상태 추적 비활성화
        Camera_EnableBallStateTracking(false);
        Camera_SetBallStateChangeCallback(nullptr, nullptr);

        Camera_EnableRealtimeDetection(false);
        Camera_SetRealtimeDetectionCallback(nullptr, nullptr);

        KillTimer(TIMER_BALL_STATE_UPDATE);

        if (m_btnResetTracking) {
            m_btnResetTracking->EnableWindow(FALSE);
        }

        if (m_staticBallPosition) {
            m_staticBallPosition->SetWindowText(_T("Not detected"));
        }
        if (m_staticBallInfo) {
            m_staticBallInfo->SetWindowText(_T("-"));
        }
        if (m_staticDetectionFPS) {
            m_staticDetectionFPS->SetWindowText(_T("0.0"));
        }
        if (m_staticBallState) {
            m_staticBallState->SetWindowText(_T("NOT DETECTED"));
        }
        if (m_staticStateTime) {
            m_staticStateTime->SetWindowText(_T("0 ms"));
        }
        if (m_staticStableTime) {
            m_staticStableTime->SetWindowText(_T("0 ms"));
        }

        TRACE(_T("Real-time ball detection and state tracking disabled\n"));
    }
}

// 볼 상태 표시 업데이트
void CXIMEASensorDiagDlg::UpdateBallStateDisplay()
{
    if (!Camera_IsBallStateTrackingEnabled()) return;

    BallStateInfo info;
    if (Camera_GetBallStateInfo(&info)) {
        // 현재 상태 표시
        if (m_staticBallState) {
            CString stateStr = GetBallStateDisplayString(info.currentState);

            // READY 상태에서 실패 카운터 정보 추가 (디버그용)
            if (info.currentState == BallState::READY) {
                // 필요시 실패 카운터 정보도 표시 가능
                // stateStr += _T(" (stable)");
            }

            m_staticBallState->SetWindowText(stateStr);
        }

        // 현재 상태 지속 시간
        int timeInState = Camera_GetTimeInCurrentState();
        if (m_staticStateTime) {
            CString timeStr;
            if (timeInState < 1000) {
                timeStr.Format(_T("%d ms"), timeInState);
            }
            else {
                timeStr.Format(_T("%.1f s"), timeInState / 1000.0f);
            }
            m_staticStateTime->SetWindowText(timeStr);
        }

        // 안정화 시간 (STABILIZING, READY, STOPPED 상태에서만)
        if (m_staticStableTime) {
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
                m_staticStableTime->SetWindowText(stableStr);
            }
            else {
                m_staticStableTime->SetWindowText(_T("0 ms"));
            }
        }
    }
}

// 볼 상태 업데이트 메시지 핸들러
LRESULT CXIMEASensorDiagDlg::OnUpdateBallState(WPARAM wParam, LPARAM lParam)
{
    BallState newState = static_cast<BallState>(wParam);

    // 현재 상태 가져오기 - Camera_GetBallState()는 현재 상태를 반환
    static BallState previousState = BallState::NOT_DETECTED;

    // READY에서 MOVING으로 전환 시 궤적 기록 시작
    if (previousState == BallState::READY && newState == BallState::MOVING) {
        StartTrajectoryRecording();
        TRACE(_T("Ball started moving - trajectory recording started\n"));
    }

    // 상태 저장
    previousState = newState;

    // 즉시 상태 업데이트
    UpdateBallStateDisplay();

    // 상태별 추가 동작
    switch (newState) {
    case BallState::READY:
        TRACE(_T("Ball is in READY state!\n"));
        break;
    case BallState::MOVING:
        TRACE(_T("Ball started moving\n"));
        break;
    }

    return 0;
}


// 볼 추적 리셋 버튼 핸들러
void CXIMEASensorDiagDlg::OnBnClickedButtonResetTracking()
{
    Camera_ResetBallStateTracking();
    UpdateBallStateDisplay();

    CString msg = _T("Ball state tracking has been reset.");
    SetDlgItemText(IDC_STATIC_STATUS, msg);

    TRACE(_T("Ball state tracking reset\n"));
}

// 볼 추적 설정 버튼 핸들러
void CXIMEASensorDiagDlg::OnBnClickedButtonConfigureTracking()
{
    // 현재 설정 가져오기
    BallStateConfig config;
    Camera_GetBallStateConfig(&config);

    // 간단한 설정 다이얼로그 (실제로는 별도 다이얼로그 클래스 생성 권장)
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

// 볼 상태 문자열 변환
CString CXIMEASensorDiagDlg::GetBallStateDisplayString(BallState state)
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

// 볼 상태별 색상
COLORREF CXIMEASensorDiagDlg::GetBallStateColor(BallState state)
{
    switch (state) {
    case BallState::NOT_DETECTED:
        return RGB(128, 128, 128);  // 회색
    case BallState::MOVING:
        return RGB(255, 165, 0);    // 주황색
    case BallState::STABILIZING:
        return RGB(255, 255, 0);    // 노란색
    case BallState::READY:
        return RGB(0, 255, 0);      // 초록색
    case BallState::STOPPED:
        return RGB(0, 128, 255);    // 파란색
    default:
        return RGB(255, 0, 0);      // 빨간색
    }
}


void CXIMEASensorDiagDlg::OnBnClickedCheckEnableDynamicROI()
{
    if (!m_checkEnableDynamicROI || !m_isStreaming) return;

    BOOL isChecked = (m_checkEnableDynamicROI->GetCheck() == BST_CHECKED);

    if (isChecked) {
        // 현재 설정 가져오기
        DynamicROIConfig config;
        Camera_GetDynamicROIConfig(&config);

        // UI에서 설정 읽기
        config.enabled = true;
        config.showROIOverlay = (m_checkShowROIOverlay &&
            m_checkShowROIOverlay->GetCheck() == BST_CHECKED);

        if (m_sliderROIMultiplier) {
            config.roiSizeMultiplier = m_sliderROIMultiplier->GetPos() / 10.0f;
        }

        // 설정 적용
        Camera_SetDynamicROIConfig(&config);
        Camera_EnableDynamicROI(true);

        // 타이머 시작
        SetTimer(TIMER_DYNAMIC_ROI_UPDATE, 100, nullptr);

        TRACE(_T("Dynamic ROI enabled\n"));
    }
    else {
        Camera_EnableDynamicROI(false);
        KillTimer(TIMER_DYNAMIC_ROI_UPDATE);

        UpdateDynamicROIDisplay();

        TRACE(_T("Dynamic ROI disabled\n"));
    }
}

void CXIMEASensorDiagDlg::OnBnClickedCheckShowROIOverlay()
{
    if (!m_checkShowROIOverlay) return;

    DynamicROIConfig config;
    if (Camera_GetDynamicROIConfig(&config)) {
        config.showROIOverlay = (m_checkShowROIOverlay->GetCheck() == BST_CHECKED);
        Camera_SetDynamicROIConfig(&config);
    }
}

void CXIMEASensorDiagDlg::OnBnClickedButtonResetROI()
{
    Camera_ResetDynamicROI();
    UpdateDynamicROIDisplay();

    TRACE(_T("Dynamic ROI reset\n"));
}

void CXIMEASensorDiagDlg::OnEnChangeEditROIMultiplier()
{
    if (!m_editROIMultiplier || !m_sliderROIMultiplier) return;

    CString str;
    m_editROIMultiplier->GetWindowText(str);
    float multiplier = (float)_ttof(str);

    if (multiplier >= 6.0f && multiplier <= 10.0f) {
        m_sliderROIMultiplier->SetPos((int)(multiplier * 10));

        if (Camera_IsDynamicROIEnabled()) {
            DynamicROIConfig config;
            if (Camera_GetDynamicROIConfig(&config)) {
                config.roiSizeMultiplier = multiplier;
                Camera_SetDynamicROIConfig(&config);
            }
        }
    }
}

void CXIMEASensorDiagDlg::UpdateDynamicROIDisplay()
{
    DynamicROIInfo info;
    if (Camera_GetDynamicROIInfo(&info)) {
        if (m_staticROIStatus) {
            m_staticROIStatus->SetWindowText(info.active ? _T("Active") : _T("Inactive"));
        }
        
        if (m_staticROISize) {
            if (info.active) {
                CString sizeStr;
                sizeStr.Format(_T("%dx%d"), info.width, info.height);
                m_staticROISize->SetWindowText(sizeStr);
            } else {
                m_staticROISize->SetWindowText(_T("-"));
            }
        }
        
        if (m_staticROIReduction) {
            if (info.active) {
                CString reductionStr;
                reductionStr.Format(_T("%.1f%%"), info.processingTimeReduction);
                m_staticROIReduction->SetWindowText(reductionStr);
            } else {
                m_staticROIReduction->SetWindowText(_T("0%"));
            }
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

    // Get display buffer dimensions for proper scaling
    int displayIdx = m_displayBufferIndex.load();
    FrameBuffer& displayBuffer = m_frameBuffers[displayIdx];

    if (displayBuffer.width == 0 || displayBuffer.height == 0) {
        return;
    }

    float scaleX = (float)rect.Width() / displayBuffer.width;
    float scaleY = (float)rect.Height() / displayBuffer.height;

    // Calculate ROI rectangle
    int x = (int)((info.centerX - info.width / 2) * scaleX);
    int y = (int)((info.centerY - info.height / 2) * scaleY);
    int w = (int)(info.width * scaleX);
    int h = (int)(info.height * scaleY);

    // Draw ROI boundary
    CPen pen(PS_DASH, 2, RGB(255, 128, 0)); // Orange dashed line
    CPen* pOldPen = dc.SelectObject(&pen);
    CBrush* pOldBrush = (CBrush*)dc.SelectStockObject(NULL_BRUSH);

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


LRESULT CXIMEASensorDiagDlg::OnUpdateDynamicROI(WPARAM wParam, LPARAM lParam)
{
    UpdateDynamicROIDisplay();
    return 0;
}

// Shot completed 메시지 핸들러
LRESULT CXIMEASensorDiagDlg::OnUpdateShotCompleted(WPARAM wParam, LPARAM lParam)
{
    const ShotCompletedInfo* info = reinterpret_cast<const ShotCompletedInfo*>(lParam);
    if (info) {
        OnShotCompleted(info);
    }
    return 0;
}

// 궤적 기록 시작
void CXIMEASensorDiagDlg::StartTrajectoryRecording()
{
    std::lock_guard<std::mutex> lock(m_trajectoryMutex);

    m_trajectoryPoints.clear();
    m_isRecordingTrajectory = true;
    m_showTrajectory = true;
    m_trajectoryAlpha = 255;

    // 페이드 아웃 타이머가 실행 중이면 중지
    KillTimer(TIMER_TRAJECTORY_FADE);

    TRACE(_T("Started trajectory recording\n"));
}

// 궤적 기록 중지
void CXIMEASensorDiagDlg::StopTrajectoryRecording()
{
    std::lock_guard<std::mutex> lock(m_trajectoryMutex);

    m_isRecordingTrajectory = false;

    TRACE(_T("Stopped trajectory recording - %zu points recorded\n"),
        m_trajectoryPoints.size());
}

// 궤적 지우기
void CXIMEASensorDiagDlg::ClearTrajectory()
{
    std::lock_guard<std::mutex> lock(m_trajectoryMutex);

    m_trajectoryPoints.clear();
    m_isRecordingTrajectory = false;
    m_showTrajectory = false;
    m_trajectoryAlpha = 255;

    KillTimer(TIMER_TRAJECTORY_FADE);
}