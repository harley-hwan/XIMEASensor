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
    m_usbErrorCount(0)
{
    InitializeCriticalSection(&m_detectionCriticalSection);
    m_cameraCallback = std::make_unique<CameraCallback>();
    memset(&m_lastDetectionResult, 0, sizeof(m_lastDetectionResult));
    s_pThis = this;
}

CXIMEASensorDiagDlg::~CXIMEASensorDiagDlg()
{
    DeleteCriticalSection(&m_detectionCriticalSection);
}

void CXIMEASensorDiagDlg::InitializeFrameBuffers()
{
    size_t maxBufferSize = 2048 * 2048;
    for (int i = 0; i < 3; i++) {
        if (m_frameBuffers[i].data == nullptr) {
            m_frameBuffers[i].data = new unsigned char[maxBufferSize];
            memset(m_frameBuffers[i].data, 0, maxBufferSize);
        }
        m_frameBuffers[i].ready = false;
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

        // Reallocate if necessary
        if (requiredSize > 1280 * 960) {
            delete[] writeBuffer.data;
            writeBuffer.data = new unsigned char[requiredSize];
        }

        if (frameInfo.data && requiredSize > 0) {
            memcpy(writeBuffer.data, frameInfo.data, requiredSize);
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
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_lastFPSUpdate).count();

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

    if (displayBuffer.ready && displayBuffer.data &&
        displayBuffer.width > 0 && displayBuffer.height > 0) {

        // Mark last draw time
        m_lastFrameDrawTime = std::chrono::steady_clock::now();

        size_t bmpInfoSize = sizeof(BITMAPINFOHEADER) + 256 * sizeof(RGBQUAD);
        BITMAPINFO* pBmpInfo = (BITMAPINFO*)malloc(bmpInfoSize);
        memset(pBmpInfo, 0, bmpInfoSize);

        pBmpInfo->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        pBmpInfo->bmiHeader.biWidth = displayBuffer.width;
        pBmpInfo->bmiHeader.biHeight = -displayBuffer.height;  // Top-down DIB
        pBmpInfo->bmiHeader.biPlanes = 1;
        pBmpInfo->bmiHeader.biBitCount = 8;
        pBmpInfo->bmiHeader.biCompression = BI_RGB;
        pBmpInfo->bmiHeader.biSizeImage = displayBuffer.width * displayBuffer.height;
        pBmpInfo->bmiHeader.biXPelsPerMeter = 0;
        pBmpInfo->bmiHeader.biYPelsPerMeter = 0;
        pBmpInfo->bmiHeader.biClrUsed = 256;
        pBmpInfo->bmiHeader.biClrImportant = 256;

        RGBQUAD* pPalette = (RGBQUAD*)(&pBmpInfo->bmiColors[0]);
        for (int i = 0; i < 256; i++) {
            pPalette[i].rgbBlue = i;
            pPalette[i].rgbGreen = i;
            pPalette[i].rgbRed = i;
            pPalette[i].rgbReserved = 0;
        }

        int oldStretchMode = SetStretchBltMode(dc.GetSafeHdc(), HALFTONE);
        SetBrushOrgEx(dc.GetSafeHdc(), 0, 0, NULL);

        int result = StretchDIBits(dc.GetSafeHdc(),
            0, 0, rect.Width(), rect.Height(),
            0, 0, displayBuffer.width, displayBuffer.height,
            displayBuffer.data,
            pBmpInfo,
            DIB_RGB_COLORS,
            SRCCOPY);

        if (result == GDI_ERROR) {
            DWORD error = GetLastError();
            TRACE(_T("StretchDIBits failed with error: %d\n"), error);
        }

        SetStretchBltMode(dc.GetSafeHdc(), oldStretchMode);

        free(pBmpInfo);

        // Draw detection overlay if enabled
        if (Camera_IsRealtimeDetectionEnabled()) {
            DrawDetectionOverlay(dc, rect);
        }
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
    EnterCriticalSection(&m_detectionCriticalSection);
    m_lastDetectionResult = *result;
    LeaveCriticalSection(&m_detectionCriticalSection);

    PostMessage(WM_UPDATE_BALL_DETECTION);
}

void CXIMEASensorDiagDlg::OnBnClickedCheckRealtimeDetection()
{
    if (!m_checkRealtimeDetection || !m_isStreaming) return;

    BOOL isChecked = (m_checkRealtimeDetection->GetCheck() == BST_CHECKED);

    if (isChecked) {
        Camera_SetRealtimeDetectionCallback(RealtimeDetectionCallback, this);

        if (Camera_EnableRealtimeDetection(true)) {
            m_lastDetectionStatsUpdate = std::chrono::steady_clock::now();

            if (m_staticBallPosition) {
                m_staticBallPosition->SetWindowText(_T("Detecting..."));
            }

            TRACE(_T("Real-time ball detection enabled\n"));
        }
        else {
            AfxMessageBox(_T("Failed to enable real-time detection!"));
            m_checkRealtimeDetection->SetCheck(BST_UNCHECKED);
        }
    }
    else {
        Camera_EnableRealtimeDetection(false);
        Camera_SetRealtimeDetectionCallback(nullptr, nullptr);

        if (m_staticBallPosition) {
            m_staticBallPosition->SetWindowText(_T("Not detected"));
        }
        if (m_staticBallInfo) {
            m_staticBallInfo->SetWindowText(_T("-"));
        }
        if (m_staticDetectionFPS) {
            m_staticDetectionFPS->SetWindowText(_T("0.0"));
        }

        TRACE(_T("Real-time ball detection disabled\n"));
    }
}

LRESULT CXIMEASensorDiagDlg::OnUpdateBallDetection(WPARAM wParam, LPARAM lParam)
{
    RealtimeDetectionResult result;
    EnterCriticalSection(&m_detectionCriticalSection);
    result = m_lastDetectionResult;
    LeaveCriticalSection(&m_detectionCriticalSection);

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
    EnterCriticalSection(&m_detectionCriticalSection);
    result = m_lastDetectionResult;
    LeaveCriticalSection(&m_detectionCriticalSection);

    if (result.ballFound && result.ballCount > 0) {
        // Get display buffer dimensions for proper scaling
        int displayIdx = m_displayBufferIndex.load();
        FrameBuffer& displayBuffer = m_frameBuffers[displayIdx];

        if (displayBuffer.width == 0 || displayBuffer.height == 0) {
            return;
        }

        float scaleX = (float)rect.Width() / displayBuffer.width;
        float scaleY = (float)rect.Height() / displayBuffer.height;

        CPen redPen(PS_SOLID, 3, RGB(255, 0, 0));
        CPen* pOldPen = dc.SelectObject(&redPen);
        CBrush* pOldBrush = (CBrush*)dc.SelectStockObject(NULL_BRUSH);

        for (int i = 0; i < result.ballCount; i++) {
            int x = (int)(result.balls[i].centerX * scaleX);
            int y = (int)(result.balls[i].centerY * scaleY);
            int radius = (int)(result.balls[i].radius * scaleX);

            dc.Ellipse(x - radius, y - radius, x + radius, y + radius);

            CPen yellowPen(PS_SOLID, 2, RGB(255, 255, 0));
            dc.SelectObject(&yellowPen);
            dc.MoveTo(x - 5, y);
            dc.LineTo(x + 5, y);
            dc.MoveTo(x, y - 5);
            dc.LineTo(x, y + 5);
            dc.SelectObject(&redPen);

            if (result.balls[i].confidence > 0.8f) {
                CString confStr;
                confStr.Format(_T("%.0f%%"), result.balls[i].confidence * 100);
                dc.SetBkMode(TRANSPARENT);
                dc.SetTextColor(RGB(0, 255, 0));
                dc.TextOut(x + radius + 5, y - 10, confStr);
            }
        }

        dc.SelectObject(pOldPen);
        dc.SelectObject(pOldBrush);
    }
}