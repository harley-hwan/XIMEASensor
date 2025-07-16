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

CXIMEASensorDiagDlg* CXIMEASensorDiagDlg::s_pThis = nullptr;

CXIMEASensorDiagDlg::CXIMEASensorDiagDlg(CWnd* pParent)
    : CDialogEx(IDD_XIMEASENSORDIAG_DIALOG, pParent),
    m_isStreaming(false),
    m_pDisplayBuffer(nullptr),
    m_displayWidth(0),
    m_displayHeight(0),
    m_hasNewFrame(false),
    m_frameCount(0),
    m_currentFPS(0.0),
    m_defaultExposureUs(4000),  // Fallback values
    m_defaultGainDb(0.0f),
    m_defaultFps(60.0f)
{
    InitializeCriticalSection(&m_frameCriticalSection);
    m_cameraCallback = std::make_unique<CameraCallback>();
    s_pThis = this;
}

CXIMEASensorDiagDlg::~CXIMEASensorDiagDlg()
{
    if (m_pDisplayBuffer) {
        delete[] m_pDisplayBuffer;
    }
    DeleteCriticalSection(&m_frameCriticalSection);
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
    ON_CBN_SELCHANGE(IDC_COMBO_DEVICES, &CXIMEASensorDiagDlg::OnCbnSelchangeComboDevices)
    ON_MESSAGE(WM_UPDATE_FRAME, &CXIMEASensorDiagDlg::OnUpdateFrame)
    ON_MESSAGE(WM_UPDATE_STATUS, &CXIMEASensorDiagDlg::OnUpdateStatus)
    ON_MESSAGE(WM_UPDATE_ERROR, &CXIMEASensorDiagDlg::OnUpdateError)
    ON_MESSAGE(WM_UPDATE_FPS, &CXIMEASensorDiagDlg::OnUpdateFPS)
    ON_MESSAGE(WM_CONTINUOUS_CAPTURE_COMPLETE, &CXIMEASensorDiagDlg::OnContinuousCaptureComplete)
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

    if (!Camera_Initialize("./logs/XIMEASensor.log", 1)) {
        AfxMessageBox(_T("Failed to initialize camera system!"));
        return FALSE;
    }

    LoadDefaultSettings();

    if (m_sliderExposure) {
        m_sliderExposure->SetRange(CameraDefaults::MIN_EXPOSURE_US, CameraDefaults::MAX_EXPOSURE_US);
        m_sliderExposure->SetPos(m_defaultExposureUs);
        m_sliderExposure->SetTicFreq(10000);

        CString strExposure;
        strExposure.Format(_T("Exposure: %d us"), m_defaultExposureUs);
        GetDlgItem(IDC_STATIC_EXPOSURE)->SetWindowText(strExposure);
    }

    // gain slider
    if (m_sliderGain) {
        m_sliderGain->SetRange(static_cast<int>(CameraDefaults::MIN_GAIN_DB * 10),
            static_cast<int>(CameraDefaults::MAX_GAIN_DB * 10));
        m_sliderGain->SetPos(static_cast<int>(m_defaultGainDb * 10));
        m_sliderGain->SetTicFreq(30);

        CString strGain;
        strGain.Format(_T("Gain: %.1f dB"), m_defaultGainDb);
        GetDlgItem(IDC_STATIC_GAIN)->SetWindowText(strGain);
    }

    // framerate slider
    if (m_sliderFramerate) {
        m_sliderFramerate->SetRange(static_cast<int>(CameraDefaults::MIN_FPS * 10),
            static_cast<int>(CameraDefaults::MAX_FPS * 10));
        m_sliderFramerate->SetPos(static_cast<int>(m_defaultFps * 10));
        m_sliderFramerate->SetTicFreq(100);

        CString strFPS;
        strFPS.Format(_T("%.1f FPS"), m_defaultFps);
        GetDlgItem(IDC_STATIC_FRAMERATE)->SetWindowText(strFPS);
    }

    // Setup callbacks
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

    size_t maxBufferSize = 2048 * 2048;  // 4MP max
    m_pDisplayBuffer = new unsigned char[maxBufferSize];
    memset(m_pDisplayBuffer, 0, maxBufferSize);

    UpdateDeviceList();
    UpdateUI(false);
    SetTimer(TIMER_UPDATE_STATISTICS, 1000, nullptr);

    return TRUE;
}

void CXIMEASensorDiagDlg::OnDestroy()
{
    CDialogEx::OnDestroy();

    Camera_SetContinuousCaptureProgressCallback(nullptr);
    s_pThis = nullptr;

    KillTimer(TIMER_UPDATE_STATISTICS);

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
}


void CXIMEASensorDiagDlg::SyncSlidersWithCamera()
{
    // Sync exposure slider
    int currentExposure = Camera_GetExposure();
    float currentGain = Camera_GetGain();
    float currentFramerate = Camera_GetFrameRate();

    if (m_sliderExposure && currentExposure > 0) {
        m_sliderExposure->SetPos(currentExposure);

        CString strExposure;
        strExposure.Format(_T("Exposure: %d us"), currentExposure);
        GetDlgItem(IDC_STATIC_EXPOSURE)->SetWindowText(strExposure);
    }

    if (m_sliderGain) {
        m_sliderGain->SetPos((int)(currentGain * 10));

        CString strGain;
        strGain.Format(_T("Gain: %.1f dB"), currentGain);
        GetDlgItem(IDC_STATIC_GAIN)->SetWindowText(strGain);
    }

    if (m_sliderFramerate && currentFramerate > 0) {
        m_sliderFramerate->SetPos((int)(currentFramerate * 10));

        CString strFPS;
        strFPS.Format(_T("%.1f FPS"), currentFramerate);
        GetDlgItem(IDC_STATIC_FRAMERATE)->SetWindowText(strFPS);
    }
}


void CXIMEASensorDiagDlg::OnBnClickedButtonStop()
{
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

    if (Camera_IsContinuousCapturing()) {
        AfxMessageBox(_T("Already in continuous capture mode!"));
        return;
    }

    BOOL isContinuous = (m_checkContinuous && m_checkContinuous->GetCheck() == BST_CHECKED);

    if (isContinuous) {
        // Continuous capture mode with ball detection
        ContinuousCaptureDefaults defaults;
        Camera_GetContinuousCaptureDefaults(&defaults);

        int result = MessageBox(_T("Select image format:\n\nPNG (Yes) - Lossless, larger files\nJPG (No) - Compressed, smaller files"),
            _T("Image Format"), MB_YESNOCANCEL | MB_ICONQUESTION);

        if (result == IDCANCEL) {
            return;
        }

        defaults.format = (result == IDYES) ? 0 : 1;

        // Ball detection 설정 추가
        defaults.enableBallDetection = true;  // Ball detection 활성화
        defaults.saveOriginalImages = true;
        defaults.saveDetectionImages = true;

        // 수정된 defaults를 설정
        Camera_SetContinuousCaptureDefaults(&defaults);

        if (defaults.enableBallDetection) {
            MessageBox(_T("Ball detection will be performed on captured frames.\n\n"
                "Results will be saved in:\n"
                "- original/ : Original captured frames\n"
                "- detection/ : Frames with detection results\n\n"
                "Detection results will be included in metadata."),
                _T("Ball Detection Enabled"), MB_OK | MB_ICONINFORMATION);
        }

        Camera_SetContinuousCaptureProgressCallback(ContinuousCaptureProgressCallback);

        // Start continuous capture with defaults (이제 ball detection 설정이 포함됨)
        if (Camera_StartContinuousCaptureWithDefaults()) {
            if (m_btnSnapshot) m_btnSnapshot->EnableWindow(FALSE);
            if (m_checkContinuous) m_checkContinuous->EnableWindow(FALSE);
            if (m_staticStatus) m_staticStatus->SetWindowText(_T("Continuous capture with ball detection in progress..."));
        }
        else {
            AfxMessageBox(_T("Failed to start continuous capture!"));
        }
    }
    else {
        // Single snapshot code remains the same
        SnapshotDefaults defaults;
        Camera_GetSnapshotDefaults(&defaults);

        int result = MessageBox(_T("PNG (Yes)\nJPG (No)"),
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
        if (pSlider == m_sliderExposure) {
            int exposure = m_sliderExposure->GetPos();
            CString str;
            str.Format(_T("Exposure: %d us"), exposure);
            GetDlgItem(IDC_STATIC_EXPOSURE)->SetWindowText(str);
        }
        else if (pSlider == m_sliderGain) {
            float gain = m_sliderGain->GetPos() / 10.0f;
            CString str;
            str.Format(_T("Gain: %.1f dB"), gain);
            GetDlgItem(IDC_STATIC_GAIN)->SetWindowText(str);
        }
        else if (pSlider == m_sliderFramerate) {
            float fps = m_sliderFramerate->GetPos() / 10.0f;
            CString str;
            str.Format(_T("%.1f FPS"), fps);
            GetDlgItem(IDC_STATIC_FRAMERATE)->SetWindowText(str);
        }

        CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
        return;
    }

    // Apply settings to camera when streaming
    if (pSlider == m_sliderExposure) {
        int exposure = m_sliderExposure->GetPos();

        if (Camera_SetExposure(exposure)) {
            CString str;
            str.Format(_T("Exposure: %d us"), exposure);
            GetDlgItem(IDC_STATIC_EXPOSURE)->SetWindowText(str);
        }
        else {
            // Revert to actual value if failed
            int currentExposure = Camera_GetExposure();
            m_sliderExposure->SetPos(currentExposure);

            CString str;
            str.Format(_T("Exposure: %d us"), currentExposure);
            GetDlgItem(IDC_STATIC_EXPOSURE)->SetWindowText(str);
        }
    }
    else if (pSlider == m_sliderGain) {
        float gain = m_sliderGain->GetPos() / 10.0f;

        if (Camera_SetGain(gain)) {
            CString str;
            str.Format(_T("Gain: %.1f dB"), gain);
            GetDlgItem(IDC_STATIC_GAIN)->SetWindowText(str);
        }
        else {
            // Revert to actual value if failed
            float currentGain = Camera_GetGain();
            m_sliderGain->SetPos((int)(currentGain * 10));

            CString str;
            str.Format(_T("Gain: %.1f dB"), currentGain);
            GetDlgItem(IDC_STATIC_GAIN)->SetWindowText(str);
        }
    }
    else if (pSlider == m_sliderFramerate) {
        float fps = m_sliderFramerate->GetPos() / 10.0f;

        if (Camera_SetFrameRate(fps)) {
            CString str;
            str.Format(_T("%.1f FPS"), fps);
            GetDlgItem(IDC_STATIC_FRAMERATE)->SetWindowText(str);
        }
        else {
            // exposure time limit
            int currentExposure = Camera_GetExposure();
            float maxPossibleFPS = 1000000.0f / currentExposure;

            CString msg;
            msg.Format(_T("Cannot set %.1f FPS with current exposure time (%d us).\nMaximum possible FPS: %.1f"),
                fps, currentExposure, maxPossibleFPS);
            MessageBox(msg, _T("FPS Limitation"), MB_OK | MB_ICONWARNING);

            // Revert to actual value
            float currentFPS = Camera_GetFrameRate();
            m_sliderFramerate->SetPos((int)(currentFPS * 10));

            CString str;
            str.Format(_T("%.1f FPS"), currentFPS);
            GetDlgItem(IDC_STATIC_FRAMERATE)->SetWindowText(str);
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

    CDialogEx::OnTimer(nIDEvent);
}

void CXIMEASensorDiagDlg::OnFrameReceivedCallback(const FrameInfo& frameInfo)
{
    EnterCriticalSection(&m_frameCriticalSection);

    int requiredSize = frameInfo.width * frameInfo.height;
    if (requiredSize > 1280 * 1024) {
        delete[] m_pDisplayBuffer;
        m_pDisplayBuffer = new unsigned char[requiredSize];
    }

    if (frameInfo.data && requiredSize > 0) {
        memcpy(m_pDisplayBuffer, frameInfo.data, requiredSize);
        m_displayWidth = frameInfo.width;
        m_displayHeight = frameInfo.height;
        m_hasNewFrame = true;
    }

    LeaveCriticalSection(&m_frameCriticalSection);

    PostMessage(WM_UPDATE_FRAME);

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

void CXIMEASensorDiagDlg::OnErrorCallback(CameraError error, const std::string& errorMessage)
{
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
    str.Format(_T("%.1f"), fps);    // FPS: %.1f

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

    EnterCriticalSection(&m_frameCriticalSection);

    if (m_pDisplayBuffer && m_displayWidth > 0 && m_displayHeight > 0) {

        size_t bmpInfoSize = sizeof(BITMAPINFOHEADER) + 256 * sizeof(RGBQUAD);
        BITMAPINFO* pBmpInfo = (BITMAPINFO*)malloc(bmpInfoSize);
        memset(pBmpInfo, 0, bmpInfoSize);

        pBmpInfo->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        pBmpInfo->bmiHeader.biWidth = m_displayWidth;
        pBmpInfo->bmiHeader.biHeight = -m_displayHeight;  // Top-down DIB
        pBmpInfo->bmiHeader.biPlanes = 1;
        pBmpInfo->bmiHeader.biBitCount = 8;
        pBmpInfo->bmiHeader.biCompression = BI_RGB;
        pBmpInfo->bmiHeader.biSizeImage = m_displayWidth * m_displayHeight;
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
            0, 0, m_displayWidth, m_displayHeight,
            m_pDisplayBuffer,
            pBmpInfo,
            DIB_RGB_COLORS,
            SRCCOPY);

        if (result == GDI_ERROR) {
            DWORD error = GetLastError();
            TRACE(_T("StretchDIBits failed with error: %d\n"), error);
        }

        SetStretchBltMode(dc.GetSafeHdc(), oldStretchMode);

        free(pBmpInfo);
    }

    LeaveCriticalSection(&m_frameCriticalSection);
}


void CXIMEASensorDiagDlg::ShowError(const CString& message)
{
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

        if (m_hasNewFrame) {
            DrawFrame();
        }
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
    // State: 0=IDLE, 1=CAPTURING, 2=STOPPING, 3=COMPLETED, 4=ERROR

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

        // Ball detection 결과가 있다면 추가로 표시
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