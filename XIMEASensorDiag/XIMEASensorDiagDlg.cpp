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
    m_currentFPS(0.0)
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
    m_staticStatus = (CStatic*)GetDlgItem(IDC_STATIC_STATUS);
    m_staticFPS = (CStatic*)GetDlgItem(IDC_STATIC_FPS);
    m_sliderExposure = (CSliderCtrl*)GetDlgItem(IDC_SLIDER_EXPOSURE);
    m_sliderGain = (CSliderCtrl*)GetDlgItem(IDC_SLIDER_GAIN);
    m_comboDevices = (CComboBox*)GetDlgItem(IDC_COMBO_DEVICES);

    if (m_sliderExposure) {
        m_sliderExposure->SetRange(10, 100000);  // 10us ~ 100ms
        m_sliderExposure->SetPos(4000);  // 4ms
        m_sliderExposure->SetTicFreq(10000);
    }

    if (m_sliderGain) {
        m_sliderGain->SetRange(0, 240);  // 0.0 ~ 24.0 dB (x10)
        m_sliderGain->SetPos(0);
        m_sliderGain->SetTicFreq(30);
    }

    if (!Camera_Initialize("./logs/XIMEASensor.log", 1)) {
        AfxMessageBox(_T("카메라 시스템 초기화 실패!"));
        return FALSE;
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

    // register callback
    Camera_RegisterCallback(m_cameraCallback.get());

    // PYTHON1300 최대 1280x1024, 여유있게.
    size_t maxBufferSize = 2048 * 2048;  // 4MP
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
        m_comboDevices->AddString(_T("디바이스 없음"));
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
}

void CXIMEASensorDiagDlg::OnBnClickedButtonStart()
{
    int deviceIndex = m_comboDevices->GetCurSel();
    if (deviceIndex < 0) {
        AfxMessageBox(_T("디바이스를 선택하세요!"));
        return;
    }

    if (!Camera_Open(deviceIndex)) {
        AfxMessageBox(_T("카메라를 열 수 없습니다!"));
        return;
    }

    // default
    Camera_SetExposure(4000);  // 4ms
    Camera_SetGain(0.0f);      // 0dB

    // 이전 슬라이더 위치값 적용 ?
    // if (m_sliderExposure) {
    //     Camera_SetExposure(m_sliderExposure->GetPos());
    // }
    // if (m_sliderGain) {
    //     Camera_SetGain(m_sliderGain->GetPos() / 10.0f);
    // }

    SyncSlidersWithCamera();

    if (!Camera_Start()) {
        AfxMessageBox(_T("카메라 스트리밍을 시작할 수 없습니다!"));
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
    int currentExposure = Camera_GetExposure();
    float currentGain = Camera_GetGain();

    if (m_sliderExposure && currentExposure > 0) {
        m_sliderExposure->SetPos(currentExposure);

        CString strExposure;
        strExposure.Format(_T("노출: %d us"), currentExposure);
        GetDlgItem(IDC_STATIC_EXPOSURE)->SetWindowText(strExposure);
    }

    if (m_sliderGain) {
        m_sliderGain->SetPos((int)(currentGain * 10));

        CString strGain;
        strGain.Format(_T("게인: %.1f dB"), currentGain);
        GetDlgItem(IDC_STATIC_GAIN)->SetWindowText(strGain);
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
        AfxMessageBox(_T("카메라가 실행 중이 아닙니다!"));
        return;
    }

    // Check if already in continuous capture
    if (Camera_IsContinuousCapturing()) {
        AfxMessageBox(_T("이미 연속 촬영 중입니다!"));
        return;
    }

    // Check continuous capture checkbox
    BOOL isContinuous = (m_checkContinuous && m_checkContinuous->GetCheck() == BST_CHECKED);

    if (isContinuous) {
        // Continuous capture settings
        double duration = 1.0;
        int format = 0;         // PNG
        int quality = 90;
        bool asyncSave = true;

        // Ask user for format
        int result = MessageBox(_T("PNG 형식으로 저장하시겠습니까?\n(아니오를 선택하면 JPG로 저장됩니다.)"),
            _T("연속 촬영 형식 선택"), MB_YESNOCANCEL | MB_ICONQUESTION);

        if (result == IDCANCEL) {
            return;
        }

        format = (result == IDYES) ? 0 : 1;

        // Configure continuous capture
        Camera_SetContinuousCaptureConfig(duration, format, quality, asyncSave);

        // Set progress callback
        Camera_SetContinuousCaptureProgressCallback(ContinuousCaptureProgressCallback);

        // Start continuous capture
        if (Camera_StartContinuousCapture()) {
            // Update UI
            if (m_btnSnapshot) m_btnSnapshot->EnableWindow(FALSE);
            if (m_checkContinuous) m_checkContinuous->EnableWindow(FALSE);
            if (m_staticStatus) m_staticStatus->SetWindowText(_T("연속 촬영 중..."));
        }
        else {
            AfxMessageBox(_T("연속 촬영을 시작할 수 없습니다!"));
        }
    }
    else {
        // Single snapshot
        SYSTEMTIME st;
        GetLocalTime(&st);

        int result = MessageBox(_T("PNG 형식으로 저장하시겠습니까?\n(아니오를 선택하면 JPG로 저장됩니다.)"),
            _T("이미지 형식 선택"), MB_YESNOCANCEL | MB_ICONQUESTION);

        if (result == IDCANCEL) {
            return;
        }

        CString filename;
        int format = (result == IDYES) ? 0 : 1;

        if (format == 0) {
            filename.Format(_T("snapshot_%04d%02d%02d_%02d%02d%02d.png"),
                st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
        }
        else {
            filename.Format(_T("snapshot_%04d%02d%02d_%02d%02d%02d.jpg"),
                st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
        }

        CStringA filenameA(filename);
        bool saveSuccess = Camera_SaveSnapshot(filenameA.GetString(), format, 90);

        if (saveSuccess) {
            int width = 0, height = 0;
            Camera_GetROI(nullptr, nullptr, &width, &height);

            CString msg;
            msg.Format(_T("스냅샷 저장됨: %s\n크기: %dx%d"),
                filename.GetString(), width, height);
            AfxMessageBox(msg);
        }
        else {
            AfxMessageBox(_T("이미지 파일을 저장할 수 없습니다!"));
        }
    }
}


void CXIMEASensorDiagDlg::OnBnClickedButtonSettings()
{
    AfxMessageBox(_T("Coming soonnnnn~."));
}

void CXIMEASensorDiagDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
    if (!m_isStreaming) {
        CSliderCtrl* pSlider = (CSliderCtrl*)pScrollBar;

        if (pSlider == m_sliderExposure) {
            int exposure = m_sliderExposure->GetPos();
            CString str;
            str.Format(_T("노출: %d us"), exposure);
            GetDlgItem(IDC_STATIC_EXPOSURE)->SetWindowText(str);
        }
        else if (pSlider == m_sliderGain) {
            float gain = m_sliderGain->GetPos() / 10.0f;
            CString str;
            str.Format(_T("게인: %.1f dB"), gain);
            GetDlgItem(IDC_STATIC_GAIN)->SetWindowText(str);
        }

        CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
        return;
    }

    CSliderCtrl* pSlider = (CSliderCtrl*)pScrollBar;

    if (pSlider == m_sliderExposure) {
        int exposure = m_sliderExposure->GetPos();

        if (Camera_SetExposure(exposure)) {
            CString str;
            str.Format(_T("노출: %d us"), exposure);
            GetDlgItem(IDC_STATIC_EXPOSURE)->SetWindowText(str);
        }
        else {
            int currentExposure = Camera_GetExposure();
            m_sliderExposure->SetPos(currentExposure);

            CString str;
            str.Format(_T("노출: %d us"), currentExposure);
            GetDlgItem(IDC_STATIC_EXPOSURE)->SetWindowText(str);
        }
    }
    else if (pSlider == m_sliderGain) {
        float gain = m_sliderGain->GetPos() / 10.0f;

        if (Camera_SetGain(gain)) {
            CString str;
            str.Format(_T("게인: %.1f dB"), gain);
            GetDlgItem(IDC_STATIC_GAIN)->SetWindowText(str);
        }
        else {
            float currentGain = Camera_GetGain();
            m_sliderGain->SetPos((int)(currentGain * 10));

            CString str;
            str.Format(_T("게인: %.1f dB"), currentGain);
            GetDlgItem(IDC_STATIC_GAIN)->SetWindowText(str);
        }
    }

    CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
}

void CXIMEASensorDiagDlg::OnCbnSelchangeComboDevices()
{
    // choose device
}

void CXIMEASensorDiagDlg::OnTimer(UINT_PTR nIDEvent)
{
    if (nIDEvent == TIMER_UPDATE_STATISTICS && m_isStreaming) {
        unsigned long totalFrames, droppedFrames;
        double avgFPS, minFPS, maxFPS;

        if (Camera_GetStatistics(&totalFrames, &droppedFrames,
            &avgFPS, &minFPS, &maxFPS)) {
            CString str;
            str.Format(_T("총 프레임: %lu, 드롭: %lu, 평균 FPS: %.1f"),
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

    m_frameCount++;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_lastFPSUpdate).count();

    if (elapsed >= 500) {  // fps update every 500ms
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
        *pMsg = _T("카메라 연결 해제됨");
        break;
    case CameraState::CONNECTED:
        *pMsg = _T("카메라 연결됨");
        break;
    case CameraState::CAPTURING:
        *pMsg = _T("캡처 중...");
        break;
    case CameraState::kERROR:
        *pMsg = _T("카메라 오류");
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
    str.Format(_T("FPS: %.1f"), fps);

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
        
        size_t bmpInfoSize = sizeof(BITMAPINFOHEADER) + 256 * sizeof(RGBQUAD); // BITMAPINFO + 256개 RGBQUAD
        BITMAPINFO* pBmpInfo = (BITMAPINFO*)malloc(bmpInfoSize);
        memset(pBmpInfo, 0, bmpInfoSize);

        pBmpInfo->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        pBmpInfo->bmiHeader.biWidth = m_displayWidth;
        pBmpInfo->bmiHeader.biHeight = -m_displayHeight;  // top-down DIB
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
            0, 0, rect.Width(), rect.Height(),              // 대상 영역
            0, 0, m_displayWidth, m_displayHeight,          // 소스 영역
            m_pDisplayBuffer,                               // 이미지 데이터
            pBmpInfo,                                       // 비트맵 정보
            DIB_RGB_COLORS,                                 // 팔레트 사용
            SRCCOPY);                                       // 복사 모드

        if (result == GDI_ERROR) {
            DWORD error = GetLastError();
            TRACE(_T("StretchDIBits failed with error: %d\n"), error);
        }

        SetStretchBltMode(dc.GetSafeHdc(), oldStretchMode);

        free(pBmpInfo);

        // m_hasNewFrame = false;
    }

    LeaveCriticalSection(&m_frameCriticalSection);
}


void CXIMEASensorDiagDlg::ShowError(const CString& message)
{
    MessageBox(message, _T("카메라 오류"), MB_OK | MB_ICONERROR);
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
        // Update progress
        CString status;
        status.Format(_T("연속 촬영 중... %d프레임 (%.1f초)"), currentFrame, elapsedSeconds);

        if (m_staticStatus && ::IsWindow(m_staticStatus->GetSafeHwnd())) {
            m_staticStatus->SetWindowText(status);
        }
    }
    else if (state == 3) {  // COMPLETED
        // Send completion message
        PostMessage(WM_CONTINUOUS_CAPTURE_COMPLETE);
    }
}

LRESULT CXIMEASensorDiagDlg::OnContinuousCaptureComplete(WPARAM wParam, LPARAM lParam)
{
    // Get results
    int totalFrames = 0, savedFrames = 0, droppedFrames = 0;
    double duration = 0.0;
    char folderPath[256] = { 0 };

    bool success = Camera_GetContinuousCaptureResult(&totalFrames, &savedFrames,
        &droppedFrames, &duration,
        folderPath, sizeof(folderPath));

    // Restore UI
    if (m_btnSnapshot) m_btnSnapshot->EnableWindow(TRUE);
    if (m_checkContinuous) m_checkContinuous->EnableWindow(TRUE);

    // Show results
    CString msg;
    if (success) {
        msg.Format(_T("연속 촬영 완료!\n\n")
            _T("총 프레임: %d\n")
            _T("저장된 프레임: %d\n")
            _T("드롭된 프레임: %d\n")
            _T("실제 시간: %.3f초\n")
            _T("평균 FPS: %.1f\n")
            _T("저장 폴더: %s"),
            totalFrames, savedFrames, droppedFrames,
            duration, totalFrames / duration,
            CString(folderPath).GetString());
    }
    else {
        msg = _T("연속 촬영 실패!");
    }

    AfxMessageBox(msg);

    // Restore status display
    if (m_staticStatus) {
        m_staticStatus->SetWindowText(_T("캡처 중..."));
    }

    return 0;
}