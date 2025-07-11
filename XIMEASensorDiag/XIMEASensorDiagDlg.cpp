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

// 타이머 ID
#define TIMER_UPDATE_STATISTICS 1001

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

    m_pDisplayBuffer = new unsigned char[1280 * 1024];  // 최대 크기

    UpdateDeviceList();

    UpdateUI(false);

    SetTimer(TIMER_UPDATE_STATISTICS, 1000, nullptr);

    return TRUE;
}

void CXIMEASensorDiagDlg::OnDestroy()
{
    CDialogEx::OnDestroy();

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

    Camera_SetExposure(4000);  // 4ms
    Camera_SetGain(0.0f);      // 0dB

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

void CXIMEASensorDiagDlg::OnBnClickedButtonStop()
{
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
    if (!m_isStreaming || !m_hasNewFrame) {
        AfxMessageBox(_T("캡처할 프레임이 없습니다!"));
        return;
    }

    SYSTEMTIME st;
    GetLocalTime(&st);

    CString filename;
    filename.Format(_T("snapshot_%04d%02d%02d_%02d%02d%02d.raw"),
        st.wYear, st.wMonth, st.wDay,
        st.wHour, st.wMinute, st.wSecond);

    EnterCriticalSection(&m_frameCriticalSection);

    CFile file;
    if (file.Open(filename, CFile::modeCreate | CFile::modeWrite)) {
        file.Write(m_pDisplayBuffer, m_displayWidth * m_displayHeight);
        file.Close();

        CString msg;
        msg.Format(_T("스냅샷 저장됨: %s\n크기: %dx%d"),
            filename, m_displayWidth, m_displayHeight);
        AfxMessageBox(msg);
    }

    LeaveCriticalSection(&m_frameCriticalSection);
}

void CXIMEASensorDiagDlg::OnBnClickedButtonSettings()
{
    AfxMessageBox(_T("Coming soonnnnn~."));
}

void CXIMEASensorDiagDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
    if (!m_isStreaming) return;

    CSliderCtrl* pSlider = (CSliderCtrl*)pScrollBar;

    if (pSlider == m_sliderExposure) {
        int exposure = m_sliderExposure->GetPos();
        Camera_SetExposure(exposure);

        CString str;
        str.Format(_T("노출: %d us"), exposure);
        GetDlgItem(IDC_STATIC_EXPOSURE)->SetWindowText(str);
    }
    else if (pSlider == m_sliderGain) {
        float gain = m_sliderGain->GetPos() / 10.0f;
        Camera_SetGain(gain);

        CString str;
        str.Format(_T("게인: %.1f dB"), gain);
        GetDlgItem(IDC_STATIC_GAIN)->SetWindowText(str);
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

    memcpy(m_pDisplayBuffer, frameInfo.data,
        frameInfo.width * frameInfo.height);
    m_displayWidth = frameInfo.width;
    m_displayHeight = frameInfo.height;
    m_hasNewFrame = true;

    LeaveCriticalSection(&m_frameCriticalSection);

    PostMessage(WM_UPDATE_FRAME);

    // FPS calc
    m_frameCount++;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_lastFPSUpdate).count();

    if (elapsed >= 500) {  // 0.5초마다 FPS 업데이트
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
    CString* pMsg = new CString(errorMessage.c_str());
    PostMessage(WM_UPDATE_ERROR, (WPARAM)error, (LPARAM)pMsg);
}

void CXIMEASensorDiagDlg::OnPropertyChangedCallback(const std::string& propertyName,
    const std::string& value)
{
    // 속성 변경 로그 (디버그용)
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
    if (!m_hasNewFrame || !m_pictureCtrl) return;

    CClientDC dc(m_pictureCtrl);
    CRect rect;
    m_pictureCtrl->GetClientRect(&rect);

    EnterCriticalSection(&m_frameCriticalSection);

    if (m_hasNewFrame) {
        // 8bit grayscale bitmap
        BITMAPINFO bmpInfo = { 0 };
        bmpInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bmpInfo.bmiHeader.biWidth = m_displayWidth;
        bmpInfo.bmiHeader.biHeight = -m_displayHeight;  // top-down
        bmpInfo.bmiHeader.biPlanes = 1;
        bmpInfo.bmiHeader.biBitCount = 8;
        bmpInfo.bmiHeader.biCompression = BI_RGB;

        RGBQUAD grayscalePalette[256];
        for (int i = 0; i < 256; i++) {
            grayscalePalette[i].rgbBlue = i;
            grayscalePalette[i].rgbGreen = i;
            grayscalePalette[i].rgbRed = i;
            grayscalePalette[i].rgbReserved = 0;
        }

        SetDIBColorTable(dc.GetSafeHdc(), 0, 256, grayscalePalette);
        SetStretchBltMode(dc.GetSafeHdc(), HALFTONE);

        // draw
        StretchDIBits(dc.GetSafeHdc(),
            0, 0, rect.Width(), rect.Height(),
            0, 0, m_displayWidth, m_displayHeight,
            m_pDisplayBuffer,
            &bmpInfo,
            DIB_RGB_COLORS,
            SRCCOPY);
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