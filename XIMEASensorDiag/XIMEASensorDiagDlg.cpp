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

CXIMEASensorDiagDlg::CXIMEASensorDiagDlg(CWnd* pParent)
    : CDialogEx(IDD_XIMEASENSORDIAG_DIALOG, pParent)
    , m_pictureCtrl(nullptr)
    , m_isStreaming(false)
    , m_pMemDC(nullptr)
    , m_pBackBuffer(nullptr)
    , m_renderFrameCount(0)
    , m_displayFPS(0.0f)
    , m_currentExposure(4000)
    , m_currentGain(0.0f)
    , m_currentFPS(210.0f)
{
}

CXIMEASensorDiagDlg::~CXIMEASensorDiagDlg()
{
    if (m_isStreaming) {
        OnBnClickedButtonStop();
    }

    if (m_pMemDC) delete m_pMemDC;
    if (m_pBackBuffer) delete m_pBackBuffer;
}

void CXIMEASensorDiagDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_SLIDER_EXPOSURE, m_sliderExposure);
    DDX_Control(pDX, IDC_SLIDER_GAIN, m_sliderGain);
    DDX_Control(pDX, IDC_SLIDER_FPS, m_sliderFPS);
    DDX_Control(pDX, IDC_CHECK_AUTO_EXPOSURE, m_checkAutoExposure);
    DDX_Control(pDX, IDC_CHECK_AUTO_GAIN, m_checkAutoGain);
    DDX_Control(pDX, IDC_EDIT_LOG, m_editLog);
}

BEGIN_MESSAGE_MAP(CXIMEASensorDiagDlg, CDialogEx)
    ON_WM_PAINT()
    ON_WM_QUERYDRAGICON()
    ON_WM_TIMER()
    ON_WM_HSCROLL()
    ON_BN_CLICKED(IDC_BUTTON_START, &CXIMEASensorDiagDlg::OnBnClickedButtonStart)
    ON_BN_CLICKED(IDC_BUTTON_STOP, &CXIMEASensorDiagDlg::OnBnClickedButtonStop)
    ON_BN_CLICKED(IDC_BUTTON_CAPTURE, &CXIMEASensorDiagDlg::OnBnClickedButtonCapture)
    ON_BN_CLICKED(IDC_BUTTON_SAVE_CONFIG, &CXIMEASensorDiagDlg::OnBnClickedButtonSaveConfig)
    ON_BN_CLICKED(IDC_BUTTON_LOAD_CONFIG, &CXIMEASensorDiagDlg::OnBnClickedButtonLoadConfig)
    ON_BN_CLICKED(IDC_CHECK_AUTO_EXPOSURE, &CXIMEASensorDiagDlg::OnBnClickedCheckAutoExposure)
    ON_BN_CLICKED(IDC_CHECK_AUTO_GAIN, &CXIMEASensorDiagDlg::OnBnClickedCheckAutoGain)
END_MESSAGE_MAP()

BOOL CXIMEASensorDiagDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    // 고정밀 타이머 주파수
    QueryPerformanceFrequency(&m_frequency);

    // Picture Control 초기화
    m_pictureCtrl = (CStatic*)GetDlgItem(IDC_STATIC_VIDEO);
    m_pictureCtrl->GetClientRect(&m_displayRect);

    // 더블 버퍼링 초기화
    CDC* pDC = m_pictureCtrl->GetDC();
    m_pMemDC = new CDC();
    m_pMemDC->CreateCompatibleDC(pDC);
    m_pBackBuffer = new CBitmap();
    m_pBackBuffer->CreateCompatibleBitmap(pDC, m_displayRect.Width(), m_displayRect.Height());
    m_pMemDC->SelectObject(m_pBackBuffer);
    m_pictureCtrl->ReleaseDC(pDC);

    // BITMAPINFO 초기화
    memset(&m_bmpInfo, 0, sizeof(m_bmpInfo));
    m_bmpInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    m_bmpInfo.bmiHeader.biPlanes = 1;
    m_bmpInfo.bmiHeader.biBitCount = 8;
    m_bmpInfo.bmiHeader.biCompression = BI_RGB;

    // 그레이스케일 팔레트
    for (int i = 0; i < 256; i++) {
        m_bmpInfo.bmiColors[i].rgbBlue = i;
        m_bmpInfo.bmiColors[i].rgbGreen = i;
        m_bmpInfo.bmiColors[i].rgbRed = i;
        m_bmpInfo.bmiColors[i].rgbReserved = 0;
    }

    // 슬라이더 초기화
    m_sliderExposure.SetRange(10, 50000);  // 10us ~ 50ms
    m_sliderExposure.SetPos(4000);         // 4ms

    m_sliderGain.SetRange(0, 240);         // 0.0 ~ 24.0 dB (x10)
    m_sliderGain.SetPos(0);

    m_sliderFPS.SetRange(1, 210);          // 1 ~ 210 FPS
    m_sliderFPS.SetPos(210);

    // 콜백 설정
    Camera_SetFrameCallback(FrameCallback, this);
    Camera_SetErrorCallback(ErrorCallback, this);
    Camera_SetLogCallback(LogCallback, this);

    // 로그 파일 설정
    Camera_SetLogFile("XIMEACamera.log");
    Camera_SetLogLevel(XIMEA_LOG_INFO);

    // UI 업데이트 타이머
    SetTimer(TIMER_UPDATE_UI, 100, NULL);  // 100ms마다 UI 업데이트

    UpdateParameterDisplay();

    return TRUE;
}

void CXIMEASensorDiagDlg::OnPaint()
{
    if (IsIconic()) {
        CPaintDC dc(this);
        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);
    }
    else {
        CDialogEx::OnPaint();
        RenderFrame();
    }
}

HCURSOR CXIMEASensorDiagDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(AfxGetApp()->LoadStandardIcon(IDI_APPLICATION));
}

void CXIMEASensorDiagDlg::OnTimer(UINT_PTR nIDEvent)
{
    if (nIDEvent == TIMER_UPDATE_UI) {
        UpdateUI();
    }
    else if (nIDEvent == TIMER_RENDER) {
        RenderFrame();
    }

    CDialogEx::OnTimer(nIDEvent);
}

void CXIMEASensorDiagDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
    if (!m_isStreaming) {
        CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
        return;
    }

    CSliderCtrl* pSlider = (CSliderCtrl*)pScrollBar;

    if (pSlider == &m_sliderExposure) {
        int exposure = m_sliderExposure.GetPos();
        Camera_SetExposure(exposure);
        m_currentExposure = exposure;
    }
    else if (pSlider == &m_sliderGain) {
        float gain = m_sliderGain.GetPos() / 10.0f;
        Camera_SetGain(gain);
        m_currentGain = gain;
    }
    else if (pSlider == &m_sliderFPS) {
        float fps = (float)m_sliderFPS.GetPos();
        Camera_SetFPS(fps);
        m_currentFPS = fps;
    }

    UpdateParameterDisplay();

    CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
}

void CXIMEASensorDiagDlg::OnBnClickedCheckAutoExposure()
{
    bool enable = (m_checkAutoExposure.GetCheck() == BST_CHECKED);
    Camera_SetAutoExposure(enable, 0.5f);
    m_sliderExposure.EnableWindow(!enable);
}

void CXIMEASensorDiagDlg::OnBnClickedCheckAutoGain()
{
    bool enable = (m_checkAutoGain.GetCheck() == BST_CHECKED);
    Camera_SetAutoGain(enable);
    m_sliderGain.EnableWindow(!enable);
}

void CXIMEASensorDiagDlg::OnBnClickedButtonStart()
{
    // 카메라 초기화
    if (!Camera_Initialize(0)) {
        AfxMessageBox(_T("카메라 초기화 실패: ") + CString(Camera_GetLastError()));
        return;
    }

    // 카메라 정보 표시
    XIMEA_CAMERA_INFO info;
    if (Camera_GetInfo(&info)) {
        CString strInfo;
        strInfo.Format(_T("카메라: %s\n시리얼: %s\n센서: %dx%d"),
            CString(info.deviceName), CString(info.serialNumber),
            info.sensorWidth, info.sensorHeight);
        SetDlgItemText(IDC_STATIC_CAMERA_INFO, strInfo);
    }

    // 초기 파라미터 설정
    Camera_SetExposure(m_currentExposure);
    Camera_SetGain(m_currentGain);
    Camera_SetFPS(m_currentFPS);

    // 캡처 시작
    if (!Camera_StartCapture()) {
        AfxMessageBox(_T("캡처 시작 실패: ") + CString(Camera_GetLastError()));
        Camera_Shutdown();
        return;
    }

    m_isStreaming = true;

    // 렌더링 타이머 시작
    SetTimer(TIMER_RENDER, 16, NULL);  // ~60 FPS 렌더링

    // UI 업데이트
    GetDlgItem(IDC_BUTTON_START)->EnableWindow(FALSE);
    GetDlgItem(IDC_BUTTON_STOP)->EnableWindow(TRUE);
    GetDlgItem(IDC_BUTTON_CAPTURE)->EnableWindow(TRUE);
    SetDlgItemText(IDC_STATIC_STATUS, _T("스트리밍 중..."));

    AddLogMessage("카메라 스트리밍 시작");
}

void CXIMEASensorDiagDlg::OnBnClickedButtonStop()
{
    m_isStreaming = false;

    KillTimer(TIMER_RENDER);

    Camera_StopCapture();
    Camera_Shutdown();

    // UI 업데이트
    GetDlgItem(IDC_BUTTON_START)->EnableWindow(TRUE);
    GetDlgItem(IDC_BUTTON_STOP)->EnableWindow(FALSE);
    GetDlgItem(IDC_BUTTON_CAPTURE)->EnableWindow(FALSE);
    SetDlgItemText(IDC_STATIC_STATUS, _T("정지됨"));
    SetDlgItemText(IDC_STATIC_FPS, _T("FPS: 0.0 / 0.0"));

    AddLogMessage("카메라 스트리밍 정지");
}

void CXIMEASensorDiagDlg::OnBnClickedButtonCapture()
{
    CString filename;
    CTime time = CTime::GetCurrentTime();
    filename.Format(_T("Capture_%s.png"), time.Format(_T("%Y%m%d_%H%M%S")));

    if (Camera_CaptureImage(CT2A(filename), XIMEA_FORMAT_MONO8)) {
        AddLogMessage("이미지 저장: " + std::string(CT2A(filename)));
        AfxMessageBox(_T("이미지가 저장되었습니다: ") + filename);
    }
    else {
        AfxMessageBox(_T("이미지 저장 실패"));
    }
}

void CXIMEASensorDiagDlg::OnBnClickedButtonSaveConfig()
{
    CFileDialog dlg(FALSE, _T("ini"), _T("camera_config.ini"),
        OFN_OVERWRITEPROMPT, _T("Config Files (*.ini)|*.ini|All Files (*.*)|*.*||"));

    if (dlg.DoModal() == IDOK) {
        if (Camera_SaveParameters(CT2A(dlg.GetPathName()))) {
            AddLogMessage("설정 저장: " + std::string(CT2A(dlg.GetPathName())));
            AfxMessageBox(_T("설정이 저장되었습니다"));
        }
        else {
            AfxMessageBox(_T("설정 저장 실패"));
        }
    }
}

void CXIMEASensorDiagDlg::OnBnClickedButtonLoadConfig()
{
    CFileDialog dlg(TRUE, _T("ini"), NULL,
        OFN_FILEMUSTEXIST, _T("Config Files (*.ini)|*.ini|All Files (*.*)|*.*||"));

    if (dlg.DoModal() == IDOK) {
        if (Camera_LoadParameters(CT2A(dlg.GetPathName()))) {
            AddLogMessage("설정 로드: " + std::string(CT2A(dlg.GetPathName())));

            // UI 업데이트
            m_currentExposure = Camera_GetExposure();
            m_currentGain = Camera_GetGain();
            m_sliderExposure.SetPos(m_currentExposure);
            m_sliderGain.SetPos((int)(m_currentGain * 10));
            UpdateParameterDisplay();

            AfxMessageBox(_T("설정이 로드되었습니다"));
        }
        else {
            AfxMessageBox(_T("설정 로드 실패"));
        }
    }
}

void CXIMEASensorDiagDlg::FrameCallback(const unsigned char* data, int width, int height,
    unsigned long frameNumber, double timestamp, void* userContext)
{
    auto* pDlg = reinterpret_cast<CXIMEASensorDiagDlg*>(userContext);
    if (!pDlg || !pDlg->m_isStreaming) return;

    std::lock_guard<std::mutex> lock(pDlg->m_frameMutex);

    // 프레임 큐에 추가 (최대 3개 유지)
    if (pDlg->m_frameQueue.size() >= 3) {
        pDlg->m_frameQueue.pop_front();
    }

    FrameData frame;
    frame.width = width;
    frame.height = height;
    frame.frameNumber = frameNumber;
    frame.timestamp = timestamp;
    frame.data.resize(width * height);
    memcpy(frame.data.data(), data, width * height);

    pDlg->m_frameQueue.push_back(std::move(frame));
}

void CXIMEASensorDiagDlg::ErrorCallback(int errorCode, const char* errorMessage, void* userContext)
{
    auto* pDlg = reinterpret_cast<CXIMEASensorDiagDlg*>(userContext);
    if (!pDlg) return;

    CString msg;
    msg.Format(_T("[ERROR %d] %s"), errorCode, CString(errorMessage));
    pDlg->AddLogMessage(std::string(CT2A(msg)));
}

void CXIMEASensorDiagDlg::LogCallback(int logLevel, const char* message, void* userContext)
{
    auto* pDlg = reinterpret_cast<CXIMEASensorDiagDlg*>(userContext);
    if (!pDlg) return;

    pDlg->AddLogMessage(std::string(message));
}

void CXIMEASensorDiagDlg::UpdateUI()
{
    if (!m_isStreaming) return;

    // 성능 통계 업데이트
    XIMEA_PERFORMANCE_STATS stats;
    if (Camera_GetPerformanceStats(&stats)) {
        CString strFPS;
        strFPS.Format(_T("FPS: %.1f / %.1f"), m_displayFPS, stats.currentFPS);
        SetDlgItemText(IDC_STATIC_FPS, strFPS);

        CString strStats;
        strStats.Format(_T("캡처: %lu / 드롭: %lu"),
            stats.framesCaptured, stats.framesDropped);
        SetDlgItemText(IDC_STATIC_STATS, strStats);
    }

    // 현재 파라미터 업데이트 (자동 모드인 경우)
    if (m_checkAutoExposure.GetCheck() == BST_CHECKED) {
        m_currentExposure = Camera_GetExposure();
        m_sliderExposure.SetPos(m_currentExposure);
    }
    if (m_checkAutoGain.GetCheck() == BST_CHECKED) {
        m_currentGain = Camera_GetGain();
        m_sliderGain.SetPos((int)(m_currentGain * 10));
    }

    UpdateParameterDisplay();
}

void CXIMEASensorDiagDlg::RenderFrame()
{
    std::lock_guard<std::mutex> lock(m_frameMutex);

    if (m_frameQueue.empty()) return;

    // 최신 프레임 가져오기
    const FrameData& frame = m_frameQueue.back();

    // BITMAPINFO 업데이트
    m_bmpInfo.bmiHeader.biWidth = frame.width;
    m_bmpInfo.bmiHeader.biHeight = -frame.height;  // top-down

    // 백버퍼에 렌더링
    SetStretchBltMode(m_pMemDC->GetSafeHdc(), HALFTONE);

    int result = StretchDIBits(m_pMemDC->GetSafeHdc(),
        0, 0, m_displayRect.Width(), m_displayRect.Height(),
        0, 0, frame.width, frame.height,
        frame.data.data(),
        &m_bmpInfo,
        DIB_RGB_COLORS,
        SRCCOPY);

    if (result != 0) {
        // 화면에 복사
        CDC* pDC = m_pictureCtrl->GetDC();
        pDC->BitBlt(0, 0, m_displayRect.Width(), m_displayRect.Height(),
            m_pMemDC, 0, 0, SRCCOPY);
        m_pictureCtrl->ReleaseDC(pDC);

        // 디스플레이 FPS 계산
        LARGE_INTEGER currentTime;
        QueryPerformanceCounter(&currentTime);

        if (m_renderFrameCount > 0) {
            double elapsed = (double)(currentTime.QuadPart - m_lastRenderTime.QuadPart) / m_frequency.QuadPart;
            if (elapsed >= 1.0) {
                m_displayFPS = m_renderFrameCount / (float)elapsed;
                m_renderFrameCount = 0;
                m_lastRenderTime = currentTime;
            }
        }
        else {
            m_lastRenderTime = currentTime;
        }

        m_renderFrameCount++;
    }
}

void CXIMEASensorDiagDlg::AddLogMessage(const std::string& message)
{
    std::lock_guard<std::mutex> lock(m_logMutex);

    m_logMessages.push_back(message);
    if (m_logMessages.size() > 100) {
        m_logMessages.pop_front();
    }

    // UI 업데이트 (메인 스레드에서만)
    if (::IsWindow(m_editLog.GetSafeHwnd())) {
        CString logText;
        for (const auto& msg : m_logMessages) {
            logText += CString(msg.c_str()) + _T("\r\n");
        }
        m_editLog.SetWindowText(logText);
        m_editLog.LineScroll(m_editLog.GetLineCount());
    }
}

void CXIMEASensorDiagDlg::UpdateParameterDisplay()
{
    CString str;

    str.Format(_T("노출: %d us"), m_currentExposure);
    SetDlgItemText(IDC_STATIC_EXPOSURE_VALUE, str);

    str.Format(_T("게인: %.1f dB"), m_currentGain);
    SetDlgItemText(IDC_STATIC_GAIN_VALUE, str);

    str.Format(_T("목표 FPS: %.0f"), m_currentFPS);
    SetDlgItemText(IDC_STATIC_FPS_VALUE, str);
}