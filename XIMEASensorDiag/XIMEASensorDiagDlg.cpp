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

    // ������ Ÿ�̸� ���ļ�
    QueryPerformanceFrequency(&m_frequency);

    // Picture Control �ʱ�ȭ
    m_pictureCtrl = (CStatic*)GetDlgItem(IDC_STATIC_VIDEO);
    m_pictureCtrl->GetClientRect(&m_displayRect);

    // ���� ���۸� �ʱ�ȭ
    CDC* pDC = m_pictureCtrl->GetDC();
    m_pMemDC = new CDC();
    m_pMemDC->CreateCompatibleDC(pDC);
    m_pBackBuffer = new CBitmap();
    m_pBackBuffer->CreateCompatibleBitmap(pDC, m_displayRect.Width(), m_displayRect.Height());
    m_pMemDC->SelectObject(m_pBackBuffer);
    m_pictureCtrl->ReleaseDC(pDC);

    // BITMAPINFO �ʱ�ȭ
    memset(&m_bmpInfo, 0, sizeof(m_bmpInfo));
    m_bmpInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    m_bmpInfo.bmiHeader.biPlanes = 1;
    m_bmpInfo.bmiHeader.biBitCount = 8;
    m_bmpInfo.bmiHeader.biCompression = BI_RGB;

    // �׷��̽����� �ȷ�Ʈ
    for (int i = 0; i < 256; i++) {
        m_bmpInfo.bmiColors[i].rgbBlue = i;
        m_bmpInfo.bmiColors[i].rgbGreen = i;
        m_bmpInfo.bmiColors[i].rgbRed = i;
        m_bmpInfo.bmiColors[i].rgbReserved = 0;
    }

    // �����̴� �ʱ�ȭ
    m_sliderExposure.SetRange(10, 50000);  // 10us ~ 50ms
    m_sliderExposure.SetPos(4000);         // 4ms

    m_sliderGain.SetRange(0, 240);         // 0.0 ~ 24.0 dB (x10)
    m_sliderGain.SetPos(0);

    m_sliderFPS.SetRange(1, 210);          // 1 ~ 210 FPS
    m_sliderFPS.SetPos(210);

    // �ݹ� ����
    Camera_SetFrameCallback(FrameCallback, this);
    Camera_SetErrorCallback(ErrorCallback, this);
    Camera_SetLogCallback(LogCallback, this);

    // �α� ���� ����
    Camera_SetLogFile("XIMEACamera.log");
    Camera_SetLogLevel(XIMEA_LOG_INFO);

    // UI ������Ʈ Ÿ�̸�
    SetTimer(TIMER_UPDATE_UI, 100, NULL);  // 100ms���� UI ������Ʈ

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
    // ī�޶� �ʱ�ȭ
    if (!Camera_Initialize(0)) {
        AfxMessageBox(_T("ī�޶� �ʱ�ȭ ����: ") + CString(Camera_GetLastError()));
        return;
    }

    // ī�޶� ���� ǥ��
    XIMEA_CAMERA_INFO info;
    if (Camera_GetInfo(&info)) {
        CString strInfo;
        strInfo.Format(_T("ī�޶�: %s\n�ø���: %s\n����: %dx%d"),
            CString(info.deviceName), CString(info.serialNumber),
            info.sensorWidth, info.sensorHeight);
        SetDlgItemText(IDC_STATIC_CAMERA_INFO, strInfo);
    }

    // �ʱ� �Ķ���� ����
    Camera_SetExposure(m_currentExposure);
    Camera_SetGain(m_currentGain);
    Camera_SetFPS(m_currentFPS);

    // ĸó ����
    if (!Camera_StartCapture()) {
        AfxMessageBox(_T("ĸó ���� ����: ") + CString(Camera_GetLastError()));
        Camera_Shutdown();
        return;
    }

    m_isStreaming = true;

    // ������ Ÿ�̸� ����
    SetTimer(TIMER_RENDER, 16, NULL);  // ~60 FPS ������

    // UI ������Ʈ
    GetDlgItem(IDC_BUTTON_START)->EnableWindow(FALSE);
    GetDlgItem(IDC_BUTTON_STOP)->EnableWindow(TRUE);
    GetDlgItem(IDC_BUTTON_CAPTURE)->EnableWindow(TRUE);
    SetDlgItemText(IDC_STATIC_STATUS, _T("��Ʈ���� ��..."));

    AddLogMessage("ī�޶� ��Ʈ���� ����");
}

void CXIMEASensorDiagDlg::OnBnClickedButtonStop()
{
    m_isStreaming = false;

    KillTimer(TIMER_RENDER);

    Camera_StopCapture();
    Camera_Shutdown();

    // UI ������Ʈ
    GetDlgItem(IDC_BUTTON_START)->EnableWindow(TRUE);
    GetDlgItem(IDC_BUTTON_STOP)->EnableWindow(FALSE);
    GetDlgItem(IDC_BUTTON_CAPTURE)->EnableWindow(FALSE);
    SetDlgItemText(IDC_STATIC_STATUS, _T("������"));
    SetDlgItemText(IDC_STATIC_FPS, _T("FPS: 0.0 / 0.0"));

    AddLogMessage("ī�޶� ��Ʈ���� ����");
}

void CXIMEASensorDiagDlg::OnBnClickedButtonCapture()
{
    CString filename;
    CTime time = CTime::GetCurrentTime();
    filename.Format(_T("Capture_%s.png"), time.Format(_T("%Y%m%d_%H%M%S")));

    if (Camera_CaptureImage(CT2A(filename), XIMEA_FORMAT_MONO8)) {
        AddLogMessage("�̹��� ����: " + std::string(CT2A(filename)));
        AfxMessageBox(_T("�̹����� ����Ǿ����ϴ�: ") + filename);
    }
    else {
        AfxMessageBox(_T("�̹��� ���� ����"));
    }
}

void CXIMEASensorDiagDlg::OnBnClickedButtonSaveConfig()
{
    CFileDialog dlg(FALSE, _T("ini"), _T("camera_config.ini"),
        OFN_OVERWRITEPROMPT, _T("Config Files (*.ini)|*.ini|All Files (*.*)|*.*||"));

    if (dlg.DoModal() == IDOK) {
        if (Camera_SaveParameters(CT2A(dlg.GetPathName()))) {
            AddLogMessage("���� ����: " + std::string(CT2A(dlg.GetPathName())));
            AfxMessageBox(_T("������ ����Ǿ����ϴ�"));
        }
        else {
            AfxMessageBox(_T("���� ���� ����"));
        }
    }
}

void CXIMEASensorDiagDlg::OnBnClickedButtonLoadConfig()
{
    CFileDialog dlg(TRUE, _T("ini"), NULL,
        OFN_FILEMUSTEXIST, _T("Config Files (*.ini)|*.ini|All Files (*.*)|*.*||"));

    if (dlg.DoModal() == IDOK) {
        if (Camera_LoadParameters(CT2A(dlg.GetPathName()))) {
            AddLogMessage("���� �ε�: " + std::string(CT2A(dlg.GetPathName())));

            // UI ������Ʈ
            m_currentExposure = Camera_GetExposure();
            m_currentGain = Camera_GetGain();
            m_sliderExposure.SetPos(m_currentExposure);
            m_sliderGain.SetPos((int)(m_currentGain * 10));
            UpdateParameterDisplay();

            AfxMessageBox(_T("������ �ε�Ǿ����ϴ�"));
        }
        else {
            AfxMessageBox(_T("���� �ε� ����"));
        }
    }
}

void CXIMEASensorDiagDlg::FrameCallback(const unsigned char* data, int width, int height,
    unsigned long frameNumber, double timestamp, void* userContext)
{
    auto* pDlg = reinterpret_cast<CXIMEASensorDiagDlg*>(userContext);
    if (!pDlg || !pDlg->m_isStreaming) return;

    std::lock_guard<std::mutex> lock(pDlg->m_frameMutex);

    // ������ ť�� �߰� (�ִ� 3�� ����)
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

    // ���� ��� ������Ʈ
    XIMEA_PERFORMANCE_STATS stats;
    if (Camera_GetPerformanceStats(&stats)) {
        CString strFPS;
        strFPS.Format(_T("FPS: %.1f / %.1f"), m_displayFPS, stats.currentFPS);
        SetDlgItemText(IDC_STATIC_FPS, strFPS);

        CString strStats;
        strStats.Format(_T("ĸó: %lu / ���: %lu"),
            stats.framesCaptured, stats.framesDropped);
        SetDlgItemText(IDC_STATIC_STATS, strStats);
    }

    // ���� �Ķ���� ������Ʈ (�ڵ� ����� ���)
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

    // �ֽ� ������ ��������
    const FrameData& frame = m_frameQueue.back();

    // BITMAPINFO ������Ʈ
    m_bmpInfo.bmiHeader.biWidth = frame.width;
    m_bmpInfo.bmiHeader.biHeight = -frame.height;  // top-down

    // ����ۿ� ������
    SetStretchBltMode(m_pMemDC->GetSafeHdc(), HALFTONE);

    int result = StretchDIBits(m_pMemDC->GetSafeHdc(),
        0, 0, m_displayRect.Width(), m_displayRect.Height(),
        0, 0, frame.width, frame.height,
        frame.data.data(),
        &m_bmpInfo,
        DIB_RGB_COLORS,
        SRCCOPY);

    if (result != 0) {
        // ȭ�鿡 ����
        CDC* pDC = m_pictureCtrl->GetDC();
        pDC->BitBlt(0, 0, m_displayRect.Width(), m_displayRect.Height(),
            m_pMemDC, 0, 0, SRCCOPY);
        m_pictureCtrl->ReleaseDC(pDC);

        // ���÷��� FPS ���
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

    // UI ������Ʈ (���� �����忡����)
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

    str.Format(_T("����: %d us"), m_currentExposure);
    SetDlgItemText(IDC_STATIC_EXPOSURE_VALUE, str);

    str.Format(_T("����: %.1f dB"), m_currentGain);
    SetDlgItemText(IDC_STATIC_GAIN_VALUE, str);

    str.Format(_T("��ǥ FPS: %.0f"), m_currentFPS);
    SetDlgItemText(IDC_STATIC_FPS_VALUE, str);
}