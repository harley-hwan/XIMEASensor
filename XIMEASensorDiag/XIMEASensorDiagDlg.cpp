#include "pch.h"
#include "framework.h"
#include "XIMEASensorDiag.h"
#include "XIMEASensorDiagDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

CXIMEASensorDiagDlg::CXIMEASensorDiagDlg(CWnd* pParent)
    : CDialogEx(IDD_XIMEASENSORDIAG_DIALOG, pParent),
    m_pictureCtrl(nullptr),
    m_isStreaming(false),
    m_pFrameBuffer(nullptr),
    m_frameWidth(0),
    m_frameHeight(0)
{
}

CXIMEASensorDiagDlg::~CXIMEASensorDiagDlg()
{
    if (m_pFrameBuffer) delete[] m_pFrameBuffer;
}

void CXIMEASensorDiagDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CXIMEASensorDiagDlg, CDialogEx)
    ON_WM_PAINT()
    ON_WM_QUERYDRAGICON()
    ON_BN_CLICKED(IDC_BUTTON_START, &CXIMEASensorDiagDlg::OnBnClickedButtonStart)
    ON_BN_CLICKED(IDC_BUTTON_STOP, &CXIMEASensorDiagDlg::OnBnClickedButtonStop)
    ON_MESSAGE(WM_APP + 1, &CXIMEASensorDiagDlg::OnFrameReady)
    ON_MESSAGE(WM_APP + 2, &CXIMEASensorDiagDlg::OnCameraError)
END_MESSAGE_MAP()

BOOL CXIMEASensorDiagDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    m_pictureCtrl = (CStatic*)GetDlgItem(IDC_STATIC_VIDEO);
    m_pFrameBuffer = new unsigned char[MAX_FRAME_SIZE];  // 힙에 버퍼 할당

    // Set static instance pointer for callbacks
    s_instance = this;
    // Register callbacks for camera events
    Camera_SetFrameCallback(FrameCallback);
    Camera_SetErrorCallback(ErrorCallback);
    Camera_SetLogCallback(LogCallback);

    return TRUE;
}

void CXIMEASensorDiagDlg::OnPaint()
{
    if (IsIconic())
    {
        CPaintDC dc(this);
        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);
    }
    else
    {
        CDialogEx::OnPaint();
    }
}

HCURSOR CXIMEASensorDiagDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(AfxGetApp()->LoadStandardIcon(IDI_APPLICATION));
}

void CXIMEASensorDiagDlg::OnBnClickedButtonStart()
{
    // Open camera device
    if (!Camera_Open(0)) {
        // If open fails, error callback will handle the message box
        return;
    }

    // Set optimal settings for MQ013MG-ON
    Camera_SetExposure(2000);  // 2ms exposure (fast capture)
    // Optionally adjust gain for low-light (e.g., Camera_SetGain(6.0f); )
    // Optionally set ROI (e.g., Camera_SetROI(0, 0, 1280, 1024); )

    // Start camera streaming
    if (!Camera_Start()) {
        // If start fails, error callback handles error message.
        // Ensure camera is closed if start failed.
        Camera_Close();
        return;
    }

    // Update UI on successful start
    GetDlgItem(IDC_BUTTON_START)->EnableWindow(FALSE);
    GetDlgItem(IDC_BUTTON_STOP)->EnableWindow(TRUE);
    SetDlgItemText(IDC_STATIC_STATUS, _T("MQ013MG-ON 카메라 실행 중..."));

    // Mark streaming state
    m_isStreaming = true;
}

void CXIMEASensorDiagDlg::OnBnClickedButtonStop()
{
    if (!m_isStreaming) {
        return;
    }
    m_isStreaming = false;
    // Stop camera streaming and close camera
    Camera_Stop();
    Camera_Close();
    // Update UI controls
    GetDlgItem(IDC_BUTTON_START)->EnableWindow(TRUE);
    GetDlgItem(IDC_BUTTON_STOP)->EnableWindow(FALSE);
    // Update status
    SetDlgItemText(IDC_STATIC_STATUS, _T("정지됨"));
    SetDlgItemText(IDC_STATIC_FPS, _T("FPS: 0.0"));
}

// Static instance pointer initialization
CXIMEASensorDiagDlg* CXIMEASensorDiagDlg::s_instance = nullptr;

// Static callback function: Frame ready
void CXIMEASensorDiagDlg::FrameCallback(const unsigned char* frame, int width, int height)
{
    if (!s_instance) return;
    // Copy frame data to dialog's buffer in a thread-safe manner
    {
        std::lock_guard<std::mutex> lock(s_instance->m_frameMutex);
        int copyWidth = width;
        int copyHeight = height;
        int frameSize = copyWidth * copyHeight;
        if (frameSize > s_instance->MAX_FRAME_SIZE) {
            // Frame is larger than buffer (should not happen given predefined max size)
            copyWidth = s_instance->MAX_FRAME_SIZE;
            copyHeight = 1;
            frameSize = copyWidth * copyHeight;
        }
        memcpy(s_instance->m_pFrameBuffer, frame, frameSize);
        s_instance->m_frameWidth = copyWidth;
        s_instance->m_frameHeight = copyHeight;
    }
    // Post message to main thread to handle frame display
    s_instance->PostMessage(WM_APP + 1, 0, 0);
}

// Static callback function: Error event
void CXIMEASensorDiagDlg::ErrorCallback(int errorCode, const char* errorMsg)
{
    if (!s_instance) return;
    // Copy error message to heap for posting to UI thread
    std::string msgStr = errorMsg ? std::string(errorMsg) : std::string("Unknown error");
    char* msgCopy = new char[msgStr.size() + 1];
    strcpy_s(msgCopy, msgStr.size() + 1, msgStr.c_str());
    s_instance->PostMessage(WM_APP + 2, (WPARAM)errorCode, (LPARAM)msgCopy);
}

// Static callback function: Log event
void CXIMEASensorDiagDlg::LogCallback(const char* logMsg)
{
    // In this example, we do not display general log messages in the UI.
    // They are already written to file and debug output by the Logger.
    (void)logMsg;
}

// Message handler for frame ready (WM_APP+1)
LRESULT CXIMEASensorDiagDlg::OnFrameReady(WPARAM wParam, LPARAM lParam)
{
    std::lock_guard<std::mutex> lock(m_frameMutex);
    int width = m_frameWidth;
    int height = m_frameHeight;
    if (width <= 0 || height <= 0) {
        return 0;
    }
    // Prepare BITMAPINFO for 8-bit grayscale
    static BITMAPINFO bmpInfo;
    static RGBQUAD grayPalette[256];
    static bool infoInitialized = false;
    if (!infoInitialized) {
        memset(&bmpInfo, 0, sizeof(bmpInfo));
        bmpInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bmpInfo.bmiHeader.biPlanes = 1;
        bmpInfo.bmiHeader.biBitCount = 8;
        bmpInfo.bmiHeader.biCompression = BI_RGB;
        // Initialize grayscale palette
        for (int i = 0; i < 256; ++i) {
            grayPalette[i].rgbBlue = (BYTE)i;
            grayPalette[i].rgbGreen = (BYTE)i;
            grayPalette[i].rgbRed = (BYTE)i;
            grayPalette[i].rgbReserved = 0;
        }
        infoInitialized = true;
    }
    // Update bitmap dimensions
    bmpInfo.bmiHeader.biWidth = width;
    bmpInfo.bmiHeader.biHeight = -height;  // top-down image
    bmpInfo.bmiHeader.biSizeImage = 0;
    // Draw the image to the picture control
    CClientDC dc(m_pictureCtrl);
    SetDIBColorTable(dc.GetSafeHdc(), 0, 256, grayPalette);
    SetStretchBltMode(dc.GetSafeHdc(), HALFTONE);
    CRect rect;
    m_pictureCtrl->GetClientRect(&rect);
    StretchDIBits(dc.GetSafeHdc(),
        0, 0, rect.Width(), rect.Height(),    // destination area (control size)
        0, 0, width, height,                  // source image size
        m_pFrameBuffer,
        &bmpInfo,
        DIB_RGB_COLORS,
        SRCCOPY);
    // Update FPS display every second
    static DWORD lastTick = GetTickCount();
    static int frameCount = 0;
    frameCount++;
    DWORD currentTick = GetTickCount();
    if (currentTick - lastTick >= 1000) {
        float fps = frameCount * 1000.0f / (currentTick - lastTick);
        CString strFPS;
        strFPS.Format(_T("FPS: %.1f"), fps);
        SetDlgItemText(IDC_STATIC_FPS, strFPS);
        frameCount = 0;
        lastTick = currentTick;
    }
    return 0;
}

// Message handler for camera error (WM_APP+2)
LRESULT CXIMEASensorDiagDlg::OnCameraError(WPARAM wParam, LPARAM lParam)
{
    int errorCode = (int)wParam;
    char* msgPtr = reinterpret_cast<char*>(lParam);
    CString msg;
    if (msgPtr) {
        msg = CString(msgPtr);
    }
    // Determine user-facing message based on error code
    CString userMsg;
    switch (errorCode) {
    case -1:
        userMsg = _T("MQ013MG-ON 카메라를 열 수 없습니다!"); // Cannot open camera
        break;
    case -2:
        userMsg = _T("카메라 스트리밍을 시작할 수 없습니다!"); // Cannot start streaming
        break;
    case -3:
        userMsg = _T("노출 시간을 설정할 수 없습니다!"); // Cannot set exposure time
        break;
    case -4:
        userMsg = _T("ROI 영역을 설정할 수 없습니다!"); // Cannot set ROI region
        break;
    case -5:
        userMsg = _T("카메라 게인을 설정할 수 없습니다!"); // Cannot set camera gain
        break;
    default:
        if (errorCode > 0) {
            userMsg.Format(_T("카메라 오류 발생 (코드 %d)"), errorCode);
        }
        else if (!msg.IsEmpty()) {
            userMsg = CString(_T("오류: ")) + msg;
        }
        else {
            userMsg = _T("알 수 없는 카메라 오류가 발생했습니다.");
        }
        break;
    }
    AfxMessageBox(userMsg, MB_ICONERROR | MB_OK);
    // Clean up allocated message string
    if (msgPtr) {
        delete[] msgPtr;
    }
    // If an error occurred during streaming, stop and clean up
    if (m_isStreaming) {
        OnBnClickedButtonStop();
    }
    return 0;
}
