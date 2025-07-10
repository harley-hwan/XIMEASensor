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
    m_pStreamThread(nullptr),
    m_pFrameBuffer(nullptr)
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
END_MESSAGE_MAP()

BOOL CXIMEASensorDiagDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    m_pictureCtrl = (CStatic*)GetDlgItem(IDC_STATIC_VIDEO);
    m_pFrameBuffer = new unsigned char[MAX_FRAME_SIZE];  // 힙에 버퍼 할당

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
    // MQ013MG-ON 카메라 열기
    if (!Camera_Open(0)) {
        AfxMessageBox(_T("MQ013MG-ON 카메라를 열 수 없습니다!"));
        return;
    }

    // MQ013MG-ON 최적 설정
    // Global Shutter 특성을 활용한 짧은 노출 시간 설정
    Camera_SetExposure(2000);  // 2ms - 고속 촬영에 적합

    // 필요시 게인 조정 (낮은 조도 환경)
    // Camera_SetGain(6.0f);  // 6dB 게인

    // ROI 설정 예시 (전체 해상도 사용)
    // Camera_SetROI(0, 0, 1280, 1024);

    // 카메라 스트리밍 시작
    if (!Camera_Start()) {
        AfxMessageBox(_T("카메라 스트리밍을 시작할 수 없습니다!"));
        Camera_Close();
        return;
    }

    // UI 업데이트
    GetDlgItem(IDC_BUTTON_START)->EnableWindow(FALSE);
    GetDlgItem(IDC_BUTTON_STOP)->EnableWindow(TRUE);

    // 노출 시간 조정 컨트롤 활성화 (있는 경우)
    // GetDlgItem(IDC_SLIDER_EXPOSURE)->EnableWindow(TRUE);

    // 상태 표시
    SetDlgItemText(IDC_STATIC_STATUS, _T("MQ013MG-ON 카메라 실행 중..."));

    // 스트리밍 스레드 시작
    m_isStreaming = true;
    m_pStreamThread = AfxBeginThread(Thread_Stream, this);

    // 스레드 우선순위를 높여서 고속 프레임 처리
    m_pStreamThread->SetThreadPriority(THREAD_PRIORITY_ABOVE_NORMAL);
}

void CXIMEASensorDiagDlg::OnBnClickedButtonStop()
{
    m_isStreaming = false;

    if (m_pStreamThread) {
        WaitForSingleObject(m_pStreamThread->m_hThread, 2000);
        m_pStreamThread = nullptr;
    }

    Camera_Stop();
    Camera_Close();

    Invalidate();
}

UINT CXIMEASensorDiagDlg::Thread_Stream(LPVOID pParam)
{
    auto* pDlg = reinterpret_cast<CXIMEASensorDiagDlg*>(pParam);

    int width = 0, height = 0;

    // 8비트 모노크롬 비트맵 정보 설정
    BITMAPINFO bmpInfo = { 0 };
    bmpInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmpInfo.bmiHeader.biPlanes = 1;
    bmpInfo.bmiHeader.biBitCount = 8;  // 8비트 그레이스케일
    bmpInfo.bmiHeader.biCompression = BI_RGB;

    // 8비트 그레이스케일용 팔레트 설정
    RGBQUAD grayscalePalette[256];
    for (int i = 0; i < 256; i++) {
        grayscalePalette[i].rgbBlue = i;
        grayscalePalette[i].rgbGreen = i;
        grayscalePalette[i].rgbRed = i;
        grayscalePalette[i].rgbReserved = 0;
    }

    // 프레임 카운터 (성능 측정용)
    DWORD lastTickCount = GetTickCount();
    int frameCount = 0;

    // 더블 버퍼링용 로컬 버퍼
    unsigned char* localBuffer = new unsigned char[pDlg->MAX_FRAME_SIZE];

    while (pDlg->m_isStreaming)
    {
        // 카메라에서 프레임 획득
        if (Camera_GetFrame(localBuffer, pDlg->MAX_FRAME_SIZE, &width, &height))
        {
            // 비트맵 헤더 업데이트
            bmpInfo.bmiHeader.biWidth = width;
            bmpInfo.bmiHeader.biHeight = -height;  // top-down 비트맵

            // Picture Control의 DC 획득
            CClientDC dc(pDlg->m_pictureCtrl);

            // 그레이스케일 팔레트 설정
            SetDIBColorTable(dc.GetSafeHdc(), 0, 256, grayscalePalette);

            // 고속 렌더링을 위해 스트레칭 모드 설정
            SetStretchBltMode(dc.GetSafeHdc(), HALFTONE);

            // Picture Control의 크기 획득
            CRect rect;
            pDlg->m_pictureCtrl->GetClientRect(&rect);

            // 이미지가 컨트롤 크기에 맞게 스케일링되도록 렌더링
            StretchDIBits(dc.GetSafeHdc(),
                0, 0, rect.Width(), rect.Height(),  // 대상 영역
                0, 0, width, height,                 // 소스 영역
                localBuffer,                         // 이미지 데이터
                &bmpInfo,                           // 비트맵 정보
                DIB_RGB_COLORS,                     // 색상 사용
                SRCCOPY);                           // 복사 모드

            // FPS 계산 및 표시 (1초마다)
            frameCount++;
            DWORD currentTick = GetTickCount();
            if (currentTick - lastTickCount >= 1000)
            {
                float fps = (float)frameCount * 1000.0f / (currentTick - lastTickCount);
                CString strFPS;
                strFPS.Format(_T("FPS: %.1f"), fps);
                pDlg->SetDlgItemText(IDC_STATIC_FPS, strFPS);  // FPS 표시용 Static 컨트롤 필요

                frameCount = 0;
                lastTickCount = currentTick;
            }
        }

        // MQ013MG-ON은 210 FPS 지원하므로 짧은 대기
        // 약 2ms 대기 (실제 프레임레이트는 카메라가 제어)
        Sleep(2);
    }

    // 로컬 버퍼 해제
    delete[] localBuffer;

    return 0;
}