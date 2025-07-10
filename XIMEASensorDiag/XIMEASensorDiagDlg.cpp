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
    m_pFrameBuffer = new unsigned char[MAX_FRAME_SIZE];  // ���� ���� �Ҵ�

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
    // MQ013MG-ON ī�޶� ����
    if (!Camera_Open(0)) {
        AfxMessageBox(_T("MQ013MG-ON ī�޶� �� �� �����ϴ�!"));
        return;
    }

    // MQ013MG-ON ���� ����
    // Global Shutter Ư���� Ȱ���� ª�� ���� �ð� ����
    Camera_SetExposure(2000);  // 2ms - ��� �Կ��� ����

    // �ʿ�� ���� ���� (���� ���� ȯ��)
    // Camera_SetGain(6.0f);  // 6dB ����

    // ROI ���� ���� (��ü �ػ� ���)
    // Camera_SetROI(0, 0, 1280, 1024);

    // ī�޶� ��Ʈ���� ����
    if (!Camera_Start()) {
        AfxMessageBox(_T("ī�޶� ��Ʈ������ ������ �� �����ϴ�!"));
        Camera_Close();
        return;
    }

    // UI ������Ʈ
    GetDlgItem(IDC_BUTTON_START)->EnableWindow(FALSE);
    GetDlgItem(IDC_BUTTON_STOP)->EnableWindow(TRUE);

    // ���� �ð� ���� ��Ʈ�� Ȱ��ȭ (�ִ� ���)
    // GetDlgItem(IDC_SLIDER_EXPOSURE)->EnableWindow(TRUE);

    // ���� ǥ��
    SetDlgItemText(IDC_STATIC_STATUS, _T("MQ013MG-ON ī�޶� ���� ��..."));

    // ��Ʈ���� ������ ����
    m_isStreaming = true;
    m_pStreamThread = AfxBeginThread(Thread_Stream, this);

    // ������ �켱������ ������ ��� ������ ó��
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

    // 8��Ʈ ���ũ�� ��Ʈ�� ���� ����
    BITMAPINFO bmpInfo = { 0 };
    bmpInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmpInfo.bmiHeader.biPlanes = 1;
    bmpInfo.bmiHeader.biBitCount = 8;  // 8��Ʈ �׷��̽�����
    bmpInfo.bmiHeader.biCompression = BI_RGB;

    // 8��Ʈ �׷��̽����Ͽ� �ȷ�Ʈ ����
    RGBQUAD grayscalePalette[256];
    for (int i = 0; i < 256; i++) {
        grayscalePalette[i].rgbBlue = i;
        grayscalePalette[i].rgbGreen = i;
        grayscalePalette[i].rgbRed = i;
        grayscalePalette[i].rgbReserved = 0;
    }

    // ������ ī���� (���� ������)
    DWORD lastTickCount = GetTickCount();
    int frameCount = 0;

    // ���� ���۸��� ���� ����
    unsigned char* localBuffer = new unsigned char[pDlg->MAX_FRAME_SIZE];

    while (pDlg->m_isStreaming)
    {
        // ī�޶󿡼� ������ ȹ��
        if (Camera_GetFrame(localBuffer, pDlg->MAX_FRAME_SIZE, &width, &height))
        {
            // ��Ʈ�� ��� ������Ʈ
            bmpInfo.bmiHeader.biWidth = width;
            bmpInfo.bmiHeader.biHeight = -height;  // top-down ��Ʈ��

            // Picture Control�� DC ȹ��
            CClientDC dc(pDlg->m_pictureCtrl);

            // �׷��̽����� �ȷ�Ʈ ����
            SetDIBColorTable(dc.GetSafeHdc(), 0, 256, grayscalePalette);

            // ��� �������� ���� ��Ʈ��Ī ��� ����
            SetStretchBltMode(dc.GetSafeHdc(), HALFTONE);

            // Picture Control�� ũ�� ȹ��
            CRect rect;
            pDlg->m_pictureCtrl->GetClientRect(&rect);

            // �̹����� ��Ʈ�� ũ�⿡ �°� �����ϸ��ǵ��� ������
            StretchDIBits(dc.GetSafeHdc(),
                0, 0, rect.Width(), rect.Height(),  // ��� ����
                0, 0, width, height,                 // �ҽ� ����
                localBuffer,                         // �̹��� ������
                &bmpInfo,                           // ��Ʈ�� ����
                DIB_RGB_COLORS,                     // ���� ���
                SRCCOPY);                           // ���� ���

            // FPS ��� �� ǥ�� (1�ʸ���)
            frameCount++;
            DWORD currentTick = GetTickCount();
            if (currentTick - lastTickCount >= 1000)
            {
                float fps = (float)frameCount * 1000.0f / (currentTick - lastTickCount);
                CString strFPS;
                strFPS.Format(_T("FPS: %.1f"), fps);
                pDlg->SetDlgItemText(IDC_STATIC_FPS, strFPS);  // FPS ǥ�ÿ� Static ��Ʈ�� �ʿ�

                frameCount = 0;
                lastTickCount = currentTick;
            }
        }

        // MQ013MG-ON�� 210 FPS �����ϹǷ� ª�� ���
        // �� 2ms ��� (���� �����ӷ���Ʈ�� ī�޶� ����)
        Sleep(2);
    }

    // ���� ���� ����
    delete[] localBuffer;

    return 0;
}