#pragma once
#include "XIMEASensor.h"

class CXIMEASensorDiagDlg : public CDialogEx
{
public:
    CXIMEASensorDiagDlg(CWnd* pParent = nullptr);
    virtual ~CXIMEASensorDiagDlg();

#ifdef AFX_DESIGN_TIME
    enum { IDD = IDD_XIMEASENSORDIAG_DIALOG };
#endif

protected:
    virtual void DoDataExchange(CDataExchange* pDX);
    virtual BOOL OnInitDialog();
    afx_msg void OnPaint();
    afx_msg HCURSOR OnQueryDragIcon();
    DECLARE_MESSAGE_MAP()

private:
    CStatic* m_pictureCtrl;
    bool m_isStreaming;
    CWinThread* m_pStreamThread;

    unsigned char* m_pFrameBuffer; // 힙에 할당된 영상 버퍼
    const int MAX_FRAME_SIZE = 1280 * 1024;

    static UINT Thread_Stream(LPVOID pParam);

public:
    afx_msg void OnBnClickedButtonStart();
    afx_msg void OnBnClickedButtonStop();
};
