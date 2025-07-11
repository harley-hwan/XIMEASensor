#pragma once
#include "XIMEASensor.h"
#include <mutex>
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
    // Removed thread usage; using callbacks for streaming
    // CWinThread* m_pStreamThread;

    unsigned char* m_pFrameBuffer; // 힙에 할당된 영상 버퍼
    const int MAX_FRAME_SIZE = 1280 * 1024;
    int m_frameWidth;
    int m_frameHeight;

    // Mutex to protect frame buffer access between threads
    std::mutex m_frameMutex;

    // Callback functions for camera events
    static void FrameCallback(const unsigned char* frame, int width, int height);
    static void ErrorCallback(int errorCode, const char* errorMsg);
    static void LogCallback(const char* logMsg);
    static CXIMEASensorDiagDlg* s_instance;

public:
    afx_msg void OnBnClickedButtonStart();
    afx_msg void OnBnClickedButtonStop();
    afx_msg LRESULT OnFrameReady(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnCameraError(WPARAM wParam, LPARAM lParam);
};
