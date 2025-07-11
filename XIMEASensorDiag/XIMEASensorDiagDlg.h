#pragma once
#include "XIMEASensor.h"
#include <memory>
#include <atomic>
#include <deque>
#include <vector>
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
    afx_msg void OnTimer(UINT_PTR nIDEvent);
    afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
    afx_msg void OnBnClickedCheckAutoExposure();
    afx_msg void OnBnClickedCheckAutoGain();
    DECLARE_MESSAGE_MAP()

private:
    // UI 컨트롤
    CStatic* m_pictureCtrl;
    CSliderCtrl m_sliderExposure;
    CSliderCtrl m_sliderGain;
    CSliderCtrl m_sliderFPS;
    CButton m_checkAutoExposure;
    CButton m_checkAutoGain;
    CEdit m_editLog;

    // 상태
    std::atomic<bool> m_isStreaming;

    // 프레임 버퍼
    struct FrameData {
        std::vector<unsigned char> data;
        int width;
        int height;
        unsigned long frameNumber;
        double timestamp;
    };
    std::deque<FrameData> m_frameQueue;
    std::mutex m_frameMutex;

    // 렌더링
    CDC* m_pMemDC;
    CBitmap* m_pBackBuffer;
    CRect m_displayRect;
    BITMAPINFO m_bmpInfo;

    // 성능 통계
    LARGE_INTEGER m_frequency;
    LARGE_INTEGER m_lastRenderTime;
    int m_renderFrameCount;
    float m_displayFPS;

    // 로그
    std::deque<std::string> m_logMessages;
    std::mutex m_logMutex;

    // 파라미터 캐시
    int m_currentExposure;
    float m_currentGain;
    float m_currentFPS;

    // 타이머 ID
    static const UINT TIMER_UPDATE_UI = 1;
    static const UINT TIMER_RENDER = 2;

    // 콜백 함수 (static)
    static void FrameCallback(const unsigned char* data, int width, int height,
        unsigned long frameNumber, double timestamp, void* userContext);
    static void ErrorCallback(int errorCode, const char* errorMessage, void* userContext);
    static void LogCallback(int logLevel, const char* message, void* userContext);

    // 내부 함수
    void UpdateUI();
    void RenderFrame();
    void AddLogMessage(const std::string& message);
    void UpdateParameterDisplay();

public:
    // 버튼 이벤트 핸들러
    afx_msg void OnBnClickedButtonStart();
    afx_msg void OnBnClickedButtonStop();
    afx_msg void OnBnClickedButtonCapture();
    afx_msg void OnBnClickedButtonSaveConfig();
    afx_msg void OnBnClickedButtonLoadConfig();
};