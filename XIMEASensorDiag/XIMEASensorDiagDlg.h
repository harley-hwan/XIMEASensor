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
    // UI ��Ʈ��
    CStatic* m_pictureCtrl;
    CSliderCtrl m_sliderExposure;
    CSliderCtrl m_sliderGain;
    CSliderCtrl m_sliderFPS;
    CButton m_checkAutoExposure;
    CButton m_checkAutoGain;
    CEdit m_editLog;

    // ����
    std::atomic<bool> m_isStreaming;

    // ������ ����
    struct FrameData {
        std::vector<unsigned char> data;
        int width;
        int height;
        unsigned long frameNumber;
        double timestamp;
    };
    std::deque<FrameData> m_frameQueue;
    std::mutex m_frameMutex;

    // ������
    CDC* m_pMemDC;
    CBitmap* m_pBackBuffer;
    CRect m_displayRect;
    BITMAPINFO m_bmpInfo;

    // ���� ���
    LARGE_INTEGER m_frequency;
    LARGE_INTEGER m_lastRenderTime;
    int m_renderFrameCount;
    float m_displayFPS;

    // �α�
    std::deque<std::string> m_logMessages;
    std::mutex m_logMutex;

    // �Ķ���� ĳ��
    int m_currentExposure;
    float m_currentGain;
    float m_currentFPS;

    // Ÿ�̸� ID
    static const UINT TIMER_UPDATE_UI = 1;
    static const UINT TIMER_RENDER = 2;

    // �ݹ� �Լ� (static)
    static void FrameCallback(const unsigned char* data, int width, int height,
        unsigned long frameNumber, double timestamp, void* userContext);
    static void ErrorCallback(int errorCode, const char* errorMessage, void* userContext);
    static void LogCallback(int logLevel, const char* message, void* userContext);

    // ���� �Լ�
    void UpdateUI();
    void RenderFrame();
    void AddLogMessage(const std::string& message);
    void UpdateParameterDisplay();

public:
    // ��ư �̺�Ʈ �ڵ鷯
    afx_msg void OnBnClickedButtonStart();
    afx_msg void OnBnClickedButtonStop();
    afx_msg void OnBnClickedButtonCapture();
    afx_msg void OnBnClickedButtonSaveConfig();
    afx_msg void OnBnClickedButtonLoadConfig();
};