#pragma once
#include "XIMEASensor.h"
#include "IXIMEACallback.h"
#include "CameraCallback.h"
#include <memory>
#include <chrono>
#include <atomic>

#define WM_UPDATE_FRAME     (WM_USER + 100)
#define WM_UPDATE_STATUS    (WM_USER + 101)
#define WM_UPDATE_ERROR     (WM_USER + 102)
#define WM_UPDATE_FPS       (WM_USER + 103)
#define WM_CONTINUOUS_CAPTURE_COMPLETE (WM_USER + 104)

#define TIMER_CONTINUOUS_CAPTURE 1002

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
    CStatic* m_pictureCtrl = nullptr;
    CButton* m_btnStart = nullptr;
    CButton* m_btnStop = nullptr;
    CButton* m_btnSnapshot = nullptr;
    CButton* m_checkContinuous = nullptr;
    CStatic* m_staticStatus = nullptr;
    CStatic* m_staticFPS = nullptr;
    CSliderCtrl* m_sliderExposure = nullptr;
    CSliderCtrl* m_sliderGain = nullptr;
    CComboBox* m_comboDevices = nullptr;

    // Camera
    std::unique_ptr<CameraCallback> m_cameraCallback;
    std::atomic<bool> m_isStreaming;

    // frame buffer
    CRITICAL_SECTION m_frameCriticalSection;
    unsigned char* m_pDisplayBuffer;
    int m_displayWidth;
    int m_displayHeight;
    bool m_hasNewFrame;

    std::chrono::steady_clock::time_point m_lastFPSUpdate;
    int m_frameCount;
    double m_currentFPS;

    void UpdateDeviceList();
    void UpdateUI(bool isStreaming);
    void DrawFrame();
    void ShowError(const CString& message);
    void SyncSlidersWithCamera();

    // Callback handler
    void OnFrameReceivedCallback(const FrameInfo& frameInfo);
    void OnStateChangedCallback(CameraState newState, CameraState oldState);
    void OnErrorCallback(CameraError error, const std::string& errorMessage);
    void OnPropertyChangedCallback(const std::string& propertyName, const std::string& value);

    static void ContinuousCaptureProgressCallback(int currentFrame, double elapsedSeconds, int state);
    static CXIMEASensorDiagDlg* s_pThis;
    void OnContinuousCaptureProgress(int currentFrame, double elapsedSeconds, int state);

public:
    afx_msg void OnBnClickedButtonStart();
    afx_msg void OnBnClickedButtonStop();
    afx_msg void OnBnClickedButtonRefresh();
    afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
    afx_msg void OnCbnSelchangeComboDevices();
    afx_msg void OnBnClickedButtonSnapshot();
    afx_msg LRESULT OnContinuousCaptureComplete(WPARAM wParam, LPARAM lParam);
    afx_msg void OnBnClickedButtonSettings();

    afx_msg LRESULT OnUpdateFrame(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateStatus(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateError(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateFPS(WPARAM wParam, LPARAM lParam);

    afx_msg void OnDestroy();
    afx_msg void OnTimer(UINT_PTR nIDEvent);
};