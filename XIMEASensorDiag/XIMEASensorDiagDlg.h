#pragma once
#include "XIMEASensor.h"
#include "IXIMEACallback.h"
#include "CameraCallback.h"
#include <memory>
#include <chrono>

#define WM_UPDATE_FRAME     (WM_USER + 100)
#define WM_UPDATE_STATUS    (WM_USER + 101)
#define WM_UPDATE_ERROR     (WM_USER + 102)
#define WM_UPDATE_FPS       (WM_USER + 103)

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
    // UI 컨트롤
    CStatic* m_pictureCtrl;
    CButton* m_btnStart;
    CButton* m_btnStop;
    CStatic* m_staticStatus;
    CStatic* m_staticFPS;
    CSliderCtrl* m_sliderExposure;
    CSliderCtrl* m_sliderGain;
    CComboBox* m_comboDevices;

    // Camera
    std::unique_ptr<CameraCallback> m_cameraCallback;
    bool m_isStreaming;

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

    // Callback handler (다른 스레드에서 호출됨)
    void OnFrameReceivedCallback(const FrameInfo& frameInfo);
    void OnStateChangedCallback(CameraState newState, CameraState oldState);
    void OnErrorCallback(CameraError error, const std::string& errorMessage);
    void OnPropertyChangedCallback(const std::string& propertyName, const std::string& value);

public:
    afx_msg void OnBnClickedButtonStart();
    afx_msg void OnBnClickedButtonStop();
    afx_msg void OnBnClickedButtonRefresh();
    afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
    afx_msg void OnCbnSelchangeComboDevices();
    afx_msg void OnBnClickedButtonSnapshot();
    afx_msg void OnBnClickedButtonSettings();

    afx_msg LRESULT OnUpdateFrame(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateStatus(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateError(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnUpdateFPS(WPARAM wParam, LPARAM lParam);

    afx_msg void OnDestroy();
    afx_msg void OnTimer(UINT_PTR nIDEvent);
};