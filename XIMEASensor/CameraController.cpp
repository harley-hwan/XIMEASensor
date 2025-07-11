#include "pch.h"
#include "CameraController.h"
#include <cstring>

// MQ013MG-ON 카메라 사양
#define PYTHON1300_WIDTH    1280
#define PYTHON1300_HEIGHT   1024
#define PYTHON1300_MAX_FPS  210
#define DEFAULT_EXPOSURE_US 4000  // 4ms (고속 촬영용)

CameraController::CameraController()
    : xiH(nullptr), running(false), frameBuffer(nullptr),
    width(PYTHON1300_WIDTH), height(PYTHON1300_HEIGHT)
{
    // PYTHON1300 센서에 맞는 버퍼 할당
    frameBuffer = new unsigned char[width * height];
}

CameraController::~CameraController()
{
    StopCapture();
    CloseCamera();
    delete[] frameBuffer;
}

CameraController& CameraController::GetInstance()
{
    static CameraController instance;
    return instance;
}

bool CameraController::OpenCamera(int deviceIndex)
{
    if (xiOpenDevice(deviceIndex, &xiH) != XI_OK)
        return false;

    // MQ013MG-ON 전용 설정
    // 픽셀 포맷을 8비트 모노크롬으로 명시적 설정
    xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);

    // PYTHON1300 센서 해상도 설정
    xiSetParamInt(xiH, XI_PRM_WIDTH, PYTHON1300_WIDTH);
    xiSetParamInt(xiH, XI_PRM_HEIGHT, PYTHON1300_HEIGHT);

    // 고속 촬영을 위한 노출 시간 설정 (4ms)
    xiSetParamInt(xiH, XI_PRM_EXPOSURE, DEFAULT_EXPOSURE_US);

    // 게인 초기값 (노이즈 최소화를 위해 0.0)
    xiSetParamFloat(xiH, XI_PRM_GAIN, 0.0f);

    // 프레임레이트 제한 해제 (최대 속도로 동작)
    xiSetParamInt(xiH, XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FREE_RUN);

    // 버퍼 정책 설정 (최신 프레임 우선)
    xiSetParamInt(xiH, XI_PRM_BUFFER_POLICY, XI_BP_UNSAFE);

    // 자동 대역폭 계산 활성화
    xiSetParamInt(xiH, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_ON);

    return true;
}

void CameraController::CloseCamera()
{
    if (xiH) {
        xiCloseDevice(xiH);
        xiH = nullptr;
    }
}

bool CameraController::StartCapture()
{
    if (!xiH) return false;

    if (xiStartAcquisition(xiH) != XI_OK)
        return false;

    running = true;
    captureThread = std::thread(&CameraController::CaptureLoop, this);
    return true;
}

void CameraController::StopCapture()
{
    running = false;
    if (captureThread.joinable())
        captureThread.join();

    if (xiH)
        xiStopAcquisition(xiH);
}

void CameraController::CaptureLoop()
{
    XI_IMG image;
    memset(&image, 0, sizeof(image));
    image.size = sizeof(XI_IMG);

    while (running)
    {
        // 고속 카메라를 위해 타임아웃을 짧게 설정 (100ms)
        XI_RETURN stat = xiGetImage(xiH, 100, &image);
        if (stat == XI_OK)
        {
            std::lock_guard<std::mutex> lock(frameMutex);

            // PYTHON1300은 항상 1280x1024 출력
            memcpy(frameBuffer, image.bp, width * height);

            // 실제 획득된 이미지 정보 저장 (디버깅용)
            actualWidth = image.width;
            actualHeight = image.height;
            frameCounter++;
        }
        else if (stat == XI_TIMEOUT)
        {
            // 타임아웃은 정상적인 상황일 수 있음
            continue;
        }

        // CPU 부하 최소화를 위한 짧은 대기
        // 210 FPS = 약 4.76ms/frame이므로 1ms 대기
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}

bool CameraController::GetFrame(unsigned char* buffer, int bufferSize, int& outWidth, int& outHeight)
{
    std::lock_guard<std::mutex> lock(frameMutex);

    int requiredSize = width * height;
    if (bufferSize < requiredSize)
        return false;

    memcpy(buffer, frameBuffer, requiredSize);
    outWidth = width;
    outHeight = height;
    return true;
}

bool CameraController::SetExposure(int microsec)
{
    // PYTHON1300의 최소/최대 노출 시간 확인
    // Global shutter 특성상 짧은 노출 가능
    if (microsec < 10) microsec = 10;        // 최소 10us
    if (microsec > 1000000) microsec = 1000000; // 최대 1초

    return xiSetParamInt(xiH, XI_PRM_EXPOSURE, microsec) == XI_OK;
}

bool CameraController::SetROI(int offsetX, int offsetY, int w, int h)
{
    // ROI는 4의 배수로 정렬되어야 함 (PYTHON1300 요구사항)
    offsetX = (offsetX / 4) * 4;
    offsetY = (offsetY / 4) * 4;
    w = (w / 4) * 4;
    h = (h / 4) * 4;

    // 최소 크기 제한
    if (w < 32) w = 32;
    if (h < 32) h = 32;

    // 경계 체크
    if (offsetX + w > PYTHON1300_WIDTH) w = PYTHON1300_WIDTH - offsetX;
    if (offsetY + h > PYTHON1300_HEIGHT) h = PYTHON1300_HEIGHT - offsetY;

    xiSetParamInt(xiH, XI_PRM_OFFSET_X, offsetX);
    xiSetParamInt(xiH, XI_PRM_OFFSET_Y, offsetY);
    xiSetParamInt(xiH, XI_PRM_WIDTH, w);
    xiSetParamInt(xiH, XI_PRM_HEIGHT, h);

    // 내부 버퍼 크기도 업데이트
    width = w;
    height = h;

    return true;
}

bool CameraController::SetGain(float gain)
{
    // PYTHON1300의 게인 범위 (일반적으로 0.0 ~ 24.0 dB)
    if (gain < 0.0f) gain = 0.0f;
    if (gain > 24.0f) gain = 24.0f;

    return xiSetParamFloat(xiH, XI_PRM_GAIN, gain) == XI_OK;
}

// 추가 함수: 현재 프레임레이트 확인
float CameraController::GetFramerate()
{
    if (!xiH) return 0.0f;

    float fps = 0.0f;
    xiGetParamFloat(xiH, XI_PRM_FRAMERATE, &fps);
    return fps;
}

// 추가 함수: 프레임 카운터 확인 (디버깅용)
unsigned long CameraController::GetFrameCounter()
{
    std::lock_guard<std::mutex> lock(frameMutex);
    return frameCounter;
}