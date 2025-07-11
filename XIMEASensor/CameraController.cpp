#include "pch.h"
#include "CameraController.h"
#include <cstring>

// MQ013MG-ON ī�޶� ���
#define PYTHON1300_WIDTH    1280
#define PYTHON1300_HEIGHT   1024
#define PYTHON1300_MAX_FPS  210
#define DEFAULT_EXPOSURE_US 4000  // 4ms (��� �Կ���)

CameraController::CameraController()
    : xiH(nullptr), running(false), frameBuffer(nullptr),
    width(PYTHON1300_WIDTH), height(PYTHON1300_HEIGHT)
{
    // PYTHON1300 ������ �´� ���� �Ҵ�
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

    // MQ013MG-ON ���� ����
    // �ȼ� ������ 8��Ʈ ���ũ������ ����� ����
    xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);

    // PYTHON1300 ���� �ػ� ����
    xiSetParamInt(xiH, XI_PRM_WIDTH, PYTHON1300_WIDTH);
    xiSetParamInt(xiH, XI_PRM_HEIGHT, PYTHON1300_HEIGHT);

    // ��� �Կ��� ���� ���� �ð� ���� (4ms)
    xiSetParamInt(xiH, XI_PRM_EXPOSURE, DEFAULT_EXPOSURE_US);

    // ���� �ʱⰪ (������ �ּ�ȭ�� ���� 0.0)
    xiSetParamFloat(xiH, XI_PRM_GAIN, 0.0f);

    // �����ӷ���Ʈ ���� ���� (�ִ� �ӵ��� ����)
    xiSetParamInt(xiH, XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FREE_RUN);

    // ���� ��å ���� (�ֽ� ������ �켱)
    xiSetParamInt(xiH, XI_PRM_BUFFER_POLICY, XI_BP_UNSAFE);

    // �ڵ� �뿪�� ��� Ȱ��ȭ
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
        // ��� ī�޶� ���� Ÿ�Ӿƿ��� ª�� ���� (100ms)
        XI_RETURN stat = xiGetImage(xiH, 100, &image);
        if (stat == XI_OK)
        {
            std::lock_guard<std::mutex> lock(frameMutex);

            // PYTHON1300�� �׻� 1280x1024 ���
            memcpy(frameBuffer, image.bp, width * height);

            // ���� ȹ��� �̹��� ���� ���� (������)
            actualWidth = image.width;
            actualHeight = image.height;
            frameCounter++;
        }
        else if (stat == XI_TIMEOUT)
        {
            // Ÿ�Ӿƿ��� �������� ��Ȳ�� �� ����
            continue;
        }

        // CPU ���� �ּ�ȭ�� ���� ª�� ���
        // 210 FPS = �� 4.76ms/frame�̹Ƿ� 1ms ���
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
    // PYTHON1300�� �ּ�/�ִ� ���� �ð� Ȯ��
    // Global shutter Ư���� ª�� ���� ����
    if (microsec < 10) microsec = 10;        // �ּ� 10us
    if (microsec > 1000000) microsec = 1000000; // �ִ� 1��

    return xiSetParamInt(xiH, XI_PRM_EXPOSURE, microsec) == XI_OK;
}

bool CameraController::SetROI(int offsetX, int offsetY, int w, int h)
{
    // ROI�� 4�� ����� ���ĵǾ�� �� (PYTHON1300 �䱸����)
    offsetX = (offsetX / 4) * 4;
    offsetY = (offsetY / 4) * 4;
    w = (w / 4) * 4;
    h = (h / 4) * 4;

    // �ּ� ũ�� ����
    if (w < 32) w = 32;
    if (h < 32) h = 32;

    // ��� üũ
    if (offsetX + w > PYTHON1300_WIDTH) w = PYTHON1300_WIDTH - offsetX;
    if (offsetY + h > PYTHON1300_HEIGHT) h = PYTHON1300_HEIGHT - offsetY;

    xiSetParamInt(xiH, XI_PRM_OFFSET_X, offsetX);
    xiSetParamInt(xiH, XI_PRM_OFFSET_Y, offsetY);
    xiSetParamInt(xiH, XI_PRM_WIDTH, w);
    xiSetParamInt(xiH, XI_PRM_HEIGHT, h);

    // ���� ���� ũ�⵵ ������Ʈ
    width = w;
    height = h;

    return true;
}

bool CameraController::SetGain(float gain)
{
    // PYTHON1300�� ���� ���� (�Ϲ������� 0.0 ~ 24.0 dB)
    if (gain < 0.0f) gain = 0.0f;
    if (gain > 24.0f) gain = 24.0f;

    return xiSetParamFloat(xiH, XI_PRM_GAIN, gain) == XI_OK;
}

// �߰� �Լ�: ���� �����ӷ���Ʈ Ȯ��
float CameraController::GetFramerate()
{
    if (!xiH) return 0.0f;

    float fps = 0.0f;
    xiGetParamFloat(xiH, XI_PRM_FRAMERATE, &fps);
    return fps;
}

// �߰� �Լ�: ������ ī���� Ȯ�� (������)
unsigned long CameraController::GetFrameCounter()
{
    std::lock_guard<std::mutex> lock(frameMutex);
    return frameCounter;
}