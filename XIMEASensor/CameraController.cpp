#include "pch.h"
#include "CameraController.h"
#include <algorithm>
#include <sstream>
#include <cstring>
#include <iomanip>

// MQ013MG-ON 카메라 사양
#define PYTHON1300_WIDTH    1280
#define PYTHON1300_HEIGHT   1024
#define PYTHON1300_MAX_FPS  210
#define DEFAULT_EXPOSURE_US 4000
#define MIN_EXPOSURE_US     10
#define MAX_EXPOSURE_US     1000000

std::unique_ptr<CameraController> CameraController::instance = nullptr;
std::mutex CameraController::instanceMutex;

CameraController::CameraController()
    : xiH(nullptr),
    isRunning(false),
    isPaused(false),
    frameBuffer(nullptr),
    workingBuffer(nullptr),
    width(PYTHON1300_WIDTH),
    height(PYTHON1300_HEIGHT),
    currentExposure(DEFAULT_EXPOSURE_US),
    currentGain(0.0f),
    currentState(CameraState::DISCONNECTED) {

    size_t bufferSize = width * height;
    frameBuffer = new unsigned char[bufferSize];
    workingBuffer = new unsigned char[bufferSize];

    stats.Reset();

    LOG_INFO("CameraController initialized");
}

CameraController::~CameraController() {
    LOG_INFO("CameraController destructor called");

    try {
        // 콜백 제거 (콜백 중에 새로운 작업이 시작되는 것을 방지)
        {
            std::lock_guard<std::mutex> lock(callbackMutex);
            callbacks.clear();
        }

        // 캡처 중이면 안전하게 정지
        if (isRunning.load()) {
            isRunning = false;

            // 스레드가 종료될 때까지 최대 3초 대기
            if (captureThread.joinable()) {
                // condition_variable로 스레드 깨우기 (만약 대기 중이라면)
                isPaused = false;

                // join with timeout 구현
                auto start = std::chrono::steady_clock::now();
                while (captureThread.joinable()) {
                    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(3)) {
                        LOG_ERROR("Capture thread did not stop in time");
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    if (!captureThread.joinable()) {
                        break;
                    }

                    captureThread.join();
                }
            }
        }

        // 카메라가 열려있으면 닫기
        if (xiH != nullptr) {
            // 먼저 acquisition 중지 시도
            XI_RETURN stat = xiStopAcquisition(xiH);
            // 에러는 무시 (이미 중지된 상태일 수 있음)

            // 잠시 대기
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // 카메라 닫기
            stat = xiCloseDevice(xiH);
            if (stat != XI_OK) {
                LOG_ERROR("Error closing camera in destructor: " + GetXiApiErrorString(stat));
            }
            xiH = nullptr;
        }

        // 버퍼 해제
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            if (frameBuffer) {
                delete[] frameBuffer;
                frameBuffer = nullptr;
            }

            if (workingBuffer) {
                delete[] workingBuffer;
                workingBuffer = nullptr;
            }
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in destructor: " + std::string(e.what()));
    }
    catch (...) {
        LOG_ERROR("Unknown exception in destructor");
    }

    LOG_INFO("CameraController destroyed");
}

CameraController& CameraController::GetInstance() {
    std::lock_guard<std::mutex> lock(instanceMutex);
    if (!instance) {
        instance = std::unique_ptr<CameraController>(new CameraController());
    }
    return *instance;
}

void CameraController::Destroy() {
    std::lock_guard<std::mutex> lock(instanceMutex);
    if (instance) {
        instance.reset();
    }
}

bool CameraController::OpenCamera(int deviceIndex) {
    LOG_INFO("Opening camera with index: " + std::to_string(deviceIndex));

    if (xiH != nullptr) {
        LOG_WARNING("Camera already open");
        return true;
    }

    XI_RETURN stat = xiOpenDevice(deviceIndex, &xiH);
    if (stat != XI_OK) {
        LOG_ERROR("Failed to open camera: " + GetXiApiErrorString(stat));
        NotifyError(static_cast<CameraError>(stat), "Failed to open camera");
        return false;
    }

    // MQ013MG-ON 초기 설정
    LOG_INFO("Configuring MQ013MG-ON camera");

    // 픽셀 포맷 설정 - XI_MONO8 사용
    stat = xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);
    if (stat != XI_OK) {
        LOG_WARNING("Failed to set image format: " + GetXiApiErrorString(stat));
    }

    // PYTHON1300 센서는 10비트이지만 8비트로 출력 설정
    xiSetParamInt(xiH, XI_PRM_OUTPUT_DATA_BIT_DEPTH, XI_BPP_8);
    xiSetParamInt(xiH, XI_PRM_SENSOR_DATA_BIT_DEPTH, XI_BPP_10);

    // 해상도 설정
    xiSetParamInt(xiH, XI_PRM_WIDTH, PYTHON1300_WIDTH);
    xiSetParamInt(xiH, XI_PRM_HEIGHT, PYTHON1300_HEIGHT);

    // 노출 시간 설정
    xiSetParamInt(xiH, XI_PRM_EXPOSURE, DEFAULT_EXPOSURE_US);
    currentExposure = DEFAULT_EXPOSURE_US;

    // 게인 설정
    xiSetParamFloat(xiH, XI_PRM_GAIN, 0.0f);
    currentGain = 0.0f;

    // 타이밍 모드 설정
    xiSetParamInt(xiH, XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FREE_RUN);

    // 버퍼 정책 설정 - SAFE 모드로 변경하여 안정성 향상
    xiSetParamInt(xiH, XI_PRM_BUFFER_POLICY, XI_BP_SAFE);

    // 자동 대역폭 계산
    xiSetParamInt(xiH, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_ON);

    // 트리거 모드 OFF (연속 캡처)
    xiSetParamInt(xiH, XI_PRM_TRG_SOURCE, XI_TRG_OFF);

    // 다운샘플링 비활성화
    xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING, XI_DWN_1x1);
    xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING_TYPE, XI_BINNING);

    // 센서 탭 설정 (PYTHON1300은 보통 2탭)
    xiSetParamInt(xiH, XI_PRM_SENSOR_TAPS, XI_TAP_CNT_2);

    // 버퍼 큐 크기 설정 - 더 크게 설정하여 프레임 드롭 방지
    xiSetParamInt(xiH, XI_PRM_BUFFERS_QUEUE_SIZE, 20);

    // 최근 프레임 모드 활성화 (항상 최신 프레임 제공)
    xiSetParamInt(xiH, XI_PRM_RECENT_FRAME, XI_ON);

    // 추가 화질 개선 설정
    // 감마 보정 비활성화 (raw 데이터 유지)
    xiSetParamFloat(xiH, XI_PRM_GAMMAY, 1.0f);

    // 샤프닝 비활성화 (raw 데이터 유지)
    xiSetParamFloat(xiH, XI_PRM_SHARPNESS, 0.0f);

    // 노이즈 감소 비활성화 (raw 데이터 유지)
    xiSetParamInt(xiH, XI_PRM_HDR, XI_OFF);

    // 블랙 레벨 자동 조정
    xiSetParamInt(xiH, XI_PRM_AUTO_WB, XI_OFF);
    xiSetParamInt(xiH, XI_PRM_MANUAL_WB, XI_OFF);

    // 센서 결함 픽셀 보정 활성화
    //xiSetParamInt(xiH, XI_PRM_SENSOR_DEFECTS_CORR, XI_ON);
    xiSetParamInt(xiH, XI_PRM_SENS_DEFECTS_CORR, XI_ON);

    // 색상 필터 배열 설정 (모노크롬 센서이므로 NONE)
    xiSetParamInt(xiH, XI_PRM_COLOR_FILTER_ARRAY, XI_CFA_NONE);

    // 전송 포맷 설정
    xiSetParamInt(xiH, XI_PRM_TRANSPORT_PIXEL_FORMAT, XI_GenTL_Image_Format_Mono8);

    // 버퍼 크기 재할당 - 안전하게 처리
    {
        std::lock_guard<std::mutex> lock(frameMutex);

        if (frameBuffer) {
            delete[] frameBuffer;
            frameBuffer = nullptr;
        }
        if (workingBuffer) {
            delete[] workingBuffer;
            workingBuffer = nullptr;
        }

        size_t bufferSize = PYTHON1300_WIDTH * PYTHON1300_HEIGHT;
        try {
            frameBuffer = new unsigned char[bufferSize];
            workingBuffer = new unsigned char[bufferSize];

            // 버퍼 초기화
            memset(frameBuffer, 0, bufferSize);
            memset(workingBuffer, 0, bufferSize);
        }
        catch (const std::bad_alloc& e) {
            LOG_ERROR("Failed to allocate frame buffers: " + std::string(e.what()));

            // 정리
            if (frameBuffer) {
                delete[] frameBuffer;
                frameBuffer = nullptr;
            }
            if (workingBuffer) {
                delete[] workingBuffer;
                workingBuffer = nullptr;
            }

            // 카메라 닫기
            xiCloseDevice(xiH);
            xiH = nullptr;

            return false;
        }
    }

    // 상태 변경 알림
    CameraState oldState = currentState.exchange(CameraState::CONNECTED);
    NotifyStateChanged(CameraState::CONNECTED);

    LOG_INFO("Camera opened successfully");
    return true;
}


void CameraController::CloseCamera() {
    LOG_INFO("Closing camera");

    if (!xiH) {
        LOG_WARNING("Camera already closed");
        return;
    }

    // 캡처 중이면 먼저 중지
    if (isRunning.load()) {
        StopCapture();
    }

    // XIMEA 디바이스 닫기
    XI_RETURN stat = xiCloseDevice(xiH);
    if (stat != XI_OK) {
        LOG_ERROR("Error closing camera: " + GetXiApiErrorString(stat));
    }

    xiH = nullptr;

    // 상태 변경
    CameraState oldState = currentState.exchange(CameraState::DISCONNECTED);
    if (oldState != CameraState::DISCONNECTED) {
        NotifyStateChanged(CameraState::DISCONNECTED);
    }

    LOG_INFO("Camera closed");
}

bool CameraController::StartCapture() {
    if (!xiH) {
        LOG_ERROR("Camera not opened");
        NotifyError(CameraError::START_FAILED, "Camera not opened");
        return false;
    }

    if (isRunning) {
        LOG_WARNING("Capture already running");
        return true;
    }

    LOG_INFO("Starting capture");

    XI_RETURN stat = xiStartAcquisition(xiH);
    if (stat != XI_OK) {
        LOG_ERROR("Failed to start acquisition: " + GetXiApiErrorString(stat));
        NotifyError(CameraError::START_FAILED, "Failed to start acquisition");
        return false;
    }

    stats.Reset();
    lastFrameTime = std::chrono::steady_clock::now();
    deviceNotReadyCount = 0;

    isRunning = true;
    isPaused = false;
    captureThread = std::thread(&CameraController::CaptureLoop, this);

    CameraState oldState = currentState.exchange(CameraState::CAPTURING);
    NotifyStateChanged(CameraState::CAPTURING);

    LOG_INFO("Capture started successfully");
    return true;
}

void CameraController::StopCapture() {
    LOG_INFO("Stopping capture");

    // 캡처 루프 종료 신호
    bool wasRunning = isRunning.exchange(false);
    // 이미 isRunning이 false였지만 스레드가 join되지 않은 경우 처리
    if (!wasRunning) {
        if (captureThread.joinable()) {
            try {
                captureThread.join();
            }
            catch (const std::system_error& e) {
                LOG_ERROR("Exception joining capture thread: " + std::string(e.what()));
            }
        }
        return;  // 더 이상 캡처 중이 아니므로 종료
    }

    // 캡처 스레드 조인 (실행 중이었으면 여기서 대기)
    if (captureThread.joinable()) {
        try {
            captureThread.join();
        }
        catch (const std::system_error& e) {
            LOG_ERROR("Exception joining capture thread: " + std::string(e.what()));
        }
    }

    // 스트리밍 중이었다면 XIMEA API에 캡처 중지 호출
    if (xiH) {
        XI_RETURN stat = xiStopAcquisition(xiH);
        if (stat != XI_OK) {
            LOG_ERROR("Error stopping acquisition: " + GetXiApiErrorString(stat));
        }
    }

    // 상태를 CAPTURING에서 CONNECTED로 변경 (오류로 인한 중단 경우 kERROR 상태는 덮어쓰지 않음)
    CameraState prevState = currentState.exchange(CameraState::CONNECTED);
    if (prevState == CameraState::CAPTURING) {
        NotifyStateChanged(CameraState::CONNECTED);
    }

    LOG_INFO("Capture stopped");
}

void CameraController::PauseCapture(bool pause) {
    isPaused = pause;
    LOG_INFO(pause ? "Capture paused" : "Capture resumed");
}

void CameraController::CaptureLoop() {
    XI_IMG image;
    memset(&image, 0, sizeof(image));
    image.size = sizeof(XI_IMG);

    LOG_INFO("Capture loop started");

    while (isRunning) {
        if (isPaused) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        XI_RETURN stat = xiGetImage(xiH, 100, &image);

        if (stat == XI_OK) {
            // 성공적으로 프레임을 받았으므로 에러 카운트 리셋
            deviceNotReadyCount = 0;

            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                now - lastFrameTime).count();
            float currentFPS = duration > 0 ? 1000000.0f / duration : 0.0f;
            lastFrameTime = now;

            if (image.frm != XI_MONO8) {
                LOG_ERROR("Unexpected image format: " + std::to_string(image.frm));
                continue;
            }

            int imageSize = image.width * image.height;

            if (image.bp == nullptr || imageSize <= 0) {
                LOG_ERROR("Invalid image data received");
                continue;
            }

            FrameInfo frameInfo;
            frameInfo.data = (unsigned char*)image.bp;
            frameInfo.width = image.width;
            frameInfo.height = image.height;
            frameInfo.frameNumber = image.nframe;
            frameInfo.acqFrameNumber = image.acq_nframe;
            frameInfo.timestamp = image.tsSec + (image.tsUSec / 1000000.0);
            frameInfo.currentFPS = currentFPS;
            frameInfo.blackLevel = image.black_level;
            frameInfo.GPI_level = image.GPI_level;
            frameInfo.exposureTime_ms = image.exposure_time_us / 1000.0f;
            frameInfo.gain_db = image.gain_db;
            frameInfo.format = image.frm;

            {
                std::lock_guard<std::mutex> lock(frameMutex);

                if (width != image.width || height != image.height) {
                    width = image.width;
                    height = image.height;

                    delete[] frameBuffer;
                    delete[] workingBuffer;

                    frameBuffer = new unsigned char[imageSize];
                    workingBuffer = new unsigned char[imageSize];
                }

                memcpy(frameBuffer, image.bp, imageSize);
            }

            NotifyFrameReceived(frameInfo);
            UpdateStatistics(true);
        }
        else if (stat == XI_TIMEOUT) {
            // 타임아웃은 정상, 에러 카운트 리셋
            deviceNotReadyCount = 0;
            UpdateStatistics(false);
        }
        else if (stat == XI_DEVICE_NOT_READY) {
            // Device not ready 에러 처리
            deviceNotReadyCount++;
            LOG_ERROR("Frame grab error: " + GetXiApiErrorString(stat) +
                " (count: " + std::to_string(deviceNotReadyCount.load()) + ")");

            if (deviceNotReadyCount >= MAX_DEVICE_NOT_READY_ERRORS) {
                LOG_ERROR("Device not ready error exceeded limit. Stopping capture.");

                // 에러 콜백 호출
                NotifyError(CameraError::DEVICE_NOT_READY,
                    "Camera disconnected - Device not ready error exceeded limit");

                // 캡처 중지
                isRunning = false;

                // 상태 변경 알림
                CameraState oldState = currentState.exchange(CameraState::kERROR);
                NotifyStateChanged(CameraState::kERROR);

                break;
            }

            UpdateStatistics(false);
        }
        else {
            // 다른 에러는 기존처럼 처리
            LOG_ERROR("Frame grab error: " + GetXiApiErrorString(stat));
            NotifyError(static_cast<CameraError>(stat),
                "Frame grab error: " + GetXiApiErrorString(stat));
            UpdateStatistics(false);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    LOG_INFO("Capture loop ended");
}


bool CameraController::GetFrame(unsigned char* buffer, int bufferSize, int& outWidth, int& outHeight) {
    if (!buffer || bufferSize <= 0) {
        LOG_ERROR("Invalid buffer provided");
        return false;
    }

    std::lock_guard<std::mutex> lock(frameMutex);

    int currentFrameSize = width * height;

    if (currentFrameSize <= 0) {
        LOG_ERROR("Invalid frame dimensions: " + std::to_string(width) + "x" + std::to_string(height));
        return false;
    }

    if (bufferSize < currentFrameSize) {
        LOG_ERROR("Buffer size too small: " + std::to_string(bufferSize) +
            " < " + std::to_string(currentFrameSize));
        return false;
    }

    if (!frameBuffer) {
        LOG_ERROR("Frame buffer not initialized");
        return false;
    }

    memcpy(buffer, frameBuffer, currentFrameSize);
    outWidth = width;
    outHeight = height;

    return true;
}

bool CameraController::SetExposure(int microsec) {
    if (!xiH) {
        LOG_ERROR("Camera not opened");
        return false;
    }

    microsec = std::max(MIN_EXPOSURE_US, std::min(MAX_EXPOSURE_US, microsec));

    XI_RETURN stat = xiSetParamInt(xiH, XI_PRM_EXPOSURE, microsec);
    if (stat != XI_OK) {
        LOG_ERROR("Failed to set exposure: " + GetXiApiErrorString(stat));
        return false;
    }

    currentExposure = microsec;
    NotifyPropertyChanged("Exposure", std::to_string(microsec) + " us");

    LOG_INFO("Exposure set to: " + std::to_string(microsec) + " us");
    return true;
}

bool CameraController::SetGain(float gain) {
    if (!xiH) {
        LOG_ERROR("Camera not opened");
        return false;
    }

    gain = std::max(0.0f, std::min(24.0f, gain));

    XI_RETURN stat = xiSetParamFloat(xiH, XI_PRM_GAIN, gain);
    if (stat != XI_OK) {
        LOG_ERROR("Failed to set gain: " + GetXiApiErrorString(stat));
        return false;
    }

    currentGain = gain;
    NotifyPropertyChanged("Gain", std::to_string(gain) + " dB");

    LOG_INFO("Gain set to: " + std::to_string(gain) + " dB");
    return true;
}

bool CameraController::SetROI(int offsetX, int offsetY, int w, int h) {
    if (!xiH) {
        LOG_ERROR("Camera not opened");
        return false;
    }

    offsetX = (offsetX / 4) * 4;
    offsetY = (offsetY / 4) * 4;
    w = (w / 4) * 4;
    h = (h / 4) * 4;

    w = std::max(32, w);
    h = std::max(32, h);

    if (offsetX + w > PYTHON1300_WIDTH) {
        w = PYTHON1300_WIDTH - offsetX;
    }
    if (offsetY + h > PYTHON1300_HEIGHT) {
        h = PYTHON1300_HEIGHT - offsetY;
    }

    LOG_INFO("Setting ROI: offset(" + std::to_string(offsetX) + "," +
        std::to_string(offsetY) + "), size(" + std::to_string(w) +
        "x" + std::to_string(h) + ")");

    // 캡처 중이면 일시 정지
    bool wasCapturing = isRunning && !isPaused;
    if (wasCapturing) {
        PauseCapture(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // set ROI
    XI_RETURN stat;
    stat = xiSetParamInt(xiH, XI_PRM_OFFSET_X, offsetX);
    if (stat != XI_OK) {
        LOG_ERROR("Failed to set X offset: " + GetXiApiErrorString(stat));
        if (wasCapturing) PauseCapture(false);
        return false;
    }

    stat = xiSetParamInt(xiH, XI_PRM_OFFSET_Y, offsetY);
    if (stat != XI_OK) {
        LOG_ERROR("Failed to set Y offset: " + GetXiApiErrorString(stat));
        if (wasCapturing) PauseCapture(false);
        return false;
    }

    stat = xiSetParamInt(xiH, XI_PRM_WIDTH, w);
    if (stat != XI_OK) {
        LOG_ERROR("Failed to set width: " + GetXiApiErrorString(stat));
        if (wasCapturing) PauseCapture(false);
        return false;
    }

    stat = xiSetParamInt(xiH, XI_PRM_HEIGHT, h);
    if (stat != XI_OK) {
        LOG_ERROR("Failed to set height: " + GetXiApiErrorString(stat));
        if (wasCapturing) PauseCapture(false);
        return false;
    }

    width = w;
    height = h;
    size_t newSize = static_cast<size_t>(width) * height;

    // new frame buffer
    unsigned char* newFrameBuffer = nullptr;
    unsigned char* newWorkingBuffer = nullptr;
    try {
        newFrameBuffer = new unsigned char[newSize];
        newWorkingBuffer = new unsigned char[newSize];
    }
    catch (const std::bad_alloc& e) {
        LOG_ERROR("Failed to allocate memory for new ROI buffer: " + std::string(e.what()));

        if (wasCapturing) {
            PauseCapture(false);
        }
        NotifyError(CameraError::MEMORY_ERROR, "Failed to allocate memory for ROI");
        return false;
    }

    delete[] frameBuffer;
    delete[] workingBuffer;
    frameBuffer = newFrameBuffer;
    workingBuffer = newWorkingBuffer;

    std::string roiStr = "offset(" + std::to_string(offsetX) + "," +
        std::to_string(offsetY) + "), size(" +
        std::to_string(w) + "x" + std::to_string(h) + ")";
    NotifyPropertyChanged("ROI", roiStr);

    if (wasCapturing) {
        PauseCapture(false);
    }

    LOG_INFO("ROI set successfully");
    return true;
}


bool CameraController::SetFrameRate(float fps) {
    if (!xiH) {
        LOG_ERROR("Camera not opened");
        return false;
    }

    fps = std::max(1.0f, std::min((float)PYTHON1300_MAX_FPS, fps));

    XI_RETURN stat = xiSetParamFloat(xiH, XI_PRM_FRAMERATE, fps);
    if (stat != XI_OK) {
        LOG_WARNING("Failed to set frame rate: " + GetXiApiErrorString(stat));
        return false;
    }

    NotifyPropertyChanged("FrameRate", std::to_string(fps) + " FPS");
    LOG_INFO("Frame rate set to: " + std::to_string(fps) + " FPS");
    return true;
}

bool CameraController::SetTriggerMode(bool enabled) {
    if (!xiH) {
        LOG_ERROR("Camera not opened");
        return false;
    }

    XI_RETURN stat = xiSetParamInt(xiH, XI_PRM_TRG_SOURCE,
        enabled ? XI_TRG_SOFTWARE : XI_TRG_OFF);
    if (stat != XI_OK) {
        LOG_ERROR("Failed to set trigger mode: " + GetXiApiErrorString(stat));
        return false;
    }

    NotifyPropertyChanged("TriggerMode", enabled ? "Enabled" : "Disabled");
    LOG_INFO("Trigger mode " + std::string(enabled ? "enabled" : "disabled"));
    return true;
}

float CameraController::GetFrameRate() {
    if (!xiH) return 0.0f;

    float fps = 0.0f;
    xiGetParamFloat(xiH, XI_PRM_FRAMERATE, &fps);
    return fps;
}

void CameraController::UpdateStatistics(bool frameReceived) {
    if (frameReceived) {
        stats.totalFrames++;

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - stats.startTime).count();

        if (elapsed > 0) {
            stats.averageFPS = static_cast<double>(stats.totalFrames) / elapsed;
        }

        float currentFPS = GetFrameRate();
        stats.minFPS = std::min(stats.minFPS, static_cast<double>(currentFPS));
        stats.maxFPS = std::max(stats.maxFPS, static_cast<double>(currentFPS));
    }
    else {
        stats.droppedFrames++;
    }
}

void CameraController::RegisterCallback(IXIMEACallback* callback) {
    if (!callback) return;

    std::lock_guard<std::mutex> lock(callbackMutex);
    auto it = std::find(callbacks.begin(), callbacks.end(), callback);
    if (it == callbacks.end()) {
        callbacks.push_back(callback);
        LOG_INFO("Callback registered");
    }
}

void CameraController::UnregisterCallback(IXIMEACallback* callback) {
    if (!callback) return;

    std::lock_guard<std::mutex> lock(callbackMutex);
    auto it = std::find(callbacks.begin(), callbacks.end(), callback);
    if (it != callbacks.end()) {
        callbacks.erase(it);
        LOG_INFO("Callback unregistered");
    }
}

void CameraController::ClearCallbacks() {
    std::lock_guard<std::mutex> lock(callbackMutex);
    callbacks.clear();
    LOG_INFO("All callbacks cleared");
}

void CameraController::NotifyFrameReceived(const FrameInfo& info) {
    // logging every 100 frames
    if (info.frameNumber % 100 == 0) {
        std::stringstream ss;
        ss << "Frame " << info.frameNumber
            << " (acq: " << info.acqFrameNumber << ")"
            << ", size: " << info.width << "x" << info.height
            << ", exp: " << info.exposureTime_ms << "ms"
            << ", gain: " << info.gain_db << "dB"
            << ", black: " << info.blackLevel
            << ", GPI: 0x" << std::hex << info.GPI_level
            << ", FPS: " << std::fixed << std::setprecision(1) << info.currentFPS;
        LOG_DEBUG(ss.str());
    }

    std::lock_guard<std::mutex> lock(callbackMutex);
    for (auto* callback : callbacks) {
        try {
            callback->OnFrameReceived(info);
        }
        catch (const std::exception& e) {
            LOG_ERROR("Exception in OnFrameReceived callback: " + std::string(e.what()));
        }
    }
}

void CameraController::NotifyStateChanged(CameraState newState) {
    std::lock_guard<std::mutex> lock(callbackMutex);
    CameraState oldState = currentState;

    for (auto* callback : callbacks) {
        try {
            callback->OnCameraStateChanged(newState, oldState);
        }
        catch (const std::exception& e) {
            LOG_ERROR("Exception in OnCameraStateChanged callback: " + std::string(e.what()));
        }
    }
}

void CameraController::NotifyError(CameraError error, const std::string& message) {
    std::lock_guard<std::mutex> lock(callbackMutex);
    for (auto* callback : callbacks) {
        try {
            callback->OnError(error, message);
        }
        catch (const std::exception& e) {
            LOG_ERROR("Exception in OnError callback: " + std::string(e.what()));
        }
        catch (...) {
            LOG_ERROR("Unknown exception in OnError callback");
        }
    }
}
void CameraController::NotifyPropertyChanged(const std::string& property, const std::string& value) {
    std::lock_guard<std::mutex> lock(callbackMutex);
    for (auto* callback : callbacks) {
        try {
            callback->OnPropertyChanged(property, value);
        }
        catch (const std::exception& e) {
            LOG_ERROR("Exception in OnPropertyChanged callback: " + std::string(e.what()));
        }
    }
}

int CameraController::GetConnectedDeviceCount() {
    DWORD deviceCount = 0;
    xiGetNumberDevices(&deviceCount);
    return static_cast<int>(deviceCount);
}

bool CameraController::GetDeviceInfo(int index, std::string& name, std::string& serial) {
    char deviceName[256] = { 0 };
    char serialNumber[256] = { 0 };

    XI_RETURN stat = xiGetDeviceInfoString(index, XI_PRM_DEVICE_NAME,
        deviceName, sizeof(deviceName));
    if (stat != XI_OK) {
        LOG_ERROR("Failed to get device name: " + GetXiApiErrorString(stat));
        return false;
    }

    stat = xiGetDeviceInfoString(index, XI_PRM_DEVICE_SN,
        serialNumber, sizeof(serialNumber));
    if (stat != XI_OK) {
        LOG_ERROR("Failed to get device serial: " + GetXiApiErrorString(stat));
        return false;
    }

    name = deviceName;
    serial = serialNumber;
    return true;
}

std::string CameraController::GetXiApiErrorString(XI_RETURN error) {
    switch (error) {
        case XI_OK: return "Success";
        case XI_INVALID_HANDLE: return "Invalid handle";
        case XI_READREG: return "Register read error";
        case XI_WRITEREG: return "Register write error";
        case XI_FREE_RESOURCES: return "Free resources error";
        case XI_FREE_CHANNEL: return "Free channel error";
        case XI_FREE_BANDWIDTH: return "Free bandwidth error";
        case XI_READBLK: return "Read block error";
        case XI_WRITEBLK: return "Write block error";
        case XI_NO_IMAGE: return "No image";
        case XI_TIMEOUT: return "Timeout";
        case XI_INVALID_ARG: return "Invalid argument";
        case XI_NOT_SUPPORTED: return "Not supported";
        case XI_ISOCH_ATTACH_BUFFERS: return "Isochronous attach buffers error";
        case XI_GET_OVERLAPPED_RESULT: return "Get overlapped result error";
        case XI_MEMORY_ALLOCATION: return "Memory allocation error";
        case XI_DLLCONTEXTISNULL: return "DLL context is null";
        case XI_DLLCONTEXTISNONZERO: return "DLL context is non-zero";
        case XI_DLLCONTEXTEXIST: return "DLL context exists";
        case XI_TOOMANYDEVICES: return "Too many devices";
        case XI_ERRORCAMCONTEXT: return "Camera context error";
        case XI_UNKNOWN_HARDWARE: return "Unknown hardware";
        case XI_INVALID_TM_FILE: return "Invalid TM file";
        case XI_INVALID_TM_TAG: return "Invalid TM tag";
        case XI_INCOMPLETE_TM: return "Incomplete TM";
        case XI_BUS_RESET_FAILED: return "Bus reset failed";
        case XI_NOT_IMPLEMENTED: return "Not implemented";
        case XI_SHADING_TOOBRIGHT: return "Shading too bright";
        case XI_SHADING_TOODARK: return "Shading too dark";
        case XI_TOO_LOW_GAIN: return "Too low gain";
        case XI_INVALID_BPL: return "Invalid BPL";
        case XI_BPL_REALLOC: return "BPL reallocation error";
        case XI_INVALID_PIXEL_LIST: return "Invalid pixel list";
        case XI_INVALID_FFS: return "Invalid FFS";
        case XI_INVALID_PROFILE: return "Invalid profile";
        case XI_INVALID_CALIBRATION: return "Invalid calibration";
        case XI_INVALID_BUFFER: return "Invalid buffer";
        case XI_INVALID_DATA: return "Invalid data";
        case XI_TGBUSY: return "Timing generator busy";
        case XI_IO_WRONG: return "Wrong I/O direction";
        case XI_ACQUISITION_ALREADY_UP: return "Acquisition already up";
        case XI_OLD_DRIVER_VERSION: return "Old driver version";
        case XI_GET_LAST_ERROR: return "Get last error";
        case XI_CANT_PROCESS: return "Can't process";
        case XI_ACQUISITION_STOPED: return "Acquisition stopped";
        case XI_ACQUISITION_STOPED_WERR: return "Acquisition stopped with error";
        case XI_INVALID_INPUT_ICC_PROFILE: return "Invalid input ICC profile";
        case XI_INVALID_OUTPUT_ICC_PROFILE: return "Invalid output ICC profile";
        case XI_DEVICE_NOT_READY: return "Device not ready";
        case XI_SHADING_TOOCONTRAST: return "Shading too contrast";
        case XI_ALREADY_INITIALIZED: return "Already initialized";
        case XI_NOT_ENOUGH_PRIVILEGES: return "Not enough privileges";
        case XI_NOT_COMPATIBLE_DRIVER: return "Not compatible driver";
        case XI_TM_INVALID_RESOURCE: return "TM invalid resource";
        case XI_DEVICE_HAS_BEEN_RESETED: return "Device has been reset";
        case XI_NO_DEVICES_FOUND: return "No devices found";
        case XI_RESOURCE_OR_FUNCTION_LOCKED: return "Resource or function locked";
        case XI_BUFFER_SIZE_TOO_SMALL: return "Buffer size too small";
        case XI_COULDNT_INIT_PROCESSOR: return "Couldn't initialize processor";
        case XI_NOT_INITIALIZED: return "Not initialized";
        case XI_RESOURCE_NOT_FOUND: return "Resource not found";
        case XI_UNKNOWN_PARAM: return "Unknown parameter";
        case XI_WRONG_PARAM_VALUE: return "Wrong parameter value";
        case XI_WRONG_PARAM_TYPE: return "Wrong parameter type";
        case XI_WRONG_PARAM_SIZE: return "Wrong parameter size";
        case XI_BUFFER_TOO_SMALL: return "Buffer too small";
        case XI_NOT_SUPPORTED_PARAM: return "Parameter not supported";
        case XI_NOT_SUPPORTED_PARAM_INFO: return "Parameter info not supported";
        case XI_NOT_SUPPORTED_DATA_FORMAT: return "Data format not supported";
        case XI_READ_ONLY_PARAM: return "Read-only parameter";
        case XI_BANDWIDTH_NOT_SUPPORTED: return "Bandwidth not supported";
        case XI_INVALID_FFS_FILE_NAME: return "Invalid FFS file name";
        case XI_FFS_FILE_NOT_FOUND: return "FFS file not found";
        case XI_PARAM_NOT_SETTABLE: return "Parameter not settable";
        case XI_SAFE_POLICY_NOT_SUPPORTED: return "Safe buffer policy not supported";
        case XI_GPUDIRECT_NOT_AVAILABLE: return "GPUDirect not available";
        case XI_INCORRECT_SENS_ID_CHECK: return "Incorrect sensor ID checksum";
        case XI_INCORRECT_FPGA_TYPE: return "Incorrect FPGA type";
        case XI_PARAM_CONDITIONALLY_NOT_AVAILABLE: return "Parameter conditionally not available";
        case XI_ERR_FRAME_BUFFER_RAM_INIT: return "Frame buffer RAM initialization error";
        case XI_PROC_OTHER_ERROR: return "Processing error - other";
        case XI_PROC_PROCESSING_ERROR: return "Error while image processing";
        case XI_PROC_INPUT_FORMAT_UNSUPPORTED: return "Input format not supported";
        case XI_PROC_OUTPUT_FORMAT_UNSUPPORTED: return "Output format not supported";
        case XI_OUT_OF_RANGE: return "Parameter value out of range";
        default: return "Unknown error (" + std::to_string(error) + ")";
    }
}