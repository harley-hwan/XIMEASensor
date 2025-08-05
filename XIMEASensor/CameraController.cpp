#include "pch.h"
#include "CameraController.h"
#include <algorithm>
#include <sstream>
#include <cstring>
#include <iomanip>
#include <algorithm>
#include <future>
#include "XIMEASensor.h"
#include "BallDetector.h"

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
    currentExposure(CameraDefaults::EXPOSURE_US),
    currentGain(CameraDefaults::GAIN_DB),
    currentState(CameraState::DISCONNECTED),
    m_realtimeDetectionEnabled(false),
    m_realtimeCallback(nullptr),
    m_realtimeCallbackContext(nullptr),
    m_detectionThreadRunning(false),
    m_realtimeProcessedFrames(0),
    m_realtimeTotalProcessingTime(0.0),
    m_ballStateTrackingEnabled(false),
    m_ballStateCallback(nullptr),
    m_ballStateCallbackContext(nullptr) {

    size_t bufferSize = width * height;
    frameBuffer = new unsigned char[bufferSize];
    workingBuffer = new unsigned char[bufferSize];

    stats.Reset();

#ifdef ENABLE_CONTINUOUS_CAPTURE
    m_continuousCapture = std::make_unique<ContinuousCaptureManager>();
#endif

    m_realtimeBallDetector = std::make_unique<BallDetector>();
    memset(&m_lastDetectionResult, 0, sizeof(m_lastDetectionResult));

    m_ballStateConfig = BallStateConfig();
    m_ballTracking = BallTrackingData();

    LOG_INFO("CameraController initialized with ball state tracking support");
}

CameraController::~CameraController() {
    LOG_INFO("CameraController destructor called");

    try {
        // 1. 모든 작업 중지 플래그 설정
        if (m_ballStateTrackingEnabled.load()) {
            m_ballStateTrackingEnabled = false;
        }

        if (m_detectionThreadRunning.load()) {
            m_detectionThreadRunning = false;
            m_detectionCV.notify_all();
        }

        if (isRunning.load()) {
            isRunning = false;
        }

        // 2. Detection 스레드 종료 대기
        if (m_detectionThread.joinable()) {
            // condition_variable에 타임아웃 추가
            {
                std::unique_lock<std::mutex> lock(m_detectionQueueMutex);
                // 큐를 비우고 종료 신호
                std::queue<std::pair<std::vector<unsigned char>, FrameInfo>> empty;
                std::swap(m_detectionQueue, empty);
            }
            m_detectionCV.notify_all();

            // 직접 join with 제한 시간
            try {
                m_detectionThread.join();
            }
            catch (const std::exception& e) {
                LOG_ERROR("Failed to join detection thread: " + std::string(e.what()));
                m_detectionThread.detach();
            }
        }

        // 3. 콜백 정리
        {
            std::lock_guard<std::mutex> lock(callbackMutex);
            callbacks.clear();
        }

        // 4. Capture 스레드 종료
        if (captureThread.joinable()) {
            isPaused = false;
            captureThread.join();
        }

        // 5. 카메라 하드웨어 정리
        if (xiH != nullptr) {
            XI_RETURN stat = xiStopAcquisition(xiH);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            stat = xiCloseDevice(xiH);
            if (stat != XI_OK) {
                LOG_ERROR("Error closing camera in destructor: " + GetXiApiErrorString(stat));
            }
            xiH = nullptr;
        }

        // 6. 메모리 정리
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

        // 7. Detection queue 정리
        {
            std::lock_guard<std::mutex> lock(m_detectionQueueMutex);
            std::queue<std::pair<std::vector<unsigned char>, FrameInfo>> empty;
            std::swap(m_detectionQueue, empty);
        }

    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in destructor: " + std::string(e.what()));
    }
    catch (...) {
        LOG_ERROR("Unknown exception in destructor");
    }

    LOG_INFO("CameraController destroyed safely");
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

    // MQ013MG-ON init
    LOG_INFO("Configuring MQ013MG-ON camera");

    // pixel format
    stat = xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);
    if (stat != XI_OK) {
        LOG_WARNING("Failed to set image format: " + GetXiApiErrorString(stat));
    }

    // camera settings
    xiSetParamInt(xiH, XI_PRM_OUTPUT_DATA_BIT_DEPTH, XI_BPP_8);
    xiSetParamInt(xiH, XI_PRM_SENSOR_DATA_BIT_DEPTH, XI_BPP_10);
    xiSetParamInt(xiH, XI_PRM_WIDTH, PYTHON1300_WIDTH);
    xiSetParamInt(xiH, XI_PRM_HEIGHT, PYTHON1300_HEIGHT);
    xiSetParamInt(xiH, XI_PRM_EXPOSURE, CameraDefaults::EXPOSURE_US);
    currentExposure = CameraDefaults::EXPOSURE_US;

    xiSetParamFloat(xiH, XI_PRM_GAIN, CameraDefaults::GAIN_DB);
    currentGain = CameraDefaults::GAIN_DB;

    xiSetParamFloat(xiH, XI_PRM_FRAMERATE, CameraDefaults::FRAMERATE_FPS);
    currentFrameRate = CameraDefaults::FRAMERATE_FPS;

    xiSetParamInt(xiH, XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FRAME_RATE);
    xiSetParamInt(xiH, XI_PRM_BUFFER_POLICY, XI_BP_SAFE);
    xiSetParamInt(xiH, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_ON);
    xiSetParamInt(xiH, XI_PRM_TRG_SOURCE, XI_TRG_OFF);
    xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING, XI_DWN_1x1);
    xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING_TYPE, XI_BINNING);

    // Advanced settings for high-speed capture
    xiSetParamInt(xiH, XI_PRM_SENSOR_TAPS, XI_TAP_CNT_2);
    xiSetParamInt(xiH, XI_PRM_BUFFERS_QUEUE_SIZE, 20);
    xiSetParamInt(xiH, XI_PRM_RECENT_FRAME, XI_ON);

    // Disable image enhancements for raw data
    xiSetParamFloat(xiH, XI_PRM_GAMMAY, 1.0f);
    xiSetParamFloat(xiH, XI_PRM_SHARPNESS, 0.0f);
    xiSetParamInt(xiH, XI_PRM_HDR, XI_OFF);
    xiSetParamInt(xiH, XI_PRM_AUTO_WB, XI_OFF);
    xiSetParamInt(xiH, XI_PRM_MANUAL_WB, XI_OFF);
    xiSetParamInt(xiH, XI_PRM_SENS_DEFECTS_CORR, XI_ON);
    xiSetParamInt(xiH, XI_PRM_COLOR_FILTER_ARRAY, XI_CFA_NONE);
    xiSetParamInt(xiH, XI_PRM_TRANSPORT_PIXEL_FORMAT, XI_GenTL_Image_Format_Mono8);

    // RAII 패턴을 사용한 프레임 버퍼 할당
    {
        std::lock_guard<std::mutex> lock(frameMutex);

        // 기존 버퍼 정리
        if (frameBuffer) {
            delete[] frameBuffer;
            frameBuffer = nullptr;
        }
        if (workingBuffer) {
            delete[] workingBuffer;
            workingBuffer = nullptr;
        }

        size_t bufferSize = PYTHON1300_WIDTH * PYTHON1300_HEIGHT;

        // unique_ptr로 임시 할당
        std::unique_ptr<unsigned char[]> tempFrame(new(std::nothrow) unsigned char[bufferSize]);
        std::unique_ptr<unsigned char[]> tempWorking(new(std::nothrow) unsigned char[bufferSize]);

        if (!tempFrame || !tempWorking) {
            LOG_ERROR("Failed to allocate frame buffers");
            xiCloseDevice(xiH);
            xiH = nullptr;
            return false;
        }

        // 초기화
        memset(tempFrame.get(), 0, bufferSize);
        memset(tempWorking.get(), 0, bufferSize);

        // 성공 시에만 소유권 이전
        frameBuffer = tempFrame.release();
        workingBuffer = tempWorking.release();
    }

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

    if (isRunning.load()) {
        StopCapture();
    }

    XI_RETURN stat = xiCloseDevice(xiH);
    if (stat != XI_OK) {
        LOG_ERROR("Error closing camera: " + GetXiApiErrorString(stat));
    }

    xiH = nullptr;

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

    bool wasRunning = isRunning.exchange(false);
    if (!wasRunning) {
        if (captureThread.joinable()) {
            try {
                captureThread.join();
            }
            catch (const std::system_error& e) {
                LOG_ERROR("Exception joining capture thread: " + std::string(e.what()));
            }
        }
        return;
    }

    if (captureThread.joinable()) {
        try {
            captureThread.join();
        }
        catch (const std::system_error& e) {
            LOG_ERROR("Exception joining capture thread: " + std::string(e.what()));
        }
    }

    if (xiH) {
        XI_RETURN stat = xiStopAcquisition(xiH);
        if (stat != XI_OK) {
            LOG_ERROR("Error stopping acquisition: " + GetXiApiErrorString(stat));
        }
    }

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

    auto frameInterval = std::chrono::microseconds((int)(1000000.0f / currentFrameRate));
    auto nextFrameTime = std::chrono::steady_clock::now();

    const int IMAGE_TIMEOUT_MS = 50;
    int consecutiveTimeouts = 0;
    const int MAX_CONSECUTIVE_TIMEOUTS = 10;

    while (isRunning.load()) {
        if (isPaused) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            consecutiveTimeouts = 0;
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        if (now < nextFrameTime) {
            std::this_thread::sleep_until(nextFrameTime);
        }
        nextFrameTime += frameInterval;

        XI_RETURN stat = xiGetImage(xiH, IMAGE_TIMEOUT_MS, &image);

        if (stat == XI_OK) {
            deviceNotReadyCount = 0;
            consecutiveTimeouts = 0;

            auto frameTime = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                frameTime - lastFrameTime).count();
            float currentFPS = duration > 0 ? 1000000.0f / duration : 0.0f;
            lastFrameTime = frameTime;

            if (image.frm != XI_MONO8) {
                LOG_ERROR("Unexpected image format: " + std::to_string(image.frm));
                continue;
            }

            int imageSize = image.width * image.height;

            if (image.bp == nullptr || imageSize <= 0) {
                LOG_ERROR("Invalid image data received");
                continue;
            }

            // 더블 버퍼링을 사용한 프레임 복사
            {
                auto& writeBuffer = buffers[writeIndex.load()];

                // 크기 변경 시에만 재할당
                if (!writeBuffer.data ||
                    writeBuffer.width != image.width ||
                    writeBuffer.height != image.height) {
                    writeBuffer.data = std::make_unique<unsigned char[]>(imageSize);
                    writeBuffer.width = image.width;
                    writeBuffer.height = image.height;
                }

                // 직접 쓰기 버퍼에 복사
                memcpy(writeBuffer.data.get(), image.bp, imageSize);
                writeBuffer.ready = true;

                // 원자적 스왑
                {
                    std::lock_guard<std::mutex> lock(bufferSwapMutex);
                    int oldWrite = writeIndex.load();
                    int oldRead = readIndex.load();
                    writeIndex.store(oldRead);
                    readIndex.store(oldWrite);
                }
            }

            // 기존 호환성을 위한 프레임 버퍼 업데이트
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

#ifdef ENABLE_CONTINUOUS_CAPTURE
            // Continuous capture
            if (m_continuousCapture && m_continuousCapture->IsCapturing()) {
                m_continuousCapture->ProcessFrame(frameBuffer, width, height);
            }
#endif

            // Add to detection queue with non-blocking approach
            if (m_realtimeDetectionEnabled.load()) {
                std::unique_lock<std::mutex> queueLock(m_detectionQueueMutex, std::try_to_lock);

                if (queueLock.owns_lock() && m_detectionQueue.size() < 5) {
                    std::vector<unsigned char> frameData(imageSize);
                    memcpy(frameData.data(), image.bp, imageSize);
                    m_detectionQueue.push({ std::move(frameData), frameInfo });
                    m_detectionCV.notify_one();
                }
            }

            NotifyFrameReceived(frameInfo);
            UpdateStatistics(true);
        }
        else if (stat == XI_TIMEOUT) {
            consecutiveTimeouts++;

            if (consecutiveTimeouts >= MAX_CONSECUTIVE_TIMEOUTS) {
                LOG_WARNING("Multiple consecutive timeouts detected");
                consecutiveTimeouts = 0;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            deviceNotReadyCount = 0;
            UpdateStatistics(false);
        }
        else if (stat == XI_DEVICE_NOT_READY) {
            deviceNotReadyCount++;
            LOG_ERROR("Frame grab error: " + GetXiApiErrorString(stat) +
                " (count: " + std::to_string(deviceNotReadyCount.load()) + ")");

            if (deviceNotReadyCount >= MAX_DEVICE_NOT_READY_ERRORS) {
                LOG_ERROR("Device not ready error exceeded limit. Stopping capture.");

                NotifyError(CameraError::DEVICE_NOT_READY,
                    "Camera disconnected - Device not ready error exceeded limit");

                isRunning = false;

                CameraState oldState = currentState.exchange(CameraState::kERROR);
                NotifyStateChanged(CameraState::kERROR);

                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            UpdateStatistics(false);
        }
        else {
            LOG_ERROR("Frame grab error: " + GetXiApiErrorString(stat));
            NotifyError(static_cast<CameraError>(stat),
                "Frame grab error: " + GetXiApiErrorString(stat));
            UpdateStatistics(false);

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    LOG_INFO("Capture loop ended");
}


bool CameraController::GetFrame(unsigned char* buffer, int bufferSize, int& outWidth, int& outHeight) {
    if (!buffer || bufferSize <= 0) {
        LOG_ERROR("Invalid buffer provided");
        return false;
    }

    // 새로운 더블 버퍼링 방식 사용
    auto& readBuffer = buffers[readIndex.load()];

    if (!readBuffer.ready.load()) {
        // 폴백: 기존 방식 사용
        std::lock_guard<std::mutex> lock(frameMutex);

        int currentFrameSize = width * height;

        if (currentFrameSize <= 0) {
            LOG_ERROR("Invalid frame dimensions: " + std::to_string(width) + "x" + std::to_string(height));
            return false;
        }

        if (bufferSize < currentFrameSize) {
            LOG_ERROR("Buffer size too small: " + std::to_string(bufferSize) + " < " + std::to_string(currentFrameSize));
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

    int frameSize = readBuffer.width * readBuffer.height;
    if (bufferSize < frameSize) {
        LOG_ERROR("Buffer size too small");
        return false;
    }

    memcpy(buffer, readBuffer.data.get(), frameSize);
    outWidth = readBuffer.width;
    outHeight = readBuffer.height;

    return true;
}

bool CameraController::SetExposure(int microsec) {
    if (!xiH) {
        LOG_ERROR("Camera not opened");
        return false;
    }

    const int minExposure = CameraDefaults::MIN_EXPOSURE_US;
    const int maxExposure = CameraDefaults::MAX_EXPOSURE_US;

    if (microsec < minExposure) {
        microsec = minExposure;
        LOG_WARNING("Exposure clamped to minimum: " + std::to_string(minExposure) + " us");
    }
    else if (microsec > maxExposure) {
        microsec = maxExposure;
        LOG_WARNING("Exposure clamped to maximum: " + std::to_string(maxExposure) + " us");
    }

    XI_RETURN stat = xiSetParamInt(xiH, XI_PRM_EXPOSURE, microsec);
    if (stat != XI_OK) {
        LOG_ERROR("Failed to set exposure: " + GetXiApiErrorString(stat));
        return false;
    }

    currentExposure = microsec;
    NotifyPropertyChanged("Exposure", std::to_string(microsec) + " us");
    LOG_INFO("Exposure set to: " + std::to_string(microsec) + " us");

    float currentFPS = GetFrameRate();
    float maxPossibleFPS = 1000000.0f / microsec;

    if (currentFPS > maxPossibleFPS) {
        LOG_INFO("Adjusting frame rate due to exposure time change");
        SetFrameRate(maxPossibleFPS * 0.95f); // Set to 95% of max
    }

    return true;
}

bool CameraController::SetGain(float gain) {
    if (!xiH) {
        LOG_ERROR("Camera not opened");
        return false;
    }

    const float minGain = CameraDefaults::MIN_GAIN_DB;
    const float maxGain = CameraDefaults::MAX_GAIN_DB;

    if (gain < minGain) {
        gain = minGain;
        LOG_WARNING("Gain clamped to minimum: " + std::to_string(minGain) + " dB");
    }
    else if (gain > maxGain) {
        gain = maxGain;
        LOG_WARNING("Gain clamped to maximum: " + std::to_string(maxGain) + " dB");
    }

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

    // Align to 4-pixel boundary
    offsetX = (offsetX / 4) * 4;
    offsetY = (offsetY / 4) * 4;
    w = (w / 4) * 4;
    h = (h / 4) * 4;

    // Minimum size
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

    bool wasCapturing = isRunning && !isPaused;
    if (wasCapturing) {
        PauseCapture(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Set ROI
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

    std::string roiStr = "offset(" + std::to_string(offsetX) + "," + std::to_string(offsetY) + "), size(" + std::to_string(w) + "x" + std::to_string(h) + ")";
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

    const float minFPS = CameraDefaults::MIN_FPS;
    const float maxFPS = CameraDefaults::MAX_FPS;

    if (fps < minFPS) {
        fps = minFPS;
        LOG_WARNING("FPS clamped to minimum: " + std::to_string(minFPS));
    }
    else if (fps > maxFPS) {
        fps = maxFPS;
        LOG_WARNING("FPS clamped to hardware maximum: " + std::to_string(maxFPS));
    }

    float maxPossibleFPS = 1000000.0f / currentExposure;
    if (fps > maxPossibleFPS) {
        LOG_WARNING("Requested FPS (" + std::to_string(fps) +
            ") exceeds maximum possible with current exposure time (" +
            std::to_string(currentExposure) + "us). Max possible: " +
            std::to_string(maxPossibleFPS) + " FPS");
        fps = maxPossibleFPS * 0.95f; // Set to 95% of max
    }

    XI_RETURN stat = xiSetParamFloat(xiH, XI_PRM_FRAMERATE, fps);
    if (stat != XI_OK) {
        LOG_WARNING("Failed to set frame rate: " + GetXiApiErrorString(stat));

        // get actual frame rate
        float actualFPS = 0.0f;
        if (xiGetParamFloat(xiH, XI_PRM_FRAMERATE, &actualFPS) == XI_OK) {
            LOG_INFO("Camera reported actual frame rate: " + std::to_string(actualFPS) + " FPS");
            NotifyPropertyChanged("FrameRate", std::to_string(actualFPS) + " FPS");
        }

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
    XI_RETURN stat = xiGetParamFloat(xiH, XI_PRM_FRAMERATE, &fps);
    if (stat == XI_OK) {
        return fps;
    }
    return currentFrameRate;
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

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            now - lastFrameTime).count();
        if (duration > 0) {
            double currentFPS = 1000000.0 / duration;
            stats.minFPS = std::min(stats.minFPS, currentFPS);
            stats.maxFPS = std::max(stats.maxFPS, currentFPS);
        }
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
    // Log every 100 frames
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



bool CameraController::EnableRealtimeDetection(bool enable) {
    if (enable == m_realtimeDetectionEnabled.load()) {
        return true;
    }

    if (enable) {
        // start detect
        if (currentState != CameraState::CAPTURING) {
            LOG_ERROR("Camera must be capturing to enable realtime detection");
            return false;
        }

        m_realtimeDetectionEnabled = true;
        m_detectionThreadRunning = true;

        // init statistics
        m_realtimeProcessedFrames = 0;
        {
            std::lock_guard<std::mutex> lock(m_realtimeStatsMutex);
            m_realtimeTotalProcessingTime = 0.0;
        }
        m_realtimeStartTime = std::chrono::steady_clock::now();

        // start detection thread
        m_detectionThread = std::thread(&CameraController::RealtimeDetectionWorker, this);

        LOG_INFO("Realtime ball detection enabled");
    }
    else {
        m_realtimeDetectionEnabled = false;

        if (m_detectionThreadRunning) {
            m_detectionThreadRunning = false;
            m_detectionCV.notify_all();

            if (m_detectionThread.joinable()) {
                m_detectionThread.join();
            }
        }

        // clear queue
        {
            std::lock_guard<std::mutex> lock(m_detectionQueueMutex);
            std::queue<std::pair<std::vector<unsigned char>, FrameInfo>> empty;
            std::swap(m_detectionQueue, empty);
        }

        LOG_INFO("Realtime ball detection disabled");
    }

    return true;
}

// worker thread
void CameraController::RealtimeDetectionWorker() {
    LOG_INFO("Realtime detection thread started");

    while (m_detectionThreadRunning.load()) {
        std::unique_lock<std::mutex> lock(m_detectionQueueMutex);

        m_detectionCV.wait(lock, [this] {
            return !m_detectionQueue.empty() || !m_detectionThreadRunning.load();
            });

        while (!m_detectionQueue.empty()) {
            auto item = std::move(m_detectionQueue.front());
            m_detectionQueue.pop();
            lock.unlock();

            auto startTime = std::chrono::high_resolution_clock::now();

            ProcessRealtimeDetection(
                item.first.data(),
                item.second.width,
                item.second.height,
                item.second.frameNumber
            );

            auto endTime = std::chrono::high_resolution_clock::now();
            double processingTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();

            // statistics update - thread-safe
            m_realtimeProcessedFrames.fetch_add(1);

            {
                std::lock_guard<std::mutex> statsLock(m_realtimeStatsMutex);
                m_realtimeTotalProcessingTime += processingTime;
            }

            lock.lock();
        }
    }

    LOG_INFO("Realtime detection thread ended");
}

void CameraController::ProcessRealtimeDetection(const unsigned char* data, int width, int height, int frameIndex) {
    if (!m_realtimeBallDetector) return;

    auto result = m_realtimeBallDetector->DetectBall(data, width, height, frameIndex);

    RealtimeDetectionResult detectionResult;
    memset(&detectionResult, 0, sizeof(detectionResult));

    detectionResult.ballFound = result.found;
    detectionResult.ballCount = std::min(5, static_cast<int>(result.balls.size()));
    detectionResult.detectionTimeMs = m_realtimeBallDetector->GetLastPerformanceMetrics().totalDetectionTime_ms;

    for (int i = 0; i < detectionResult.ballCount; i++) {
        detectionResult.balls[i].centerX = result.balls[i].center.x;
        detectionResult.balls[i].centerY = result.balls[i].center.y;
        detectionResult.balls[i].radius = result.balls[i].radius;
        detectionResult.balls[i].confidence = result.balls[i].confidence;
        detectionResult.balls[i].frameIndex = result.balls[i].frameIndex;
    }

    {
        std::lock_guard<std::mutex> lock(m_lastResultMutex);
        m_lastDetectionResult = detectionResult;
    }

    // Update ball state if tracking is enabled
    if (m_ballStateTrackingEnabled.load()) {
        UpdateBallState(&detectionResult);
    }

    // Call real-time detection callback
    if (m_realtimeCallback) {
        m_realtimeCallback(&detectionResult, m_realtimeCallbackContext);
    }
}

void CameraController::SetRealtimeDetectionCallback(RealtimeDetectionCallback callback, void* context) {
    m_realtimeCallback = callback;
    m_realtimeCallbackContext = context;
}

bool CameraController::GetLastDetectionResult(RealtimeDetectionResult* result) {
    if (!result) return false;

    std::lock_guard<std::mutex> lock(m_lastResultMutex);
    *result = m_lastDetectionResult;
    return true;
}

bool CameraController::SetRealtimeDetectionROI(float roiScale) {
    if (!m_realtimeBallDetector) return false;

    auto params = m_realtimeBallDetector->GetParameters();
    params.roiScale = roiScale;
    m_realtimeBallDetector->SetParameters(params);

    LOG_INFO("Realtime detection ROI scale set to: " + std::to_string(roiScale));
    return true;
}

bool CameraController::SetRealtimeDetectionDownscale(int factor) {
    if (!m_realtimeBallDetector) return false;

    auto params = m_realtimeBallDetector->GetParameters();
    params.downscaleFactor = factor;
    m_realtimeBallDetector->SetParameters(params);

    LOG_INFO("Realtime detection downscale factor set to: " + std::to_string(factor));
    return true;
}

bool CameraController::SetRealtimeDetectionMaxCandidates(int maxCandidates) {
    if (!m_realtimeBallDetector) return false;

    auto params = m_realtimeBallDetector->GetParameters();
    params.maxCandidates = maxCandidates;
    m_realtimeBallDetector->SetParameters(params);

    LOG_INFO("Realtime detection max candidates set to: " + std::to_string(maxCandidates));
    return true;
}

void CameraController::GetRealtimeDetectionStats(int* processedFrames, double* avgProcessingTimeMs, double* detectionFPS) {
    int frames = m_realtimeProcessedFrames.load();

    if (processedFrames) {
        *processedFrames = frames;
    }

    if (frames > 0) {
        if (avgProcessingTimeMs) {
            std::lock_guard<std::mutex> lock(m_realtimeStatsMutex);
            *avgProcessingTimeMs = m_realtimeTotalProcessingTime / frames;
        }

        if (detectionFPS) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - m_realtimeStartTime).count();
            *detectionFPS = elapsed > 0 ? frames / elapsed : 0.0;
        }
    }
    else {
        if (avgProcessingTimeMs) *avgProcessingTimeMs = 0.0;
        if (detectionFPS) *detectionFPS = 0.0;
    }
}



//========================================================
// Ball state tracking methods : 2025-07-30
//========================================================

// Ball state tracking implementation
float CameraController::CalculateDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

void CameraController::ResetBallStateTracking() {
    {
        std::lock_guard<std::mutex> lock(m_ballStateMutex);
        ResetBallTracking();
    }
    LOG_INFO("Ball state tracking reset by user request");
}

void CameraController::ResetBallTracking() {
    // mutex는 이미 잠겨있다고 가정
    m_ballTracking = BallTrackingData();
}

void CameraController::NotifyBallStateChanged(BallState newState, BallState oldState, const BallStateInfo& info) {
    if (m_ballStateCallback && m_ballStateConfig.enableStateCallback) {
        try {
            m_ballStateCallback(newState, oldState, &info, m_ballStateCallbackContext);
        }
        catch (const std::exception& e) {
            LOG_ERROR("Exception in ball state change callback: " + std::string(e.what()));
        }
        catch (...) {
            LOG_ERROR("Unknown exception in ball state change callback");
        }
    }

    LOG_INFO("Ball state changed: " + std::string(Camera_GetBallStateString(oldState)) +
        " -> " + std::string(Camera_GetBallStateString(newState)));
}


void CameraController::UpdateBallState(const RealtimeDetectionResult* result) {
    BallState previousState;
    BallState newState;
    bool stateChanged = false;
    BallStateInfo stateInfo;  // 미리 정보 복사

    {
        std::lock_guard<std::mutex> lock(m_ballStateMutex);

        auto now = std::chrono::steady_clock::now();
        previousState = m_ballTracking.currentState;

        if (!result || !result->ballFound || result->ballCount == 0) {
            // No ball detected
            if (m_ballTracking.isTracking) {
                m_ballTracking.consecutiveDetections = 0;
                m_ballTracking.isTracking = false;
                m_ballTracking.previousState = m_ballTracking.currentState;
                m_ballTracking.currentState = BallState::NOT_DETECTED;
                m_ballTracking.lastStateChangeTime = now;
            }
        }
        else {
            // Ball detected
            float currentX = result->balls[0].centerX;
            float currentY = result->balls[0].centerY;

            if (!m_ballTracking.isTracking) {
                // Start new tracking
                m_ballTracking.lastPositionX = currentX;
                m_ballTracking.lastPositionY = currentY;
                m_ballTracking.lastDetectionTime = now;
                m_ballTracking.stableStartTime = now;
                m_ballTracking.isTracking = true;
                m_ballTracking.consecutiveDetections = 1;
                m_ballTracking.previousState = m_ballTracking.currentState;
                m_ballTracking.currentState = BallState::MOVING;
                m_ballTracking.lastStateChangeTime = now;
            }
            else {
                // Continue tracking
                float distance = CalculateDistance(
                    m_ballTracking.lastPositionX,
                    m_ballTracking.lastPositionY,
                    currentX,
                    currentY
                );

                m_ballTracking.consecutiveDetections++;

                if (distance > m_ballStateConfig.movementThreshold) {
                    // Ball is moving
                    if (m_ballTracking.currentState != BallState::MOVING) {
                        m_ballTracking.previousState = m_ballTracking.currentState;
                        m_ballTracking.currentState = BallState::MOVING;
                        m_ballTracking.lastStateChangeTime = now;
                    }
                    m_ballTracking.stableStartTime = now;
                    m_ballTracking.lastPositionX = currentX;
                    m_ballTracking.lastPositionY = currentY;
                }
                else if (distance <= m_ballStateConfig.positionTolerance) {
                    // Ball is at same position
                    auto stableDuration = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_ballTracking.stableStartTime).count();

                    if (m_ballTracking.currentState == BallState::MOVING) {
                        // Transition from moving to stabilizing
                        m_ballTracking.previousState = m_ballTracking.currentState;
                        m_ballTracking.currentState = BallState::STABILIZING;
                        m_ballTracking.stableStartTime = now;
                        m_ballTracking.lastStateChangeTime = now;
                    }
                    else if (m_ballTracking.currentState == BallState::STABILIZING) {
                        // Check for READY transition
                        if (stableDuration >= m_ballStateConfig.stableTimeMs &&
                            m_ballTracking.consecutiveDetections >= m_ballStateConfig.minConsecutiveDetections) {
                            m_ballTracking.previousState = m_ballTracking.currentState;
                            m_ballTracking.currentState = BallState::READY;
                            m_ballTracking.lastStateChangeTime = now;
                        }
                    }
                    else if (m_ballTracking.currentState == BallState::READY) {
                        // READY state
                    }

                    // Update position with small changes
                    m_ballTracking.lastPositionX = currentX;
                    m_ballTracking.lastPositionY = currentY;
                }
                else {
                    // Small movement detected
                    if (m_ballTracking.currentState == BallState::READY ||
                        m_ballTracking.currentState == BallState::STOPPED) {
                        // Ball started moving again
                        m_ballTracking.previousState = m_ballTracking.currentState;
                        m_ballTracking.currentState = BallState::MOVING;
                        m_ballTracking.stableStartTime = now;
                        m_ballTracking.lastStateChangeTime = now;
                    }
                    else if (m_ballTracking.currentState == BallState::STABILIZING) {
                        // Restart stabilization
                        m_ballTracking.stableStartTime = now;
                    }

                    m_ballTracking.lastPositionX = currentX;
                    m_ballTracking.lastPositionY = currentY;
                }

                m_ballTracking.lastDetectionTime = now;
            }
        }

        newState = m_ballTracking.currentState;
        stateChanged = (previousState != newState);

        if (stateChanged) {
            // 락이 걸려있는 동안 정보 복사
            GetBallStateInfoInternal(&stateInfo);
        }
    } // 여기서 mutex lock 해제됨

    // 락 해제 후 콜백 호출
    if (stateChanged) {
        NotifyBallStateChanged(newState, previousState, stateInfo);
    }
}

// Ball state tracking public methods
bool CameraController::EnableBallStateTracking(bool enable) {
    if (enable == m_ballStateTrackingEnabled.load()) {
        return true;
    }

    if (enable) {
        // Enable real-time detection if not already enabled
        if (!m_realtimeDetectionEnabled.load()) {
            LOG_WARNING("Ball state tracking requires real-time detection. Enabling it automatically.");
            if (!EnableRealtimeDetection(true)) {
                LOG_ERROR("Failed to enable real-time detection for ball state tracking");
                return false;
            }
        }

        ResetBallTracking();
        m_ballStateTrackingEnabled = true;
        LOG_INFO("Ball state tracking enabled");
    }
    else {
        m_ballStateTrackingEnabled = false;
        ResetBallTracking();
        LOG_INFO("Ball state tracking disabled");
    }

    return true;
}

BallState CameraController::GetBallState() const {
    std::lock_guard<std::mutex> lock(m_ballStateMutex);
    return m_ballTracking.currentState;
}

bool CameraController::GetBallStateInfo(BallStateInfo* info) const {
    if (!info) {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_ballStateMutex);
    GetBallStateInfoInternal(info);
    return true;
}

// 내부용 락 없는 버전 (private 메서드)
void CameraController::GetBallStateInfoInternal(BallStateInfo* info) const {
    // mutex는 이미 잠겨있다고 가정
    info->currentState = m_ballTracking.currentState;
    info->previousState = m_ballTracking.previousState;
    info->lastPositionX = m_ballTracking.lastPositionX;
    info->lastPositionY = m_ballTracking.lastPositionY;
    info->consecutiveDetections = m_ballTracking.consecutiveDetections;
    info->isTracking = m_ballTracking.isTracking;

    // Calculate stable duration
    if (m_ballTracking.currentState == BallState::STABILIZING ||
        m_ballTracking.currentState == BallState::READY ||
        m_ballTracking.currentState == BallState::STOPPED) {
        auto now = std::chrono::steady_clock::now();
        info->stableDurationMs = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now - m_ballTracking.stableStartTime).count());
    }
    else {
        info->stableDurationMs = 0;
    }

    // Last state change time
    info->lastStateChangeTime = std::chrono::duration<double>(
        m_ballTracking.lastStateChangeTime.time_since_epoch()).count();
}

bool CameraController::SetBallStateConfig(const BallStateConfig& config) {
    // Validate configuration first (without lock)
    if (config.positionTolerance <= 0 || config.movementThreshold <= 0 ||
        config.stableTimeMs <= 0 || config.minConsecutiveDetections <= 0) {
        LOG_ERROR("Invalid ball state configuration");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_ballStateMutex);
        m_ballStateConfig = config;
    }

    LOG_INFO("Ball state configuration updated");
    return true;
}

BallStateConfig CameraController::GetBallStateConfig() const {
    std::lock_guard<std::mutex> lock(m_ballStateMutex);
    return m_ballStateConfig;  // 복사본 반환으로 안전
}

void CameraController::SetBallStateChangeCallback(BallStateChangeCallback callback, void* context) {
    std::lock_guard<std::mutex> lock(m_ballStateMutex);
    m_ballStateCallback = callback;
    m_ballStateCallbackContext = context;

    if (callback) {
        LOG_INFO("Ball state change callback registered");
    }
    else {
        LOG_INFO("Ball state change callback cleared");
    }
}

int CameraController::GetTimeInCurrentState() const {
    std::lock_guard<std::mutex> lock(m_ballStateMutex);

    auto now = std::chrono::steady_clock::now();
    return static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - m_ballTracking.lastStateChangeTime).count()
        );
}

bool CameraController::IsBallStable() const {
    std::lock_guard<std::mutex> lock(m_ballStateMutex);
    return (m_ballTracking.currentState == BallState::READY ||
        m_ballTracking.currentState == BallState::STOPPED);
}