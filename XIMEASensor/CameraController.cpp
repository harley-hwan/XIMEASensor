#include "pch.h"
#include "CameraController.h"
#include <algorithm>
#include <sstream>
#include <cstring>
#include <iomanip>
#include <algorithm>
#include <future>
#include "BallDetector.h"
#include <filesystem>

std::unique_ptr<CameraController> CameraController::instance = nullptr;
std::mutex CameraController::instanceMutex;

CameraController::CameraController()
    : cameraHandle(nullptr),
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
    m_ballStateCallbackContext(nullptr),
    m_usingDynamicROI(false),
    currentFrameRate(0.0f)
{

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
                std::queue<DetectionQueueItem> empty;
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
        if (cameraHandle != nullptr && cameraInterface) {
            Camera::ReturnCode stat = cameraInterface->StopAcquisition(cameraHandle);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            stat = cameraInterface->CloseDevice(cameraHandle);
            if (stat != Camera::ReturnCode::OK) {
                LOG_ERROR("Error closing camera in destructor: " + cameraInterface->GetErrorString(stat));
            }
            cameraHandle = nullptr;
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
            std::queue<DetectionQueueItem> empty;  // 타입 변경
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

void CameraController::SetCameraType(Camera::CameraFactory::CameraType type) {
    if (cameraHandle) {
        LOG_WARNING("Cannot change camera type while camera is open");
        return;
    }

    cameraInterface = Camera::CameraFactory::CreateCamera(type);
    LOG_INFO("Camera type set to: " + std::to_string(static_cast<int>(type)));
}

bool CameraController::OpenCamera(int deviceIndex) {
    LOG_INFO("Opening camera with index: " + std::to_string(deviceIndex));

    if (cameraHandle != nullptr) {
        LOG_WARNING("Camera already open");
        return true;
    }

    // Create camera interface if not already created
    if (!cameraInterface) {
        SetCameraType(Camera::CameraFactory::CameraType::XIMEA);
    }

    Camera::ReturnCode stat = cameraInterface->OpenDevice(deviceIndex, &cameraHandle);
    if (stat != Camera::ReturnCode::OK) {
        LOG_ERROR("Failed to open camera: " + cameraInterface->GetErrorString(stat));
        NotifyError(ConvertReturnCodeToError(stat), "Failed to open camera");
        return false;
    }

    // MQ013MG-ON init
    LOG_INFO("Configuring MQ013MG-ON camera");

    // pixel format
    stat = cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::IMAGE_DATA_FORMAT, static_cast<int>(Camera::ImageFormat::MONO8));
    if (stat != Camera::ReturnCode::OK) {
        LOG_WARNING("Failed to set image format: " + cameraInterface->GetErrorString(stat));
    }

    // camera settings
    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::OUTPUT_DATA_BIT_DEPTH, 8);
    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::SENSOR_DATA_BIT_DEPTH, 10);
    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::WIDTH, PYTHON1300_WIDTH);
    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::HEIGHT, PYTHON1300_HEIGHT);
    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::EXPOSURE, CameraDefaults::EXPOSURE_US);
    currentExposure = CameraDefaults::EXPOSURE_US;

    cameraInterface->SetParamFloat(cameraHandle, Camera::ParamType::GAIN, CameraDefaults::GAIN_DB);
    currentGain = CameraDefaults::GAIN_DB;

    cameraInterface->SetParamFloat(cameraHandle, Camera::ParamType::FRAMERATE, CameraDefaults::FRAMERATE_FPS);
    currentFrameRate = CameraDefaults::FRAMERATE_FPS;

    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::ACQ_TIMING_MODE,              static_cast<int>(Camera::AcqTimingMode::FRAME_RATE));
    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::BUFFER_POLICY,                static_cast<int>(Camera::BufferPolicy::SAFE));
    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::AUTO_BANDWIDTH_CALCULATION,   static_cast<int>(Camera::OnOff::ON));
    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::TRG_SOURCE,                   static_cast<int>(Camera::TriggerSource::OFF));
    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::DOWNSAMPLING,                 static_cast<int>(Camera::Downsampling::DWN_1x1));
    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::DOWNSAMPLING_TYPE,            static_cast<int>(Camera::DownsamplingType::BINNING));

    // Advanced settings for high-speed capture
    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::SENSOR_TAPS, static_cast<int>(Camera::SensorTaps::TAP_CNT_2));
    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::BUFFERS_QUEUE_SIZE, 20);
    cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::RECENT_FRAME, static_cast<int>(Camera::OnOff::ON));

    // Disable image enhancements for raw data
    cameraInterface->SetParamFloat(cameraHandle, Camera::ParamType::GAMMAY, 1.0f);
    cameraInterface->SetParamFloat(cameraHandle, Camera::ParamType::SHARPNESS, 0.0f);
    cameraInterface->SetParamInt  (cameraHandle, Camera::ParamType::HDR, static_cast<int>(Camera::OnOff::OFF));
    cameraInterface->SetParamInt  (cameraHandle, Camera::ParamType::AUTO_WB, static_cast<int>(Camera::OnOff::OFF));
    cameraInterface->SetParamInt  (cameraHandle, Camera::ParamType::MANUAL_WB, static_cast<int>(Camera::OnOff::OFF));
    cameraInterface->SetParamInt  (cameraHandle, Camera::ParamType::SENS_DEFECTS_CORR, static_cast<int>(Camera::OnOff::ON));
    cameraInterface->SetParamInt  (cameraHandle, Camera::ParamType::COLOR_FILTER_ARRAY, static_cast<int>(Camera::ColorFilterArray::CFA_NONE));
    cameraInterface->SetParamInt  (cameraHandle, Camera::ParamType::TRANSPORT_PIXEL_FORMAT, static_cast<int>(Camera::GenTLImageFormat::Mono8));

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
            cameraInterface->CloseDevice(cameraHandle);
            cameraHandle = nullptr;
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

    if (!cameraHandle || !cameraInterface) {
        LOG_WARNING("Camera already closed");
        return;
    }

    if (isRunning.load()) {
        StopCapture();
    }

    Camera::ReturnCode stat = cameraInterface->CloseDevice(cameraHandle);
    if (stat != Camera::ReturnCode::OK) {
        LOG_ERROR("Error closing camera: " + cameraInterface->GetErrorString(stat));
    }

    cameraHandle = nullptr;

    CameraState oldState = currentState.exchange(CameraState::DISCONNECTED);
    if (oldState != CameraState::DISCONNECTED) {
        NotifyStateChanged(CameraState::DISCONNECTED);
    }

    LOG_INFO("Camera closed");
}

bool CameraController::StartCapture() {
    if (!cameraHandle || !cameraInterface) {
        LOG_ERROR("Camera not opened");
        NotifyError(CameraError::START_FAILED, "Camera not opened");
        return false;
    }

    if (isRunning) {
        LOG_WARNING("Capture already running");
        return true;
    }

    LOG_INFO("Starting capture");

    Camera::ReturnCode stat = cameraInterface->StartAcquisition(cameraHandle);
    if (stat != Camera::ReturnCode::OK) {
        LOG_ERROR("Failed to start acquisition: " + cameraInterface->GetErrorString(stat));
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

    if (cameraHandle && cameraInterface) {
        Camera::ReturnCode stat = cameraInterface->StopAcquisition(cameraHandle);
        if (stat != Camera::ReturnCode::OK) {
            LOG_ERROR("Error stopping acquisition: " + cameraInterface->GetErrorString(stat));
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
    Camera::ImageData image;

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

        Camera::ReturnCode stat = cameraInterface->GetImage(cameraHandle, IMAGE_TIMEOUT_MS, &image);

        if (stat == Camera::ReturnCode::OK) {
            deviceNotReadyCount = 0;
            consecutiveTimeouts = 0;

            auto frameTime = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                frameTime - lastFrameTime).count();
            float currentFPS = duration > 0 ? 1000000.0f / duration : 0.0f;
            lastFrameTime = frameTime;

            if (image.frm != Camera::ImageFormat::MONO8) {
                LOG_ERROR("Unexpected image format: " + std::to_string(static_cast<int>(image.frm)));
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

            // Add to detection queue with ROI info
            if (m_realtimeDetectionEnabled.load()) {
                std::unique_lock<std::mutex> queueLock(m_detectionQueueMutex, std::try_to_lock);

                if (queueLock.owns_lock() && m_detectionQueue.size() < 5) {
                    DetectionQueueItem item;
                    item.frameData.resize(imageSize);
                    memcpy(item.frameData.data(), image.bp, imageSize);
                    item.frameInfo = frameInfo;

                    // 현재 ROI 상태 저장
                    if (m_usingDynamicROI && m_dynamicROIEnabled.load()) {
                        std::lock_guard<std::mutex> roiLock(m_dynamicROIMutex);

                        // 전체 프레임에서 ROI 추출
                        item.isROI = true;
                        item.roiRect = m_currentROI;

                        // ROI 데이터만 저장
                        item.frameData.resize(m_currentROI.width * m_currentROI.height);
                        for (int y = 0; y < m_currentROI.height; y++) {
                            const unsigned char* srcRow = (unsigned char*)image.bp + (m_currentROI.y + y) * image.width + m_currentROI.x;
                            unsigned char* dstRow = item.frameData.data() + y * m_currentROI.width;
                            memcpy(dstRow, srcRow, m_currentROI.width);     // Access violation reading error (vcruntime140d.dll)
                        }

                        // frameInfo 업데이트
                        item.frameInfo.width = m_currentROI.width;
                        item.frameInfo.height = m_currentROI.height;
                    }
                    else {
                        item.isROI = false;
                        item.roiRect = cv::Rect(0, 0, image.width, image.height);
                    }

                    m_detectionQueue.push(std::move(item));
                    m_detectionCV.notify_one();
                }
            }

            NotifyFrameReceived(frameInfo);
            UpdateStatistics(true);
        }
        else if (stat == Camera::ReturnCode::TIMEOUT) {
            consecutiveTimeouts++;

            if (consecutiveTimeouts >= MAX_CONSECUTIVE_TIMEOUTS) {
                LOG_WARNING("Multiple consecutive timeouts detected");
                consecutiveTimeouts = 0;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            deviceNotReadyCount = 0;
            UpdateStatistics(false);
        }
        else if (stat == Camera::ReturnCode::DEVICE_NOT_READY) {
            deviceNotReadyCount++;
            LOG_ERROR("Frame grab error: " + cameraInterface->GetErrorString(stat) +
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
            LOG_ERROR("Frame grab error: " + cameraInterface->GetErrorString(stat));
            NotifyError(ConvertReturnCodeToError(stat),
                "Frame grab error: " + cameraInterface->GetErrorString(stat));
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
    if (!cameraHandle || !cameraInterface) {
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

    Camera::ReturnCode stat = cameraInterface->SetParamInt(
        cameraHandle, Camera::ParamType::EXPOSURE, microsec);
    if (stat != Camera::ReturnCode::OK) {
        LOG_ERROR("Failed to set exposure: " + cameraInterface->GetErrorString(stat));
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
    if (!cameraHandle || !cameraInterface) {
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

    Camera::ReturnCode stat = cameraInterface->SetParamFloat(
        cameraHandle, Camera::ParamType::GAIN, gain);
    if (stat != Camera::ReturnCode::OK) {
        LOG_ERROR("Failed to set gain: " + cameraInterface->GetErrorString(stat));
        return false;
    }

    currentGain = gain;
    NotifyPropertyChanged("Gain", std::to_string(gain) + " dB");
    LOG_INFO("Gain set to: " + std::to_string(gain) + " dB");
    return true;
}

bool CameraController::SetROI(int offsetX, int offsetY, int w, int h) {
    if (!cameraHandle || !cameraInterface) {
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
    Camera::ReturnCode stat;
    stat = cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::OFFSET_X, offsetX);
    if (stat != Camera::ReturnCode::OK) {
        LOG_ERROR("Failed to set X offset: " + cameraInterface->GetErrorString(stat));
        if (wasCapturing) PauseCapture(false);
        return false;
    }

    stat = cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::OFFSET_Y, offsetY);
    if (stat != Camera::ReturnCode::OK) {
        LOG_ERROR("Failed to set Y offset: " + cameraInterface->GetErrorString(stat));
        if (wasCapturing) PauseCapture(false);
        return false;
    }

    stat = cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::WIDTH, w);
    if (stat != Camera::ReturnCode::OK) {
        LOG_ERROR("Failed to set width: " + cameraInterface->GetErrorString(stat));
        if (wasCapturing) PauseCapture(false);
        return false;
    }

    stat = cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::HEIGHT, h);
    if (stat != Camera::ReturnCode::OK) {
        LOG_ERROR("Failed to set height: " + cameraInterface->GetErrorString(stat));
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
    if (!cameraHandle || !cameraInterface) {
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

    Camera::ReturnCode stat = cameraInterface->SetParamFloat(
        cameraHandle, Camera::ParamType::FRAMERATE, fps);
    if (stat != Camera::ReturnCode::OK) {
        LOG_WARNING("Failed to set frame rate: " + cameraInterface->GetErrorString(stat));

        // get actual frame rate
        float actualFPS = 0.0f;
        if (cameraInterface->GetParamFloat(cameraHandle, Camera::ParamType::FRAMERATE, &actualFPS) == Camera::ReturnCode::OK) {
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
    if (!cameraHandle || !cameraInterface) {
        LOG_ERROR("Camera not opened");
        return false;
    }

    Camera::ReturnCode stat = cameraInterface->SetParamInt(cameraHandle, Camera::ParamType::TRG_SOURCE,
        enabled ? static_cast<int>(Camera::TriggerSource::SOFTWARE) : static_cast<int>(Camera::TriggerSource::OFF));
    if (stat != Camera::ReturnCode::OK) {
        LOG_ERROR("Failed to set trigger mode: " + cameraInterface->GetErrorString(stat));
        return false;
    }

    NotifyPropertyChanged("TriggerMode", enabled ? "Enabled" : "Disabled");
    LOG_INFO("Trigger mode " + std::string(enabled ? "enabled" : "disabled"));
    return true;
}

float CameraController::GetFrameRate() {
    if (!cameraHandle || !cameraInterface) return 0.0f;

    float fps = 0.0f;
    Camera::ReturnCode stat = cameraInterface->GetParamFloat(cameraHandle, Camera::ParamType::FRAMERATE, &fps);
    if (stat == Camera::ReturnCode::OK) {
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
            stats.averageFPS = static_cast<float>(stats.totalFrames) / elapsed;
        }

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            now - lastFrameTime).count();
        if (duration > 0) {
            float currentFPS = 1000000.0f / duration;
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
    if (!cameraInterface) {
        SetCameraType(Camera::CameraFactory::CameraType::XIMEA);
    }

    uint32_t deviceCount = 0;
    cameraInterface->GetNumberDevices(&deviceCount);
    return static_cast<int>(deviceCount);
}

bool CameraController::GetDeviceInfo(int index, std::string& name, std::string& serial) {
    if (!cameraInterface) {
        SetCameraType(Camera::CameraFactory::CameraType::XIMEA);
    }

    char deviceName[256] = { 0 };
    char serialNumber[256] = { 0 };

    Camera::ReturnCode stat = cameraInterface->GetDeviceInfoString(
        index, Camera::ParamType::DEVICE_NAME, deviceName, sizeof(deviceName));
    if (stat != Camera::ReturnCode::OK) {
        LOG_ERROR("Failed to get device name: " + cameraInterface->GetErrorString(stat));
        return false;
    }

    stat = cameraInterface->GetDeviceInfoString(
        index, Camera::ParamType::DEVICE_SN, serialNumber, sizeof(serialNumber));
    if (stat != Camera::ReturnCode::OK) {
        LOG_ERROR("Failed to get device serial: " + cameraInterface->GetErrorString(stat));
        return false;
    }

    name = deviceName;
    serial = serialNumber;
    return true;
}

std::string CameraController::GetCameraErrorString(Camera::ReturnCode error) {
    if (cameraInterface) {
        return cameraInterface->GetErrorString(error);
    }
    return "Unknown error (" + std::to_string(static_cast<int>(error)) + ")";
}

CameraError CameraController::ConvertReturnCodeToError(Camera::ReturnCode code) {
    switch (code) {
    case Camera::ReturnCode::OK:
        return CameraError::NONE;
    case Camera::ReturnCode::NO_DEVICES_FOUND:
        return CameraError::DEVICE_NOT_FOUND;
    case Camera::ReturnCode::INVALID_HANDLE:
        return CameraError::OPEN_FAILED;
    case Camera::ReturnCode::ACQUISITION_STOPED:
        return CameraError::START_FAILED;
    case Camera::ReturnCode::WRONG_PARAM_VALUE:
        return CameraError::PARAMETER_ERROR;
    case Camera::ReturnCode::TIMEOUT:
        return CameraError::TIMEOUT;
    case Camera::ReturnCode::MEMORY_ALLOCATION:
        return CameraError::MEMORY_ERROR;
    case Camera::ReturnCode::DEVICE_NOT_READY:
        return CameraError::DEVICE_NOT_READY;
    default:
        return CameraError::UNKNOWN;
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

        // clear queue - 타입 수정
        {
            std::lock_guard<std::mutex> lock(m_detectionQueueMutex);
            std::queue<DetectionQueueItem> empty;  // 타입 변경
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

            // ROI 정보와 함께 처리
            ProcessRealtimeDetectionWithROI(
                item.frameData.data(),
                item.frameInfo.width,
                item.frameInfo.height,
                item.frameInfo.frameNumber,
                item.isROI,
                item.roiRect
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
    ProcessRealtimeDetectionWithROI(data, width, height, frameIndex, false, cv::Rect(0, 0, width, height));
}

void CameraController::ProcessRealtimeDetectionWithROI(
    const unsigned char* data,
    int width,
    int height,
    int frameIndex,
    bool isROI,
    const cv::Rect& roiRect) {

    if (!m_realtimeBallDetector) return;

    const unsigned char* processData = data;
    int processWidth = width;
    int processHeight = height;

    int roiOffsetX = 0;
    int roiOffsetY = 0;

    // 이미 ROI 이미지인 경우
    if (isROI) {
        roiOffsetX = roiRect.x;
        roiOffsetY = roiRect.y;
        processWidth = roiRect.width;
        processHeight = roiRect.height;

        LOG_DEBUG("Processing pre-extracted ROI: offset(" + std::to_string(roiOffsetX) + "," + std::to_string(roiOffsetY) + "), size(" + std::to_string(processWidth) + "x" + std::to_string(processHeight) + ")");
    }

    // Detect ball
    auto result = m_realtimeBallDetector->DetectBall(
        processData, processWidth, processHeight, frameIndex);

    // 좌표 변환 - ROI 이미지를 처리한 경우에만
    if (isROI && result.found) {
        for (auto& ball : result.balls) {
            ball.center.x += roiOffsetX;
            ball.center.y += roiOffsetY;

            LOG_INFO("Ball detected at (" + std::to_string(static_cast<int>(ball.center.x)) + ", " + std::to_string(static_cast<int>(ball.center.y)) + ") in full frame coordinates");
        }
    }

    // Convert to RealtimeDetectionResult
    RealtimeDetectionResult detectionResult;
    memset(&detectionResult, 0, sizeof(detectionResult));

    detectionResult.ballFound = result.found;
    detectionResult.ballCount = std::min(5, static_cast<int>(result.balls.size()));
    detectionResult.detectionTimeMs = m_realtimeBallDetector->GetLastPerformanceMetrics().totalDetectionTime_ms;

    for (int i = 0; i < detectionResult.ballCount; i++) {
        // These coordinates are now in full frame space
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

    // Update dynamic ROI if enabled
    if (m_dynamicROIEnabled.load()) {
        UpdateDynamicROI(&detectionResult);
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
    LOG_INFO("Ball tracking reset - all counters cleared");
}

void CameraController::RecordTrajectoryPoint(float x, float y, float radius,
    float confidence, int frameNumber,
    BallState state) {
    auto now = std::chrono::steady_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

    TrajectoryPoint point(cv::Point2f(x, y), radius, confidence, timestamp, frameNumber, state);

    // Add to trajectory buffer
    m_ballTracking.trajectoryBuffer.push_back(point);

    // Limit buffer size
    if (m_ballTracking.trajectoryBuffer.size() > MAX_TRAJECTORY_BUFFER_SIZE) {
        m_ballTracking.trajectoryBuffer.pop_front();
    }

    // Add to shot data if recording
    if (m_ballTracking.recordingTrajectory) {
        std::lock_guard<std::mutex> lock(m_shotDataMutex);
        m_currentShotData.fullTrajectory.push_back(point);

        if (state == BallState::MOVING) {
            m_currentShotData.movingTrajectory.push_back(point);
        }
    }
}

void CameraController::StartTrajectoryRecording() {
    std::lock_guard<std::mutex> lock(m_shotDataMutex);

    m_currentShotData.Clear();
    m_ballTracking.recordingTrajectory = true;

    // Add READY position as first trajectory point if available
    if (m_ballTracking.currentState == BallState::READY ||
        m_ballTracking.previousState == BallState::READY) {
        auto now = std::chrono::steady_clock::now();
        double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

        TrajectoryPoint readyPoint(m_ballTracking.readyPosition,
            0, 1.0f, timestamp, 0, BallState::READY);
        m_currentShotData.fullTrajectory.push_back(readyPoint);
    }

    LOG_INFO("Started trajectory recording");
}

void CameraController::StopTrajectoryRecording() {
    m_ballTracking.recordingTrajectory = false;
    LOG_INFO("Stopped trajectory recording");
}


void CameraController::ProcessShotCompleted() {
    {
        std::lock_guard<std::mutex> lock(m_shotDataMutex);

        // Calculate shot metrics
        m_currentShotData.CalculateMetrics();

        LOG_INFO("Shot completed - Distance: " +
            std::to_string(m_currentShotData.totalDistance) + " pixels, " +
            "Duration: " + std::to_string(m_currentShotData.shotDuration) + " seconds, " +
            "Max velocity: " + std::to_string(m_currentShotData.maxVelocity) + " pixels/sec");
    }

    // Stop recording
    StopTrajectoryRecording();

    // 자동으로 궤적 데이터를 파일로 저장
    SaveTrajectoryDataAutomatic();

    // Notify shot completion
    NotifyShotCompleted();
}


void CameraController::SaveTrajectoryDataAutomatic() {
    // 현재 시간으로 파일명 생성
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    struct tm localTime;
#ifdef _WIN32
    localtime_s(&localTime, &time_t);
#else
    localtime_r(&time_t, &localTime);
#endif

    std::ostringstream filename;
    filename << "PuttingTrajectory_"
        << std::put_time(&localTime, "%Y%m%d_%H%M%S")
        << ".txt";

    std::string filepath = "./trajectory_data/" + filename.str();

    // 디렉토리 생성 (없으면)
    try {
        std::filesystem::create_directories("./trajectory_data/");
    }
    catch (...) {
        LOG_ERROR("Failed to create trajectory_data directory");
    }

    // 파일 저장
    if (SaveTrajectoryData(filepath)) {
        LOG_INFO("Trajectory automatically saved to: " + filepath);
    }
    else {
        LOG_ERROR("Failed to automatically save trajectory data");
    }
}

void CameraController::NotifyShotCompleted() {
    if (m_shotCompletedCallback) {
        std::lock_guard<std::mutex> lock(m_shotDataMutex);

        try {
            m_shotCompletedCallback(&m_currentShotData, m_shotCompletedCallbackContext);
        }
        catch (const std::exception& e) {
            LOG_ERROR("Exception in shot completed callback: " + std::string(e.what()));
        }
    }
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
    BallStateInfo stateInfo;

    {
        std::lock_guard<std::mutex> lock(m_ballStateMutex);
        auto now = std::chrono::steady_clock::now();
        previousState = m_ballTracking.currentState;

        // ========== 탐지 실패 처리 ==========
        if (!result || !result->ballFound || result->ballCount == 0) {
            HandleBallNotDetected(now);
        }
        else {
            // ========== Ball detected ==========
            float currentX = result->balls[0].centerX;
            float currentY = result->balls[0].centerY;
            float currentRadius = result->balls[0].radius;
            float currentConfidence = result->balls[0].confidence;
            int currentFrame = result->balls[0].frameIndex;

            cv::Point2f currentPos(currentX, currentY);

            // 새로운 추적 시작
            if (!m_ballTracking.isTracking) {
                StartNewTracking(currentPos, now);
            }
            else {
                // 기존 추적 계속
                float pixelDelta = static_cast<float>(cv::norm(currentPos - cv::Point2f(m_ballTracking.lastPositionX, m_ballTracking.lastPositionY)));

                // 움직임 데이터 처리
                ProcessMovementData(m_ballTracking, currentX, currentY, pixelDelta);

                // 상태별 전환 로직
                switch (m_ballTracking.currentState) {
                case BallState::NOT_DETECTED:
                    HandleNotDetectedState(pixelDelta, currentPos, now);
                    break;

                case BallState::READY:
                    HandleReadyState(pixelDelta, currentPos, now);
                    break;

                case BallState::MOVING:
                    HandleMovingState(pixelDelta, now);
                    break;

                case BallState::STABILIZING:
                    HandleStabilizingState(pixelDelta, now);
                    break;

                case BallState::STOPPED:
                    HandleStoppedState(currentPos, now);
                    break;
                }

                // 위치 업데이트
                m_ballTracking.lastPositionX = currentX;
                m_ballTracking.lastPositionY = currentY;
                m_ballTracking.lastDetectionTime = now;
            }

            // 궤적 기록
            if (m_ballTracking.recordingTrajectory ||
                m_ballTracking.currentState == BallState::READY) {
                RecordTrajectoryPoint(currentX, currentY, currentRadius, currentConfidence, currentFrame, m_ballTracking.currentState);
            }
        }

        newState = m_ballTracking.currentState;
        stateChanged = (previousState != newState);

        if (stateChanged) {
            GetBallStateInfoInternal(&stateInfo);

            // 상태 전환 로그 - Camera_GetBallStateString 사용
            LOG_INFO("Ball state changed: " +
                std::string(Camera_GetBallStateString(previousState)) + " -> " +
                std::string(Camera_GetBallStateString(newState)) +
                " (Moving frames: " + std::to_string(m_ballTracking.movingStateFrameCount) +
                ", Stable frames: " + std::to_string(m_ballTracking.consecutiveStableFrames) +
                ", Accumulated: " + std::to_string(m_ballTracking.totalAccumulatedMovement) + " px)");
        }
    }

    if (stateChanged) {
        NotifyBallStateChanged(newState, previousState, stateInfo);

        if (newState == BallState::STOPPED) {
            ProcessShotCompleted();
        }
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


// 움직임 데이터 처리
void CameraController::ProcessMovementData(BallTrackingData& tracking, 
                                          float currentX, float currentY, 
                                          float pixelDelta) {
    cv::Point2f currentPos(currentX, currentY);
    
    // 위치 히스토리 업데이트
    tracking.UpdatePositionHistory(currentPos);
    
    // 프레임 델타 업데이트
    tracking.UpdateFrameDeltas(pixelDelta);
    
    // 누적 움직임 계산
    tracking.CalculateRecentAccumulatedMovement();
    
    // 총 누적 이동 업데이트
    tracking.totalAccumulatedMovement += pixelDelta;
    
    // 동적 임계값 업데이트
    tracking.UpdateDynamicThresholds();
    
    // 움직임/정지 카운터 업데이트
    if (pixelDelta > tracking.currentMovementThreshold) {
        tracking.consecutiveMovingFrames++;
        tracking.consecutiveStableFrames = 0;
        
        if (pixelDelta < tracking.currentMovementThreshold * 2) {
            tracking.microMovementCount++;
        }
    } else if (pixelDelta < tracking.currentStabilityThreshold) {
        tracking.consecutiveStableFrames++;
        tracking.consecutiveMovingFrames = 0;
    } else {
        // 애매한 영역 - 미세 움직임으로 처리
        tracking.microMovementCount++;
    }
    
    // 디버그 로깅 (10프레임마다)
    if (tracking.currentState == BallState::MOVING && 
        tracking.movingStateFrameCount % 10 == 0) {
        LOG_DEBUG("Movement Analysis - Delta: " + std::to_string(pixelDelta) +
                 " px, Avg: " + std::to_string(tracking.averageFrameDelta) +
                 " px, Recent Accumulated: " + std::to_string(tracking.recentAccumulatedMovement) +
                 " px, Noise: " + std::to_string(tracking.movementNoiseLevel) +
                 " px, Thresholds(Move/Stable): " + 
                 std::to_string(tracking.currentMovementThreshold) + "/" +
                 std::to_string(tracking.currentStabilityThreshold));
    }
}


// 볼이 감지되지 않았을 때 처리
void CameraController::HandleBallNotDetected(std::chrono::steady_clock::time_point now) {
    // READY 상태에서의 특별 처리
    if (m_ballTracking.currentState == BallState::READY) {
        m_ballTracking.missedDetectionCount++;

        if (m_ballTracking.missedDetectionCount == 1) {
            m_ballTracking.lastMissedTime = now;
        }

        if (m_ballTracking.missedDetectionCount > m_ballStateConfig.maxMissedDetections) {
            LOG_INFO("Ball detection lost after " +
                std::to_string(m_ballTracking.missedDetectionCount) +
                " missed detections in READY state");

            m_ballTracking.consecutiveDetections = 0;
            m_ballTracking.isTracking = false;
            m_ballTracking.previousState = m_ballTracking.currentState;
            m_ballTracking.currentState = BallState::NOT_DETECTED;
            m_ballTracking.lastStateChangeTime = now;
            m_ballTracking.missedDetectionCount = 0;

            // Reset tracking data
            m_ballTracking.Reset();

            if (m_ballTracking.recordingTrajectory) {
                StopTrajectoryRecording();
            }
        }
        else {
            LOG_DEBUG("Missed detection " +
                std::to_string(m_ballTracking.missedDetectionCount) +
                "/" + std::to_string(m_ballStateConfig.maxMissedDetections) +
                " in READY state - maintaining state");
        }
    }
    else if (m_ballTracking.isTracking) {
        // 추적 중 놓침 - 짧은 시간 대기
        auto timeSinceLast = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - m_ballTracking.lastDetectionTime).count();

        if (timeSinceLast > 200) {  // 200ms 이상 감지 안되면 추적 중단
            m_ballTracking.consecutiveDetections = 0;
            m_ballTracking.isTracking = false;
            m_ballTracking.previousState = m_ballTracking.currentState;
            m_ballTracking.currentState = BallState::NOT_DETECTED;
            m_ballTracking.lastStateChangeTime = now;
            m_ballTracking.missedDetectionCount = 0;

            // Reset tracking data
            m_ballTracking.Reset();

            if (m_ballTracking.recordingTrajectory) {
                StopTrajectoryRecording();
            }

            LOG_INFO("Ball tracking lost - no detection for " +
                std::to_string(timeSinceLast) + " ms");
        }
    }
}

// 새로운 추적 시작
void CameraController::StartNewTracking(const cv::Point2f& currentPos,
    std::chrono::steady_clock::time_point now) {
    m_ballTracking.lastPositionX = currentPos.x;
    m_ballTracking.lastPositionY = currentPos.y;
    m_ballTracking.lastDetectionTime = now;
    m_ballTracking.stableStartTime = now;
    m_ballTracking.isTracking = true;
    m_ballTracking.consecutiveDetections = 1;
    m_ballTracking.missedDetectionCount = 0;

    // Reset tracking data for new tracking
    m_ballTracking.Reset();
    m_ballTracking.UpdatePositionHistory(currentPos);

    // 초기 상태 설정
    if (m_ballTracking.currentState == BallState::NOT_DETECTED) {
        m_ballTracking.previousState = m_ballTracking.currentState;
        m_ballTracking.currentState = BallState::MOVING;
        m_ballTracking.lastStateChangeTime = now;
        m_ballTracking.movingStateFrameCount = 0;

        LOG_INFO("New ball tracking started - initial state: MOVING");
    }
}

// NOT_DETECTED 상태 처리
void CameraController::HandleNotDetectedState(float pixelDelta, const cv::Point2f& currentPos, std::chrono::steady_clock::time_point now) {
    m_ballTracking.consecutiveDetections++;

    // 충분한 연속 감지 후 상태 전환
    if (m_ballTracking.consecutiveDetections >= m_ballStateConfig.minConsecutiveDetections) {
        if (ShouldStartMoving(m_ballTracking, pixelDelta)) {
            // MOVING으로 전환
            m_ballTracking.previousState = m_ballTracking.currentState;
            m_ballTracking.currentState = BallState::MOVING;
            m_ballTracking.lastStateChangeTime = now;
            m_ballTracking.movingStateFrameCount = 0;
            m_ballTracking.movementStartPosition = currentPos;

            LOG_INFO("NOT_DETECTED->MOVING: Movement detected after " +
                std::to_string(m_ballTracking.consecutiveDetections) + " detections");
        }
        else {
            // STABILIZING으로 전환 (정지 상태)
            m_ballTracking.previousState = m_ballTracking.currentState;
            m_ballTracking.currentState = BallState::STABILIZING;
            m_ballTracking.lastStateChangeTime = now;
            m_ballTracking.stableStartTime = now;

            LOG_INFO("NOT_DETECTED->STABILIZING: Stable ball detected");
        }
    }
}


// READY 상태 처리
void CameraController::HandleReadyState(float pixelDelta, const cv::Point2f& currentPos,
    std::chrono::steady_clock::time_point now) {
    // READY에서 벗어난 거리 계산
    float distanceFromReady = static_cast<float>(cv::norm(currentPos - m_ballTracking.readyPosition));

    // 움직임 시작 조건
    bool shouldStartMoving = false;

    // 1. 즉각적인 큰 움직임
    if (pixelDelta > m_ballStateConfig.movementThreshold) {
        shouldStartMoving = true;
        LOG_INFO("READY->MOVING: Large movement detected (" +
            std::to_string(pixelDelta) + " px/frame)");
    }
    // 2. READY 위치에서 일정 거리 이상 벗어남
    else if (distanceFromReady > m_ballStateConfig.movementThreshold * 2) {
        shouldStartMoving = true;
        LOG_INFO("READY->MOVING: Moved away from ready position (" +
            std::to_string(distanceFromReady) + " px total)");
    }
    // 3. 일관된 작은 움직임
    else if (m_ballTracking.HasConsistentMovement()) {
        shouldStartMoving = true;
        LOG_INFO("READY->MOVING: Consistent small movements detected (avg: " +
            std::to_string(m_ballTracking.averageFrameDelta) + " px/frame)");
    }
    // 4. 느린 움직임 감지
    else if (m_ballTracking.DetectSlowMovement()) {
        shouldStartMoving = true;
        LOG_INFO("READY->MOVING: Slow movement pattern detected (accumulated: " +
            std::to_string(m_ballTracking.recentAccumulatedMovement) + " px)");
    }

    if (shouldStartMoving) {
        // MOVING 상태로 전환
        m_ballTracking.previousState = m_ballTracking.currentState;
        m_ballTracking.currentState = BallState::MOVING;
        m_ballTracking.lastStateChangeTime = now;
        m_ballTracking.movingStartTime = now;
        m_ballTracking.movingStateFrameCount = 0;
        m_ballTracking.consecutiveStableFrames = 0;
        m_ballTracking.movementStartPosition = m_ballTracking.readyPosition;

        // 궤적 기록 시작
        StartTrajectoryRecording();

        {
            std::lock_guard<std::mutex> shotLock(m_shotDataMutex);
            m_currentShotData.startPosition = m_ballTracking.readyPosition;
            m_currentShotData.shotStartTime = m_ballTracking.movingStartTime;
        }

        LOG_INFO("Shot started - Ball moving from READY state");
    }
}

// MOVING 상태 처리
void CameraController::HandleMovingState(float pixelDelta,
    std::chrono::steady_clock::time_point now) {
    m_ballTracking.movingStateFrameCount++;

    // 최소 MOVING 지속 시간 보장 (15프레임 = 0.25초 @ 60fps)
    const int MIN_MOVING_FRAMES = 15;

    if (m_ballTracking.movingStateFrameCount < MIN_MOVING_FRAMES) {
        // 최소 시간 동안은 MOVING 유지
        return;
    }

    // STABILIZING 전환 조건 체크
    bool shouldStabilize = false;

    // 1. 연속 안정 프레임이 임계값 초과
    if (m_ballTracking.consecutiveStableFrames >= m_ballTracking.requiredStableFramesForStop) {
        shouldStabilize = true;
        LOG_INFO("MOVING->STABILIZING: Consecutive stable frames (" +
            std::to_string(m_ballTracking.consecutiveStableFrames) + ")");
    }
    // 2. 확실한 정지 패턴
    else if (m_ballTracking.IsDefinitelyStable() &&
        m_ballTracking.averageFrameDelta < m_ballTracking.currentStabilityThreshold) {
        shouldStabilize = true;
        LOG_INFO("MOVING->STABILIZING: Definitely stable pattern (avg delta: " +
            std::to_string(m_ballTracking.averageFrameDelta) + " px)");
    }
    // 3. 매우 낮은 움직임이 지속
    else if (m_ballTracking.consecutiveStableFrames >= 10 &&
        m_ballTracking.recentAccumulatedMovement < m_ballTracking.currentStabilityThreshold * 5) {
        shouldStabilize = true;
        LOG_INFO("MOVING->STABILIZING: Very low movement sustained (accumulated: " +
            std::to_string(m_ballTracking.recentAccumulatedMovement) + " px)");
    }

    // 하지만 여전히 움직임이 감지되면 MOVING 유지
    if (shouldStabilize && pixelDelta > m_ballTracking.currentMovementThreshold * 1.5f) {
        shouldStabilize = false;
        m_ballTracking.consecutiveStableFrames = 0;
        LOG_DEBUG("Stabilization cancelled - movement detected (" +
            std::to_string(pixelDelta) + " px)");
    }

    if (shouldStabilize) {
        m_ballTracking.previousState = m_ballTracking.currentState;
        m_ballTracking.currentState = BallState::STABILIZING;
        m_ballTracking.lastStateChangeTime = now;
        m_ballTracking.stableStartTime = now;

        LOG_INFO("Ball stabilizing after " +
            std::to_string(m_ballTracking.movingStateFrameCount) +
            " frames of movement");
    }
}

// STABILIZING 상태 처리
void CameraController::HandleStabilizingState(float pixelDelta,
    std::chrono::steady_clock::time_point now) {
    auto stableDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_ballTracking.stableStartTime).count();

    // 다시 움직이기 시작했는지 확인
    if (pixelDelta > m_ballTracking.currentMovementThreshold * 2 ||
        m_ballTracking.consecutiveMovingFrames >= 3) {

        // MOVING으로 복귀
        m_ballTracking.previousState = m_ballTracking.currentState;
        m_ballTracking.currentState = BallState::MOVING;
        m_ballTracking.lastStateChangeTime = now;
        m_ballTracking.consecutiveStableFrames = 0;

        LOG_INFO("Ball resumed movement from STABILIZING state");
        return;
    }

    // STOPPED 또는 READY 전환 확인
    if (stableDuration >= m_ballStateConfig.stabilizingTimeMs) {
        if (m_ballTracking.recordingTrajectory) {
            // 샷 완료 - STOPPED
            m_ballTracking.previousState = m_ballTracking.currentState;
            m_ballTracking.currentState = BallState::STOPPED;
            m_ballTracking.lastStateChangeTime = now;

            cv::Point2f currentPos(m_ballTracking.lastPositionX, m_ballTracking.lastPositionY);

            {
                std::lock_guard<std::mutex> shotLock(m_shotDataMutex);
                m_currentShotData.endPosition = currentPos;
                m_currentShotData.shotEndTime = now;
            }

            float totalDistance = static_cast<float>(cv::norm(currentPos - m_ballTracking.movementStartPosition));

            LOG_INFO("Shot completed - Ball STOPPED (distance: " +
                std::to_string(totalDistance) + " px, duration: " +
                std::to_string(stableDuration) + " ms)");
        }
        else {
            // 일반 READY 전환
            m_ballTracking.previousState = m_ballTracking.currentState;
            m_ballTracking.currentState = BallState::READY;
            m_ballTracking.lastStateChangeTime = now;
            m_ballTracking.readyPosition = cv::Point2f(m_ballTracking.lastPositionX,
                m_ballTracking.lastPositionY);

            // 추적 데이터 리셋
            m_ballTracking.Reset();
            m_ballTracking.totalAccumulatedMovement = 0.0f;

            LOG_INFO("Ball READY for shot");
        }
    }
}

// STOPPED 상태 처리
void CameraController::HandleStoppedState(const cv::Point2f& currentPos,
    std::chrono::steady_clock::time_point now) {
    auto stoppedDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_ballTracking.lastStateChangeTime).count();

    // 3초 후 READY로 전환
    if (stoppedDuration > 3000) {
        m_ballTracking.previousState = m_ballTracking.currentState;
        m_ballTracking.currentState = BallState::READY;
        m_ballTracking.lastStateChangeTime = now;
        m_ballTracking.readyPosition = currentPos;

        // 추적 데이터 리셋
        m_ballTracking.Reset();
        m_ballTracking.totalAccumulatedMovement = 0.0f;

        LOG_INFO("Ball READY for next shot after " +
            std::to_string(stoppedDuration) + " ms in STOPPED state");
    }
}


// 움직임 시작 판단 (helper)
bool CameraController::ShouldStartMoving(const BallTrackingData& tracking, float pixelDelta) {
    // 1. 즉각적인 큰 움직임
    if (pixelDelta > tracking.currentMovementThreshold * 2) {
        return true;
    }

    // 2. 일관된 움직임 패턴
    if (tracking.HasConsistentMovement()) {
        return true;
    }

    // 3. 느린 움직임 감지
    if (tracking.DetectSlowMovement()) {
        return true;
    }

    // 4. 연속 움직임 프레임
    if (tracking.consecutiveMovingFrames >= 3) {
        return true;
    }

    return false;
}

// 움직임 정지 판단 (helper)
bool CameraController::ShouldStopMoving(const BallTrackingData& tracking) {
    // 1. 확실한 정지 패턴
    if (tracking.IsDefinitelyStable()) {
        return true;
    }

    // 2. 연속 안정 프레임이 충분
    if (tracking.consecutiveStableFrames >= tracking.requiredStableFramesForStop) {
        return true;
    }

    // 3. 매우 낮은 평균 델타
    if (tracking.averageFrameDelta < tracking.currentStabilityThreshold * 0.5f &&
        tracking.frameDeltas.size() >= 10) {
        return true;
    }

    return false;
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
        info->stableDurationMs = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - m_ballTracking.stableStartTime).count());
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

void CameraController::SetShotCompletedCallback(InternalShotCompletedCallback callback, void* context) {
    m_shotCompletedCallback = callback;
    m_shotCompletedCallbackContext = context;

    if (callback) {
        LOG_INFO("Shot completed callback registered");
    }
    else {
        LOG_INFO("Shot completed callback cleared");
    }
}

bool CameraController::GetLastShotTrajectory(ShotTrajectoryData* data) const {
    if (!data) return false;

    std::lock_guard<std::mutex> lock(m_shotDataMutex);
    *data = m_currentShotData;
    return !m_currentShotData.fullTrajectory.empty();
}

void CameraController::ClearShotTrajectory() {
    std::lock_guard<std::mutex> lock(m_shotDataMutex);
    m_currentShotData.Clear();
    LOG_INFO("Shot trajectory data cleared");
}

bool CameraController::SaveTrajectoryData(const std::string& filename) const {
    std::lock_guard<std::mutex> lock(m_shotDataMutex);

    if (m_currentShotData.fullTrajectory.empty()) {
        LOG_ERROR("No trajectory data to save");
        return false;
    }

    try {
        std::ofstream file(filename);
        if (!file.is_open()) {
            LOG_ERROR("Failed to open file: " + filename);
            return false;
        }

        // Write header with metadata
        file << "# Golf Ball Putting Trajectory Data\n";
        file << "# Generated: " << std::chrono::system_clock::now().time_since_epoch().count() << "\n";
        file << "# Format: Frame,Timestamp,X,Y,Radius,Confidence,State,StateString\n";
        file << "#\n";

        // Write shot summary at the top
        file << "# Shot Summary:\n";
        file << "# Start Position: (" << std::fixed << std::setprecision(2)
            << m_currentShotData.startPosition.x << ", "
            << m_currentShotData.startPosition.y << ")\n";
        file << "# End Position: (" << m_currentShotData.endPosition.x << ", "
            << m_currentShotData.endPosition.y << ")\n";
        file << "# Total Distance: " << m_currentShotData.totalDistance << " pixels\n";
        file << "# Shot Duration: " << std::fixed << std::setprecision(3)
            << m_currentShotData.shotDuration << " seconds\n";
        file << "# Max Velocity: " << std::fixed << std::setprecision(2)
            << m_currentShotData.maxVelocity << " pixels/sec\n";
        file << "# Average Velocity: " << m_currentShotData.averageVelocity << " pixels/sec\n";
        file << "# Total Points: " << m_currentShotData.fullTrajectory.size() << "\n";
        file << "# Moving Points: " << m_currentShotData.movingTrajectory.size() << "\n";
        file << "#\n";

        // Write column headers
        file << "Frame,Timestamp,X,Y,Radius,Confidence,State,StateString\n";

        // Write all trajectory points
        for (const auto& point : m_currentShotData.fullTrajectory) {
            // Get state string
            std::string stateStr;
            switch (point.state) {
            case BallState::NOT_DETECTED: stateStr = "NOT_DETECTED"; break;
            case BallState::MOVING: stateStr = "MOVING"; break;
            case BallState::STABILIZING: stateStr = "STABILIZING"; break;
            case BallState::READY: stateStr = "READY"; break;
            case BallState::STOPPED: stateStr = "STOPPED"; break;
            default: stateStr = "UNKNOWN"; break;
            }

            file << point.frameNumber << ","
                << std::fixed << std::setprecision(6) << point.timestamp << ","
                << std::fixed << std::setprecision(2) << point.position.x << ","
                << std::fixed << std::setprecision(2) << point.position.y << ","
                << std::fixed << std::setprecision(2) << point.radius << ","
                << std::fixed << std::setprecision(4) << point.confidence << ","
                << static_cast<int>(point.state) << ","
                << stateStr << "\n";
        }

        // Add trajectory analysis section
        file << "\n# Trajectory Analysis\n";

        // Calculate and write velocity profile
        file << "# Velocity Profile (for MOVING state only):\n";
        file << "# PointIndex,Time,X,Y,Velocity,Acceleration\n";

        if (m_currentShotData.movingTrajectory.size() >= 2) {
            for (size_t i = 1; i < m_currentShotData.movingTrajectory.size(); ++i) {
                const auto& prev = m_currentShotData.movingTrajectory[i - 1];
                const auto& curr = m_currentShotData.movingTrajectory[i];

                float distance = static_cast<float>(cv::norm(curr.position - prev.position));
                double timeDiff = curr.timestamp - prev.timestamp;
                double velocity = (timeDiff > 0) ? distance / timeDiff : 0.0;

                double acceleration = 0.0;
                if (i >= 2) {
                    const auto& prev2 = m_currentShotData.movingTrajectory[i - 2];
                    float distance2 = static_cast<float>(cv::norm(prev.position - prev2.position));
                    double timeDiff2 = prev.timestamp - prev2.timestamp;
                    double velocity2 = (timeDiff2 > 0) ? distance2 / timeDiff2 : 0.0;
                    double totalTimeDiff = curr.timestamp - prev2.timestamp;
                    acceleration = (totalTimeDiff > 0) ? (velocity - velocity2) / totalTimeDiff : 0.0;
                }

                file << i << ","
                    << std::fixed << std::setprecision(6) << curr.timestamp << ","
                    << std::fixed << std::setprecision(2) << curr.position.x << ","
                    << std::fixed << std::setprecision(2) << curr.position.y << ","
                    << std::fixed << std::setprecision(2) << velocity << ","
                    << std::fixed << std::setprecision(2) << acceleration << "\n";
            }
        }

        // Add section markers for easy parsing
        file << "\n# END OF DATA\n";

        file.close();
        LOG_INFO("Trajectory data saved to: " + filename);

        // Also save a simplified CSV version for Excel
        std::string csvFilename = filename.substr(0, filename.find_last_of('.')) + "_simple.csv";
        SaveTrajectoryDataCSV(csvFilename);

        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Failed to save trajectory data: " + std::string(e.what()));
        return false;
    }
}


bool CameraController::SaveTrajectoryDataCSV(const std::string& filename) const {
    try {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        
        // Simple CSV header
        file << "Frame,Time(sec),X,Y,State\n";
        
        // Write data
        double startTime = m_currentShotData.fullTrajectory.empty() ? 0.0 : 
                          m_currentShotData.fullTrajectory[0].timestamp;
        
        for (const auto& point : m_currentShotData.fullTrajectory) {
            file << point.frameNumber << ","
                 << std::fixed << std::setprecision(3) << (point.timestamp - startTime) << ","
                 << std::fixed << std::setprecision(1) << point.position.x << ","
                 << std::fixed << std::setprecision(1) << point.position.y << ","
                 << static_cast<int>(point.state) << "\n";
        }
        
        file.close();
        LOG_INFO("Simple CSV saved to: " + filename);
        return true;
    }
    catch (...) {
        return false;
    }
}

// Dynamic ROI implementation
void CameraController::UpdateDynamicROI(const RealtimeDetectionResult* result) {
    if (!m_dynamicROIEnabled.load() || !result) return;

    std::lock_guard<std::mutex> lock(m_dynamicROIMutex);

    BallState currentBallState = m_ballTracking.currentState;

    // Determine ROI mode based on ball state
    switch (currentBallState) {
    case BallState::NOT_DETECTED:
    case BallState::STABILIZING:
        // Full frame processing when no ball or stabilizing
        if (m_usingDynamicROI) {
            ClearDynamicROI();
            SetROIMode(ROIMode::FULL_FRAME);
        }
        break;

    case BallState::READY:
        // Small ROI for READY state
        if (result->ballFound && result->ballCount > 0) {
            const RealtimeBallInfo& ball = result->balls[0];

            // Use smaller multiplier for READY state
            float readyMultiplier = m_dynamicROIConfig.roiSizeMultiplier;
            cv::Rect newROI = CalculateDynamicROI(ball.centerX, ball.centerY,
                ball.radius, BallState::READY);

            if (ShouldUpdateROI(newROI)) {
                ApplyDynamicROI(newROI);
                SetROIMode(ROIMode::READY_ROI);
            }
        }
        break;

    case BallState::MOVING:
        // Larger ROI for MOVING state to handle sudden movements
        if (result->ballFound && result->ballCount > 0) {
            const RealtimeBallInfo& ball = result->balls[0];

            // Use larger multiplier for MOVING state
            cv::Rect newROI = CalculateDynamicROI(ball.centerX, ball.centerY,
                ball.radius, BallState::MOVING);

            if (ShouldUpdateROI(newROI)) {
                ApplyDynamicROI(newROI);
                SetROIMode(ROIMode::MOVING_ROI);
            }
        }
        else {
            // If we lose the ball while moving, expand ROI or go full frame
            if (m_usingDynamicROI) {
                // Expand current ROI by 50%
                cv::Rect expandedROI = m_currentROI;
                int expansion = static_cast<int>(m_currentROI.width * 0.25);
                expandedROI.x = std::max(0, expandedROI.x - expansion);
                expandedROI.y = std::max(0, expandedROI.y - expansion);
                expandedROI.width = std::min(width - expandedROI.x, expandedROI.width + 2 * expansion);
                expandedROI.height = std::min(height - expandedROI.y, expandedROI.height + 2 * expansion);

                ApplyDynamicROI(expandedROI);
            }
        }
        break;

    case BallState::STOPPED:
        // Maintain current ROI for STOPPED state
        // Don't change ROI to allow for shot completion processing
        break;
    }

    m_lastROIUpdateTime = std::chrono::steady_clock::now();
}


bool CameraController::ShouldUpdateROI(const cv::Rect& newROI) {
    const int POSITION_THRESHOLD = 10;  // pixels
    const int SIZE_THRESHOLD = 20;      // pixels

    return !m_usingDynamicROI ||
        std::abs(newROI.x - m_currentROI.x) > POSITION_THRESHOLD ||
        std::abs(newROI.y - m_currentROI.y) > POSITION_THRESHOLD ||
        std::abs(newROI.width - m_currentROI.width) > SIZE_THRESHOLD ||
        std::abs(newROI.height - m_currentROI.height) > SIZE_THRESHOLD;
}

cv::Rect CameraController::CalculateDynamicROI(float centerX, float centerY, float radius, BallState state) {
    float roiMultiplier = m_dynamicROIConfig.roiSizeMultiplier;

    // Adjust multiplier based on state
    switch (state) {
    case BallState::READY:
        // Smaller ROI for stationary ball
        roiMultiplier = m_dynamicROIConfig.roiSizeMultiplier;
        break;

    case BallState::MOVING:
        // Larger ROI to accommodate sudden movements
        roiMultiplier = m_dynamicROIConfig.roiSizeMultiplier * 2.0f;
        break;

    default:
        break;
    }

    // Calculate ROI size
    float roiSize = radius * 2.0f * roiMultiplier;

    // Apply min/max constraints
    float maxSize = (state == BallState::MOVING) ?
        m_dynamicROIConfig.maxROISize * 1.5f :
        m_dynamicROIConfig.maxROISize;

    roiSize = std::max(m_dynamicROIConfig.minROISize,
        std::min(maxSize, roiSize));

    // Make size even for better alignment
    int size = static_cast<int>(roiSize);
    size = (size / 2) * 2;

    // Calculate position
    int halfSize = size / 2;
    int x = static_cast<int>(centerX) - halfSize;
    int y = static_cast<int>(centerY) - halfSize;

    // Clamp to image boundaries
    x = std::max(0, x);
    y = std::max(0, y);

    if (x + size > width) {
        size = width - x;
    }
    if (y + size > height) {
        size = height - y;
    }

    // Ensure minimum size
    const int MIN_ROI_SIZE = static_cast<int>(m_dynamicROIConfig.minROISize);
    if (size < MIN_ROI_SIZE) {
        if (x > 0 && x + size < width) {
            int expand = MIN_ROI_SIZE - size;
            x = std::max(0, x - expand / 2);
            size = MIN_ROI_SIZE;
        }
        if (y > 0 && y + size < height) {
            int expand = MIN_ROI_SIZE - size;
            y = std::max(0, y - expand / 2);
            size = MIN_ROI_SIZE;
        }
    }

    return cv::Rect(x, y, size, size);
}

void CameraController::ApplyDynamicROI(const cv::Rect& roi) {
    // Update detection parameters
    if (m_realtimeBallDetector) {
        auto params = m_realtimeBallDetector->GetParameters();
        //params.useROI = true;
        params.roiScale = 1.0f; // Full scale within ROI
        m_realtimeBallDetector->SetParameters(params);
    }

    // Store ROI info
    m_currentROI = roi;
    m_usingDynamicROI = true;

    // Update info structure
    m_dynamicROIInfo.active = true;
    m_dynamicROIInfo.centerX = roi.x + roi.width / 2;
    m_dynamicROIInfo.centerY = roi.y + roi.height / 2;
    m_dynamicROIInfo.width = roi.width;
    m_dynamicROIInfo.height = roi.height;

    // Calculate processing time reduction
    float fullFramePixels = static_cast<float>(width * height);
    float roiPixels = static_cast<float>(roi.width * roi.height);
    m_dynamicROIInfo.processingTimeReduction = (1.0f - roiPixels / fullFramePixels) * 100.0f;

    LOG_INFO("Dynamic ROI applied: " + std::to_string(roi.x) + "," +
        std::to_string(roi.y) + " " + std::to_string(roi.width) + "x" +
        std::to_string(roi.height) + " (reduction: " +
        std::to_string(m_dynamicROIInfo.processingTimeReduction) + "%)");
}

void CameraController::ClearDynamicROI() {
    if (m_realtimeBallDetector) {
        auto params = m_realtimeBallDetector->GetParameters();
        //params.useROI = false;
        params.roiScale = 0.75f; // Default
        m_realtimeBallDetector->SetParameters(params);
    }

    m_currentROI = cv::Rect(0, 0, width, height);
    m_usingDynamicROI = false;

    m_dynamicROIInfo.active = false;
    m_dynamicROIInfo.processingTimeReduction = 0.0f;

    LOG_INFO("Dynamic ROI cleared - using full frame");
}

void CameraController::SetROIMode(ROIMode mode) {
    m_currentROIMode = mode;

    std::string modeStr;
    switch (mode) {
    case ROIMode::FULL_FRAME:
        modeStr = "FULL_FRAME";
        break;
    case ROIMode::READY_ROI:
        modeStr = "READY_ROI";
        break;
    case ROIMode::MOVING_ROI:
        modeStr = "MOVING_ROI";
        break;
    case ROIMode::TRACKING_ROI:
        modeStr = "TRACKING_ROI";
        break;
    }

    LOG_INFO("ROI mode changed to: " + modeStr);
}

bool CameraController::EnableDynamicROI(bool enable) {
    if (enable == m_dynamicROIEnabled.load()) {
        return true;
    }

    m_dynamicROIEnabled = enable;

    if (!enable) {
        std::lock_guard<std::mutex> lock(m_dynamicROIMutex);
        ClearDynamicROI();
    }

    LOG_INFO("Dynamic ROI " + std::string(enable ? "enabled" : "disabled"));
    return true;
}

bool CameraController::SetDynamicROIConfig(const DynamicROIConfig& config) {
    std::lock_guard<std::mutex> lock(m_dynamicROIMutex);
    m_dynamicROIConfig = config;

    LOG_INFO("Dynamic ROI config updated: multiplier=" +
        std::to_string(config.roiSizeMultiplier) +
        ", min=" + std::to_string(config.minROISize) +
        ", max=" + std::to_string(config.maxROISize));
    return true;
}

DynamicROIConfig CameraController::GetDynamicROIConfig() const {
    std::lock_guard<std::mutex> lock(m_dynamicROIMutex);
    return m_dynamicROIConfig;
}

bool CameraController::GetDynamicROIInfo(DynamicROIInfo* info) const {
    if (!info) return false;

    std::lock_guard<std::mutex> lock(m_dynamicROIMutex);
    *info = m_dynamicROIInfo;
    return true;
}

void CameraController::ResetDynamicROI() {
    std::lock_guard<std::mutex> lock(m_dynamicROIMutex);
    ClearDynamicROI();
}