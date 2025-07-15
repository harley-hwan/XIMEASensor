#include "pch.h"
#include "CameraController.h"
#include <algorithm>
#include <sstream>
#include <cstring>
#include <iomanip>
#include <algorithm>


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
    currentExposure(CameraDefaults::EXPOSURE_US),  // Use DLL default
    currentGain(CameraDefaults::GAIN_DB),          // Use DLL default
    currentState(CameraState::DISCONNECTED) {

    size_t bufferSize = width * height;
    frameBuffer = new unsigned char[bufferSize];
    workingBuffer = new unsigned char[bufferSize];

    stats.Reset();

    m_continuousCapture = std::make_unique<ContinuousCaptureManager>();

    LOG_INFO("CameraController initialized");
}

CameraController::~CameraController() {
    LOG_INFO("CameraController destructor called");

    try {
        // Clear callbacks first
        {
            std::lock_guard<std::mutex> lock(callbackMutex);
            callbacks.clear();
        }

        if (isRunning.load()) {
            isRunning = false;

            if (captureThread.joinable()) {
                isPaused = false;

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

        if (xiH != nullptr) {
            XI_RETURN stat = xiStopAcquisition(xiH);

            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            stat = xiCloseDevice(xiH);
            if (stat != XI_OK) {
                LOG_ERROR("Error closing camera in destructor: " + GetXiApiErrorString(stat));
            }
            xiH = nullptr;
        }

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

    // MQ013MG-ON initialization
    LOG_INFO("Configuring MQ013MG-ON camera");

    // Set pixel format
    stat = xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);
    if (stat != XI_OK) {
        LOG_WARNING("Failed to set image format: " + GetXiApiErrorString(stat));
    }

    // Basic camera settings
    xiSetParamInt(xiH, XI_PRM_OUTPUT_DATA_BIT_DEPTH, XI_BPP_8);
    xiSetParamInt(xiH, XI_PRM_SENSOR_DATA_BIT_DEPTH, XI_BPP_10);
    xiSetParamInt(xiH, XI_PRM_WIDTH, PYTHON1300_WIDTH);
    xiSetParamInt(xiH, XI_PRM_HEIGHT, PYTHON1300_HEIGHT);
    xiSetParamInt(xiH, XI_PRM_EXPOSURE, CameraDefaults::EXPOSURE_US);  // Use DLL default
    currentExposure = CameraDefaults::EXPOSURE_US;

    xiSetParamFloat(xiH, XI_PRM_GAIN, CameraDefaults::GAIN_DB);        // Use DLL default
    currentGain = CameraDefaults::GAIN_DB;

    // Set default framerate
    xiSetParamFloat(xiH, XI_PRM_FRAMERATE, CameraDefaults::FRAMERATE_FPS);
    currentFrameRate = CameraDefaults::FRAMERATE_FPS;

    // Configure acquisition
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

    // Allocate frame buffers
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

            memset(frameBuffer, 0, bufferSize);
            memset(workingBuffer, 0, bufferSize);
        }
        catch (const std::bad_alloc& e) {
            LOG_ERROR("Failed to allocate frame buffers: " + std::string(e.what()));

            if (frameBuffer) {
                delete[] frameBuffer;
                frameBuffer = nullptr;
            }
            if (workingBuffer) {
                delete[] workingBuffer;
                workingBuffer = nullptr;
            }

            xiCloseDevice(xiH);
            xiH = nullptr;

            return false;
        }
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

    // Calculate frame period based on current framerate
    auto frameInterval = std::chrono::microseconds((int)(1000000.0f / currentFrameRate));
    auto nextFrameTime = std::chrono::steady_clock::now();

    while (isRunning.load()) {
        if (isPaused) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Frame rate limiting
        auto now = std::chrono::steady_clock::now();
        if (now < nextFrameTime) {
            std::this_thread::sleep_until(nextFrameTime);
        }
        nextFrameTime += frameInterval;

        // Update frame interval if framerate changed
        frameInterval = std::chrono::microseconds((int)(1000000.0f / currentFrameRate));

        XI_RETURN stat = xiGetImage(xiH, 100, &image);

        if (stat == XI_OK) {
            deviceNotReadyCount = 0;

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

            // Process continuous capture if active
            if (m_continuousCapture && m_continuousCapture->IsCapturing()) {
                m_continuousCapture->ProcessFrame(frameBuffer, width, height);
            }

            NotifyFrameReceived(frameInfo);
            UpdateStatistics(true);
        }
        else if (stat == XI_TIMEOUT) {
            // Timeout is normal
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

            UpdateStatistics(false);
        }
        else {
            LOG_ERROR("Frame grab error: " + GetXiApiErrorString(stat));
            NotifyError(static_cast<CameraError>(stat),
                "Frame grab error: " + GetXiApiErrorString(stat));
            UpdateStatistics(false);
        }
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

    // Clamp value to valid range using DLL limits
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

    // Check if current frame rate is still valid
    float currentFPS = GetFrameRate();
    float maxPossibleFPS = 1000000.0f / microsec;

    if (currentFPS > maxPossibleFPS) {
        LOG_INFO("Adjusting frame rate due to exposure time change");
        SetFrameRate(maxPossibleFPS * 0.95f); // Set to 95% of max to ensure stability
    }

    return true;
}

bool CameraController::SetGain(float gain) {
    if (!xiH) {
        LOG_ERROR("Camera not opened");
        return false;
    }

    // Clamp value to valid range using DLL limits
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

    // Boundary check
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

    // Set ROI parameters
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

    // Reallocate frame buffers
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

    // Clamp to hardware limits first
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

    // Check if requested FPS is possible with current exposure time
    float maxPossibleFPS = 1000000.0f / currentExposure;
    if (fps > maxPossibleFPS) {
        LOG_WARNING("Requested FPS (" + std::to_string(fps) +
            ") exceeds maximum possible with current exposure time (" +
            std::to_string(currentExposure) + "us). Max possible: " +
            std::to_string(maxPossibleFPS) + " FPS");
        fps = maxPossibleFPS * 0.95f; // Set to 95% of max to ensure stability
    }

    XI_RETURN stat = xiSetParamFloat(xiH, XI_PRM_FRAMERATE, fps);
    if (stat != XI_OK) {
        LOG_WARNING("Failed to set frame rate: " + GetXiApiErrorString(stat));

        // Try to get actual frame rate from camera
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
    return currentFrameRate; // Return cached value if API call fails
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

        // Update min/max FPS based on actual measured frame rate
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