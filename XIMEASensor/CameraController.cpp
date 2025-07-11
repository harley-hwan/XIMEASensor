#include "pch.h"
#include "CameraController.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>

// Static 멤버 초기화
std::unique_ptr<CameraController> CameraController::instance = nullptr;
std::mutex CameraController::instanceMutex;

// 상수 정의
constexpr int DEFAULT_BUFFER_COUNT = 10;
constexpr int MIN_EXPOSURE_US = 10;
constexpr int MAX_EXPOSURE_US = 10000000;  // 10초
constexpr float MIN_GAIN_DB = 0.0f;
constexpr float MAX_GAIN_DB = 24.0f;

CameraController::CameraController()
    : xiH(nullptr)
    , state(CameraState::UNINITIALIZED)
    , isCapturing(false)
    , paramsChanged(false)
    , frameCallback(nullptr)
    , errorCallback(nullptr)
    , logCallback(nullptr)
{
    perfStats.framesCaptures = 0;
    perfStats.framesDropped = 0;
    perfStats.currentFps = 0.0f;
    perfStats.startTime = std::chrono::steady_clock::now();
}

CameraController::~CameraController()
{
    Shutdown();
}

CameraController& CameraController::GetInstance()
{
    std::lock_guard<std::mutex> lock(instanceMutex);
    if (!instance) {
        instance.reset(new CameraController());
    }
    return *instance;
}

void CameraController::DestroyInstance()
{
    std::lock_guard<std::mutex> lock(instanceMutex);
    instance.reset();
}

bool CameraController::Initialize(int deviceIndex)
{
    std::lock_guard<std::mutex> lock(paramMutex);

    if (state != CameraState::UNINITIALIZED) {
        Log(LogLevel::LOG_WARNING, "Camera already initialized");
        return false;
    }

    // 카메라 열기
    XI_RETURN status = xiOpenDevice(deviceIndex, &xiH);
    if (status != XI_OK) {
        Log(LogLevel::LOG_ERROR, "Failed to open camera: " + GetXiErrorString(status));
        return false;
    }

    Log(LogLevel::LOG_INFO, "Camera opened successfully");

    // 기본 파라미터 설정
    if (!ApplyParameters()) {
        xiCloseDevice(xiH);
        xiH = nullptr;
        return false;
    }

    // 버퍼 초기화
    InitializeBuffers();

    state = CameraState::INITIALIZED;
    return true;
}

bool CameraController::StartCapture()
{
    if (state != CameraState::INITIALIZED) {
        Log(LogLevel::LOG_ERROR, "Camera not initialized");
        return false;
    }

    XI_RETURN status = xiStartAcquisition(xiH);
    if (status != XI_OK) {
        Log(LogLevel::LOG_ERROR, "Failed to start acquisition: " + GetXiErrorString(status));
        return false;
    }

    isCapturing = true;
    state = CameraState::STREAMING;

    // 캡처 스레드 시작
    captureThread = std::make_unique<std::thread>(&CameraController::CaptureLoop, this);

    // 콜백 스레드 시작 (콜백이 설정된 경우)
    if (frameCallback) {
        callbackThread = std::make_unique<std::thread>(&CameraController::CallbackLoop, this);
    }

    Log(LogLevel::LOG_INFO, "Capture started");
    perfStats.startTime = std::chrono::steady_clock::now();

    return true;
}

void CameraController::StopCapture()
{
    if (!isCapturing) return;

    isCapturing = false;
    frameCV.notify_all();

    // 스레드 종료 대기
    if (captureThread && captureThread->joinable()) {
        captureThread->join();
    }
    if (callbackThread && callbackThread->joinable()) {
        callbackThread->join();
    }

    // 획득 중지
    if (xiH) {
        xiStopAcquisition(xiH);
    }

    state = CameraState::INITIALIZED;
    Log(LogLevel::LOG_INFO, "Capture stopped");
}

void CameraController::Shutdown()
{
    StopCapture();

    if (xiH) {
        xiCloseDevice(xiH);
        xiH = nullptr;
    }

    CleanupBuffers();
    state = CameraState::UNINITIALIZED;
    Log(LogLevel::LOG_INFO, "Camera shutdown");
}

void CameraController::CaptureLoop()
{
    XI_IMG image;
    memset(&image, 0, sizeof(image));
    image.size = sizeof(XI_IMG);

    auto lastStatsUpdate = std::chrono::steady_clock::now();
    uint64_t frameCount = 0;

    while (isCapturing)
    {
        // 파라미터 변경 확인
        if (paramsChanged) {
            std::lock_guard<std::mutex> lock(paramMutex);
            if (ApplyParameters()) {
                params = targetParams;
                paramsChanged = false;
            }
        }

        // 이미지 획득
        XI_RETURN status = xiGetImage(xiH, 100, &image);

        if (status == XI_OK) {
            std::lock_guard<std::mutex> lock(bufferMutex);

            if (!availableBuffers.empty()) {
                int bufferIndex = availableBuffers.front();
                availableBuffers.pop();

                // 프레임 정보 복사
                FrameBuffer& buffer = frameBuffers[bufferIndex];
                size_t imageSize = image.width * image.height * ((image.bp_size > 0) ? image.bp_size : 1);

                if (buffer.data.size() < imageSize) {
                    buffer.data.resize(imageSize);
                }

                memcpy(buffer.data.data(), image.bp, imageSize);

                // 메타데이터 저장
                buffer.info.data = buffer.data.data();
                buffer.info.width = image.width;
                buffer.info.height = image.height;
                buffer.info.stride = image.width;
                buffer.info.frameNumber = image.nframe;
                buffer.info.timestamp = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - perfStats.startTime).count();
                buffer.info.exposureTime = params.exposureUs / 1000.0f;  // ms
                buffer.info.gain = params.gain;
                buffer.info.format = image.frm;

                buffer.ready = true;
                readyBuffers.push(bufferIndex);

                frameCV.notify_one();
                perfStats.framesCaptures++;
                frameCount++;
            }
            else {
                perfStats.framesDropped++;
                Log(LogLevel::LOG_WARNING, "Frame dropped - no available buffers");
            }
        }
        else if (status != XI_TIMEOUT) {
            Log(LogLevel::LOG_ERROR, "Image acquisition error: " + GetXiErrorString(status));
        }

        // 성능 통계 업데이트 (1초마다)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastStatsUpdate);
        if (elapsed.count() >= 1000) {
            perfStats.currentFps = frameCount * 1000.0f / elapsed.count();
            frameCount = 0;
            lastStatsUpdate = now;
        }
    }
}

void CameraController::CallbackLoop()
{
    while (isCapturing)
    {
        std::unique_lock<std::mutex> lock(bufferMutex);
        frameCV.wait(lock, [this] { return !readyBuffers.empty() || !isCapturing; });

        if (!isCapturing) break;

        if (!readyBuffers.empty()) {
            int bufferIndex = readyBuffers.front();
            readyBuffers.pop();

            FrameBuffer& buffer = frameBuffers[bufferIndex];

            // 콜백 호출 (락 해제 후)
            lock.unlock();

            if (frameCallback) {
                frameCallback(buffer.info);
            }

            // 버퍼 반환
            lock.lock();
            buffer.ready = false;
            availableBuffers.push(bufferIndex);
        }
    }
}

bool CameraController::ApplyParameters()
{
    if (!xiH) return false;

    XI_RETURN status;

    // 이미지 포맷
    status = xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);
    if (status != XI_OK) {
        Log(LogLevel::LOG_ERROR, "Failed to set image format: " + GetXiErrorString(status));
        return false;
    }

    // ROI 설정
    status = xiSetParamInt(xiH, XI_PRM_WIDTH, targetParams.width);
    if (status != XI_OK) {
        Log(LogLevel::LOG_ERROR, "Failed to set width: " + GetXiErrorString(status));
        return false;
    }

    status = xiSetParamInt(xiH, XI_PRM_HEIGHT, targetParams.height);
    if (status != XI_OK) {
        Log(LogLevel::LOG_ERROR, "Failed to set height: " + GetXiErrorString(status));
        return false;
    }

    // 노출 시간
    status = xiSetParamInt(xiH, XI_PRM_EXPOSURE, targetParams.exposureUs);
    if (status != XI_OK) {
        Log(LogLevel::LOG_ERROR, "Failed to set exposure: " + GetXiErrorString(status));
        return false;
    }

    // 게인
    status = xiSetParamFloat(xiH, XI_PRM_GAIN, targetParams.gain);
    if (status != XI_OK) {
        Log(LogLevel::LOG_ERROR, "Failed to set gain: " + GetXiErrorString(status));
        return false;
    }

    // FPS 제한
    if (targetParams.fps > 0) {
        status = xiSetParamFloat(xiH, XI_PRM_FRAMERATE, targetParams.fps);
        if (status != XI_OK) {
            Log(LogLevel::LOG_WARNING, "Failed to set FPS limit: " + GetXiErrorString(status));
        }
    }

    // 트리거 모드
    int triggerSource = XI_TRG_OFF;
    switch (targetParams.triggerMode) {
    case TriggerMode::SOFTWARE_TRIGGER:
        triggerSource = XI_TRG_SOFTWARE;
        break;
    case TriggerMode::HARDWARE_TRIGGER:
        triggerSource = XI_TRG_EDGE_RISING;
        break;
    default:
        triggerSource = XI_TRG_OFF;
        break;
    }

    status = xiSetParamInt(xiH, XI_PRM_TRG_SOURCE, triggerSource);
    if (status != XI_OK) {
        Log(LogLevel::LOG_WARNING, "Failed to set trigger mode: " + GetXiErrorString(status));
    }

    // 자동 노출/게인
    status = xiSetParamInt(xiH, XI_PRM_AEAG, targetParams.autoExposure || targetParams.autoGain ? XI_ON : XI_OFF);
    if (status != XI_OK) {
        Log(LogLevel::LOG_WARNING, "Failed to set auto exposure/gain: " + GetXiErrorString(status));
    }

    // 비닝
    if (targetParams.binningX > 1 || targetParams.binningY > 1) {
        xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING, targetParams.binningX);
        xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING_TYPE, XI_BINNING);
    }

    // 버퍼 정책
    xiSetParamInt(xiH, XI_PRM_BUFFER_POLICY, XI_BP_UNSAFE);
    xiSetParamInt(xiH, XI_PRM_BUFFERS_QUEUE_SIZE, targetParams.bufferCount);

    // 대역폭 자동 계산
    xiSetParamInt(xiH, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_ON);

    Log(LogLevel::LOG_INFO, "Camera parameters applied successfully");
    return true;
}

void CameraController::InitializeBuffers()
{
    std::lock_guard<std::mutex> lock(bufferMutex);

    frameBuffers.clear();
    frameBuffers.resize(params.bufferCount);

    // 버퍼 큐 초기화
    std::queue<int> empty1, empty2;
    availableBuffers.swap(empty1);
    readyBuffers.swap(empty2);

    for (int i = 0; i < params.bufferCount; i++) {
        frameBuffers[i].data.reserve(params.width * params.height * 2);  // 여유있게 할당
        frameBuffers[i].ready = false;
        availableBuffers.push(i);
    }
}

void CameraController::CleanupBuffers()
{
    std::lock_guard<std::mutex> lock(bufferMutex);
    frameBuffers.clear();
}

bool CameraController::SetParameter(const std::string& name, int value)
{
    if (!xiH) {
        Log(LogLevel::LOG_ERROR, "Camera not initialized");
        return false;
    }

    XI_RETURN status = XI_INVALID_ARG;

    // 파라미터 이름을 XI_PRM_* 형식으로 변환
    if (name == "exposure") {
        status = xiSetParamInt(xiH, XI_PRM_EXPOSURE, value);
    }
    else if (name == "width") {
        status = xiSetParamInt(xiH, XI_PRM_WIDTH, value);
    }
    else if (name == "height") {
        status = xiSetParamInt(xiH, XI_PRM_HEIGHT, value);
    }
    else if (name == "offset_x") {
        status = xiSetParamInt(xiH, XI_PRM_OFFSET_X, value);
    }
    else if (name == "offset_y") {
        status = xiSetParamInt(xiH, XI_PRM_OFFSET_Y, value);
    }
    else if (name == "trg_source") {
        status = xiSetParamInt(xiH, XI_PRM_TRG_SOURCE, value);
    }
    else if (name == "buffer_policy") {
        status = xiSetParamInt(xiH, XI_PRM_BUFFER_POLICY, value);
    }
    else if (name == "buffers_queue_size") {
        status = xiSetParamInt(xiH, XI_PRM_BUFFERS_QUEUE_SIZE, value);
    }
    else if (name == "aeag") {
        status = xiSetParamInt(xiH, XI_PRM_AEAG, value);
    }
    else if (name == "downsampling") {
        status = xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING, value);
    }
    else if (name == "downsampling_type") {
        status = xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING_TYPE, value);
    }
    else {
        Log(LogLevel::LOG_WARNING, "Unknown parameter: " + name);
        return false;
    }

    if (status != XI_OK) {
        Log(LogLevel::LOG_ERROR, "Failed to set parameter " + name + ": " + GetXiErrorString(status));
        return false;
    }

    Log(LogLevel::LOG_DEBUG, "Set parameter " + name + " = " + std::to_string(value));
    return true;
}

bool CameraController::SetParameter(const std::string& name, float value)
{
    if (!xiH) {
        Log(LogLevel::LOG_ERROR, "Camera not initialized");
        return false;
    }

    XI_RETURN status = XI_INVALID_ARG;

    // 파라미터 이름을 XI_PRM_* 형식으로 변환
    if (name == "gain") {
        status = xiSetParamFloat(xiH, XI_PRM_GAIN, value);
    }
    else if (name == "framerate") {
        status = xiSetParamFloat(xiH, XI_PRM_FRAMERATE, value);
    }
    else if (name == "aeag_level") {
        status = xiSetParamFloat(xiH, XI_PRM_AEAG_LEVEL, value);
    }
    else {
        Log(LogLevel::LOG_WARNING, "Unknown float parameter: " + name);
        return false;
    }

    if (status != XI_OK) {
        Log(LogLevel::LOG_ERROR, "Failed to set parameter " + name + ": " + GetXiErrorString(status));
        return false;
    }

    Log(LogLevel::LOG_DEBUG, "Set parameter " + name + " = " + std::to_string(value));
    return true;
}


bool CameraController::GetParameter(const std::string& name, int& value) const
{
    if (!xiH) {
        return false;
    }

    XI_RETURN status = XI_INVALID_ARG;

    // 파라미터 이름을 XI_PRM_* 형식으로 변환
    if (name == "exposure") {
        status = xiGetParamInt(xiH, XI_PRM_EXPOSURE, &value);
    }
    else if (name == "width") {
        status = xiGetParamInt(xiH, XI_PRM_WIDTH, &value);
    }
    else if (name == "height") {
        status = xiGetParamInt(xiH, XI_PRM_HEIGHT, &value);
    }
    else if (name == "offset_x") {
        status = xiGetParamInt(xiH, XI_PRM_OFFSET_X, &value);
    }
    else if (name == "offset_y") {
        status = xiGetParamInt(xiH, XI_PRM_OFFSET_Y, &value);
    }
    else if (name == "trg_source") {
        status = xiGetParamInt(xiH, XI_PRM_TRG_SOURCE, &value);
    }
    else if (name == "buffer_policy") {
        status = xiGetParamInt(xiH, XI_PRM_BUFFER_POLICY, &value);
    }
    else if (name == "buffers_queue_size") {
        status = xiGetParamInt(xiH, XI_PRM_BUFFERS_QUEUE_SIZE, &value);
    }
    else if (name == "aeag") {
        status = xiGetParamInt(xiH, XI_PRM_AEAG, &value);
    }
    else if (name == "downsampling") {
        status = xiGetParamInt(xiH, XI_PRM_DOWNSAMPLING, &value);
    }
    else if (name == "downsampling_type") {
        status = xiGetParamInt(xiH, XI_PRM_DOWNSAMPLING_TYPE, &value);
    }
    else if (name == "image_data_format") {
        status = xiGetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, &value);
    }
    else {
        return false;
    }

    return (status == XI_OK);
}

bool CameraController::GetParameter(const std::string& name, float& value) const
{
    if (!xiH) {
        return false;
    }

    XI_RETURN status = XI_INVALID_ARG;

    // 파라미터 이름을 XI_PRM_* 형식으로 변환
    if (name == "gain") {
        status = xiGetParamFloat(xiH, XI_PRM_GAIN, &value);
    }
    else if (name == "framerate") {
        status = xiGetParamFloat(xiH, XI_PRM_FRAMERATE, &value);
    }
    else if (name == "aeag_level") {
        status = xiGetParamFloat(xiH, XI_PRM_AEAG_LEVEL, &value);
    }
    else if (name == "temperature") {
        status = xiGetParamFloat(xiH, XI_PRM_TEMP_SELECTOR, &value);
    }
    else {
        return false;
    }

    return (status == XI_OK);
}

bool CameraController::SetExposure(int microsec)
{
    std::lock_guard<std::mutex> lock(paramMutex);

    if (microsec < MIN_EXPOSURE_US) microsec = MIN_EXPOSURE_US;
    if (microsec > MAX_EXPOSURE_US) microsec = MAX_EXPOSURE_US;

    targetParams.exposureUs = microsec;
    paramsChanged = true;

    Log(LogLevel::LOG_INFO, "Exposure set to " + std::to_string(microsec) + " us");
    return true;
}

bool CameraController::SetGain(float gain)
{
    std::lock_guard<std::mutex> lock(paramMutex);

    if (gain < MIN_GAIN_DB) gain = MIN_GAIN_DB;
    if (gain > MAX_GAIN_DB) gain = MAX_GAIN_DB;

    targetParams.gain = gain;
    paramsChanged = true;

    Log(LogLevel::LOG_INFO, "Gain set to " + std::to_string(gain) + " dB");
    return true;
}

bool CameraController::SetFPS(float fps)
{
    std::lock_guard<std::mutex> lock(paramMutex);

    if (fps < 0) fps = 0;  // 0 = unlimited
    if (fps > 1000) fps = 1000;

    targetParams.fps = fps;
    paramsChanged = true;

    Log(LogLevel::LOG_INFO, "FPS limit set to " + std::to_string(fps));
    return true;
}

bool CameraController::SetROI(int offsetX, int offsetY, int width, int height)
{
    std::lock_guard<std::mutex> lock(paramMutex);

    // 4의 배수로 정렬
    offsetX = (offsetX / 4) * 4;
    offsetY = (offsetY / 4) * 4;
    width = (width / 4) * 4;
    height = (height / 4) * 4;

    // 최소 크기 확인
    if (width < 32) width = 32;
    if (height < 32) height = 32;

    // TODO: 센서 크기 확인 및 범위 제한

    if (xiH && state == CameraState::INITIALIZED) {
        // 스트리밍 중이 아닐 때만 직접 설정
        xiSetParamInt(xiH, XI_PRM_OFFSET_X, offsetX);
        xiSetParamInt(xiH, XI_PRM_OFFSET_Y, offsetY);
    }

    targetParams.width = width;
    targetParams.height = height;
    paramsChanged = true;

    Log(LogLevel::LOG_INFO, "ROI set to " + std::to_string(width) + "x" +
        std::to_string(height) + " at (" + std::to_string(offsetX) +
        ", " + std::to_string(offsetY) + ")");

    return true;
}

bool CameraController::GetLatestFrame(unsigned char* buffer, int bufferSize,
    int& width, int& height, uint64_t& frameNumber)
{
    std::lock_guard<std::mutex> lock(bufferMutex);

    if (readyBuffers.empty()) {
        return false;
    }

    // 가장 최근 프레임 가져오기
    int bufferIndex = readyBuffers.back();

    // 이전 프레임들은 버림
    while (!readyBuffers.empty()) {
        int idx = readyBuffers.front();
        readyBuffers.pop();
        if (idx != bufferIndex) {
            frameBuffers[idx].ready = false;
            availableBuffers.push(idx);
        }
    }

    FrameBuffer& frameBuffer = frameBuffers[bufferIndex];

    int requiredSize = frameBuffer.info.width * frameBuffer.info.height;
    if (bufferSize < requiredSize) {
        return false;
    }

    memcpy(buffer, frameBuffer.data.data(), requiredSize);
    width = frameBuffer.info.width;
    height = frameBuffer.info.height;
    frameNumber = frameBuffer.info.frameNumber;

    // 버퍼 반환
    frameBuffer.ready = false;
    availableBuffers.push(bufferIndex);

    return true;
}

void CameraController::SetFrameCallback(FrameCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex);
    frameCallback = callback;
}

void CameraController::SetErrorCallback(ErrorCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex);
    errorCallback = callback;
}

void CameraController::SetLogCallback(LogCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex);
    logCallback = callback;
}

void CameraController::Log(LogLevel level, const std::string& message)
{
    // 로그 레벨 확인
    if (level < minLogLevel) return;

    std::lock_guard<std::mutex> lock(logMutex);

    // 타임스탬프 생성
    auto now = std::chrono::system_clock::now();
    auto now_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm tm_buf;
    localtime_s(&tm_buf, &now_t);
    std::stringstream ss;
    ss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << now_ms.count();


    // 로그 레벨 문자열
    std::string levelStr;
    switch (level) {
    case LogLevel::LOG_DEBUG: levelStr = "DEBUG"; break;
    case LogLevel::LOG_INFO: levelStr = "INFO"; break;
    case LogLevel::LOG_WARNING: levelStr = "WARN"; break;
    case LogLevel::LOG_ERROR: levelStr = "ERROR"; break;
    case LogLevel::LOG_CRITICAL: levelStr = "CRIT"; break;
    }

    std::string logMessage = ss.str() + " [" + levelStr + "] " + message;

    // 콜백 호출
    if (logCallback) {
        logCallback(level, logMessage);
    }

    // 파일 로깅
    if (!logFilePath.empty()) {
        std::ofstream logFile(logFilePath, std::ios::app);
        if (logFile.is_open()) {
            logFile << logMessage << std::endl;
        }
    }

    // 디버그 출력
#ifdef _DEBUG
    OutputDebugStringA((logMessage + "\n").c_str());
#endif
}

void CameraController::SetLogLevel(LogLevel level)
{
    minLogLevel = level;
}

void CameraController::SetLogFile(const std::string& filepath)
{
    std::lock_guard<std::mutex> lock(logMutex);
    logFilePath = filepath;

    // 로그 파일 헤더 작성
    if (!logFilePath.empty()) {
        std::ofstream logFile(logFilePath, std::ios::app);
        if (logFile.is_open()) {
            logFile << "\n=== Camera Controller Log Started ===" << std::endl;
        }
    }
}

std::string CameraController::GetXiErrorString(XI_RETURN error)
{
    switch (error) {
    case XI_OK: return "Success";
    case XI_INVALID_HANDLE: return "Invalid handle";
    case XI_READREG: return "Register read error";
    case XI_WRITEREG: return "Register write error";
    case XI_FREE_RESOURCES: return "Freeing resources error";
    case XI_FREE_CHANNEL: return "Freeing channel error";
    case XI_FREE_BANDWIDTH: return "Freeing bandwidth error";
    case XI_READBLK: return "Read block error";
    case XI_WRITEBLK: return "Write block error";
    case XI_NO_IMAGE: return "No image";
    case XI_TIMEOUT: return "Timeout";
    case XI_INVALID_ARG: return "Invalid argument";
    case XI_NOT_SUPPORTED: return "Not supported";
    case XI_ISOCH_ATTACH_BUFFERS: return "Attach buffers error";
    case XI_GET_OVERLAPPED_RESULT: return "Overlapped result";
    case XI_MEMORY_ALLOCATION: return "Memory allocation error";
    case XI_DLLCONTEXTISNULL: return "DLL context is NULL";
    case XI_DLLCONTEXTISNONZERO: return "DLL context is non zero";
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
    case XI_TOO_LOW_GAIN: return "Gain too low";
    case XI_INVALID_BPL: return "Invalid bad pixel list";
    case XI_BPL_REALLOC: return "Bad pixel list reallocation error";
    case XI_INVALID_PIXEL_LIST: return "Invalid pixel list";
    case XI_INVALID_FFS: return "Invalid flash file system";
    case XI_INVALID_PROFILE: return "Invalid profile";
    case XI_INVALID_CALIBRATION: return "Invalid calibration";
    case XI_INVALID_BUFFER: return "Invalid buffer";
    case XI_INVALID_DATA: return "Invalid data";
    case XI_TGBUSY: return "Timing generator busy";
    case XI_IO_WRONG: return "Wrong I/O direction";
    case XI_ACQUISITION_ALREADY_UP: return "Acquisition already started";
    case XI_OLD_DRIVER_VERSION: return "Old driver version";
    case XI_GET_LAST_ERROR: return "Get last error";
    case XI_CANT_PROCESS: return "Cannot process";
    case XI_ACQUISITION_STOPED: return "Acquisition stopped";
    case XI_ACQUISITION_STOPED_WERR: return "Acquisition stopped with error";
    case XI_INVALID_INPUT_ICC_PROFILE: return "Invalid input ICC profile";
    case XI_INVALID_OUTPUT_ICC_PROFILE: return "Invalid output ICC profile";
    case XI_DEVICE_NOT_READY: return "Device not ready";
    case XI_SHADING_TOOCONTRAST: return "Shading too contrast";
    case XI_ALREADY_INITIALIZED: return "Already initialized";
    case XI_NOT_ENOUGH_PRIVILEGES: return "Not enough privileges";
    case XI_NOT_COMPATIBLE_DRIVER: return "Not compatible driver";
    case XI_TM_INVALID_RESOURCE: return "Invalid resource in transport manifest";
    case XI_DEVICE_HAS_BEEN_RESETED: return "Device has been reset";
    case XI_NO_DEVICES_FOUND: return "No devices found";
    case XI_RESOURCE_OR_FUNCTION_LOCKED: return "Resource or function locked";
    case XI_BUFFER_SIZE_TOO_SMALL: return "Buffer size too small";
    case XI_COULDNT_INIT_PROCESSOR: return "Could not initialize processor";
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
    case XI_READ_ONLY_PARAM: return "Read only parameter";
    case XI_BANDWIDTH_NOT_SUPPORTED: return "Bandwidth not supported";
    //case XI_INVALID_TM_ENV: return "Invalid transport manifest environment";
    //case XI_INVALID_TM_RANGE: return "Invalid transport manifest range";
    //case XI_INVALID_DEVICE_MON_FRAME: return "Invalid device monitor frame";
    case XI_PROC_OTHER_ERROR: return "Processing error";
    case XI_PROC_PROCESSING_ERROR: return "Processing error";
    case XI_PROC_INPUT_FORMAT_UNSUPPORTED: return "Input format unsupported";
    case XI_PROC_OUTPUT_FORMAT_UNSUPPORTED: return "Output format unsupported";
    case XI_OUT_OF_RANGE: return "Parameter out of range";
    default: return "Unknown error (" + std::to_string(error) + ")";
    }
}

// 추가 구현이 필요한 함수들의 스텁
bool CameraController::SetBinning(int horizontal, int vertical)
{
    std::lock_guard<std::mutex> lock(paramMutex);
    targetParams.binningX = horizontal;
    targetParams.binningY = vertical;
    paramsChanged = true;
    return true;
}

bool CameraController::SetDecimation(int horizontal, int vertical)
{
    std::lock_guard<std::mutex> lock(paramMutex);

    if (horizontal < 1) horizontal = 1;
    if (vertical < 1) vertical = 1;
    if (horizontal > 10) horizontal = 10;
    if (vertical > 10) vertical = 10;

    targetParams.decimationX = horizontal;
    targetParams.decimationY = vertical;
    paramsChanged = true;

    Log(LogLevel::LOG_INFO, "Decimation set to " + std::to_string(horizontal) +
        "x" + std::to_string(vertical));

    return true;
}

bool CameraController::SetTriggerMode(TriggerMode mode)
{
    std::lock_guard<std::mutex> lock(paramMutex);
    targetParams.triggerMode = mode;
    paramsChanged = true;
    return true;
}

bool CameraController::SoftwareTrigger()
{
    if (!xiH || targetParams.triggerMode != TriggerMode::SOFTWARE_TRIGGER) {
        return false;
    }

    XI_RETURN status = xiSetParamInt(xiH, XI_PRM_TRG_SOFTWARE, 1);
    return status == XI_OK;
}

bool CameraController::SetAutoExposure(bool enable, float targetLevel)
{
    std::lock_guard<std::mutex> lock(paramMutex);
    targetParams.autoExposure = enable;
    paramsChanged = true;

    if (enable && xiH) {
        // 목표 레벨 설정 (0.0 ~ 1.0)
        xiSetParamFloat(xiH, XI_PRM_AEAG_LEVEL, targetLevel * 100.0f);
    }

    return true;
}

bool CameraController::SetAutoGain(bool enable)
{
    std::lock_guard<std::mutex> lock(paramMutex);
    targetParams.autoGain = enable;
    paramsChanged = true;
    return true;
}

bool CameraController::SetAutoWhiteBalance(bool enable)
{
    if (!xiH) return false;

    XI_RETURN status = xiSetParamInt(xiH, XI_PRM_AUTO_WB, enable ? XI_ON : XI_OFF);
    return status == XI_OK;
}

bool CameraController::SetImageFormat(int format)
{
    if (!xiH) {
        Log(LogLevel::LOG_ERROR, "Camera not initialized");
        return false;
    }

    XI_RETURN status = xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, format);
    if (status != XI_OK) {
        Log(LogLevel::LOG_ERROR, "Failed to set image format: " + GetXiErrorString(status));
        return false;
    }

    Log(LogLevel::LOG_INFO, "Image format set to " + std::to_string(format));
    return true;
}

std::vector<int> CameraController::GetSupportedFormats() const
{
    std::vector<int> formats;

    if (!xiH) return formats;

    // XIMEA 카메라가 지원하는 일반적인 포맷들
    formats.push_back(XI_MONO8);
    formats.push_back(XI_MONO16);
    formats.push_back(XI_RGB24);
    formats.push_back(XI_RGB32);
    formats.push_back(XI_RAW8);
    formats.push_back(XI_RAW16);

    return formats;
}

std::string CameraController::GetDeviceName() const
{
    if (!xiH) return "";

    char name[256] = { 0 };
    xiGetParamString(xiH, XI_PRM_DEVICE_NAME, name, sizeof(name));
    return std::string(name);
}

std::string CameraController::GetSerialNumber() const
{
    if (!xiH) return "";

    char serial[256] = { 0 };
    xiGetParamString(xiH, XI_PRM_DEVICE_SN, serial, sizeof(serial));
    return std::string(serial);
}

bool CameraController::GetSensorSize(int& width, int& height) const
{
    if (!xiH) return false;

    xiGetParamInt(xiH, XI_PRM_WIDTH XI_PRM_INFO_MAX, &width);
    xiGetParamInt(xiH, XI_PRM_HEIGHT XI_PRM_INFO_MAX, &height);
    return true;
}

bool CameraController::GetTemperature(float& temp) const
{
    if (!xiH) return false;

    XI_RETURN status = xiGetParamFloat(xiH, XI_PRM_TEMP_SELECTOR, &temp);
    return status == XI_OK;
}

CameraParameters CameraController::GetCurrentParameters() const
{
    std::lock_guard<std::mutex> lock(paramMutex);
    return params;
}

bool CameraController::SaveParametersToFile(const std::string& filepath) const
{
    std::ofstream file(filepath);
    if (!file.is_open()) return false;

    std::lock_guard<std::mutex> lock(paramMutex);

    file << "# XIMEA Camera Parameters" << std::endl;
    file << "width=" << params.width << std::endl;
    file << "height=" << params.height << std::endl;
    file << "fps=" << params.fps << std::endl;
    file << "exposure=" << params.exposureUs << std::endl;
    file << "gain=" << params.gain << std::endl;
    file << "binningX=" << params.binningX << std::endl;
    file << "binningY=" << params.binningY << std::endl;
    file << "bufferCount=" << params.bufferCount << std::endl;
    file << "autoExposure=" << (params.autoExposure ? "true" : "false") << std::endl;
    file << "autoGain=" << (params.autoGain ? "true" : "false") << std::endl;

    return true;
}

bool CameraController::LoadParametersFromFile(const std::string& filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open()) return false;

    std::lock_guard<std::mutex> lock(paramMutex);

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        size_t pos = line.find('=');
        if (pos != std::string::npos) {
            std::string key = line.substr(0, pos);
            std::string value = line.substr(pos + 1);

            if (key == "width") targetParams.width = std::stoi(value);
            else if (key == "height") targetParams.height = std::stoi(value);
            else if (key == "fps") targetParams.fps = std::stof(value);
            else if (key == "exposure") targetParams.exposureUs = std::stoi(value);
            else if (key == "gain") targetParams.gain = std::stof(value);
            else if (key == "binningX") targetParams.binningX = std::stoi(value);
            else if (key == "binningY") targetParams.binningY = std::stoi(value);
            else if (key == "bufferCount") targetParams.bufferCount = std::stoi(value);
            else if (key == "autoExposure") targetParams.autoExposure = (value == "true");
            else if (key == "autoGain") targetParams.autoGain = (value == "true");
        }
    }

    paramsChanged = true;
    return true;
}

bool CameraController::CaptureImage(const std::string& filepath, int format)
{
    if (!xiH || !isCapturing) {
        Log(LogLevel::LOG_ERROR, "Camera not capturing");
        return false;
    }

    // 최신 프레임 가져오기
    std::lock_guard<std::mutex> lock(bufferMutex);

    if (readyBuffers.empty()) {
        Log(LogLevel::LOG_ERROR, "No frame available for capture");
        return false;
    }

    int bufferIndex = readyBuffers.back();
    FrameBuffer& frameBuffer = frameBuffers[bufferIndex];

    // 파일 확장자 결정
    std::string ext = filepath.substr(filepath.find_last_of(".") + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    bool success = false;

    if (ext == "raw") {
        // RAW 포맷으로 저장
        std::ofstream file(filepath, std::ios::binary);
        if (file.is_open()) {
            // 헤더 정보 저장
            file.write(reinterpret_cast<const char*>(&frameBuffer.info.width), sizeof(int));
            file.write(reinterpret_cast<const char*>(&frameBuffer.info.height), sizeof(int));
            file.write(reinterpret_cast<const char*>(&frameBuffer.info.format), sizeof(int));

            // 이미지 데이터 저장
            size_t dataSize = frameBuffer.info.width * frameBuffer.info.height;
            file.write(reinterpret_cast<const char*>(frameBuffer.data.data()), dataSize);

            success = true;
        }
    }
    else if (ext == "bmp") {
        // BMP 포맷으로 저장
        success = SaveBMP(filepath, frameBuffer.data.data(),
            frameBuffer.info.width, frameBuffer.info.height);
    }
    else if (ext == "png") {
        // PNG 저장은 외부 라이브러리가 필요하므로, 일단 BMP로 저장
        std::string bmpPath = filepath.substr(0, filepath.find_last_of(".")) + ".bmp";
        success = SaveBMP(bmpPath, frameBuffer.data.data(),
            frameBuffer.info.width, frameBuffer.info.height);

        if (success) {
            Log(LogLevel::LOG_WARNING, "PNG format not supported, saved as BMP: " + bmpPath);
        }
    }
    else {
        Log(LogLevel::LOG_ERROR, "Unsupported file format: " + ext);
        return false;
    }

    if (success) {
        Log(LogLevel::LOG_INFO, "Image saved: " + filepath);
    }
    else {
        Log(LogLevel::LOG_ERROR, "Failed to save image: " + filepath);
    }

    return success;
}

bool CameraController::SaveBMP(const std::string& filepath, const unsigned char* data,
    int width, int height)
{
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) return false;

    // BMP 파일 헤더
    struct BMPFileHeader {
        uint16_t type = 0x4D42;  // "BM"
        uint32_t size;
        uint16_t reserved1 = 0;
        uint16_t reserved2 = 0;
        uint32_t offset = 54 + 256 * 4;  // 헤더 + 팔레트
    } fileHeader;

    // BMP 정보 헤더
    struct BMPInfoHeader {
        uint32_t size = 40;
        int32_t width;
        int32_t height;
        uint16_t planes = 1;
        uint16_t bitCount = 8;
        uint32_t compression = 0;
        uint32_t sizeImage;
        int32_t xPelsPerMeter = 0;
        int32_t yPelsPerMeter = 0;
        uint32_t clrUsed = 256;
        uint32_t clrImportant = 256;
    } infoHeader;

    // 행은 4바이트 단위로 정렬
    int rowSize = ((width + 3) / 4) * 4;

    infoHeader.width = width;
    infoHeader.height = height;
    infoHeader.sizeImage = rowSize * height;

    fileHeader.size = fileHeader.offset + infoHeader.sizeImage;

    // 헤더 쓰기
    file.write(reinterpret_cast<const char*>(&fileHeader.type), 2);
    file.write(reinterpret_cast<const char*>(&fileHeader.size), 4);
    file.write(reinterpret_cast<const char*>(&fileHeader.reserved1), 2);
    file.write(reinterpret_cast<const char*>(&fileHeader.reserved2), 2);
    file.write(reinterpret_cast<const char*>(&fileHeader.offset), 4);

    file.write(reinterpret_cast<const char*>(&infoHeader), sizeof(infoHeader));

    // 그레이스케일 팔레트 쓰기
    for (int i = 0; i < 256; i++) {
        uint8_t color[4] = { (uint8_t)i, (uint8_t)i, (uint8_t)i, 0 };
        file.write(reinterpret_cast<const char*>(color), 4);
    }

    // 이미지 데이터 쓰기 (BMP는 bottom-up)
    std::vector<uint8_t> rowBuffer(rowSize, 0);
    for (int y = height - 1; y >= 0; y--) {
        memcpy(rowBuffer.data(), data + y * width, width);
        file.write(reinterpret_cast<const char*>(rowBuffer.data()), rowSize);
    }

    return true;
}