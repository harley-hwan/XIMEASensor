#include "pch.h"
#include "ContinuousCaptureManager.h"
#include "ImageSaver.h"
#include "Logger.h"
#include "GolfBallDetector.h"
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <fstream>

ContinuousCaptureManager::ContinuousCaptureManager()
    : m_state(ContinuousCaptureState::IDLE),
    m_isCapturing(false),
    m_frameCount(0),
    m_savedCount(0),
    m_droppedCount(0),
    m_processingCount(0),
    m_saveThreadRunning(false),
    m_actualDuration(0.0) {

    // Init buffer pool
    for (int i = 0; i < 10; i++) {
        m_bufferPool.push(std::vector<unsigned char>());
    }

    // Initialize golf ball detector
    m_golfBallDetector = std::make_unique<GolfBallDetector>();

    // 골프공 검출 파라미터 설정
    GolfBallDetector::DetectionParams params;
    params.minRadius = 10;      // 최소 반지름을 좀 더 크게
    params.maxRadius = 40;      // 최대 반지름 조정
    params.dp = 1.2;
    params.minDist = 30.0;
    params.param1 = 100.0;
    params.param2 = 25.0;       // 민감도 높임
    params.brightnessThreshold = 180;  // 밝기 임계값 조정
    params.minCircularity = 0.7f;      // 원형도 기준 완화
    params.useColorFilter = true;
    params.useCircularityCheck = true;
    params.detectMultiple = true;       // 여러 개 검출 허용

    m_golfBallDetector->SetParameters(params);

    LOG_INFO("Golf ball detector initialized with parameters");
}

ContinuousCaptureManager::~ContinuousCaptureManager() {
    StopCapture();

    if (m_saveThread.joinable()) {
        m_saveThreadRunning = false;
        m_queueCV.notify_all();
        m_saveThread.join();
    }
}

void ContinuousCaptureManager::SetConfig(const ContinuousCaptureConfig& config) {
    if (m_isCapturing.load()) {
        LOG_WARNING("Cannot change config while capturing");
        return;
    }
    m_config = config;

    LOG_INFO("ContinuousCaptureConfig set: duration=" + std::to_string(config.durationSeconds) +
        "s, format=" + std::to_string(config.imageFormat) +
        ", enableGolfBallDetection=" + (config.enableGolfBallDetection ? "true" : "false") +
        ", saveOriginalImages=" + (config.saveOriginalImages ? "true" : "false") +
        ", saveDetectionImages=" + (config.saveDetectionImages ? "true" : "false"));
}

bool ContinuousCaptureManager::StartCapture() {
    ContinuousCaptureState currentState = m_state.load();
    if (currentState != ContinuousCaptureState::IDLE &&
        currentState != ContinuousCaptureState::COMPLETED) {
        LOG_ERROR("Cannot start capture - invalid state: " + std::to_string(static_cast<int>(currentState)));
        return false;
    }

    m_state = ContinuousCaptureState::CAPTURING;

    LOG_INFO("Starting continuous capture for " + std::to_string(m_config.durationSeconds) + " seconds");
    LOG_INFO("Golf ball detection: " + std::string(m_config.enableGolfBallDetection ? "ENABLED" : "DISABLED"));

    m_frameCount = 0;
    m_savedCount = 0;
    m_droppedCount = 0;
    m_processingCount = 0;
    m_actualDuration = 0.0;
    m_isCapturing = true;
    m_startTime = std::chrono::steady_clock::now();

    // Reset detection results
    m_detectionResult = ContinuousCaptureDetectionResult();

    if (!CreateCaptureFolder()) {
        m_state = ContinuousCaptureState::kERROR;
        m_isCapturing = false;
        LOG_ERROR("Failed to create capture folder");
        return false;
    }

    if (m_config.useAsyncSave && !m_saveThreadRunning) {
        m_saveThreadRunning = true;
        m_saveThread = std::thread(&ContinuousCaptureManager::SaveThreadWorker, this);
    }

    if (m_progressCallback) {
        m_progressCallback(0, 0.0, ContinuousCaptureState::CAPTURING);
    }

    return true;
}

void ContinuousCaptureManager::StopCapture() {
    if (!m_isCapturing.load()) {
        return;
    }

    LOG_INFO("Stopping continuous capture");

    m_isCapturing = false;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - m_startTime);
    m_actualDuration = elapsed.count() / 1000.0;

    m_state = ContinuousCaptureState::STOPPING;

    // Wait for all async save operations to complete
    if (m_config.useAsyncSave && m_saveThreadRunning) {
        if (!WaitForSaveCompletion(30)) {  // 30초 타임아웃
            LOG_ERROR("Timeout waiting for save operations to complete");
            int pendingFrames = m_frameCount.load() - m_savedCount.load() - m_droppedCount.load();
            if (pendingFrames > 0) {
                m_droppedCount += pendingFrames;
                LOG_WARNING("Dropped " + std::to_string(pendingFrames) + " frames due to timeout");
            }
        }
    }

    // 모든 저장 작업이 완료된 후에 메타데이터 저장
    if (m_config.createMetadata) {
        SaveMetadata();

        // Save detection metadata if golf ball detection was enabled
        if (m_config.enableGolfBallDetection) {
            SaveDetectionMetadata();
        }
    }

    m_state = ContinuousCaptureState::COMPLETED;

    if (m_progressCallback) {
        m_progressCallback(m_frameCount.load(), m_actualDuration, ContinuousCaptureState::COMPLETED);
    }

    LOG_INFO("Continuous capture completed. Frames: " + std::to_string(m_savedCount.load()) +
        "/" + std::to_string(m_frameCount.load()) +
        " (dropped: " + std::to_string(m_droppedCount.load()) + ")");

    if (m_config.enableGolfBallDetection) {
        LOG_INFO("Golf ball detection results: " +
            std::to_string(m_detectionResult.framesWithGolfBall) + " frames with golf balls, " +
            std::to_string(m_detectionResult.totalBallsDetected) + " total balls detected");
    }
}

void ContinuousCaptureManager::ProcessFrame(const unsigned char* data, int width, int height) {
    if (!m_isCapturing.load()) {
        return;
    }

    double elapsed = GetElapsedTime();
    if (elapsed >= m_config.durationSeconds) {
        StopCapture();
        return;
    }

    m_frameCount++;

    if (m_config.useAsyncSave) {
        SaveFrameAsync(data, width, height);
    }
    else {
        // 동기 저장 모드에서도 골프공 검출을 수행
        std::string saveFolder = m_captureFolder;
        if (m_config.enableGolfBallDetection && m_config.saveOriginalImages) {
            saveFolder = m_originalFolder;
        }

        std::string filename = saveFolder + "/frame_" +
            std::to_string(m_frameCount.load() - 1) +
            (m_config.imageFormat == 0 ? ".png" : ".jpg");

        if (ImageSaver::SaveGrayscaleImage(data, width, height, filename,
            static_cast<ImageFormat>(m_config.imageFormat),
            m_config.jpgQuality)) {
            m_savedCount++;

            // 골프공 검출 수행
            if (m_config.enableGolfBallDetection) {
                SaveItem item;
                item.data.resize(width * height);
                memcpy(item.data.data(), data, width * height);
                item.width = width;
                item.height = height;
                item.frameIndex = m_frameCount.load() - 1;
                item.filename = filename;

                ProcessGolfBallDetection(item);
            }
        }
        else {
            m_droppedCount++;
            LOG_ERROR("Failed to save frame: " + filename);
        }
    }

    // callback every 10 frames
    if (m_progressCallback && m_frameCount % 10 == 0) {
        m_progressCallback(m_frameCount.load(), elapsed, ContinuousCaptureState::CAPTURING);
    }
}

bool ContinuousCaptureManager::CreateCaptureFolder() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    struct tm localTime;
    localtime_s(&localTime, &time_t);

    std::stringstream ss;
    ss << m_config.baseFolder << "/Capture_"
        << std::setfill('0')
        << std::setw(4) << (localTime.tm_year + 1900)
        << std::setw(2) << (localTime.tm_mon + 1)
        << std::setw(2) << localTime.tm_mday << "_"
        << std::setw(2) << localTime.tm_hour
        << std::setw(2) << localTime.tm_min
        << std::setw(2) << localTime.tm_sec;

    m_captureFolder = ss.str();

    try {
        std::filesystem::create_directories(m_captureFolder);
        LOG_INFO("Created capture folder: " + m_captureFolder);

        // Create subfolders for golf ball detection - 항상 생성
        if (!m_config.enableGolfBallDetection) {
            // Original images folder
            m_originalFolder = m_captureFolder + "/original";
            std::filesystem::create_directories(m_originalFolder);
            LOG_INFO("Created original images folder: " + m_originalFolder);

            // Detection images folder
            m_detectionFolder = m_captureFolder + "/detection";
            std::filesystem::create_directories(m_detectionFolder);
            LOG_INFO("Created detection images folder: " + m_detectionFolder);

            m_detectionResult.detectionFolder = m_detectionFolder;
        }

        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Failed to create capture folder: " + std::string(e.what()));
        return false;
    }
}

void ContinuousCaptureManager::SaveFrameAsync(const unsigned char* data, int width, int height) {
    auto buffer = GetBufferFromPool(width * height);
    if (buffer.empty()) {
        buffer.resize(width * height);
    }

    memcpy(buffer.data(), data, width * height);

    int frameIndex = m_frameCount.load() - 1;
    std::stringstream ss;

    // Determine save path based on configuration
    std::string saveFolder = m_captureFolder;
    if (m_config.enableGolfBallDetection && m_config.saveOriginalImages) {
        saveFolder = m_originalFolder;
    }

    ss << saveFolder << "/frame_"
        << std::setfill('0') << std::setw(5) << frameIndex
        << (m_config.imageFormat == 0 ? ".png" : ".jpg");

    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        m_saveQueue.push(SaveItem{ std::move(buffer), ss.str(), width, height, frameIndex });

        if (m_saveQueue.size() > 100) {
            auto droppedItem = std::move(m_saveQueue.front());
            m_saveQueue.pop();
            m_droppedCount++;
            ReturnBufferToPool(std::move(droppedItem.data));
            LOG_WARNING("Dropped frame due to queue overflow: " + droppedItem.filename);
        }
    }
    m_queueCV.notify_one();
}

void ContinuousCaptureManager::SaveThreadWorker() {
    LOG_INFO("Save thread started");

    while (m_saveThreadRunning.load() || !m_saveQueue.empty() || m_processingCount > 0) {
        std::unique_lock<std::mutex> lock(m_queueMutex);

        m_queueCV.wait(lock, [this] {
            return !m_saveQueue.empty() || !m_saveThreadRunning.load();
            });

        while (!m_saveQueue.empty()) {
            auto item = std::move(m_saveQueue.front());
            m_saveQueue.pop();
            m_processingCount++;
            lock.unlock();

            bool saveSuccess = ImageSaver::SaveGrayscaleImage(
                item.data.data(), item.width, item.height,
                item.filename,
                static_cast<ImageFormat>(m_config.imageFormat),
                m_config.jpgQuality);

            if (saveSuccess) {
                m_savedCount++;
                LOG_DEBUG("Saved frame: " + item.filename);

                // Process golf ball detection if enabled
                if (m_config.enableGolfBallDetection) {
                    LOG_DEBUG("Processing golf ball detection for frame " + std::to_string(item.frameIndex));
                    ProcessGolfBallDetection(item);
                }
            }
            else {
                m_droppedCount++;
                LOG_ERROR("Failed to save frame: " + item.filename);
            }

            ReturnBufferToPool(std::move(item.data));

            m_processingCount--;
            m_completionCV.notify_all();

            lock.lock();
        }
    }

    LOG_INFO("Save thread ended");
}

void ContinuousCaptureManager::ProcessGolfBallDetection(const SaveItem& item) {
    if (!m_golfBallDetector) {
        LOG_ERROR("Golf ball detector not initialized");
        return;
    }

    LOG_DEBUG("Starting golf ball detection for frame " + std::to_string(item.frameIndex));

    // Detect golf ball
    auto result = m_golfBallDetector->DetectGolfBall(
        item.data.data(), item.width, item.height, item.frameIndex);

    // Update statistics
    {
        std::lock_guard<std::mutex> lock(m_detectionMutex);

        if (result.found && !result.balls.empty()) {
            m_detectionResult.framesWithGolfBall++;
            m_detectionResult.totalBallsDetected += static_cast<int>(result.balls.size());

            // Update average confidence
            float sumConfidence = 0.0f;
            for (const auto& ball : result.balls) {
                sumConfidence += ball.confidence;
            }

            // Running average calculation
            float currentAvg = m_detectionResult.averageConfidence;
            int currentCount = m_detectionResult.totalBallsDetected - static_cast<int>(result.balls.size());
            if (m_detectionResult.totalBallsDetected > 0) {
                m_detectionResult.averageConfidence =
                    (currentAvg * currentCount + sumConfidence) / m_detectionResult.totalBallsDetected;
            }

            LOG_INFO("Frame " + std::to_string(item.frameIndex) +
                ": Detected " + std::to_string(result.balls.size()) + " golf ball(s)");
        }
    }

    // Save detection image if enabled
    if (m_config.saveDetectionImages) {
        std::stringstream detectionPath;
        detectionPath << m_detectionFolder << "/frame_"
            << std::setfill('0') << std::setw(5) << item.frameIndex;

        if (result.found) {
            detectionPath << "_detected";

            // 검출된 공의 좌표 정보를 파일명에 추가
            if (!result.balls.empty()) {
                const auto& firstBall = result.balls[0];
                detectionPath << "_x" << static_cast<int>(firstBall.center.x)
                    << "_y" << static_cast<int>(firstBall.center.y);
            }
        }

        detectionPath << (m_config.imageFormat == 0 ? ".png" : ".jpg");

        // 검출 결과가 그려진 이미지 저장
        if (!m_golfBallDetector->SaveDetectionImage(
            item.data.data(), item.width, item.height,
            result, detectionPath.str())) {
            LOG_ERROR("Failed to save detection image: " + detectionPath.str());
        }
        else {
            LOG_DEBUG("Saved detection image: " + detectionPath.str());
        }
    }
}

void ContinuousCaptureManager::SaveMetadata() {
    std::string metaFile = m_captureFolder + "/metadata.txt";
    std::ofstream file(metaFile);

    if (!file.is_open()) {
        LOG_ERROR("Failed to create metadata file");
        return;
    }

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    struct tm localTime;
    localtime_s(&localTime, &time_t);

    file << "Continuous Capture Metadata\n";
    file << "===========================\n\n";

    file << "Capture Date: "
        << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S") << "\n";

    file << "Configuration:\n";
    file << "  Duration: " << m_config.durationSeconds << " seconds\n";
    file << "  Format: " << (m_config.imageFormat == 0 ? "PNG" : "JPG") << "\n";
    if (m_config.imageFormat == 1) {
        file << "  JPG Quality: " << m_config.jpgQuality << "\n";
    }
    file << "  Async Save: " << (m_config.useAsyncSave ? "Yes" : "No") << "\n";
    file << "  Golf Ball Detection: " << (m_config.enableGolfBallDetection ? "Yes" : "No") << "\n\n";

    file << "Results:\n";
    file << "  Total Frames: " << m_frameCount.load() << "\n";
    file << "  Saved Frames: " << m_savedCount.load() << "\n";
    file << "  Dropped Frames: " << m_droppedCount.load() << "\n";
    file << "  Actual Duration: " << std::fixed << std::setprecision(3)
        << m_actualDuration << " seconds\n";

    if (m_actualDuration > 0) {
        file << "  Average FPS: " << std::fixed << std::setprecision(1)
            << (m_frameCount.load() / m_actualDuration) << "\n";
    }
    else {
        file << "  Average FPS: N/A\n";
    }

    if (m_frameCount > 0) {
        float successRate = (float)m_savedCount.load() / m_frameCount.load() * 100.0f;
        file << "  Save Success Rate: " << std::fixed << std::setprecision(1)
            << successRate << "%\n";
    }

    file.close();
    LOG_INFO("Metadata saved to: " + metaFile);
}

void ContinuousCaptureManager::SaveDetectionMetadata() {
    std::string metaFile = m_captureFolder + "/detection_results.txt";
    std::ofstream file(metaFile);

    if (!file.is_open()) {
        LOG_ERROR("Failed to create detection metadata file");
        return;
    }

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    struct tm localTime;
    localtime_s(&localTime, &time_t);

    file << "Golf Ball Detection Results\n";
    file << "===========================\n\n";

    file << "Detection Date: "
        << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S") << "\n\n";

    file << "Summary:\n";
    file << "  Total Frames Analyzed: " << m_frameCount.load() << "\n";
    file << "  Frames with Golf Ball: " << m_detectionResult.framesWithGolfBall << "\n";
    file << "  Total Golf Balls Detected: " << m_detectionResult.totalBallsDetected << "\n";
    file << "  Detection Rate: " << std::fixed << std::setprecision(1)
        << (m_frameCount > 0 ?
            (float)m_detectionResult.framesWithGolfBall / m_frameCount * 100.0f : 0.0f)
        << "%\n";
    file << "  Average Confidence: " << std::fixed << std::setprecision(3)
        << m_detectionResult.averageConfidence << "\n\n";

    file << "Detection Parameters:\n";
    auto params = m_golfBallDetector->GetParameters();
    file << "  Min Radius: " << params.minRadius << " pixels\n";
    file << "  Max Radius: " << params.maxRadius << " pixels\n";
    file << "  Brightness Threshold: " << params.brightnessThreshold << "\n";
    file << "  Min Circularity: " << params.minCircularity << "\n";
    file << "  Color Filter: " << (params.useColorFilter ? "Enabled" : "Disabled") << "\n";
    file << "  Circularity Check: " << (params.useCircularityCheck ? "Enabled" : "Disabled") << "\n";
    file << "  Multiple Detection: " << (params.detectMultiple ? "Enabled" : "Disabled") << "\n\n";

    file << "Folders:\n";
    file << "  Capture Folder: " << m_captureFolder << "\n";
    if (!m_originalFolder.empty()) {
        file << "  Original Images: " << m_originalFolder << "\n";
    }
    if (!m_detectionFolder.empty()) {
        file << "  Detection Images: " << m_detectionFolder << "\n";
    }

    file.close();
    LOG_INFO("Detection metadata saved to: " + metaFile);
}

bool ContinuousCaptureManager::WaitForSaveCompletion(int timeoutSeconds) {
    auto startWait = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(timeoutSeconds);

    std::unique_lock<std::mutex> lock(m_queueMutex);

    bool completed = m_completionCV.wait_for(lock, timeout, [this, &startWait, &timeout] {
        auto elapsed = std::chrono::steady_clock::now() - startWait;
        if (elapsed >= timeout) {
            return false;
        }

        bool queueEmpty = m_saveQueue.empty();
        bool noProcessing = (m_processingCount == 0);
        bool allSaved = (m_savedCount + m_droppedCount) >= m_frameCount;

        static auto lastLog = std::chrono::steady_clock::now();
        if (std::chrono::steady_clock::now() - lastLog > std::chrono::seconds(1)) {
            LOG_DEBUG("Waiting for save completion - Queue: " + std::to_string(m_saveQueue.size()) +
                ", Processing: " + std::to_string(m_processingCount.load()) +
                ", Saved: " + std::to_string(m_savedCount.load()) +
                ", Dropped: " + std::to_string(m_droppedCount.load()) +
                ", Total: " + std::to_string(m_frameCount.load()));
            lastLog = std::chrono::steady_clock::now();
        }

        return queueEmpty && noProcessing && allSaved;
        });

    if (!completed) {
        LOG_WARNING("Save completion wait timed out after " + std::to_string(timeoutSeconds) + " seconds");
    }

    return completed;
}

ContinuousCaptureResult ContinuousCaptureManager::GetResult() const {
    ContinuousCaptureResult result;
    result.success = (m_state == ContinuousCaptureState::COMPLETED) &&
        (m_savedCount == m_frameCount);
    result.totalFrames = m_frameCount.load();
    result.savedFrames = m_savedCount.load();
    result.droppedFrames = m_droppedCount.load();
    result.actualDuration = m_actualDuration;
    result.folderPath = m_captureFolder;

    if (m_state == ContinuousCaptureState::kERROR) {
        result.errorMessage = "Capture failed";
    }
    else if (result.droppedFrames > 0) {
        result.errorMessage = "Some frames were dropped";
    }

    return result;
}

double ContinuousCaptureManager::GetElapsedTime() const {
    if (m_state == ContinuousCaptureState::CAPTURING || m_isCapturing.load()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_startTime);
        return elapsed.count() / 1000.0;
    }

    return m_actualDuration;
}

void ContinuousCaptureManager::SetProgressCallback(ContinuousCaptureProgressCallback callback) {
    std::lock_guard<std::mutex> lock(m_callbackMutex);
    m_progressCallback = callback;
}

std::vector<unsigned char> ContinuousCaptureManager::GetBufferFromPool(size_t size) {
    std::lock_guard<std::mutex> lock(m_poolMutex);
    if (!m_bufferPool.empty()) {
        auto buffer = std::move(m_bufferPool.front());
        m_bufferPool.pop();
        if (buffer.size() != size) {
            buffer.resize(size);
        }
        return buffer;
    }
    return std::vector<unsigned char>(size);
}

void ContinuousCaptureManager::ReturnBufferToPool(std::vector<unsigned char>&& buffer) {
    std::lock_guard<std::mutex> lock(m_poolMutex);
    if (m_bufferPool.size() < 20) {
        m_bufferPool.push(std::move(buffer));
    }
}

void ContinuousCaptureManager::Reset() {
    if (m_isCapturing.load()) {
        StopCapture();
    }

    if (m_saveThreadRunning && m_saveThread.joinable()) {
        m_saveThreadRunning = false;
        m_queueCV.notify_all();
        m_saveThread.join();
    }

    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        while (!m_saveQueue.empty()) {
            m_saveQueue.pop();
        }
    }

    m_state = ContinuousCaptureState::IDLE;
    m_frameCount = 0;
    m_savedCount = 0;
    m_droppedCount = 0;
    m_processingCount = 0;
    m_actualDuration = 0.0;
    m_captureFolder.clear();
    m_originalFolder.clear();
    m_detectionFolder.clear();
    m_isCapturing = false;
    m_saveThreadRunning = false;

    // Reset detection results
    m_detectionResult = ContinuousCaptureDetectionResult();

    LOG_INFO("ContinuousCaptureManager reset completed");
}

ContinuousCaptureDetectionResult ContinuousCaptureManager::GetDetectionResult() const {
    std::lock_guard<std::mutex> lock(m_detectionMutex);
    return m_detectionResult;
}