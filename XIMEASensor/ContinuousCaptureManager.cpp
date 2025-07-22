#include "pch.h"
#include "ContinuousCaptureManager.h"
#include "ImageSaver.h"
#include "Logger.h"
#include "BallDetector.h"
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <numeric>      // std::accumulate

ContinuousCaptureManager::ContinuousCaptureManager()
    : m_state(ContinuousCaptureState::IDLE),
    m_isCapturing(false),
    m_frameCount(0),
    m_savedCount(0),
    m_droppedCount(0),
    m_processingCount(0),
    m_ballDetectionPendingCount(0),
    m_ballDetectionCompletedCount(0),
    m_saveThreadRunning(false),
    m_actualDuration(0.0) {

    // Init buffer pool
    for (int i = 0; i < 10; i++) {
        m_bufferPool.push(std::vector<unsigned char>());
    }

    // Initialize ball detector - BallDetector가 내부적으로 기본 파라미터를 설정함
    m_ballDetector = std::make_unique<BallDetector>();

    LOG_INFO("ContinuousCaptureManager initialized with BallDetector");
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

    LOG_INFO("========== Setting new configuration ==========");
    LOG_INFO("Previous config:");
    LOG_INFO("  Ball Detection: " + std::string(m_config.enableBallDetection ? "ENABLED" : "DISABLED"));
    LOG_INFO("  Save Original: " + std::string(m_config.saveOriginalImages ? "YES" : "NO"));
    LOG_INFO("  Save Detection: " + std::string(m_config.saveDetectionImages ? "YES" : "NO"));

    m_config = config;

    LOG_INFO("New config:");
    LOG_INFO("  Duration: " + std::to_string(config.durationSeconds) + "s");
    LOG_INFO("  Format: " + std::to_string(config.imageFormat) + " (0=PNG, 1=JPG)");
    LOG_INFO("  JPG Quality: " + std::to_string(config.jpgQuality));
    LOG_INFO("  Async Save: " + std::string(config.useAsyncSave ? "YES" : "NO"));
    LOG_INFO("  Ball Detection: " + std::string(config.enableBallDetection ? "ENABLED" : "DISABLED"));
    LOG_INFO("  Save Original: " + std::string(config.saveOriginalImages ? "YES" : "NO"));
    LOG_INFO("  Save Detection: " + std::string(config.saveDetectionImages ? "YES" : "NO"));
    LOG_INFO("==============================================");
}

bool ContinuousCaptureManager::StartCapture() {
    ContinuousCaptureState currentState = m_state.load();
    if (currentState != ContinuousCaptureState::IDLE &&
        currentState != ContinuousCaptureState::COMPLETED) {
        LOG_ERROR("Cannot start capture - invalid state: " + std::to_string(static_cast<int>(currentState)));
        return false;
    }

    m_state = ContinuousCaptureState::CAPTURING;

    LOG_INFO("========== Starting continuous capture ==========");
    LOG_INFO("Duration: " + std::to_string(m_config.durationSeconds) + " seconds");
    LOG_INFO("Format: " + std::to_string(m_config.imageFormat) + " (0=PNG, 1=JPG)");
    LOG_INFO("JPG Quality: " + std::to_string(m_config.jpgQuality));
    LOG_INFO("Async Save: " + std::string(m_config.useAsyncSave ? "YES" : "NO"));
    LOG_INFO("Create Metadata: " + std::string(m_config.createMetadata ? "YES" : "NO"));
    LOG_INFO("Base Folder: " + m_config.baseFolder);
    LOG_INFO("Ball Detection: " + std::string(m_config.enableBallDetection ? "ENABLED" : "DISABLED"));
    LOG_INFO("Save Original Images: " + std::string(m_config.saveOriginalImages ? "YES" : "NO"));
    LOG_INFO("Save Detection Images: " + std::string(m_config.saveDetectionImages ? "YES" : "NO"));
    LOG_INFO("===============================================");

    m_frameCount = 0;
    m_savedCount = 0;
    m_droppedCount = 0;
    m_processingCount = 0;
    m_actualDuration = 0.0;
    m_isCapturing = true;
    m_startTime = std::chrono::steady_clock::now();

    // 2025-07-21
    m_ballDetectionPendingCount = 0;
    m_ballDetectionCompletedCount = 0;

    m_detectionResult = ContinuousCaptureDetectionResult();
    m_sessionPerformance.Reset();

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


void ContinuousCaptureManager::SaveSessionPerformanceReport() {
    if (m_sessionPerformance.totalFramesProcessed == 0) {
        LOG_WARNING("No frames processed for performance report");
        return;
    }

    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    struct tm localTime;
    localtime_s(&localTime, &time_t_now);

    // Calculate statistics
    m_sessionPerformance.avgFrameTime_ms =
        m_sessionPerformance.totalProcessingTime_ms / m_sessionPerformance.totalFramesProcessed;

    // Generate comprehensive performance report
    std::stringstream report;
    report << std::fixed << std::setprecision(2);
    report << "========== Continuous Capture Performance Report ==========\n";
    report << "Generated: " << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S") << "\n";
    report << "Capture Duration: " << m_actualDuration << " seconds\n";
    report << "Total Frames Processed: " << m_sessionPerformance.totalFramesProcessed << "\n";
    report << "Frames with Ball Detected: " << m_sessionPerformance.framesWithBallDetected << "\n";
    report << "Detection Rate: " << (m_sessionPerformance.framesWithBallDetected * 100.0 / m_sessionPerformance.totalFramesProcessed) << "%\n\n";

    report << "Performance Metrics:\n";
    report << "  Total Processing Time: " << m_sessionPerformance.totalProcessingTime_ms << " ms\n";
    report << "  Average Frame Processing Time: " << m_sessionPerformance.avgFrameTime_ms << " ms\n";
    report << "  Min Frame Processing Time: " << m_sessionPerformance.minFrameTime_ms << " ms\n";
    report << "  Max Frame Processing Time: " << m_sessionPerformance.maxFrameTime_ms << " ms\n";
    report << "  Standard Deviation: ";

    // Calculate standard deviation using frame timings
    if (m_sessionPerformance.frameTimings.size() > 1) {
        double variance = 0.0;
        for (const auto& timing : m_sessionPerformance.frameTimings) {
            variance += std::pow(timing.totalFrameProcessingTime_ms - m_sessionPerformance.avgFrameTime_ms, 2);
        }
        variance /= (m_sessionPerformance.frameTimings.size() - 1);
        double stdDev = std::sqrt(variance);
        report << stdDev << " ms\n";
    }
    else {
        report << "N/A\n";
    }

    report << "  Real-time Factor: " << (1000.0 / m_sessionPerformance.avgFrameTime_ms) << " FPS capability\n\n";

    // Calculate percentiles
    if (!m_sessionPerformance.frameTimings.empty()) {
        std::vector<double> sortedTimes;
        for (const auto& timing : m_sessionPerformance.frameTimings) {
            sortedTimes.push_back(timing.totalFrameProcessingTime_ms);
        }
        std::sort(sortedTimes.begin(), sortedTimes.end());

        auto getPercentile = [&sortedTimes](double percentile) -> double {
            size_t idx = static_cast<size_t>((sortedTimes.size() - 1) * percentile);
            return sortedTimes[idx];
            };

        report << "Processing Time Percentiles:\n";
        report << "  25th percentile: " << getPercentile(0.25) << " ms\n";
        report << "  50th percentile (median): " << getPercentile(0.50) << " ms\n";
        report << "  75th percentile: " << getPercentile(0.75) << " ms\n";
        report << "  90th percentile: " << getPercentile(0.90) << " ms\n";
        report << "  95th percentile: " << getPercentile(0.95) << " ms\n";
        report << "  99th percentile: " << getPercentile(0.99) << " ms\n\n";
    }

    // Processing breakdown with corrected calculations
    if (!m_sessionPerformance.frameTimings.empty()) {
        // Calculate average times for each component
        double avgImageLoadTime = 0.0;
        double avgPreprocTime = 0.0;
        double avgCoreDetectTime = 0.0;
        double avgSaveImageTime = 0.0;
        double avgOtherTime = 0.0;
        size_t frameCount = m_sessionPerformance.frameTimings.size();
        for (const auto& ft : m_sessionPerformance.frameTimings) {
            avgImageLoadTime += ft.imageLoadTime_ms;
            avgPreprocTime += ft.preprocessingTime_ms;
            avgCoreDetectTime += ft.detectionTime_ms;      // already core-only now
            avgSaveImageTime += ft.saveDetectionImageTime_ms;
            avgOtherTime += ft.otherOperationsTime_ms;
        }
        if (frameCount > 0) {
            avgImageLoadTime /= frameCount;
            avgPreprocTime /= frameCount;
            avgCoreDetectTime /= frameCount;
            avgSaveImageTime /= frameCount;
            avgOtherTime /= frameCount;
        }
        report << "Average Processing Breakdown:\n";
        report << "  Image Load: " << avgImageLoadTime << " ms ("
            << (avgImageLoadTime / m_sessionPerformance.avgFrameTime_ms * 100.0) << "%)\n";
        report << "  Preprocessing (BallDetector): " << avgPreprocTime << " ms ("
            << (avgPreprocTime / m_sessionPerformance.avgFrameTime_ms * 100.0) << "%)\n";
        report << "  Ball Detection (BallDetector core): " << avgCoreDetectTime << " ms ("
            << (avgCoreDetectTime / m_sessionPerformance.avgFrameTime_ms * 100.0) << "%)\n";
        report << "  Save Detection Image: " << avgSaveImageTime << " ms ("
            << (avgSaveImageTime / m_sessionPerformance.avgFrameTime_ms * 100.0) << "%)\n";
        report << "  Other Operations: " << avgOtherTime << " ms ("
            << (avgOtherTime / m_sessionPerformance.avgFrameTime_ms * 100.0) << "%)\n\n";
        // Verify that breakdown sums approximately to 100% of avg frame time
        double sumCheck = avgImageLoadTime + avgPreprocTime + avgCoreDetectTime + avgSaveImageTime + avgOtherTime;
        report << "  Timing Verification: Sum = " << sumCheck << " ms, Average = "
            << m_sessionPerformance.avgFrameTime_ms << " ms\n";
        double discrepancy = std::abs(sumCheck - m_sessionPerformance.avgFrameTime_ms);
        if (discrepancy > 0.1) {
            report << "  WARNING: Timing discrepancy detected (" << discrepancy << " ms)\n";
        }
        report << "\nNote: \"Preprocessing\" and \"Ball Detection (core)\" are measured inside BallDetector.\n"
            << "      \"Ball Detection (core)\" represents the detection algorithm runtime after preprocessing.\n"
            << "      All percentages are of the total frame processing time.\n\n";
    }

    // Performance analysis
    report << "Performance Analysis:\n";

    // Analyze based on target frame rate
    const double targetFPS = m_config.durationSeconds > 0 ?
        m_sessionPerformance.totalFramesProcessed / m_config.durationSeconds : 60.0;
    const double targetFrameTime = 1000.0 / targetFPS;

    if (m_sessionPerformance.avgFrameTime_ms < targetFrameTime) {
        report << "  ✓ Excellent: Processing faster than capture rate (" << targetFPS << " FPS)\n";
    }
    else if (m_sessionPerformance.avgFrameTime_ms < targetFrameTime * 1.5) {
        report << "  ⚠ Warning: Processing speed marginal for " << targetFPS << " FPS capture\n";
    }
    else {
        report << "  ✗ Poor: Cannot keep up with " << targetFPS << " FPS capture rate\n";
    }

    // Frame drop analysis
    double frameDropPercentage = (m_droppedCount.load() * 100.0) / m_frameCount.load();
    if (frameDropPercentage > 5.0) {
        report << "  ✗ High frame drop rate: " << frameDropPercentage << "%\n";
    }
    else if (frameDropPercentage > 1.0) {
        report << "  ⚠ Moderate frame drop rate: " << frameDropPercentage << "%\n";
    }
    else {
        report << "  ✓ Low frame drop rate: " << frameDropPercentage << "%\n";
    }

    // Jitter analysis
    if (m_sessionPerformance.frameTimings.size() > 1) {
        double variance = 0.0;
        for (const auto& timing : m_sessionPerformance.frameTimings) {
            variance += std::pow(timing.totalFrameProcessingTime_ms - m_sessionPerformance.avgFrameTime_ms, 2);
        }
        variance /= (m_sessionPerformance.frameTimings.size() - 1);
        double coefficientOfVariation = (std::sqrt(variance) / m_sessionPerformance.avgFrameTime_ms) * 100;

        if (coefficientOfVariation < 10) {
            report << "  ✓ Stable performance: CV = " << coefficientOfVariation << "%\n";
        }
        else if (coefficientOfVariation < 25) {
            report << "  ⚠ Moderate performance variation: CV = " << coefficientOfVariation << "%\n";
        }
        else {
            report << "  ✗ High performance variation: CV = " << coefficientOfVariation << "%\n";
        }
    }

    // Identify slowest frames for debugging
    if (!m_sessionPerformance.frameTimings.empty()) {
        report << "\nSlowest Frames (Top 5):\n";
        std::vector<SessionPerformanceData::FrameTimingData> sortedFrames = m_sessionPerformance.frameTimings;
        std::sort(sortedFrames.begin(), sortedFrames.end(),
            [](const auto& a, const auto& b) {
                return a.totalFrameProcessingTime_ms > b.totalFrameProcessingTime_ms;
            });

        int count = std::min(5, static_cast<int>(sortedFrames.size()));
        for (int i = 0; i < count; ++i) {
            const auto& timing = sortedFrames[i];
            report << "  Frame " << timing.frameIndex << ": "
                << timing.totalFrameProcessingTime_ms << " ms"
                << " (Preprocessing: " << timing.preprocessingTime_ms << " ms,"
                << " Detection: " << timing.detectionTime_ms << " ms)\n";
        }
    }

    report << "\nRecommendations:\n";

    // Generate specific recommendations based on analysis
    if (m_sessionPerformance.avgFrameTime_ms > targetFrameTime) {
        report << "  - Reduce processing complexity to match capture rate\n";
        report << "  - Consider frame skipping or parallel processing\n";
    }

    if (frameDropPercentage > 1.0) {
        report << "  - Increase buffer sizes to handle processing spikes\n";
        report << "  - Optimize I/O operations (use faster storage)\n";
    }

    if (m_sessionPerformance.maxFrameTime_ms > 2 * m_sessionPerformance.avgFrameTime_ms) {
        report << "  - Investigate cause of processing time spikes\n";
        report << "  - Consider pre-allocating resources\n";
    }

    // Component-specific recommendations
    if (!m_sessionPerformance.frameTimings.empty()) {
        double avgDetectionTime = 0.0;
        for (const auto& timing : m_sessionPerformance.frameTimings) {
            avgDetectionTime += timing.detectionTime_ms;
        }
        avgDetectionTime /= m_sessionPerformance.frameTimings.size();

        if (avgDetectionTime > 50.0) {
            report << "  - Ball detection is the bottleneck - consider optimizing detection parameters\n";
            report << "  - Reduce image resolution or detection complexity if possible\n";
        }
    }

    // Get detailed report from BallDetector
    if (m_ballDetector) {
        report << "\n" << m_ballDetector->GeneratePerformanceReport();
    }

    report << "=========================================================\n";

    // Save report to file
    std::stringstream filename;
    filename << m_captureFolder << "/performance_report_" << std::put_time(&localTime, "%Y%m%d_%H%M%S") << ".txt";

    std::string reportPath = filename.str();

    try {
        std::ofstream reportFile(reportPath);
        if (reportFile.is_open()) {
            reportFile << report.str();
            reportFile.close();
            LOG_INFO("Session performance report saved to: " + reportPath);
        }
        else {
            LOG_ERROR("Failed to open performance report file: " + reportPath);
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("Failed to save performance report: " + std::string(e.what()));
    }

    // Also log the summary
    LOG_INFO("Performance Summary - Avg: " +
        std::to_string(m_sessionPerformance.avgFrameTime_ms) + " ms/frame, " +
        "Total frames: " + std::to_string(m_sessionPerformance.totalFramesProcessed) +
        ", Detection rate: " + std::to_string(m_sessionPerformance.framesWithBallDetected * 100.0 /
            m_sessionPerformance.totalFramesProcessed) + "%");
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

    // 모든 저장 작업이 완료될 때까지 대기 - 타임아웃을 무한대로 설정
    if (m_config.useAsyncSave && m_saveThreadRunning) {
        LOG_INFO("Waiting for asynchronous save operations to complete...");

        // 타임아웃을 기본값으로 설정 (0이 아닌 값)
        bool completed = WaitForSaveCompletion(DEFAULT_WAIT_TIMEOUT_SECONDS);

        if (!completed) {
            LOG_WARNING("Save operations did not complete within timeout period");
        }
    }

    // Save performance report for the entire session
    if (m_config.enableBallDetection) {
        SaveSessionPerformanceReport();
    }

    if (m_config.createMetadata) {
        SaveMetadata();

        if (m_config.enableBallDetection) {
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

    if (m_config.enableBallDetection) {
        LOG_INFO("Ball detection results: " +
            std::to_string(m_detectionResult.framesWithBall) + " frames with balls, " +
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
        // 동기 저장의 경우에도 ball detection 카운트 관리
        if (m_config.enableBallDetection) {
            m_ballDetectionPendingCount++;
        }

        std::string saveFolder = m_captureFolder;
        if (m_config.enableBallDetection && m_config.saveOriginalImages) {
            saveFolder = m_originalFolder;
        }

        std::stringstream ss;
        ss << saveFolder << "/frame_"
            << std::setfill('0') << std::setw(5) << (m_frameCount.load() - 1)
            << (m_config.imageFormat == 0 ? ".png" : ".jpg");
        std::string filename = ss.str();

        if (ImageSaver::SaveGrayscaleImage(data, width, height, filename,
            static_cast<ImageFormat>(m_config.imageFormat),
            m_config.jpgQuality)) {
            m_savedCount++;

            if (m_config.enableBallDetection) {
                SaveItem item;
                item.data.resize(width * height);
                memcpy(item.data.data(), data, width * height);
                item.width = width;
                item.height = height;
                item.frameIndex = m_frameCount.load() - 1;
                item.filename = filename;

                ProcessBallDetection(item);
                m_ballDetectionCompletedCount++;
            }
        }
        else {
            m_droppedCount++;
            LOG_ERROR("Failed to save frame: " + filename);

            if (m_config.enableBallDetection) {
                m_ballDetectionCompletedCount++;
            }
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

        if (m_config.enableBallDetection) {
            // Original images
            m_originalFolder = m_captureFolder + "/original";
            std::filesystem::create_directories(m_originalFolder);
            LOG_INFO("Created original images folder: " + m_originalFolder);

            // Detection images
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

    std::string saveFolder = m_captureFolder;
    if (m_config.enableBallDetection && m_config.saveOriginalImages) {
        saveFolder = m_originalFolder;
    }

    ss << saveFolder << "/frame_"
        << std::setfill('0') << std::setw(5) << frameIndex
        << (m_config.imageFormat == 0 ? ".png" : ".jpg");

    if (m_config.enableBallDetection) {
        m_ballDetectionPendingCount++;
    }

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

        // 대기 조건 수정
        m_queueCV.wait_for(lock, std::chrono::milliseconds(100), [this] {
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

                // Process ball detection
                if (m_config.enableBallDetection) {
                    LOG_DEBUG("Processing ball detection for frame " + std::to_string(item.frameIndex));
                    ProcessBallDetection(item);
                    m_ballDetectionCompletedCount++;
                }
            }
            else {
                m_droppedCount++;
                LOG_ERROR("Failed to save frame: " + item.filename);

                // Ball detection 실패 시에도 카운트 업데이트
                if (m_config.enableBallDetection) {
                    m_ballDetectionCompletedCount++;
                }
            }

            ReturnBufferToPool(std::move(item.data));

            m_processingCount--;

            // 완료 신호 전송
            m_completionCV.notify_all();

            lock.lock();
        }
    }

    LOG_INFO("Save thread ended");
}

void ContinuousCaptureManager::ProcessBallDetection(const SaveItem& item) {
    if (!m_ballDetector) {
        LOG_ERROR("Ball detector not initialized");
        return;
    }

    LOG_DEBUG("Starting ball detection for frame " + std::to_string(item.frameIndex));

    // 전체 프레임 처리 시작 시간
    auto frameProcessingStart = std::chrono::high_resolution_clock::now();

    // 프레임별 타이밍 데이터
    SessionPerformanceData::FrameTimingData frameTiming;
    frameTiming.frameIndex = item.frameIndex;

    // 1. 이미지 로드/준비 시간 (이미 메모리에 있으므로 최소)
    auto imageLoadStart = std::chrono::high_resolution_clock::now();
    // 이미지는 이미 item.data에 있음
    auto imageLoadEnd = std::chrono::high_resolution_clock::now();
    frameTiming.imageLoadTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        imageLoadEnd - imageLoadStart).count() / 1000.0;

    // 2. Enable performance profiling
    m_ballDetector->EnablePerformanceProfiling(true);

    // 3. Set debug output directory
    auto setupStart = std::chrono::high_resolution_clock::now();
    auto params = m_ballDetector->GetParameters();
    params.debugOutputDir = m_captureFolder + "/detect_outputs";
    params.saveIntermediateImages = true;
    m_ballDetector->SetParameters(params);

    // Create detect_outputs directory if it doesn't exist
    try {
        std::filesystem::create_directories(params.debugOutputDir);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Failed to create detect_outputs directory: " + std::string(e.what()));
    }
    auto setupEnd = std::chrono::high_resolution_clock::now();
    double setupTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        setupEnd - setupStart).count() / 1000.0;

    // 4. Detect ball 
    auto detectionStart = std::chrono::high_resolution_clock::now();
    auto result = m_ballDetector->DetectBall(
        item.data.data(), item.width, item.height, item.frameIndex);
    auto detectionEnd = std::chrono::high_resolution_clock::now();
    double ballDetectorTotalTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        detectionEnd - detectionStart).count() / 1000.0;

    // 5. Get performance metrics from BallDetector
    auto metrics = m_ballDetector->GetLastPerformanceMetrics();
    frameTiming.preprocessingTime_ms = metrics.preprocessingTime_ms;
    // Store only the core detection time (total detection minus preprocessing)
    frameTiming.detectionTime_ms = metrics.totalDetectionTime_ms - metrics.preprocessingTime_ms;

    // 6. Save detection image
    auto saveImageStart = std::chrono::high_resolution_clock::now();
    if (m_config.saveDetectionImages) {
        std::stringstream detectionPath;
        detectionPath << m_detectionFolder << "/frame_"
            << std::setfill('0') << std::setw(5) << item.frameIndex;

        if (result.found) {
            detectionPath << "_detected";
            if (!result.balls.empty()) {
                const auto& firstBall = result.balls[0];
                detectionPath << "_x" << static_cast<int>(firstBall.center.x)
                    << "_y" << static_cast<int>(firstBall.center.y);
            }
        }
        detectionPath << (m_config.imageFormat == 0 ? ".png" : ".jpg");

        if (!m_ballDetector->SaveDetectionImage(
            item.data.data(), item.width, item.height,
            result, detectionPath.str())) {
            LOG_ERROR("Failed to save detection image: " + detectionPath.str());
        }
        else {
            LOG_DEBUG("Saved detection image: " + detectionPath.str());
        }
    }
    auto saveImageEnd = std::chrono::high_resolution_clock::now();
    frameTiming.saveDetectionImageTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        saveImageEnd - saveImageStart).count() / 1000.0;

    // 7. 전체 프레임 처리 완료
    auto frameProcessingEnd = std::chrono::high_resolution_clock::now();
    frameTiming.totalFrameProcessingTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        frameProcessingEnd - frameProcessingStart).count() / 1000.0;

    // 8. 기타 작업 시간 계산 (전체 시간 - 각 단계별 시간)
    frameTiming.otherOperationsTime_ms = frameTiming.totalFrameProcessingTime_ms
        - (frameTiming.imageLoadTime_ms
            + ballDetectorTotalTime_ms
            + frameTiming.saveDetectionImageTime_ms);

    // 9. Update session performance data
    {
        std::lock_guard<std::mutex> lock(m_detectionMutex);

        m_sessionPerformance.frameTimings.push_back(frameTiming);
        m_sessionPerformance.totalFramesProcessed++;
        m_sessionPerformance.totalProcessingTime_ms += frameTiming.totalFrameProcessingTime_ms;

        if (frameTiming.totalFrameProcessingTime_ms < m_sessionPerformance.minFrameTime_ms) {
            m_sessionPerformance.minFrameTime_ms = frameTiming.totalFrameProcessingTime_ms;
        }
        if (frameTiming.totalFrameProcessingTime_ms > m_sessionPerformance.maxFrameTime_ms) {
            m_sessionPerformance.maxFrameTime_ms = frameTiming.totalFrameProcessingTime_ms;
        }

        // Update basic detection statistics
        if (result.found && !result.balls.empty()) {
            m_sessionPerformance.framesWithBallDetected++;
            m_detectionResult.framesWithBall++;
            m_detectionResult.totalBallsDetected += static_cast<int>(result.balls.size());

            // Calculate average confidence
            float sumConfidence = 0.0f;
            for (const auto& ball : result.balls) {
                sumConfidence += ball.confidence;
            }
            float currentAvg = m_detectionResult.averageConfidence;
            int currentCount = m_detectionResult.totalBallsDetected - static_cast<int>(result.balls.size());
            if (m_detectionResult.totalBallsDetected > 0) {
                m_detectionResult.averageConfidence =
                    (currentAvg * currentCount + sumConfidence) / m_detectionResult.totalBallsDetected;
            }

            LOG_INFO("Frame " + std::to_string(item.frameIndex) +
                ": Detected " + std::to_string(result.balls.size()) + " ball(s)" +
                " in " + std::to_string(frameTiming.totalFrameProcessingTime_ms) + " ms total" +
                " (BallDetector: " + std::to_string(ballDetectorTotalTime_ms) + " ms)" +
                " (confidence: " + std::to_string(result.balls[0].confidence) + ")");
        }
    }

    // Log performance warning if too slow
    if (frameTiming.totalFrameProcessingTime_ms > 100.0) {
        LOG_WARNING("Frame " + std::to_string(item.frameIndex) +
            " processing took " + std::to_string(frameTiming.totalFrameProcessingTime_ms) +
            " ms - may impact real-time processing");
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
    file << "  Ball Detection: " << (m_config.enableBallDetection ? "Yes" : "No") << "\n\n";

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

    file << "Ball Detection Results\n";
    file << "=====================\n\n";

    file << "Detection Date: "
        << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S") << "\n\n";

    file << "Summary:\n";
    file << "  Total Frames Analyzed: " << m_frameCount.load() << "\n";
    file << "  Frames with Ball: " << m_detectionResult.framesWithBall << "\n";
    file << "  Total Balls Detected: " << m_detectionResult.totalBallsDetected << "\n";
    file << "  Detection Rate: " << std::fixed << std::setprecision(1)
        << (m_frameCount > 0 ?
            (float)m_detectionResult.framesWithBall / m_frameCount * 100.0f : 0.0f)
        << "%\n";
    file << "  Average Confidence: " << std::fixed << std::setprecision(3)
        << m_detectionResult.averageConfidence << "\n\n";

    file << "Detection Parameters:\n";
    auto params = m_ballDetector->GetParameters();
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

    // 타임아웃이 0이면 기본값 사용
    int actualTimeout = (timeoutSeconds <= 0) ? DEFAULT_WAIT_TIMEOUT_SECONDS : timeoutSeconds;
    auto timeout = std::chrono::seconds(actualTimeout);

    LOG_INFO("Waiting for save completion with timeout: " + std::to_string(actualTimeout) + " seconds");

    std::unique_lock<std::mutex> lock(m_queueMutex);

    bool completed = m_completionCV.wait_for(lock, timeout, [this, &startWait, &timeout] {
        // 현재 상태 로깅
        bool queueEmpty = m_saveQueue.empty();
        bool noProcessing = (m_processingCount == 0);
        bool allSaved = (m_savedCount + m_droppedCount) >= m_frameCount;
        bool ballDetectionComplete = true;

        if (m_config.enableBallDetection) {
            ballDetectionComplete = (m_ballDetectionCompletedCount >= m_ballDetectionPendingCount);
        }

        // 주기적인 상태 로깅
        static auto lastLog = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (now - lastLog > std::chrono::seconds(1)) {
            LOG_DEBUG("Wait status - Queue: " + std::to_string(m_saveQueue.size()) +
                ", Processing: " + std::to_string(m_processingCount.load()) +
                ", Saved: " + std::to_string(m_savedCount.load()) +
                ", Dropped: " + std::to_string(m_droppedCount.load()) +
                ", Total: " + std::to_string(m_frameCount.load()) +
                ", BallDetection Pending: " + std::to_string(m_ballDetectionPendingCount.load()) +
                ", BallDetection Completed: " + std::to_string(m_ballDetectionCompletedCount.load()));
            lastLog = now;
        }

        // 모든 조건이 충족되면 완료
        return queueEmpty && noProcessing && allSaved && ballDetectionComplete;
        });

    if (!completed) {
        LOG_WARNING("Save completion wait timed out after " + std::to_string(actualTimeout) + " seconds");
        LOG_WARNING("Final state - Queue: " + std::to_string(m_saveQueue.size()) +
            ", Processing: " + std::to_string(m_processingCount.load()) +
            ", Saved: " + std::to_string(m_savedCount.load()) +
            ", Dropped: " + std::to_string(m_droppedCount.load()) +
            ", Total: " + std::to_string(m_frameCount.load()));
    }
    else {
        auto elapsed = std::chrono::steady_clock::now() - startWait;
        auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
        LOG_INFO("Save completion successful after " + std::to_string(elapsedSeconds) + " seconds");
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
    m_ballDetectionPendingCount = 0;
    m_ballDetectionCompletedCount = 0;
    m_captureFolder.clear();
    m_originalFolder.clear();
    m_detectionFolder.clear();
    m_isCapturing = false;
    m_saveThreadRunning = false;

    m_detectionResult = ContinuousCaptureDetectionResult();

    LOG_INFO("ContinuousCaptureManager reset completed");
}

ContinuousCaptureDetectionResult ContinuousCaptureManager::GetDetectionResult() const {
    std::lock_guard<std::mutex> lock(m_detectionMutex);
    return m_detectionResult;
}