#include "pch.h"

#ifdef ENABLE_CONTINUOUS_CAPTURE

#include "ContinuousCaptureManager.h"
#include "ImageSaver.h"
#include "Logger.h"
#include "BallDetector.h"
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <numeric>
#include <tbb/global_control.h>
#include <tbb/version.h>

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
    constexpr int OPTIMAL_BUFFER_POOL_SIZE = 20;
    for (int i = 0; i < OPTIMAL_BUFFER_POOL_SIZE; i++) {
        m_bufferPool.push(std::vector<unsigned char>());
    }

    const unsigned int optimalThreads = std::max(2u, std::thread::hardware_concurrency() - 1);
    tbb::global_control global_limit(
        tbb::global_control::max_allowed_parallelism, optimalThreads
    );

    m_ballDetector = std::make_unique<BallDetector>();

    m_ballDetector->EnablePerformanceProfiling(true);

    ConfigureBallDetectorOptimal();

    LOG_INFO("ContinuousCaptureManager initialized with optimal ball detection settings");
    LOG_INFO("TBB threads: " + std::to_string(optimalThreads));
    LOG_INFO("Ball detector performance profiling: ENABLED");
}

ContinuousCaptureManager::~ContinuousCaptureManager() {
    StopCapture();

    if (m_saveThread.joinable()) {
        m_saveThreadRunning = false;
        m_queueCV.notify_all();
        m_saveThread.join();
    }
}

void ContinuousCaptureManager::ConfigureBallDetectorOptimal() {
    if (!m_ballDetector) return;

    auto params = m_ballDetector->GetParameters();
    LOG_INFO("Ball detector using optimal settings:");
    LOG_INFO("  ROI Scale: " + std::to_string(params.roiScale));
    LOG_INFO("  Downscale Factor: " + std::to_string(params.downscaleFactor));
    LOG_INFO("  Max Candidates: " + std::to_string(params.maxCandidates));
    LOG_INFO("  Processing Threads: " + std::to_string(params.processingThreads));
    LOG_INFO("  Performance Profiling: " + std::string(m_ballDetector->IsPerformanceProfilingEnabled() ? "ENABLED" : "DISABLED"));
}

void ContinuousCaptureManager::SetConfig(const ContinuousCaptureConfig& config) {
    m_config = config;

    // Update ball detector if it exists
    if (m_ballDetector) {
        auto params = m_ballDetector->GetParameters();
        params.saveIntermediateImages = config.saveBallDetectorDebugImages;

        if (!config.debugImagePath.empty()) {
            params.debugOutputDir = config.debugImagePath;
        }

        m_ballDetector->SetParameters(params);
    }
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
    LOG_INFO("Save Debug Images: " + std::string(m_config.saveBallDetectorDebugImages ? "YES" : "NO"));
    LOG_INFO("===============================================");

    m_frameCount = 0;
    m_savedCount = 0;
    m_droppedCount = 0;
    m_processingCount = 0;
    m_actualDuration = 0.0;
    m_ballDetectionPendingCount = 0;
    m_ballDetectionCompletedCount = 0;
    m_isCapturing = true;

    m_startTime = std::chrono::steady_clock::now();

    m_detectionResult = ContinuousCaptureDetectionResult();
    m_sessionPerformance.Reset();

    if (!CreateCaptureFolder()) {
        m_state = ContinuousCaptureState::kERROR;
        m_isCapturing = false;
        LOG_ERROR("Failed to create capture folder");
        return false;
    }

    if (m_config.enableBallDetection && m_ballDetector) {
        SetBallDetectorDebugOutput(m_config.saveBallDetectorDebugImages);

        LOG_INFO("Ball detector configured:");
        LOG_INFO("  Debug output: " + std::string(m_config.saveBallDetectorDebugImages ? "ENABLED" : "DISABLED"));
        if (m_config.saveBallDetectorDebugImages) {
            LOG_INFO("  Debug path: " + m_captureFolder + "/debug_images");
        }
    }

    if (m_config.useAsyncSave && !m_saveThreadRunning) {
        m_saveThreadRunning = true;
        m_saveThread = std::thread(&ContinuousCaptureManager::SaveThreadWorker, this);
    }

    if (m_progressCallback) {
        m_progressCallback(0, 0.0, ContinuousCaptureState::CAPTURING);
    }

    LOG_INFO("Continuous capture started successfully");
    return true;
}


void ContinuousCaptureManager::StopCapture() {
    if (!m_isCapturing.load()) {
        LOG_WARNING("StopCapture called but not capturing");
        return;
    }

    LOG_INFO("Stopping continuous capture");

    m_isCapturing = false;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - m_startTime);
    m_actualDuration = elapsed.count() / 1000.0;

    LOG_INFO("Actual capture duration: " + std::to_string(m_actualDuration) + " seconds");

    m_state = ContinuousCaptureState::STOPPING;

    if (m_config.useAsyncSave && m_saveThreadRunning) {
        LOG_INFO("Waiting for asynchronous save operations to complete...");
        bool completed = WaitForSaveCompletion(DEFAULT_WAIT_TIMEOUT_SECONDS);
        if (!completed) {
            LOG_WARNING("Save operations did not complete within timeout period");
        }
    }

    // 캡처 완료 후 성능 리포트 생성 및 출력
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

    LOG_INFO("Continuous capture completed. Frames: " + std::to_string(m_savedCount.load()) + "/" + std::to_string(m_frameCount.load()) + " (dropped: " + std::to_string(m_droppedCount.load()) + ")");

    if (m_config.enableBallDetection) {
        LOG_INFO("Ball detection results: " +
            std::to_string(m_detectionResult.framesWithBall) + " frames with balls, " +
            std::to_string(m_detectionResult.totalBallsDetected) + " total balls detected");
    }
}

void ContinuousCaptureManager::ProcessFrame(const unsigned char* data, int width, int height) {
    if (!m_isCapturing.load()) {
        LOG_WARNING("ProcessFrame called but not capturing");
        return;
    }

    if (m_state != ContinuousCaptureState::CAPTURING) {
        LOG_WARNING("ProcessFrame called but state is not CAPTURING");
        return;
    }

    double elapsed = GetElapsedTime();

    // every 100 frames
    if (m_frameCount % 100 == 0) {
        LOG_DEBUG("Processing frame " + std::to_string(m_frameCount) + ", elapsed: " + std::to_string(elapsed) + "s / " + std::to_string(m_config.durationSeconds) + "s");
    }

    if (elapsed >= m_config.durationSeconds) {
        LOG_INFO("Capture duration reached. Stopping capture.");
        StopCapture();
        return;
    }

    m_frameCount++;

    if (m_config.useAsyncSave) {
        SaveFrameAsync(data, width, height);
    }
    else {
        if (m_config.enableBallDetection) {
            m_ballDetectionPendingCount++;
        }

        bool shouldSaveOriginal = m_config.saveOriginalImages || !m_config.enableBallDetection;

        if (shouldSaveOriginal) {
            std::string saveFolder = m_captureFolder;
            if (m_config.enableBallDetection && m_config.saveOriginalImages) {
                saveFolder = m_originalFolder;
            }

            std::stringstream ss;
            ss << saveFolder << "/frame_" << std::setfill('0') << std::setw(5) << (m_frameCount.load() - 1) << (m_config.imageFormat == 0 ? ".png" : ".jpg");
            std::string filename = ss.str();

            if (ImageSaver::SaveGrayscaleImage(data, width, height, filename,
                static_cast<ImageFormat>(m_config.imageFormat),
                m_config.jpgQuality)) {
                m_savedCount++;
            }
            else {
                m_droppedCount++;
                LOG_ERROR("Failed to save frame: " + filename);
            }
        }

        // Ball detection 처리
        if (m_config.enableBallDetection) {
            SaveItem item;
            item.data.resize(width * height);
            memcpy(item.data.data(), data, width * height);
            item.width = width;
            item.height = height;
            item.frameIndex = m_frameCount.load() - 1;
            item.filename = ""; // Ball detection doesn't need filename

            ProcessBallDetection(item);
            m_ballDetectionCompletedCount++;
        }
    }

    if (m_progressCallback && m_frameCount % 10 == 0) {
        m_progressCallback(m_frameCount.load(), elapsed, ContinuousCaptureState::CAPTURING);
    }
}

void ContinuousCaptureManager::ProcessBallDetection(const SaveItem& item) {
    if (!m_ballDetector) {
        LOG_ERROR("Ball detector not initialized");
        return;
    }

    auto frameProcessingStart = std::chrono::high_resolution_clock::now();

    SessionPerformanceData::FrameTimingData frameTiming;
    frameTiming.frameIndex = item.frameIndex;
    frameTiming.imageLoadTime_ms = 0.01;

    // Perform ball detection
    auto detectionStart = std::chrono::high_resolution_clock::now();
    auto result = m_ballDetector->DetectBall(item.data.data(), item.width, item.height, item.frameIndex);
    auto detectionEnd = std::chrono::high_resolution_clock::now();

    frameTiming.ballDetectionTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(detectionEnd - detectionStart).count() / 1000.0;

    // 모든 성능 메트릭 복사
    auto metrics = m_ballDetector->GetLastPerformanceMetrics();

    frameTiming.roiExtractionTime_ms = metrics.roiExtractionTime_ms;
    frameTiming.downscaleTime_ms = metrics.downscaleTime_ms;
    frameTiming.preprocessingTime_ms = metrics.preprocessingTime_ms;
    frameTiming.thresholdingTime_ms = metrics.thresholdingTime_ms;
    frameTiming.morphologyTime_ms = metrics.morphologyTime_ms;
    frameTiming.contourDetectionTime_ms = metrics.contourDetectionTime_ms;
    frameTiming.houghDetectionTime_ms = metrics.houghDetectionTime_ms;
    frameTiming.candidateEvaluationTime_ms = metrics.candidateEvaluationTime_ms;
    frameTiming.debugImagesSavingTime_ms = metrics.imagesSavingTime_ms;

    frameTiming.candidatesFound = metrics.candidatesFound;
    frameTiming.candidatesEvaluated = metrics.candidatesEvaluated;
    frameTiming.ballDetected = metrics.ballDetected;

    // 검출 이미지 저장
    auto saveImageStart = std::chrono::high_resolution_clock::now();
    if (m_config.saveDetectionImages && result.found) {
        std::stringstream detectionPath;
        detectionPath << m_detectionFolder << "/frame_" << std::setfill('0') << std::setw(5) << item.frameIndex;

        if (!result.balls.empty()) {
            const auto& firstBall = result.balls[0];
            detectionPath << "_x" << static_cast<int>(firstBall.center.x)
                << "_y" << static_cast<int>(firstBall.center.y)
                << "_c" << static_cast<int>(firstBall.confidence * 100);
        }

        detectionPath << ".jpg";
        m_ballDetector->SaveDetectionImage(item.data.data(), item.width, item.height, result, detectionPath.str(), true);
    }
    auto saveImageEnd = std::chrono::high_resolution_clock::now();

    if (m_config.saveDetectionImages && result.found) {
        frameTiming.saveDetectionImageTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(saveImageEnd - saveImageStart).count() / 1000.0;
    }
    else {
        frameTiming.saveDetectionImageTime_ms = 0.0;
    }

    auto frameProcessingEnd = std::chrono::high_resolution_clock::now();
    frameTiming.totalFrameProcessingTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(frameProcessingEnd - frameProcessingStart).count() / 1000.0;

    frameTiming.otherOperationsTime_ms = frameTiming.totalFrameProcessingTime_ms - frameTiming.ballDetectionTime_ms - frameTiming.saveDetectionImageTime_ms - frameTiming.imageLoadTime_ms;

    // 통계 업데이트
    {
        std::lock_guard<std::mutex> lock(m_detectionMutex);

        m_sessionPerformance.frameTimings.push_back(frameTiming);
        m_sessionPerformance.totalFramesProcessed++;
        m_sessionPerformance.totalProcessingTime_ms += frameTiming.totalFrameProcessingTime_ms;

        // 모든 알고리즘 시간 누적
        m_sessionPerformance.totalROIExtractionTime_ms += frameTiming.roiExtractionTime_ms;
        m_sessionPerformance.totalDownscaleTime_ms += frameTiming.downscaleTime_ms;
        m_sessionPerformance.totalPreprocessingTime_ms += frameTiming.preprocessingTime_ms;
        m_sessionPerformance.totalThresholdingTime_ms += frameTiming.thresholdingTime_ms;
        m_sessionPerformance.totalMorphologyTime_ms += frameTiming.morphologyTime_ms;
        m_sessionPerformance.totalContourDetectionTime_ms += frameTiming.contourDetectionTime_ms;
        m_sessionPerformance.totalHoughDetectionTime_ms += frameTiming.houghDetectionTime_ms;
        m_sessionPerformance.totalCandidateEvaluationTime_ms += frameTiming.candidateEvaluationTime_ms;
        m_sessionPerformance.totalDebugImagesSavingTime_ms += frameTiming.debugImagesSavingTime_ms;

        if (frameTiming.totalFrameProcessingTime_ms < m_sessionPerformance.minFrameTime_ms) {
            m_sessionPerformance.minFrameTime_ms = frameTiming.totalFrameProcessingTime_ms;
        }
        if (frameTiming.totalFrameProcessingTime_ms > m_sessionPerformance.maxFrameTime_ms) {
            m_sessionPerformance.maxFrameTime_ms = frameTiming.totalFrameProcessingTime_ms;
        }

        if (result.found && !result.balls.empty()) {
            m_sessionPerformance.framesWithBallDetected++;
            m_detectionResult.framesWithBall++;
            m_detectionResult.totalBallsDetected += static_cast<int>(result.balls.size());

            float sumConfidence = 0.0f;
            for (const auto& ball : result.balls) {
                sumConfidence += ball.confidence;
            }

            if (m_detectionResult.totalBallsDetected > 0) {
                m_detectionResult.averageConfidence =
                    ((m_detectionResult.averageConfidence * (m_detectionResult.totalBallsDetected - result.balls.size()))
                        + sumConfidence) / m_detectionResult.totalBallsDetected;
            }
        }
    }
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

    m_sessionPerformance.avgFrameTime_ms = m_sessionPerformance.totalProcessingTime_ms / m_sessionPerformance.totalFramesProcessed;

    double actualFPS = m_frameCount.load() / m_actualDuration;

    // 모든 알고리즘 평균 시간 계산
    double avgROIExtraction = m_sessionPerformance.totalROIExtractionTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgDownscale = m_sessionPerformance.totalDownscaleTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgPreprocessing = m_sessionPerformance.totalPreprocessingTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgThresholding = m_sessionPerformance.totalThresholdingTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgMorphology = m_sessionPerformance.totalMorphologyTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgContour = m_sessionPerformance.totalContourDetectionTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgHough = m_sessionPerformance.totalHoughDetectionTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgEvaluation = m_sessionPerformance.totalCandidateEvaluationTime_ms / m_sessionPerformance.totalFramesProcessed;

    // I/O 시간
    double avgDebugImageSave = m_sessionPerformance.totalDebugImagesSavingTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgDetectionImageSave = 0.0;
    double avgOtherOperations = 0.0;

    for (const auto& timing : m_sessionPerformance.frameTimings) {
        avgDetectionImageSave += timing.saveDetectionImageTime_ms;
        avgOtherOperations += timing.otherOperationsTime_ms;
    }
    avgDetectionImageSave /= m_sessionPerformance.totalFramesProcessed;
    avgOtherOperations /= m_sessionPerformance.totalFramesProcessed;

    // 총 알고리즘 시간 계산
    double totalPureAlgorithmTime = avgROIExtraction + avgDownscale + avgPreprocessing +
        avgThresholding + avgMorphology + avgContour +
        avgHough + avgEvaluation;
    double totalIOTime = avgDebugImageSave + avgDetectionImageSave;
    double totalOverhead = avgOtherOperations;
    double totalAccountedTime = totalPureAlgorithmTime + totalIOTime + totalOverhead;

    std::stringstream report;
    report << std::fixed << std::setprecision(2);

    report << "========== CONTINUOUS CAPTURE PERFORMANCE REPORT ==========\n";
    report << "Generated: " << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S") << "\n\n";

    report << "CAPTURE SUMMARY\n";
    report << "---------------\n";
    report << "Duration: " << m_actualDuration << " seconds\n";
    report << "Total Frames Captured: " << m_frameCount.load() << "\n";
    report << "Frames Saved: " << m_savedCount.load() << "\n";
    report << "Frames Dropped: " << m_droppedCount.load() << "\n";
    report << "Actual FPS: " << actualFPS << "\n\n";

    report << "BALL DETECTION RESULTS\n";
    report << "---------------------\n";
    report << "Total Frames Processed: " << m_sessionPerformance.totalFramesProcessed << "\n";
    report << "Frames with Ball Detected: " << m_sessionPerformance.framesWithBallDetected << "\n";
    report << "Total Balls Detected: " << m_detectionResult.totalBallsDetected << "\n";
    report << "Detection Rate: " << (m_sessionPerformance.framesWithBallDetected * 100.0 /
        m_sessionPerformance.totalFramesProcessed) << "%\n";
    report << "Average Confidence: " << (m_detectionResult.averageConfidence * 100) << "%\n\n";

    report << "PERFORMANCE METRICS\n";
    report << "------------------\n";
    report << "Total Processing Time: " << m_sessionPerformance.totalProcessingTime_ms << " ms\n";
    report << "Average Frame Processing Time: " << m_sessionPerformance.avgFrameTime_ms << " ms\n";
    report << "Min Frame Processing Time: " << m_sessionPerformance.minFrameTime_ms << " ms\n";
    report << "Max Frame Processing Time: " << m_sessionPerformance.maxFrameTime_ms << " ms\n";

    if (m_sessionPerformance.frameTimings.size() > 1) {
        double variance = 0.0;
        for (const auto& timing : m_sessionPerformance.frameTimings) {
            variance += std::pow(timing.totalFrameProcessingTime_ms - m_sessionPerformance.avgFrameTime_ms, 2);
        }
        variance /= (m_sessionPerformance.frameTimings.size() - 1);
        double stdDev = std::sqrt(variance);
        report << "Standard Deviation: " << stdDev << " ms\n";
    }

    report << "Real-time Factor: " << (1000.0 / m_sessionPerformance.avgFrameTime_ms) << " FPS capability\n\n";

    report << "PROCESSING TIME BREAKDOWN (Average per Frame)\n";
    report << "============================================\n";
    report << "A. Detection Algorithms (" << totalPureAlgorithmTime << " ms, "
        << (totalPureAlgorithmTime / m_sessionPerformance.avgFrameTime_ms * 100) << "%):\n";

    // 이미지 전처리 단계
    report << "   1) Image Preparation:\n";
    if (avgROIExtraction > 0.001) {
        report << "      - ROI Extraction: " << avgROIExtraction << " ms ("
            << (avgROIExtraction / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    }
    if (avgDownscale > 0.001) {
        report << "      - Downscaling: " << avgDownscale << " ms ("
            << (avgDownscale / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    }
    report << "      - Preprocessing (blur, etc.): " << avgPreprocessing << " ms ("
        << (avgPreprocessing / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";

    // 이진화 단계
    report << "   2) Binary Segmentation:\n";
    report << "      - Thresholding: " << avgThresholding << " ms ("
        << (avgThresholding / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    if (avgMorphology > 0.001) {
        report << "      - Morphology Operations: " << avgMorphology << " ms ("
            << (avgMorphology / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    }

    // 검출 단계
    report << "   3) Circle Detection:\n";
    report << "      - Contour Detection: " << avgContour << " ms ("
        << (avgContour / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    if (avgHough > 0.001) {
        report << "      - Hough Circle Detection: " << avgHough << " ms ("
            << (avgHough / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    }
    report << "      - Candidate Evaluation: " << avgEvaluation << " ms ("
        << (avgEvaluation / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n\n";

    report << "B. I/O Operations (" << totalIOTime << " ms, "
        << (totalIOTime / m_sessionPerformance.avgFrameTime_ms * 100) << "%):\n";
    if (avgDebugImageSave > 0.001) {
        report << "   - Debug Images Saving: " << avgDebugImageSave << " ms ("
            << (avgDebugImageSave / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    }
    if (avgDetectionImageSave > 0.001) {
        report << "   - Detection Images Saving: " << avgDetectionImageSave << " ms ("
            << (avgDetectionImageSave / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    }
    report << "\n";

    report << "C. System Overhead (" << totalOverhead << " ms, "
        << (totalOverhead / m_sessionPerformance.avgFrameTime_ms * 100) << "%):\n";
    report << "   - Thread synchronization, memory operations, etc.\n\n";

    report << "Total Accounted Time: " << totalAccountedTime << " ms ("
        << (totalAccountedTime / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";

    double unaccountedTime = m_sessionPerformance.avgFrameTime_ms - totalAccountedTime;
    if (std::abs(unaccountedTime) > 0.01) {
        report << "Unaccounted Time: " << unaccountedTime << " ms ("
            << (unaccountedTime / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    }
    report << "\n";

    // 알고리즘 효율성 분석
    report << "ALGORITHM EFFICIENCY ANALYSIS\n";
    report << "----------------------------\n";

    // 가장 시간이 많이 걸리는 단계 찾기
    std::vector<std::pair<std::string, double>> algorithmTimes = {
        {"ROI Extraction", avgROIExtraction},
        {"Downscaling", avgDownscale},
        {"Preprocessing", avgPreprocessing},
        {"Thresholding", avgThresholding},
        {"Morphology", avgMorphology},
        {"Contour Detection", avgContour},
        {"Hough Detection", avgHough},
        {"Candidate Evaluation", avgEvaluation}
    };

    std::sort(algorithmTimes.begin(), algorithmTimes.end(),
        [](const auto& a, const auto& b) { return a.second > b.second; });

    report << "Top time-consuming algorithms:\n";
    int rank = 1;
    for (const auto& [name, time] : algorithmTimes) {
        if (time > 0.001) {
            report << "   " << rank++ << ". " << name << ": " << time << " ms ("
                << (time / totalPureAlgorithmTime * 100) << "% of algorithm time)\n";
        }
    }
    report << "\n";

    const double targetFrameTime = 1000.0 / actualFPS;
    report << "---------------------\n";
    report << "Target FPS: " << actualFPS << " (" << targetFrameTime << " ms/frame)\n";

    double ioPercentage = (totalIOTime / m_sessionPerformance.avgFrameTime_ms) * 100;

    report << "\nOPTIMIZATION SETTINGS\n";
    report << "--------------------\n";
    auto params = m_ballDetector->GetParameters();
    report << "ROI Scale: " << params.roiScale << "\n";
    report << "Downscale Factor: " << params.downscaleFactor << "\n";
    report << "Max Candidates: " << params.maxCandidates << "\n";
    report << "Processing Threads: " << params.processingThreads << "\n";
    report << "Fast Mode: " << (params.fastMode ? "Enabled" : "Disabled") << "\n";
    report << "Parallel Processing: " << (params.useParallel ? "Enabled" : "Disabled") << "\n";
    report << "Morphology Operations: " << (params.useMorphology ? "Enabled" : "Disabled") << "\n";
    report << "Adaptive Threshold: " << (params.useAdaptiveThreshold ? "Enabled" : "Disabled") << "\n";
    report << "Save Debug Images: " << (params.saveIntermediateImages ? "Enabled" : "Disabled") << "\n";

    report << "\nIMAGE SAVING CONFIGURATION\n";
    report << "--------------------------\n";
    report << "Save Original Images: " << (m_config.saveOriginalImages ? "Enabled" : "Disabled") << "\n";
    report << "Save Detection Images: " << (m_config.saveDetectionImages ? "Enabled" : "Disabled") << "\n";
    report << "Save Debug Images: " << (m_config.saveBallDetectorDebugImages ? "Enabled" : "Disabled") << "\n";

    report << "\n==========================================================\n";

    LOG_INFO("\n" + report.str());

    // 파일로 저장
    std::stringstream filename;
    filename << m_captureFolder << "/performance_report_" << std::put_time(&localTime, "%Y%m%d_%H%M%S") << ".txt";

    try {
        std::ofstream reportFile(filename.str());
        if (reportFile.is_open()) {
            reportFile << report.str();
            reportFile.close();
            LOG_INFO("Performance report saved to: " + filename.str());
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("Failed to save performance report: " + std::string(e.what()));
    }

    // 요약 로그
    LOG_INFO("\n========== PERFORMANCE SUMMARY ==========");
    LOG_INFO("Average Frame Time: " + std::to_string(m_sessionPerformance.avgFrameTime_ms) + " ms");
    LOG_INFO("  - Pure Algorithms: " + std::to_string(totalPureAlgorithmTime) + " ms ("
        + std::to_string(totalPureAlgorithmTime / m_sessionPerformance.avgFrameTime_ms * 100) + "%)");
    LOG_INFO("    * Preprocessing: " + std::to_string(avgROIExtraction + avgDownscale + avgPreprocessing) + " ms");
    LOG_INFO("    * Segmentation: " + std::to_string(avgThresholding + avgMorphology) + " ms");
    LOG_INFO("    * Detection: " + std::to_string(avgContour + avgHough + avgEvaluation) + " ms");
    LOG_INFO("  - I/O Operations: " + std::to_string(totalIOTime) + " ms (" + std::to_string(ioPercentage) + "%)");
    LOG_INFO("  - System Overhead: " + std::to_string(totalOverhead) + " ms ("
        + std::to_string(totalOverhead / m_sessionPerformance.avgFrameTime_ms * 100) + "%)");
    LOG_INFO("Detection Rate: "
        + std::to_string(m_sessionPerformance.framesWithBallDetected * 100.0 / m_sessionPerformance.totalFramesProcessed) + "%");
    LOG_INFO("========================================\n");
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
            if (m_config.saveOriginalImages) {
                m_originalFolder = m_captureFolder + "/original";
                std::filesystem::create_directories(m_originalFolder);
                LOG_INFO("Created original images folder: " + m_originalFolder);
            }

            if (m_config.saveDetectionImages) {
                m_detectionFolder = m_captureFolder + "/detection";
                std::filesystem::create_directories(m_detectionFolder);
                LOG_INFO("Created detection images folder: " + m_detectionFolder);
            }

            m_detectionResult.detectionFolder = m_detectionFolder;

            if (m_config.saveBallDetectorDebugImages) {
                std::string debugFolder = m_captureFolder + "/debug_images";
                std::filesystem::create_directories(debugFolder);
                LOG_INFO("Created debug images folder: " + debugFolder);
            }

            if (m_ballDetector) {
                m_ballDetector->SetCurrentCaptureFolder(m_captureFolder);
                LOG_INFO("Set ball detector capture folder: " + m_captureFolder);
            }
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

    bool shouldSaveOriginal = m_config.saveOriginalImages || !m_config.enableBallDetection;
    std::string filename = "";

    if (shouldSaveOriginal) {
        std::stringstream ss;
        std::string saveFolder = m_captureFolder;
        if (m_config.enableBallDetection && m_config.saveOriginalImages) {
            saveFolder = m_originalFolder;
        }

        ss << saveFolder << "/frame_" << std::setfill('0') << std::setw(5) << frameIndex << (m_config.imageFormat == 0 ? ".png" : ".jpg");
        filename = ss.str();
    }

    if (m_config.enableBallDetection) {
        m_ballDetectionPendingCount++;
    }

    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        m_saveQueue.push(SaveItem{ std::move(buffer), filename, width, height, frameIndex });

        constexpr size_t MAX_QUEUE_SIZE = 100;
        if (m_saveQueue.size() > MAX_QUEUE_SIZE) {
            auto droppedItem = std::move(m_saveQueue.front());
            m_saveQueue.pop();
            m_droppedCount++;
            ReturnBufferToPool(std::move(droppedItem.data));
            LOG_WARNING("Dropped frame due to queue overflow: " + droppedItem.filename);
        }
    }
    m_queueCV.notify_one();
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
    file << "  Actual Duration: " << std::fixed << std::setprecision(3) << m_actualDuration << " seconds\n";

    if (m_actualDuration > 0) {
        file << "  Average FPS: " << std::fixed << std::setprecision(1) << (m_frameCount.load() / m_actualDuration) << "\n";
    }
    else {
        file << "  Average FPS: N/A\n";
    }

    if (m_frameCount > 0) {
        float successRate = (float)m_savedCount.load() / m_frameCount.load() * 100.0f;
        file << "  Save Success Rate: " << std::fixed << std::setprecision(1) << successRate << "%\n";
    }

    file.close();
    LOG_INFO("Metadata saved to: " + metaFile);
}



void ContinuousCaptureManager::SaveThreadWorker() {
    LOG_INFO("Save thread started");

    while (m_saveThreadRunning.load() || !m_saveQueue.empty() || m_processingCount > 0) {
        std::unique_lock<std::mutex> lock(m_queueMutex);

        m_queueCV.wait_for(lock, std::chrono::milliseconds(100), [this] { return !m_saveQueue.empty() || !m_saveThreadRunning.load(); });

        while (!m_saveQueue.empty()) {
            auto item = std::move(m_saveQueue.front());
            m_saveQueue.pop();
            m_processingCount++;
            lock.unlock();

            bool saveSuccess = true;
            if (!item.filename.empty()) {
                saveSuccess = ImageSaver::SaveGrayscaleImage(
                    item.data.data(), item.width, item.height,
                    item.filename,
                    static_cast<ImageFormat>(m_config.imageFormat),
                    m_config.jpgQuality);

                if (saveSuccess) {
                    m_savedCount++;
                }
                else {
                    m_droppedCount++;
                    LOG_ERROR("Failed to save frame: " + item.filename);
                }
            }

            // ball detection
            if (m_config.enableBallDetection && saveSuccess) {
                ProcessBallDetection(item);
                m_ballDetectionCompletedCount++;
            }

            ReturnBufferToPool(std::move(item.data));
            m_processingCount--;
            m_completionCV.notify_all();

            lock.lock();
        }
    }

    LOG_INFO("Save thread ended");
}


void ContinuousCaptureManager::SetBallDetectorDebugOutput(bool enable, const std::string& customPath) {
    if (!m_ballDetector) {
        LOG_ERROR("Ball detector not initialized");
        return;
    }

    auto params = m_ballDetector->GetParameters();
    params.saveIntermediateImages = enable;

    if (!customPath.empty()) {
        params.debugOutputDir = customPath;
    }
    else if (!m_captureFolder.empty()) {
        params.debugOutputDir = m_captureFolder + "/debug_images";

        if (enable) {
            try {
                std::filesystem::create_directories(params.debugOutputDir);
                LOG_INFO("Ensured debug directory exists: " + params.debugOutputDir);
            }
            catch (const std::exception& e) {
                LOG_ERROR("Failed to create debug directory: " + std::string(e.what()));
                params.saveIntermediateImages = false;
            }
        }
    }
    else {
        LOG_WARNING("No capture folder set, using default debug path");
        params.debugOutputDir = "./debug_images";
    }

    m_ballDetector->SetParameters(params);

    LOG_INFO("Ball detector debug output " + std::string(enable ? "enabled" : "disabled") +
        " with path: " + params.debugOutputDir);
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

    file << "Detection Date: " << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S") << "\n\n";

    file << "Summary:\n";
    file << "  Total Frames Analyzed: " << m_frameCount.load() << "\n";
    file << "  Frames with Ball: " << m_detectionResult.framesWithBall << "\n";
    file << "  Total Balls Detected: " << m_detectionResult.totalBallsDetected << "\n";
    file << "  Detection Rate: " << std::fixed << std::setprecision(1) << (m_frameCount > 0 ? (float)m_detectionResult.framesWithBall / m_frameCount * 100.0f : 0.0f) << "%\n";
    file << "  Average Confidence: " << std::fixed << std::setprecision(3) << m_detectionResult.averageConfidence << "\n\n";

    file << "Detection Parameters (Fixed Optimal):\n";
    auto params = m_ballDetector->GetParameters();
    file << "  Min Radius: " << params.minRadius << " pixels\n";
    file << "  Max Radius: " << params.maxRadius << " pixels\n";
    file << "  Brightness Threshold: " << params.brightnessThreshold << "\n";
    file << "  Min Circularity: " << params.minCircularity << "\n";
    file << "  ROI Scale: " << params.roiScale << "\n";
    file << "  Downscale Factor: " << params.downscaleFactor << "\n";
    file << "  Max Candidates: " << params.maxCandidates << "\n";
    file << "  Debug Images Saved: " << (params.saveIntermediateImages ? "Yes" : "No") << "\n";

    file << "\nFolders:\n";
    file << "  Capture Folder: " << m_captureFolder << "\n";

    if (!m_originalFolder.empty())                                          { file << "  Original Images: " << m_originalFolder << "\n"; }
    if (!m_detectionFolder.empty())                                         { file << "  Detection Images: " << m_detectionFolder << "\n"; }
    if (params.saveIntermediateImages && !params.debugOutputDir.empty())    { file << "  Debug Images: " << params.debugOutputDir << "\n"; }

    file.close();
    LOG_INFO("Detection metadata saved to: " + metaFile);
}


bool ContinuousCaptureManager::WaitForSaveCompletion(int timeoutSeconds) {
    auto startWait = std::chrono::steady_clock::now();

    int actualTimeout = (timeoutSeconds <= 0) ? DEFAULT_WAIT_TIMEOUT_SECONDS : timeoutSeconds;
    auto timeout = std::chrono::seconds(actualTimeout);

    LOG_INFO("Waiting for save completion with timeout: " + std::to_string(actualTimeout) + " seconds");

    std::unique_lock<std::mutex> lock(m_queueMutex);

    bool completed = m_completionCV.wait_for(lock, timeout, [this, &startWait, &timeout] {
        bool queueEmpty = m_saveQueue.empty();
        bool noProcessing = (m_processingCount == 0);
        bool allSaved = (m_savedCount + m_droppedCount) >= m_frameCount;
        bool ballDetectionComplete = true;

        if (m_config.enableBallDetection) {
            ballDetectionComplete = (m_ballDetectionCompletedCount >= m_ballDetectionPendingCount);
        }

        static auto lastLog = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (now - lastLog > std::chrono::seconds(1)) {
            LOG_DEBUG("Wait status - Queue: " + std::to_string(m_saveQueue.size()) +
                    ", Processing: " + std::to_string(m_processingCount.load()) +
                    ", Saved: " + std::to_string(m_savedCount.load()) +
                    ", Dropped: " + std::to_string(m_droppedCount.load()) +
                    ", Total: " + std::to_string(m_frameCount.load()));
            lastLog = now;
        }

        return queueEmpty && noProcessing && allSaved && ballDetectionComplete;
        });

    if (!completed) {
        LOG_WARNING("Save completion wait timed out after " + std::to_string(actualTimeout) + " seconds");
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
    result.success = (m_state == ContinuousCaptureState::COMPLETED) && (m_savedCount == m_frameCount);
    result.totalFrames = m_frameCount.load();
    result.savedFrames = m_savedCount.load();
    result.droppedFrames = m_droppedCount.load();
    result.actualDuration = m_actualDuration;
    result.folderPath = m_captureFolder;

    if (m_state == ContinuousCaptureState::kERROR)  { result.errorMessage = "Capture failed"; }
    else if (result.droppedFrames > 0)              { result.errorMessage = "Some frames were dropped"; }

    return result;
}

double ContinuousCaptureManager::GetElapsedTime() const {
    if (m_state != ContinuousCaptureState::CAPTURING) {
        return m_actualDuration;
    }

    if (m_isCapturing.load()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_startTime);
        double elapsedSeconds = elapsed.count() / 1000.0;

        if (elapsedSeconds > m_config.durationSeconds) {
            LOG_DEBUG("Elapsed time (" + std::to_string(elapsedSeconds) + "s) exceeded duration limit (" + std::to_string(m_config.durationSeconds) + "s)");
        }

        return elapsedSeconds;
    }

    return 0.0;
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
    constexpr size_t MAX_POOL_SIZE = 30;
    if (m_bufferPool.size() < MAX_POOL_SIZE) {
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
    m_startTime = std::chrono::steady_clock::time_point();

    m_detectionResult = ContinuousCaptureDetectionResult();
    m_sessionPerformance.Reset();

    ConfigureBallDetectorOptimal();
    if (m_ballDetector) {
        m_ballDetector->SetCurrentCaptureFolder("");
    }

    LOG_INFO("ContinuousCaptureManager reset completed");
}

ContinuousCaptureDetectionResult ContinuousCaptureManager::GetDetectionResult() const {
    std::lock_guard<std::mutex> lock(m_detectionMutex);
    return m_detectionResult;
}


#endif