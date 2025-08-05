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

    // BallDetector의 기본 파라미터 값을 그대로 사용
    BallDetector::DetectionParams params = m_ballDetector->GetParameters();

    // 연속 캡처에 필요한 최소한의 설정만 변경
    params.saveIntermediateImages = m_config.saveBallDetectorDebugImages;
    params.debugOutputDir = m_config.debugImagePath.empty() ?
        m_captureFolder + "/debug_images" : m_config.debugImagePath;

    // 성능 프로파일링은 활성화 (기본값이 false일 수 있으므로)
    params.enableProfiling = true;

    // 변경된 파라미터 적용
    m_ballDetector->SetParameters(params);

    // 현재 캡처 폴더 설정
    m_ballDetector->SetCurrentCaptureFolder(m_captureFolder);

    // 현재 설정값 로깅
    LOG_INFO("Ball detector using current settings:");
    LOG_INFO("  ROI Scale: " + std::to_string(params.roiScale));
    LOG_INFO("  Downscale Factor: " + std::to_string(params.downscaleFactor));
    LOG_INFO("  Max Candidates: " + std::to_string(params.maxCandidates));
    LOG_INFO("  Processing Threads: " + std::to_string(params.processingThreads));
    LOG_INFO("  Fast Mode: " + std::string(params.fastMode ? "ENABLED" : "DISABLED"));
    LOG_INFO("  Min Radius: " + std::to_string(params.minRadius));
    LOG_INFO("  Max Radius: " + std::to_string(params.maxRadius));
    LOG_INFO("  Performance Profiling: " + std::string(
        m_ballDetector->IsPerformanceProfilingEnabled() ? "ENABLED" : "DISABLED"));
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

    if (m_frameCount % 100 == 0) {
        LOG_DEBUG("Processing frame " + std::to_string(m_frameCount) + ", elapsed: " +
            std::to_string(elapsed) + "s / " + std::to_string(m_config.durationSeconds) + "s");
    }

    if (elapsed >= m_config.durationSeconds) {
        LOG_INFO("Capture duration reached. Stopping capture.");
        StopCapture();
        return;
    }

    m_frameCount++;

    // 이미지 저장 활성화 여부 확인
    bool anyImageSavingEnabled = m_config.saveOriginalImages ||
        m_config.saveDetectionImages ||
        m_config.saveBallDetectorDebugImages;

    if (m_config.useAsyncSave) {
        SaveFrameAsync(data, width, height);
    }
    else {
        // 동기 처리
        if (m_config.enableBallDetection) {
            m_ballDetectionPendingCount++;
        }

        bool shouldSaveOriginal = m_config.saveOriginalImages || (!m_config.enableBallDetection && anyImageSavingEnabled);

        if (shouldSaveOriginal) {
            std::string saveFolder = m_captureFolder;
            if (m_config.enableBallDetection && m_config.saveOriginalImages) {
                saveFolder = m_originalFolder;
            }

            std::stringstream ss;
            ss << saveFolder << "/frame_" << std::setfill('0') << std::setw(5) << (m_frameCount.load() - 1)
                << (m_config.imageFormat == 0 ? ".png" : ".jpg");
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
        else if (!anyImageSavingEnabled) {
            // 이미지 저장이 비활성화된 경우에도 카운트 증가
            m_savedCount++;
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

    // Memory operation timing
    auto memOpStart = std::chrono::high_resolution_clock::now();
    frameTiming.imageLoadTime_ms = 0.01; // 이미 메모리에 있음
    auto memOpEnd = std::chrono::high_resolution_clock::now();
    frameTiming.memoryOperationsTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        memOpEnd - memOpStart).count() / 1000.0;

    // Ball Detection
    auto detectionStart = std::chrono::high_resolution_clock::now();
    BallDetectionResult result = m_ballDetector->DetectBall(
        item.data.data(), item.width, item.height, item.frameIndex);
    auto detectionEnd = std::chrono::high_resolution_clock::now();

    frameTiming.ballDetectionTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        detectionEnd - detectionStart).count() / 1000.0;

    // BallDetector의 PerformanceMetrics에서 모든 메트릭 복사
    auto metrics = m_ballDetector->GetLastPerformanceMetrics();

    // 모든 세부 시간 복사
    frameTiming.roiExtractionTime_ms = metrics.roiExtractionTime_ms;
    frameTiming.downscaleTime_ms = metrics.downscaleTime_ms;
    frameTiming.preprocessingTime_ms = metrics.preprocessingTime_ms;
    frameTiming.claheTime_ms = metrics.claheTime_ms;
    frameTiming.normalizationTime_ms = metrics.normalizationTime_ms;
    frameTiming.thresholdingTime_ms = metrics.thresholdingTime_ms;
    frameTiming.morphologyTime_ms = metrics.morphologyTime_ms;
    frameTiming.contourDetectionTime_ms = metrics.contourDetectionTime_ms;
    frameTiming.houghDetectionTime_ms = metrics.houghDetectionTime_ms;
    frameTiming.templateMatchingTime_ms = metrics.templateMatchingTime_ms;
    frameTiming.candidateEvaluationTime_ms = metrics.candidateEvaluationTime_ms;
    frameTiming.trackingTime_ms = metrics.trackingTime_ms;
    frameTiming.selectionTime_ms = metrics.selectionTime_ms;
    frameTiming.debugImagesSavingTime_ms = metrics.imagesSavingTime_ms;

    // Detection 통계
    frameTiming.candidatesFound = metrics.candidatesFound;
    frameTiming.candidatesEvaluated = metrics.candidatesEvaluated;
    frameTiming.candidatesRejected = metrics.candidatesRejected;
    frameTiming.ballDetected = metrics.ballDetected;
    frameTiming.averageConfidence = metrics.averageConfidence;

    // Detection image 저장 시간
    auto saveImageStart = std::chrono::high_resolution_clock::now();
    if (m_config.saveDetectionImages && result.found) {
        std::stringstream detectionPath;
        detectionPath << m_detectionFolder << "/frame_" << std::setfill('0')
            << std::setw(5) << item.frameIndex;

        if (!result.balls.empty()) {
            const auto& firstBall = result.balls[0];
            detectionPath << "_x" << static_cast<int>(firstBall.center.x)
                << "_y" << static_cast<int>(firstBall.center.y)
                << "_c" << static_cast<int>(firstBall.confidence * 100);
        }

        detectionPath << ".jpg";

        m_ballDetector->SaveDetectionImage(
            item.data.data(), item.width, item.height,
            result, detectionPath.str(), true);
    }
    auto saveImageEnd = std::chrono::high_resolution_clock::now();

    frameTiming.saveDetectionImageTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        saveImageEnd - saveImageStart).count() / 1000.0;

    // 전체 프레임 처리 시간
    auto frameProcessingEnd = std::chrono::high_resolution_clock::now();
    frameTiming.totalFrameProcessingTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        frameProcessingEnd - frameProcessingStart).count() / 1000.0;

    // 기타 작업 시간 계산 (더 정확하게)
    double accountedTime = frameTiming.ballDetectionTime_ms +
        frameTiming.saveDetectionImageTime_ms +
        frameTiming.memoryOperationsTime_ms;

    frameTiming.otherOperationsTime_ms = frameTiming.totalFrameProcessingTime_ms - accountedTime;

    // 통계 업데이트
    {
        std::lock_guard<std::mutex> lock(m_detectionMutex);

        m_sessionPerformance.frameTimings.push_back(frameTiming);
        m_sessionPerformance.totalFramesProcessed++;
        m_sessionPerformance.totalProcessingTime_ms += frameTiming.totalFrameProcessingTime_ms;

        // 모든 세부 시간 누적
        m_sessionPerformance.totalImageLoadTime_ms += frameTiming.imageLoadTime_ms;
        m_sessionPerformance.totalROIExtractionTime_ms += frameTiming.roiExtractionTime_ms;
        m_sessionPerformance.totalDownscaleTime_ms += frameTiming.downscaleTime_ms;
        m_sessionPerformance.totalPreprocessingTime_ms += frameTiming.preprocessingTime_ms;
        m_sessionPerformance.totalCLAHETime_ms += frameTiming.claheTime_ms;
        m_sessionPerformance.totalNormalizationTime_ms += frameTiming.normalizationTime_ms;
        m_sessionPerformance.totalThresholdingTime_ms += frameTiming.thresholdingTime_ms;
        m_sessionPerformance.totalMorphologyTime_ms += frameTiming.morphologyTime_ms;
        m_sessionPerformance.totalContourDetectionTime_ms += frameTiming.contourDetectionTime_ms;
        m_sessionPerformance.totalHoughDetectionTime_ms += frameTiming.houghDetectionTime_ms;
        m_sessionPerformance.totalTemplateMatchingTime_ms += frameTiming.templateMatchingTime_ms;
        m_sessionPerformance.totalCandidateEvaluationTime_ms += frameTiming.candidateEvaluationTime_ms;
        m_sessionPerformance.totalTrackingTime_ms += frameTiming.trackingTime_ms;
        m_sessionPerformance.totalSelectionTime_ms += frameTiming.selectionTime_ms;
        m_sessionPerformance.totalDebugImagesSavingTime_ms += frameTiming.debugImagesSavingTime_ms;
        m_sessionPerformance.totalDetectionImageSavingTime_ms += frameTiming.saveDetectionImageTime_ms;
        m_sessionPerformance.totalMemoryOperationsTime_ms += frameTiming.memoryOperationsTime_ms;
        m_sessionPerformance.totalOtherOperationsTime_ms += frameTiming.otherOperationsTime_ms;

        // Min/Max 업데이트
        if (frameTiming.totalFrameProcessingTime_ms < m_sessionPerformance.minFrameTime_ms) {
            m_sessionPerformance.minFrameTime_ms = frameTiming.totalFrameProcessingTime_ms;
        }
        if (frameTiming.totalFrameProcessingTime_ms > m_sessionPerformance.maxFrameTime_ms) {
            m_sessionPerformance.maxFrameTime_ms = frameTiming.totalFrameProcessingTime_ms;
        }

        // Detection 통계
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

    // 모든 평균 시간 계산
    double avgImageLoad = m_sessionPerformance.totalImageLoadTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgROIExtraction = m_sessionPerformance.totalROIExtractionTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgDownscale = m_sessionPerformance.totalDownscaleTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgPreprocessing = m_sessionPerformance.totalPreprocessingTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgCLAHE = m_sessionPerformance.totalCLAHETime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgNormalization = m_sessionPerformance.totalNormalizationTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgThresholding = m_sessionPerformance.totalThresholdingTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgMorphology = m_sessionPerformance.totalMorphologyTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgContour = m_sessionPerformance.totalContourDetectionTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgHough = m_sessionPerformance.totalHoughDetectionTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgTemplate = m_sessionPerformance.totalTemplateMatchingTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgEvaluation = m_sessionPerformance.totalCandidateEvaluationTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgTracking = m_sessionPerformance.totalTrackingTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgSelection = m_sessionPerformance.totalSelectionTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgDebugImageSave = m_sessionPerformance.totalDebugImagesSavingTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgDetectionImageSave = m_sessionPerformance.totalDetectionImageSavingTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgOriginalImageSave = m_sessionPerformance.totalOriginalImageSavingTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgMemoryOps = m_sessionPerformance.totalMemoryOperationsTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgQueueOps = m_sessionPerformance.totalQueueOperationsTime_ms / m_sessionPerformance.totalFramesProcessed;
    double avgOtherOps = m_sessionPerformance.totalOtherOperationsTime_ms / m_sessionPerformance.totalFramesProcessed;

    // 카테고리별 시간 계산
    double totalImagePreparation = avgROIExtraction + avgDownscale;
    double totalPreprocessing = avgPreprocessing + avgCLAHE + avgNormalization;
    double totalSegmentation = avgThresholding + avgMorphology;
    double totalDetection = avgContour + avgHough + avgTemplate;
    double totalPostProcessing = avgEvaluation + avgTracking + avgSelection;
    double totalPureAlgorithmTime = totalImagePreparation + totalPreprocessing +
        totalSegmentation + totalDetection + totalPostProcessing;
    double totalIOTime = avgImageLoad + avgDebugImageSave + avgDetectionImageSave + avgOriginalImageSave;
    double totalSystemOverhead = avgMemoryOps + avgQueueOps + avgOtherOps;
    double totalAccountedTime = totalPureAlgorithmTime + totalIOTime + totalSystemOverhead;

    std::stringstream report;
    report << std::fixed << std::setprecision(3);

    report << "========== CONTINUOUS CAPTURE PERFORMANCE REPORT ==========\n";
    report << "Generated: " << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S") << "\n";
    report << "Version: 2.0 (Complete Time Accounting)\n\n";

    report << "CAPTURE SUMMARY\n";
    report << "---------------\n";
    report << "Duration: " << m_actualDuration << " seconds\n";
    report << "Total Frames Captured: " << m_frameCount.load() << "\n";
    report << "Frames Processed: " << m_sessionPerformance.totalFramesProcessed << "\n";
    report << "Frames Saved: " << m_savedCount.load() << "\n";
    report << "Frames Dropped: " << m_droppedCount.load() << "\n";
    report << "Actual FPS: " << actualFPS << "\n";
    report << "Processing FPS Capability: " << (1000.0 / m_sessionPerformance.avgFrameTime_ms) << "\n\n";

    report << "BALL DETECTION RESULTS\n";
    report << "---------------------\n";
    report << "Frames with Ball Detected: " << m_sessionPerformance.framesWithBallDetected
        << " (" << (m_sessionPerformance.framesWithBallDetected * 100.0 / m_sessionPerformance.totalFramesProcessed) << "%)\n";
    report << "Total Balls Detected: " << m_detectionResult.totalBallsDetected << "\n";
    report << "Average Confidence: " << (m_detectionResult.averageConfidence * 100) << "%\n\n";

    report << "PERFORMANCE METRICS\n";
    report << "------------------\n";
    report << "Total Processing Time: " << m_sessionPerformance.totalProcessingTime_ms << " ms\n";
    report << "Average Frame Processing Time: " << m_sessionPerformance.avgFrameTime_ms << " ms\n";
    report << "Min Frame Processing Time: " << m_sessionPerformance.minFrameTime_ms << " ms\n";
    report << "Max Frame Processing Time: " << m_sessionPerformance.maxFrameTime_ms << " ms\n";

    // Standard deviation calculation
    if (m_sessionPerformance.frameTimings.size() > 1) {
        double variance = 0.0;
        for (const auto& timing : m_sessionPerformance.frameTimings) {
            variance += std::pow(timing.totalFrameProcessingTime_ms - m_sessionPerformance.avgFrameTime_ms, 2);
        }
        variance /= (m_sessionPerformance.frameTimings.size() - 1);
        double stdDev = std::sqrt(variance);
        report << "Standard Deviation: " << stdDev << " ms\n";
        report << "Coefficient of Variation: " << (stdDev / m_sessionPerformance.avgFrameTime_ms * 100) << "%\n";
    }
    report << "\n";

    report << "COMPLETE TIME BREAKDOWN (Average per Frame)\n";
    report << "==========================================\n\n";

    report << "A. PURE ALGORITHM TIME: " << totalPureAlgorithmTime << " ms ("
        << (totalPureAlgorithmTime / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    report << "   ---------------------------------------------\n";

    // 1. Image Preparation
    report << "   1. Image Preparation: " << totalImagePreparation << " ms ("
        << (totalImagePreparation / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    if (avgROIExtraction > 0.001) {
        report << "      • ROI Extraction: " << avgROIExtraction << " ms\n";
    }
    if (avgDownscale > 0.001) {
        report << "      • Downscaling: " << avgDownscale << " ms\n";
    }
    report << "\n";

    // 2. Preprocessing
    report << "   2. Preprocessing: " << totalPreprocessing << " ms ("
        << (totalPreprocessing / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    report << "      • General Processing: " << avgPreprocessing << " ms\n";
    if (avgCLAHE > 0.001) {
        report << "      • CLAHE Enhancement: " << avgCLAHE << " ms\n";
    }
    if (avgNormalization > 0.001) {
        report << "      • Normalization: " << avgNormalization << " ms\n";
    }
    report << "\n";

    // 3. Segmentation
    report << "   3. Binary Segmentation: " << totalSegmentation << " ms ("
        << (totalSegmentation / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    report << "      • Thresholding: " << avgThresholding << " ms\n";
    if (avgMorphology > 0.001) {
        report << "      • Morphology Operations: " << avgMorphology << " ms\n";
    }
    report << "\n";

    // 4. Detection
    report << "   4. Ball Detection: " << totalDetection << " ms ("
        << (totalDetection / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    report << "      • Contour Detection: " << avgContour << " ms\n";
    if (avgHough > 0.001) {
        report << "      • Hough Circle Detection: " << avgHough << " ms\n";
    }
    if (avgTemplate > 0.001) {
        report << "      • Template Matching: " << avgTemplate << " ms\n";
    }
    report << "\n";

    // 5. Post-processing
    report << "   5. Post-processing: " << totalPostProcessing << " ms ("
        << (totalPostProcessing / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    report << "      • Candidate Evaluation: " << avgEvaluation << " ms\n";
    if (avgTracking > 0.001) {
        report << "      • Tracking Update: " << avgTracking << " ms\n";
    }
    if (avgSelection > 0.001) {
        report << "      • Ball Selection: " << avgSelection << " ms\n";
    }
    report << "\n";

    report << "B. I/O OPERATIONS: " << totalIOTime << " ms ("
        << (totalIOTime / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    report << "   ---------------------------------------------\n";
    report << "   • Image Loading: " << avgImageLoad << " ms\n";
    if (avgOriginalImageSave > 0.001) {
        report << "   • Original Image Saving: " << avgOriginalImageSave << " ms\n";
    }
    if (avgDetectionImageSave > 0.001) {
        report << "   • Detection Image Saving: " << avgDetectionImageSave << " ms\n";
    }
    if (avgDebugImageSave > 0.001) {
        report << "   • Debug Images Saving: " << avgDebugImageSave << " ms\n";
    }
    report << "\n";

    report << "C. SYSTEM OVERHEAD: " << totalSystemOverhead << " ms ("
        << (totalSystemOverhead / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";
    report << "   ---------------------------------------------\n";
    if (avgMemoryOps > 0.001) {
        report << "   • Memory Operations: " << avgMemoryOps << " ms\n";
    }
    if (avgQueueOps > 0.001) {
        report << "   • Queue Operations: " << avgQueueOps << " ms\n";
    }
    report << "   • Other Operations: " << avgOtherOps << " ms\n";
    report << "\n";

    report << "TOTAL ACCOUNTED TIME: " << totalAccountedTime << " ms ("
        << (totalAccountedTime / m_sessionPerformance.avgFrameTime_ms * 100) << "%)\n";

    double unaccountedTime = m_sessionPerformance.avgFrameTime_ms - totalAccountedTime;
    double unaccountedPercent = (unaccountedTime / m_sessionPerformance.avgFrameTime_ms * 100);

    if (std::abs(unaccountedTime) > 0.001) {
        report << "UNACCOUNTED TIME: " << unaccountedTime << " ms (" << unaccountedPercent << "%)\n";

        if (std::abs(unaccountedPercent) > 5.0) {
            report << "WARNING: Unaccounted time exceeds 5% - consider investigating timing methodology\n";
        }
    }
    else {
        report << "UNACCOUNTED TIME: 0.000 ms (0.0%) - Perfect time accounting achieved!\n";
    }
    report << "\n";

    // Detection statistics
    report << "DETECTION ALGORITHM STATISTICS\n";
    report << "-----------------------------\n";

    // Calculate average candidates per frame
    double avgCandidatesFound = 0.0;
    double avgCandidatesEvaluated = 0.0;
    for (const auto& timing : m_sessionPerformance.frameTimings) {
        avgCandidatesFound += timing.candidatesFound;
        avgCandidatesEvaluated += timing.candidatesEvaluated;
    }
    avgCandidatesFound /= m_sessionPerformance.totalFramesProcessed;
    avgCandidatesEvaluated /= m_sessionPerformance.totalFramesProcessed;

    report << "Average Candidates Found: " << avgCandidatesFound << " per frame\n";
    report << "Average Candidates Evaluated: " << avgCandidatesEvaluated << " per frame\n";
    report << "Candidate Rejection Rate: "
        << ((avgCandidatesFound - m_sessionPerformance.framesWithBallDetected / (double)m_sessionPerformance.totalFramesProcessed)
            / avgCandidatesFound * 100) << "%\n";
    report << "\n";

    // Top time-consuming algorithms
    report << "TOP TIME-CONSUMING OPERATIONS\n";
    report << "----------------------------\n";

    std::vector<std::pair<std::string, double>> algorithmTimes = {
        {"Candidate Evaluation", avgEvaluation},
        {"Thresholding", avgThresholding},
        {"Contour Detection", avgContour},
        {"Preprocessing", avgPreprocessing},
        {"Hough Detection", avgHough},
        {"ROI Extraction", avgROIExtraction},
        {"Downscaling", avgDownscale},
        {"Morphology", avgMorphology},
        {"CLAHE", avgCLAHE},
        {"Tracking", avgTracking},
        {"Selection", avgSelection},
        {"Template Matching", avgTemplate},
        {"Normalization", avgNormalization}
    };

    std::sort(algorithmTimes.begin(), algorithmTimes.end(),
        [](const auto& a, const auto& b) { return a.second > b.second; });

    int rank = 1;
    for (const auto& [name, time] : algorithmTimes) {
        if (time > 0.001 && rank <= 5) {
            report << "   " << rank++ << ". " << name << ": " << time << " ms ("
                << (time / totalPureAlgorithmTime * 100) << "% of algorithm time)\n";
        }
    }
    report << "\n";

    // Configuration details
    auto params = m_ballDetector->GetParameters();
    report << "CONFIGURATION DETAILS\n";
    report << "--------------------\n";
    report << "Algorithm Settings:\n";
    report << "  • ROI Scale: " << params.roiScale << "\n";
    report << "  • Downscale Factor: " << params.downscaleFactor << "\n";
    report << "  • Fast Mode: " << (params.fastMode ? "Enabled" : "Disabled") << "\n";
    report << "  • Parallel Processing: " << (params.useParallel ? "Enabled" : "Disabled") << "\n";
    report << "  • Processing Threads: " << params.processingThreads << "\n";
    report << "  • Max Candidates: " << params.maxCandidates << "\n";
    report << "\n";

    report << "Detection Methods:\n";
    report << "  • Contour Detection: " << (params.useContourDetection ? "Enabled" : "Disabled") << "\n";
    report << "  • Hough Detection: " << (params.useHoughDetection ? "Enabled" : "Disabled") << "\n";
    report << "  • Template Matching: " << (params.useTemplateMatching ? "Enabled" : "Disabled") << "\n";
    report << "\n";

    report << "Enhancement Options:\n";
    report << "  • CLAHE: " << (params.useCLAHE ? "Enabled" : "Disabled") << "\n";
    report << "  • Morphology: " << (params.useMorphology ? "Enabled" : "Disabled") << "\n";
    report << "  • Normalization: " << (params.useNormalization ? "Enabled" : "Disabled") << "\n";
    report << "\n";

    report << "Image Saving:\n";
    report << "  • Save Original: " << (m_config.saveOriginalImages ? "Yes" : "No") << "\n";
    report << "  • Save Detection: " << (m_config.saveDetectionImages ? "Yes" : "No") << "\n";
    report << "  • Save Debug: " << (m_config.saveBallDetectorDebugImages ? "Yes" : "No") << "\n";
    report << "\n";

    report << "==========================================================\n";

    // Log to console
    LOG_INFO("\n" + report.str());

    // Save to file
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

    // Summary log
    LOG_INFO("\n========== PERFORMANCE SUMMARY ==========");
    LOG_INFO("Average Frame Time: " + std::to_string(m_sessionPerformance.avgFrameTime_ms) + " ms");
    LOG_INFO("Processing Capability: " + std::to_string(1000.0 / m_sessionPerformance.avgFrameTime_ms) + " FPS");
    LOG_INFO("Time Accounting:");
    LOG_INFO("  - Pure Algorithms: " + std::to_string(totalPureAlgorithmTime) + " ms ("
        + std::to_string(totalPureAlgorithmTime / m_sessionPerformance.avgFrameTime_ms * 100) + "%)");
    LOG_INFO("  - I/O Operations: " + std::to_string(totalIOTime) + " ms ("
        + std::to_string(totalIOTime / m_sessionPerformance.avgFrameTime_ms * 100) + "%)");
    LOG_INFO("  - System Overhead: " + std::to_string(totalSystemOverhead) + " ms ("
        + std::to_string(totalSystemOverhead / m_sessionPerformance.avgFrameTime_ms * 100) + "%)");
    LOG_INFO("  - Unaccounted: " + std::to_string(unaccountedTime) + " ms ("
        + std::to_string(unaccountedPercent) + "%)");
    LOG_INFO("Detection Rate: " + std::to_string(m_sessionPerformance.framesWithBallDetected * 100.0 /
        m_sessionPerformance.totalFramesProcessed) + "%");
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

        // 이미지 저장이 활성화된 경우에만 하위 폴더 생성
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
                m_detectionResult.detectionFolder = m_detectionFolder;
            }

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

        ss << saveFolder << "/frame_" << std::setfill('0') << std::setw(5) << frameIndex
            << (m_config.imageFormat == 0 ? ".png" : ".jpg");
        filename = ss.str();
    }

    if (m_config.enableBallDetection) {
        m_ballDetectionPendingCount++;
    }

    {
        auto queueOpStart = std::chrono::high_resolution_clock::now();

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

        auto queueOpEnd = std::chrono::high_resolution_clock::now();
        double queueTime = std::chrono::duration_cast<std::chrono::microseconds>(
            queueOpEnd - queueOpStart).count() / 1000.0;

        // Queue operation time을 세션 성능에 추가
        std::lock_guard<std::mutex> perfLock(m_detectionMutex);
        m_sessionPerformance.totalQueueOperationsTime_ms += queueTime;
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

        m_queueCV.wait_for(lock, std::chrono::milliseconds(100), [this] {
            return !m_saveQueue.empty() || !m_saveThreadRunning.load();
            });

        while (!m_saveQueue.empty()) {
            auto item = std::move(m_saveQueue.front());
            m_saveQueue.pop();
            m_processingCount++;
            lock.unlock();

            bool saveSuccess = true;

            // 이미지 파일명이 비어있으면 저장하지 않음
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
            else {
                // 이미지 저장이 비활성화된 경우에도 처리된 것으로 간주
                m_savedCount++;
            }

            // Ball detection
            if (m_config.enableBallDetection) {
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

    // 현재 캡처 폴더도 업데이트
    if (!m_captureFolder.empty()) {
        m_ballDetector->SetCurrentCaptureFolder(m_captureFolder);
    }

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
    file << "  Detection Rate: " << std::fixed << std::setprecision(1)
        << (m_frameCount > 0 ? (float)m_detectionResult.framesWithBall / m_frameCount * 100.0f : 0.0f)
        << "%\n";
    file << "  Average Confidence: " << std::fixed << std::setprecision(3)
        << m_detectionResult.averageConfidence << "\n\n";

    // BallDetector의 현재 파라미터 가져오기
    auto params = m_ballDetector->GetParameters();

    file << "Detection Parameters:\n";
    file << "  Min Radius: " << params.minRadius << " pixels\n";
    file << "  Max Radius: " << params.maxRadius << " pixels\n";
    file << "  Brightness Threshold: " << params.brightnessThreshold << "\n";
    file << "  Min Circularity: " << params.minCircularity << "\n";
    file << "  ROI Scale: " << params.roiScale << "\n";
    file << "  Downscale Factor: " << params.downscaleFactor << "\n";
    file << "  Max Candidates: " << params.maxCandidates << "\n";
    file << "  Fast Mode: " << (params.fastMode ? "Yes" : "No") << "\n";
    file << "  Parallel Processing: " << (params.useParallel ? "Yes" : "No") << "\n";
    file << "  Processing Threads: " << params.processingThreads << "\n";
    file << "  Debug Images Saved: " << (params.saveIntermediateImages ? "Yes" : "No") << "\n";

    file << "\nFolders:\n";
    file << "  Capture Folder: " << m_captureFolder << "\n";

    if (!m_originalFolder.empty()) {
        file << "  Original Images: " << m_originalFolder << "\n";
    }
    if (!m_detectionFolder.empty()) {
        file << "  Detection Images: " << m_detectionFolder << "\n";
    }
    if (params.saveIntermediateImages && !params.debugOutputDir.empty()) {
        file << "  Debug Images: " << params.debugOutputDir << "\n";
    }

    file.close();
    LOG_INFO("Detection metadata saved to: " + metaFile);
}


bool ContinuousCaptureManager::WaitForSaveCompletion(int timeoutSeconds) {
    auto startWait = std::chrono::steady_clock::now();

    int actualTimeout = (timeoutSeconds <= 0) ? DEFAULT_WAIT_TIMEOUT_SECONDS : timeoutSeconds;
    auto timeout = std::chrono::seconds(actualTimeout);

    LOG_INFO("Waiting for save completion with timeout: " + std::to_string(actualTimeout) + " seconds");

    // 이미지 저장이 전혀 활성화되지 않은 경우 빠른 반환
    bool anyImageSavingEnabled = m_config.saveOriginalImages ||
        m_config.saveDetectionImages ||
        m_config.saveBallDetectorDebugImages;

    if (!anyImageSavingEnabled && !m_config.useAsyncSave) {
        LOG_INFO("No image saving enabled, skipping save completion wait");
        return true;
    }

    std::unique_lock<std::mutex> lock(m_queueMutex);

    bool completed = m_completionCV.wait_for(lock, timeout, [this, anyImageSavingEnabled, &startWait, &timeout] {
        bool queueEmpty = m_saveQueue.empty();
        bool noProcessing = (m_processingCount == 0);

        // 이미지 저장이 비활성화된 경우 다른 완료 조건 사용
        bool allSaved = true;
        if (anyImageSavingEnabled) {
            allSaved = (m_savedCount + m_droppedCount) >= m_frameCount;
        }
        else {
            // 이미지 저장이 없으면 처리된 프레임 수로 판단
            allSaved = m_ballDetectionCompletedCount >= m_ballDetectionPendingCount;
        }

        bool ballDetectionComplete = true;
        if (m_config.enableBallDetection) {
            ballDetectionComplete = (m_ballDetectionCompletedCount >= m_ballDetectionPendingCount);
        }

        // 디버그 로그 - 1초마다만 출력
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

        return queueEmpty && noProcessing && allSaved && ballDetectionComplete;
        });

    if (!completed) {
        LOG_WARNING("Save completion wait timed out after " + std::to_string(actualTimeout) + " seconds");
        LOG_WARNING("Final status - Queue: " + std::to_string(m_saveQueue.size()) +
            ", Processing: " + std::to_string(m_processingCount.load()) +
            ", Saved: " + std::to_string(m_savedCount.load()) +
            ", Total frames: " + std::to_string(m_frameCount.load()) +
            ", Ball detection pending: " + std::to_string(m_ballDetectionPendingCount.load()) +
            ", Ball detection completed: " + std::to_string(m_ballDetectionCompletedCount.load()));
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

    // 이미지 저장 설정 확인
    bool anyImageSavingEnabled = m_config.saveOriginalImages ||
        m_config.saveDetectionImages ||
        m_config.saveBallDetectorDebugImages;

    // 성공 여부 판단 - 이미지 저장이 비활성화된 경우 다른 기준 사용
    if (anyImageSavingEnabled) {
        result.success = (m_state == ContinuousCaptureState::COMPLETED) && (m_savedCount == m_frameCount);
    }
    else {
        // 이미지 저장이 없으면 상태와 볼 검출 완료 여부로 판단
        result.success = (m_state == ContinuousCaptureState::COMPLETED) &&
            (m_ballDetectionCompletedCount == m_ballDetectionPendingCount);
    }

    result.totalFrames = m_frameCount.load();
    result.savedFrames = anyImageSavingEnabled ? m_savedCount.load() : m_frameCount.load();
    result.droppedFrames = m_droppedCount.load();
    result.actualDuration = m_actualDuration;
    result.folderPath = m_captureFolder;

    if (m_state == ContinuousCaptureState::kERROR) {
        result.errorMessage = "Capture failed";
    }
    else if (anyImageSavingEnabled && result.droppedFrames > 0) {
        result.errorMessage = "Some frames were dropped";
    }

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
