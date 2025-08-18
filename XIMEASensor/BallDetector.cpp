#include "pch.h"
#include "BallDetector.h"
#include "Logger.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <thread>
#include <numeric>
#include <immintrin.h>  // SIMD
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>

namespace fs = std::filesystem;

// Constants
namespace {
    constexpr int MIN_DOWNSCALE_WIDTH = 160;
    constexpr int MIN_DOWNSCALE_HEIGHT = 120;
    constexpr float MIN_CONFIDENCE_THRESHOLD = 0.7f;
    constexpr int SHADOW_THRESHOLD = 100;
    constexpr float SHADOW_ENHANCEMENT_BASE = 1.0f;
    constexpr int MAX_DISPLAY_BALLS = 5;
    constexpr int INFO_PANEL_HEIGHT = 90;
    constexpr int INFO_PANEL_WIDTH = 280;
    constexpr double CANNY_LOW_THRESHOLD = 50.0;
    constexpr double CANNY_HIGH_THRESHOLD = 150.0;
    constexpr int SOBEL_KERNEL_SIZE = 3;
    constexpr float TEMPLATE_MATCH_THRESHOLD = 0.7f;
    constexpr int TEMPLATE_SCALES = 5;
    constexpr float TEMPLATE_SCALE_STEP = 0.1f;
}

#ifdef ENABLE_PERFORMANCE_PROFILING

class ScopedTimer {
private:
    std::chrono::high_resolution_clock::time_point m_start;
    double& m_target;
    const std::string m_name;
    bool m_enabled;

public:
    ScopedTimer(const std::string& name, double& target, bool enabled = true)
        : m_name(name), m_target(target), m_enabled(enabled) {
        if (m_enabled) {
            m_start = std::chrono::high_resolution_clock::now();
        }
    }

    ~ScopedTimer() {
        if (m_enabled) {
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - m_start).count();
            m_target = duration / 1000.0;
        }
    }
};

#define MEASURE_TIME(name, code, metricsVar) \
    { \
        ScopedTimer timer(name, metricsVar); \
        code \
    }

#else
#define MEASURE_TIME(name, code, metricsVar) { code }
#endif

// Thread-local storage definition with pre-allocated resources
thread_local std::unique_ptr<BallDetector::DetectionContext> BallDetector::t_context;

// Optimized DetectionParams constructor
BallDetector::DetectionParams::DetectionParams() {
    // Initialize all members in one go to improve cache locality
    minRadius = 5;
    maxRadius = 20;
    minCircularity = 0.70f;
    dp = 2.0;
    minDist = 40.0;
    param1 = 130.0;
    param2 = 0.87;
    brightnessThreshold = 120;
    useAdaptiveThreshold = false;
    adaptiveBlockSize = 51;
    adaptiveConstant = 5.0;
    useColorFilter = false;
    useCircularityCheck = false;
    contrastThreshold = 20.0f;
    detectMultiple = false;
    edgeThreshold = 30.0f;
    skipPreprocessing = false;
    useEnhanceShadows = true;
    shadowEnhanceFactor = 0.7f;
    useMorphology = false;
    useNormalization = true;
    useCLAHE = false;
    claheClipLimit = 2.0;
    useContourDetection = true;
    useHoughDetection = true;
    useThresholding = true;
    useTemplateMatching = false;
    fastMode = true;
    useROI = true;
    roiScale = 0.75f;
    downscaleFactor = 2;
    useParallel = true;
    maxCandidates = 15;
    processingThreads = std::max(2u, std::thread::hardware_concurrency() / 2);
    useGPU = false;
    saveIntermediateImages = false;
    debugOutputDir = "";
    enableProfiling = false;
    enableTracking = false;
    maxTrackingDistance = 100.0f;
    trackingHistorySize = 10;
}

// Optimized PerformanceMetrics::Reset
void BallDetector::PerformanceMetrics::Reset() {
    // Use memset for faster reset
    memset(this, 0, sizeof(PerformanceMetrics));
}

// Optimized BallDetector::Impl class
class BallDetector::Impl {
public:
    // Pre-allocated working images to avoid repeated allocations
    cv::Mat m_lastProcessedImage;
    cv::Mat m_lastBinaryImage;
    cv::Mat m_lastShadowEnhanced;
    cv::Mat m_lastCLAHE;
    cv::Mat m_lastEdgeImage;
    cv::Mat m_downscaledImage;

    std::string m_currentCaptureFolder;
    const DetectionParams* m_paramsPtr;
    DetectionContext* m_context = nullptr;

    // Enhanced Mat pool with size tracking
    struct MatPoolEntry {
        cv::Mat mat;
        std::chrono::steady_clock::time_point lastUsed;
    };
    std::vector<MatPoolEntry> m_matPool;
    std::mutex m_poolMutex;
    static constexpr size_t MAX_POOL_SIZE = 20;
    static constexpr auto POOL_CLEANUP_INTERVAL = std::chrono::seconds(30);

    // Async save queue
    struct SaveTask {
        cv::Mat image;
        std::string path;
    };
    std::queue<SaveTask> m_saveQueue;
    std::mutex m_saveQueueMutex;
    std::condition_variable m_saveCV;
    std::thread m_saveThread;
    std::atomic<bool> m_saveThreadRunning{ false };

    // Pre-allocated buffers for processing
    std::vector<unsigned char> m_tempBuffer;
    std::vector<float> m_floatBuffer;

    // Lookup tables for common operations
    std::array<unsigned char, 256> m_shadowLUT = {};
    bool m_shadowLUTInitialized = false;

    Impl() : m_currentCaptureFolder(""), m_paramsPtr(nullptr) {
        m_saveThreadRunning = true;
        m_saveThread = std::thread(&Impl::saveWorker, this);
        m_matPool.reserve(MAX_POOL_SIZE);

        // Pre-allocate buffers
        m_tempBuffer.reserve(2048 * 2048);
        m_floatBuffer.reserve(2048 * 2048);

        m_shadowLUT.fill(0);
    }

    ~Impl() {
        m_saveThreadRunning = false;
        m_saveCV.notify_all();
        if (m_saveThread.joinable()) {
            m_saveThread.join();
        }
    }

    void setContext(DetectionContext* ctx) { m_context = ctx; }
    void setCurrentCaptureFolder(const std::string& folder) { m_currentCaptureFolder = folder; }
    void setParamsReference(const DetectionParams* params) {
        m_paramsPtr = params;
        // Initialize shadow LUT when params are set
        if (!m_shadowLUTInitialized && params) {
            initializeShadowLUT(params->shadowEnhanceFactor);
            m_shadowLUTInitialized = true;
        }
    }

    // Optimized methods
    cv::Mat preprocessImage(const cv::Mat& grayImage, const DetectionParams& params);
    cv::Mat applyCLAHE(const cv::Mat& image, double clipLimit);
    cv::Mat enhanceShadowRegionsOptimized(const cv::Mat& image, float factor);
    cv::Mat computeEdgeMapOptimized(const cv::Mat& image, int method = 0);
    std::vector<cv::Vec3f> detectCirclesHoughOptimized(const cv::Mat& image, const DetectionParams& params);
    bool quickValidateCircleOptimized(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
    float calculateConfidenceOptimized(const cv::Mat& image, const cv::Vec3f& circle,
        const DetectionParams& params, const cv::Mat& edgeMap = cv::Mat());
    float calculateCircularityOptimized(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
    float calculateEdgeStrengthOptimized(const cv::Mat& edgeMap, const cv::Vec3f& circle);
    cv::Mat extractROIOptimized(const cv::Mat& image, float scale);
    std::vector<BallInfo> evaluateCandidatesOptimized(const cv::Mat& image,
        const std::vector<cv::Vec3f>& candidates,
        const DetectionParams& params,
        float scaleFactor,
        const cv::Rect& roiRect,
        int frameIndex,
        const cv::Mat& edgeMap = cv::Mat());
    void saveIntermediateImagesAsync(const std::string& basePath, int frameIndex);
    cv::Mat getMatFromPoolOptimized(int rows, int cols, int type);
    void returnMatToPool(cv::Mat& mat);
    void cleanupMatPool();
    void initializeShadowLUT(float factor);

private:
    void saveWorker() {
        while (m_saveThreadRunning.load()) {
            std::unique_lock<std::mutex> lock(m_saveQueueMutex);
            m_saveCV.wait(lock, [this] {
                return !m_saveQueue.empty() || !m_saveThreadRunning.load();
                });

            // Process batch saves
            std::vector<SaveTask> tasks;
            while (!m_saveQueue.empty() && tasks.size() < 5) {
                tasks.push_back(std::move(m_saveQueue.front()));
                m_saveQueue.pop();
            }
            lock.unlock();

            // Save without holding lock
            for (auto& task : tasks) {
                try {
                    cv::imwrite(task.path, task.image);
                }
                catch (const cv::Exception& e) {
                    LOG_ERROR("Failed to save image: " + std::string(e.what()));
                }
            }
        }
    }
};

// Initialize shadow enhancement lookup table
void BallDetector::Impl::initializeShadowLUT(float factor) {
    for (int i = 0; i < 256; ++i) {
        if (i < SHADOW_THRESHOLD) {
            float enhancementRatio = 1.0f - (i / static_cast<float>(SHADOW_THRESHOLD));
            float enhancementFactor = SHADOW_ENHANCEMENT_BASE + factor * enhancementRatio;
            m_shadowLUT[i] = cv::saturate_cast<uchar>(i * enhancementFactor);
        }
        else {
            m_shadowLUT[i] = i;
        }
    }
}

// BallDetector constructor - optimized
BallDetector::BallDetector()
    : m_params(),
    pImpl(std::make_unique<Impl>()),
    m_performanceProfilingEnabled(false),
    m_nextTrackId(0),
    m_templateInitialized(false) {

    cv::setNumThreads(0);  // Use all available threads
    cv::setUseOptimized(true);

    LOG_INFO("BallDetector initialized with optimized TBB support");
    LOG_INFO("Performance profiling: " +
        std::string(m_performanceProfilingEnabled ? "ENABLED" : "DISABLED"));

    InitializeDefaultParams();
    m_lastMetrics.Reset();
}

BallDetector::~BallDetector() = default;

void BallDetector::InitializeDefaultParams() {
    m_params = DetectionParams();
    LOG_INFO("BallDetector parameters initialized with default values");
}

void BallDetector::ResetToDefaults() {
    InitializeDefaultParams();
    pImpl->m_shadowLUTInitialized = false;
    LOG_INFO("BallDetector parameters reset to default values");
}

void BallDetector::EnablePerformanceProfiling(bool enable) {
    m_performanceProfilingEnabled = enable;
    LOG_INFO("BallDetector performance profiling " + std::string(enable ? "ENABLED" : "DISABLED"));
}

void BallDetector::SetCurrentCaptureFolder(const std::string& folder) {
    if (pImpl) {
        pImpl->setCurrentCaptureFolder(folder);
    }
}

// Optimized main detection function
BallDetectionResult BallDetector::DetectBall(const unsigned char* imageData, int width, int height, int frameIndex) {
    auto totalStartTime = std::chrono::high_resolution_clock::now();

    // Initialize thread-local context with pre-allocated resources
    auto contextInitStart = std::chrono::high_resolution_clock::now();
    if (!t_context) {
        t_context = std::make_unique<DetectionContext>();
    }

    auto& context = *t_context;

    // 이미지 크기가 변경될 수 있으므로 매번 체크하여 재할당
    if (context.tempMat1.empty() || context.tempMat1.rows != height || context.tempMat1.cols != width) {
        context.tempMat1.create(height, width, CV_8UC1);
        context.tempMat2.create(height, width, CV_8UC1);
        context.tempMat3.create(height, width, CV_8UC1);
    }
    context.tempCandidates.reserve(100);
    auto contextInitEnd = std::chrono::high_resolution_clock::now();

    context.metrics.Reset();
    context.metrics.contextInitTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        contextInitEnd - contextInitStart).count() / 1000.0;

    // Get thread-safe copy of parameters
    auto paramCopyStart = std::chrono::high_resolution_clock::now();
    DetectionParams localParams;
    {
        std::lock_guard<std::mutex> lock(m_paramsMutex);
        localParams = m_params;
    }
    auto paramCopyEnd = std::chrono::high_resolution_clock::now();
    context.metrics.parameterCopyTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(paramCopyEnd - paramCopyStart).count() / 1000.0;

    // Pass context to implementation
    pImpl->setContext(&context);
    pImpl->setParamsReference(&localParams);

    BallDetectionResult result;
    result.found = false;
    result.balls.clear();
    result.balls.reserve(MAX_DISPLAY_BALLS);  // Pre-allocate

    // Input validation
    if (!imageData || width <= 0 || height <= 0) {
        result.errorMessage = "Invalid input parameters";
        LOG_ERROR("DetectBall: " + result.errorMessage);
        return result;
    }

    try {
        // Create Mat wrapper without copying data
        auto matCreationStart = std::chrono::high_resolution_clock::now();
        cv::Mat grayImage(height, width, CV_8UC1, const_cast<unsigned char*>(imageData));
        auto matCreationEnd = std::chrono::high_resolution_clock::now();
        context.metrics.matCreationTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(matCreationEnd - matCreationStart).count() / 1000.0;

        cv::Mat workingImage = grayImage;
        cv::Rect roiRect(0, 0, width, height);
        float scaleFactor = 1.0f;

        // ROI extraction - optimized with timing
        if (localParams.useROI && localParams.roiScale < 1.0f) {
            MEASURE_TIME("ROI extraction",
                workingImage = pImpl->extractROIOptimized(grayImage, localParams.roiScale);
                int offsetX = (width - workingImage.cols) / 2;
                int offsetY = (height - workingImage.rows) / 2;
                roiRect = cv::Rect(offsetX, offsetY, workingImage.cols, workingImage.rows);
                , context.metrics.roiExtractionTime_ms);
        }

        // Downscaling - optimized with timing
        if (localParams.fastMode && localParams.downscaleFactor > 1) {
            int newWidth = workingImage.cols / localParams.downscaleFactor;
            int newHeight = workingImage.rows / localParams.downscaleFactor;

            if (newWidth >= MIN_DOWNSCALE_WIDTH && newHeight >= MIN_DOWNSCALE_HEIGHT) {
                MEASURE_TIME("Image downscaling",
                    cv::Mat & downscaled = pImpl->m_downscaledImage;
                if (downscaled.empty() || downscaled.rows != newHeight || downscaled.cols != newWidth) {
                    downscaled.create(newHeight, newWidth, CV_8UC1);
                }
                cv::resize(workingImage, downscaled, cv::Size(newWidth, newHeight), 0, 0, cv::INTER_LINEAR);
                workingImage = downscaled;
                scaleFactor = static_cast<float>(localParams.downscaleFactor);
                , context.metrics.downscaleTime_ms);
            }
        }

        // Preprocessing - optimized with timing
        cv::Mat processed = workingImage;
        if (!localParams.skipPreprocessing) {
            MEASURE_TIME("Preprocessing",
                processed = pImpl->preprocessImage(workingImage, localParams);
                , context.metrics.preprocessingTime_ms);
        }

        // Edge detection - FIXED: now properly tracked
        cv::Mat edgeMap;
        if (localParams.edgeThreshold > 0) {
            MEASURE_TIME("Edge detection",
                edgeMap = pImpl->computeEdgeMapOptimized(processed, 0);
                , context.metrics.edgeDetectionTime_ms);
        }

        // Detection - use pre-allocated vector
        context.tempCandidates.clear();
        std::vector<cv::Vec3f>& candidates = context.tempCandidates;

        // Contour-based detection - optimized with timing
        if (localParams.useContourDetection && localParams.useThresholding) {
            cv::Mat& binary = context.tempMat2;

            MEASURE_TIME("Thresholding",
                if (localParams.useAdaptiveThreshold) {
                    cv::adaptiveThreshold(processed, binary, 255,
                        cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY,
                        localParams.adaptiveBlockSize, localParams.adaptiveConstant);
                }
                else {
                    double threshVal = localParams.brightnessThreshold > 0 ? localParams.brightnessThreshold : 0;
                    int threshType = localParams.brightnessThreshold > 0 ? cv::THRESH_BINARY : (cv::THRESH_BINARY | cv::THRESH_OTSU);
                    cv::threshold(processed, binary, threshVal, 255, threshType);
                }
            , context.metrics.thresholdingTime_ms);

            if (localParams.useMorphology) {
                MEASURE_TIME("Morphology operations",
                    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
                    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
                    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
                    , context.metrics.morphologyTime_ms);
            }

            if (localParams.saveIntermediateImages) {
                pImpl->m_lastBinaryImage = binary.clone();
            }

            MEASURE_TIME("Contour detection",
                auto contourCandidates = detectByContoursOptimized(binary, processed, scaleFactor);
                candidates.insert(candidates.end(), contourCandidates.begin(), contourCandidates.end());
                , context.metrics.contourDetectionTime_ms);

            LOG_DEBUG("Contour detection found " + std::to_string(candidates.size()) + " candidates");
        }

        // Hough-based detection - optimized with timing
        if (localParams.useHoughDetection && (candidates.empty() || !localParams.useContourDetection)) {
            MEASURE_TIME("Hough circle detection",
                auto houghCandidates = pImpl->detectCirclesHoughOptimized(processed, localParams);
                candidates.insert(candidates.end(), houghCandidates.begin(), houghCandidates.end());
                , context.metrics.houghDetectionTime_ms);

            if (!candidates.empty()) {
                LOG_DEBUG("Hough detection found " + std::to_string(candidates.size()) + " candidates");
            }
        }

        // Template matching - optimized with timing
        if (localParams.useTemplateMatching && m_templateInitialized) {
            MEASURE_TIME("Template matching",
                auto templateCandidates = detectByTemplateOptimized(processed, scaleFactor);
                candidates.insert(candidates.end(), templateCandidates.begin(), templateCandidates.end());
                , context.metrics.templateMatchingTime_ms);
        }

        context.metrics.candidatesFound = static_cast<int>(candidates.size());

        // Candidate evaluation - optimized with timing
        std::vector<BallInfo> validBalls;
        if (!candidates.empty()) {
            MEASURE_TIME("Candidate evaluation",
                validBalls = pImpl->evaluateCandidatesOptimized(processed, candidates, localParams, scaleFactor, roiRect, frameIndex, edgeMap);
                , context.metrics.candidateEvaluationTime_ms);
        }
        context.metrics.candidatesEvaluated = static_cast<int>(candidates.size());
        context.metrics.candidatesRejected = static_cast<int>(candidates.size() - validBalls.size());

        // Tracking update - optimized with timing
        if (localParams.enableTracking && !validBalls.empty()) {
            MEASURE_TIME("Tracking update",
                updateTrackingOptimized(validBalls);
                , context.metrics.trackingTime_ms);
        }

        // Final selection - optimized with timing
        if (!validBalls.empty()) {
            MEASURE_TIME("Ball selection",
                selectBestBallsOptimized(validBalls, result);
                , context.metrics.selectionTime_ms);

                // Final confidence filter and processing
                auto filterStart = std::chrono::high_resolution_clock::now();

                // Final confidence filter
                result.balls.erase(std::remove_if(result.balls.begin(), result.balls.end(), 
                    [](const BallInfo& ball) { return ball.confidence < MIN_CONFIDENCE_THRESHOLD; }),
                result.balls.end()
            );

            auto filterEnd = std::chrono::high_resolution_clock::now();
            context.metrics.resultFilteringTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(filterEnd - filterStart).count() / 1000.0;

            if (!result.balls.empty()) {
                context.metrics.ballDetected = true;

                if (localParams.enableTracking) {
                    updateTracking(result);
                }

                // Calculate average confidence
                auto confCalcStart = std::chrono::high_resolution_clock::now();
                float totalConfidence = 0.0f;
                for (const auto& ball : result.balls) {
                    totalConfidence += ball.confidence;
                }
                context.metrics.averageConfidence = totalConfidence / result.balls.size();
                auto confCalcEnd = std::chrono::high_resolution_clock::now();
                context.metrics.confidenceCalculationTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(confCalcEnd - confCalcStart).count() / 1000.0;

                const BallInfo& ball = result.balls[0];
                LOG_INFO("Ball detected at (" + std::to_string(static_cast<int>(ball.center.x)) + ", " +
                    std::to_string(static_cast<int>(ball.center.y)) + "), radius=" +
                    std::to_string(static_cast<int>(ball.radius)) + ", confidence=" +
                    std::to_string(ball.confidence * 100) + "%");
            }
            else {
                result.found = false;
                LOG_DEBUG("All detected balls below 50% confidence threshold");
            }
        }

        // Debug output - async with timing
        if (localParams.saveIntermediateImages) {
            MEASURE_TIME("Debug image saving",
                saveDebugImagesAsync(imageData, width, height, frameIndex, result);
                , context.metrics.imagesSavingTime_ms);
        }

    }
    catch (const cv::Exception& e) {
        result.errorMessage = "OpenCV error: " + std::string(e.what());
        LOG_ERROR(result.errorMessage);
    }
    catch (const std::exception& e) {
        result.errorMessage = "Detection error: " + std::string(e.what());
        LOG_ERROR(result.errorMessage);
    }

    auto totalEndTime = std::chrono::high_resolution_clock::now();
    context.metrics.totalDetectionTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(totalEndTime - totalStartTime).count() / 1000.0;

    // Save metrics thread-safely
    auto metricsUpdateStart = std::chrono::high_resolution_clock::now();
    {
        std::lock_guard<std::mutex> lock(m_metricsMutex);
        m_lastMetrics = context.metrics;
    }
    auto metricsUpdateEnd = std::chrono::high_resolution_clock::now();

    // Add synchronization time
    context.metrics.synchronizationTime_ms = context.metrics.parameterCopyTime_ms +
        std::chrono::duration_cast<std::chrono::microseconds>(metricsUpdateEnd - metricsUpdateStart).count() / 1000.0;

    // Update metrics one more time to include synchronization time
    {
        std::lock_guard<std::mutex> lock(m_metricsMutex);
        m_lastMetrics.synchronizationTime_ms = context.metrics.synchronizationTime_ms;
        m_lastMetrics.metricsUpdateTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(metricsUpdateEnd - metricsUpdateStart).count() / 1000.0;
    }

    return result;
}

// Optimized contour-based detection
std::vector<cv::Vec3f> BallDetector::detectByContoursOptimized(const cv::Mat& binary, const cv::Mat& grayImage, float downscaleFactor) {

    std::vector<std::vector<cv::Point>> contours;
    // Use RETR_EXTERNAL and CHAIN_APPROX_SIMPLE for efficiency
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    float minR = static_cast<float>(m_params.minRadius);
    float maxR = static_cast<float>(m_params.maxRadius);
    if (downscaleFactor > 1) {
        minR = std::max(3.0f, minR / downscaleFactor);
        maxR = maxR / downscaleFactor;
    }

    const float minArea = static_cast<float>(CV_PI * minR * minR);
    const float maxArea = static_cast<float>(CV_PI * maxR * maxR);
    const float minCircularityThreshold = m_params.minCircularity;

    // Pre-allocate result vector
    std::vector<cv::Vec3f> circleList;
    circleList.reserve(contours.size() / 2);  // Estimate

    // Process contours in parallel for large sets
    if (contours.size() > 20 && m_params.useParallel) {
        tbb::concurrent_vector<cv::Vec3f> concurrentCircles;

        tbb::parallel_for(tbb::blocked_range<size_t>(0, contours.size()),
            [&](const tbb::blocked_range<size_t>& range) {
                for (size_t i = range.begin(); i != range.end(); ++i) {
                    // Quick area check
                    double area = cv::contourArea(contours[i]);
                    if (area < minArea || area > maxArea) {
                        continue;
                    }

                    cv::Point2f center;
                    float radius;
                    cv::minEnclosingCircle(contours[i], center, radius);

                    if (radius < minR || radius > maxR) {
                        continue;
                    }

                    // Circularity check
                    double perimeter = cv::arcLength(contours[i], true);
                    if (perimeter > 0) {
                        double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
                        if (circularity >= minCircularityThreshold) {
                            // Optional color filter
                            if (m_params.useColorFilter) {
                                int cx = static_cast<int>(center.x);
                                int cy = static_cast<int>(center.y);
                                // Fixed: Add proper boundary check
                                if (cx >= 0 && cy >= 0 && cx < grayImage.cols && cy < grayImage.rows) {
                                    if (grayImage.at<uchar>(cy, cx) >= m_params.brightnessThreshold * 0.5) {
                                        concurrentCircles.push_back(cv::Vec3f(center.x, center.y, radius));
                                    }
                                }
                            }
                            else {
                                concurrentCircles.push_back(cv::Vec3f(center.x, center.y, radius));
                            }
                        }
                    }
                }
            }
        );

        // Copy to result
        circleList.assign(concurrentCircles.begin(), concurrentCircles.end());
    }
    else {
        // Sequential processing for small sets
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < minArea || area > maxArea) {
                continue;
            }

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);

            if (radius < minR || radius > maxR) {
                continue;
            }

            double perimeter = cv::arcLength(contour, true);
            if (perimeter > 0) {
                double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
                if (circularity >= minCircularityThreshold) {
                    // Fixed: Add proper boundary check for sequential processing
                    if (m_params.useColorFilter) {
                        int cx = static_cast<int>(center.x);
                        int cy = static_cast<int>(center.y);
                        if (cx >= 0 && cy >= 0 && cx < grayImage.cols && cy < grayImage.rows) {
                            if (grayImage.at<uchar>(cy, cx) >= m_params.brightnessThreshold * 0.5) {
                                circleList.push_back(cv::Vec3f(center.x, center.y, radius));
                            }
                        }
                    }
                    else {
                        circleList.push_back(cv::Vec3f(center.x, center.y, radius));
                    }
                }
            }
        }
    }

    return circleList;
}

// Original contour detection (backward compatibility)
std::vector<cv::Vec3f> BallDetector::detectByContours(const cv::Mat& binary,
    const cv::Mat& grayImage,
    float downscaleFactor) {
    return detectByContoursOptimized(binary, grayImage, downscaleFactor);
}

// template matching
std::vector<cv::Vec3f> BallDetector::detectByTemplateOptimized(const cv::Mat& image, float downscaleFactor) {
    std::vector<cv::Vec3f> candidates;
    candidates.reserve(TEMPLATE_SCALES * 5);

    if (m_ballTemplate.empty()) {
        return candidates;
    }

    // Process multiple scales in parallel
    tbb::concurrent_vector<cv::Vec3f> concurrentCandidates;

    tbb::parallel_for(0, TEMPLATE_SCALES, [&](int s) {
        float scale = 1.0f - s * TEMPLATE_SCALE_STEP;
        cv::Mat scaledTemplate;
        cv::resize(m_ballTemplate, scaledTemplate, cv::Size(static_cast<int>(m_ballTemplate.cols * scale), static_cast<int>(m_ballTemplate.rows * scale)));

        cv::Mat result;
        cv::matchTemplate(image, scaledTemplate, result, cv::TM_CCOEFF_NORMED);

        // Find local maxima above threshold
        cv::Mat mask;
        cv::threshold(result, mask, TEMPLATE_MATCH_THRESHOLD, 1, cv::THRESH_BINARY);
        mask.convertTo(mask, CV_8U);

        // Find peaks
        std::vector<cv::Point> locations;
        cv::findNonZero(mask, locations);

        for (const auto& loc : locations) {
            if (result.at<float>(loc) > TEMPLATE_MATCH_THRESHOLD) {
                float cx = loc.x + scaledTemplate.cols / 2.0f;
                float cy = loc.y + scaledTemplate.rows / 2.0f;
                float radius = (scaledTemplate.cols + scaledTemplate.rows) / 4.0f;
                concurrentCandidates.push_back(cv::Vec3f(cx, cy, radius));
            }
        }
        });

    // Copy results
    candidates.assign(concurrentCandidates.begin(), concurrentCandidates.end());
    return candidates;
}

// Original template detection (backward compatibility)
std::vector<cv::Vec3f> BallDetector::detectByTemplate(const cv::Mat& image, float downscaleFactor) {
    return detectByTemplateOptimized(image, downscaleFactor);
}

// Optimized ball selection
void BallDetector::selectBestBallsOptimized(const std::vector<BallInfo>& validBalls,
    BallDetectionResult& result) {

    if (!m_params.detectMultiple) {
        // Find best ball using parallel reduction for large sets
        if (m_params.useParallel && validBalls.size() > 10) {
            BallInfo bestBall = tbb::parallel_reduce(
                tbb::blocked_range<size_t>(0, validBalls.size()),
                validBalls[0],
                [&validBalls](const tbb::blocked_range<size_t>& r, BallInfo currentBest) {
                    for (size_t i = r.begin(); i != r.end(); ++i) {
                        if (validBalls[i].confidence > currentBest.confidence) {
                            currentBest = validBalls[i];
                        }
                    }
                    return currentBest;
                },
                [](const BallInfo& a, const BallInfo& b) {
                    return (a.confidence > b.confidence) ? a : b;
                }
            );
            result.balls.push_back(bestBall);
        }
        else {
            // Sequential search for small sets
            auto bestIt = std::max_element(validBalls.begin(), validBalls.end(),
                [](const BallInfo& a, const BallInfo& b) {
                    return a.confidence < b.confidence;
                });
            result.balls.push_back(*bestIt);
        }
    }
    else {
        // Multiple ball detection
        std::vector<BallInfo> sortedBalls = validBalls;

        // Sort by confidence
        if (m_params.useParallel && sortedBalls.size() > 10) {
            tbb::parallel_sort(sortedBalls.begin(), sortedBalls.end(),
                [](const BallInfo& a, const BallInfo& b) {
                    return a.confidence > b.confidence;
                });
        }
        else {
            std::sort(sortedBalls.begin(), sortedBalls.end(),
                [](const BallInfo& a, const BallInfo& b) {
                    return a.confidence > b.confidence;
                });
        }

        // Non-maximum suppression
        std::vector<bool> selected(sortedBalls.size(), false);
        for (size_t i = 0; i < sortedBalls.size() && result.balls.size() < MAX_DISPLAY_BALLS; ++i) {
            if (!selected[i]) {
                result.balls.push_back(sortedBalls[i]);
                selected[i] = true;

                // Mark overlapping balls
                for (size_t j = i + 1; j < sortedBalls.size(); ++j) {
                    if (!selected[j]) {
                        float dist = static_cast<float>(cv::norm(sortedBalls[i].center - sortedBalls[j].center));
                        if (dist < (sortedBalls[i].radius + sortedBalls[j].radius) * 0.5f) {
                            selected[j] = true;
                        }
                    }
                }
            }
        }
    }

    result.found = true;
}

// Original ball selection (backward compatibility)
void BallDetector::selectBestBalls(const std::vector<BallInfo>& validBalls,
    BallDetectionResult& result) {
    selectBestBallsOptimized(validBalls, result);
}

// Optimized debug image saving
void BallDetector::saveDebugImagesAsync(const unsigned char* imageData, int width, int height,
    int frameIndex, const BallDetectionResult& result) {

    if (!m_params.saveIntermediateImages) return;

    std::string debugPath;
    if (!pImpl->m_currentCaptureFolder.empty()) {
        debugPath = pImpl->m_currentCaptureFolder + "/debug_images";
    }
    else if (!m_params.debugOutputDir.empty()) {
        debugPath = m_params.debugOutputDir;
    }
    else {
        debugPath = "./debug_images";
    }

    try {
        if (!fs::exists(debugPath)) {
            fs::create_directories(debugPath);
        }

        // Queue intermediate images for async saving
        pImpl->saveIntermediateImagesAsync(debugPath, frameIndex);

        // Save result image if ball found
        if (result.found) {
            std::string resultPath = debugPath + "/frame_" + std::to_string(frameIndex) + "_result.png";

            // Create result image in background
            std::thread([this, imageData, width, height, result, resultPath]() {
                SaveDetectionImage(imageData, width, height, result, resultPath, true);
                }).detach();
        }
    }
    catch (const fs::filesystem_error& e) {
        LOG_ERROR("Failed to save debug images: " + std::string(e.what()));
    }
}

// Original debug image saving (backward compatibility)
void BallDetector::saveDebugImages(const unsigned char* imageData, int width, int height,
    int frameIndex, const BallDetectionResult& result) {
    saveDebugImagesAsync(imageData, width, height, frameIndex, result);
}

// Optimized tracking update
void BallDetector::updateTrackingOptimized(std::vector<BallInfo>& validBalls) {
    if (validBalls.empty()) return;

    // Calculate motion scores in parallel
    tbb::parallel_for(tbb::blocked_range<size_t>(0, validBalls.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                float bestMotionScore = 0.0f;

                for (const auto& [trackId, track] : m_tracks) {
                    float motionScore = calculateMotionConsistency(validBalls[i], track);
                    if (motionScore > bestMotionScore) {
                        bestMotionScore = motionScore;
                    }
                }

                validBalls[i].motionScore = bestMotionScore;
                validBalls[i].confidence = validBalls[i].confidence * 0.8f + bestMotionScore * 0.2f;
            }
        }
    );
}


cv::Mat BallDetector::Impl::preprocessImage(const cv::Mat& grayImage, const DetectionParams& params) {
    if (!m_context) return grayImage;

    cv::Mat processed = grayImage;

    // Initialize all timing variables
    m_context->metrics.filterTime_ms = 0.0;
    m_context->metrics.claheTime_ms = 0.0;
    m_context->metrics.shadowEnhancementTime_ms = 0.0;
    m_context->metrics.sharpenTime_ms = 0.0;
    m_context->metrics.normalizationTime_ms = 0.0;

    double totalPreprocessingStart = std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000000.0;

    // Apply filters
    if (!params.fastMode) {
        MEASURE_TIME("Bilateral filter",
            cv::bilateralFilter(grayImage, m_context->tempMat1, 5, 50, 50);
            processed = m_context->tempMat1;
            , m_context->metrics.filterTime_ms);
    }
    else {
        MEASURE_TIME("Gaussian blur",
            cv::GaussianBlur(grayImage, m_context->tempMat1, cv::Size(3, 3), 0.75, 0.75, cv::BORDER_REPLICATE);
            processed = m_context->tempMat1;
            , m_context->metrics.filterTime_ms);
    }

    // Apply CLAHE if needed
    if (params.useCLAHE) {
        MEASURE_TIME("CLAHE",
            processed = applyCLAHE(processed, params.claheClipLimit);
            if (params.saveIntermediateImages) {
                m_lastCLAHE = processed.clone();
            }
            , m_context->metrics.claheTime_ms);
    }

    // Shadow enhancement
    if (params.useEnhanceShadows) {
        MEASURE_TIME("Shadow enhancement",
            processed = enhanceShadowRegionsOptimized(processed, params.shadowEnhanceFactor);
            , m_context->metrics.shadowEnhancementTime_ms);
    }

    // Sharpening
    if (!params.fastMode && params.contrastThreshold > 0) {
        MEASURE_TIME("Sharpening",
            cv::Mat blurred;
            cv::GaussianBlur(processed, blurred, cv::Size(0, 0), 1.0);
            cv::addWeighted(processed, 1.5, blurred, -0.5, 0, m_context->tempMat2);
            processed = m_context->tempMat2;
            , m_context->metrics.sharpenTime_ms);
    }

    // Normalize with timing
    if (params.useNormalization) {
        MEASURE_TIME("Normalization",
            cv::normalize(processed, processed, 0, 255, cv::NORM_MINMAX);
            , m_context->metrics.normalizationTime_ms);
    }

    if (params.saveIntermediateImages) {
        m_lastProcessedImage = processed.clone();
    }

    // Calculate total preprocessing time
    double totalPreprocessingEnd = std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000000.0;
    m_context->metrics.preprocessingTime_ms = totalPreprocessingEnd - totalPreprocessingStart;

    return processed;
}


cv::Mat BallDetector::Impl::applyCLAHE(const cv::Mat& image, double clipLimit) {
    static thread_local cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clipLimit, cv::Size(8, 8));
    clahe->setClipLimit(clipLimit);

    cv::Mat enhanced;
    clahe->apply(image, enhanced);
    return enhanced;
}

cv::Mat BallDetector::Impl::enhanceShadowRegionsOptimized(const cv::Mat& image, float factor) {
    cv::Mat result = image.clone();

    // Use pre-computed LUT for shadow enhancement
    if (m_shadowLUTInitialized) {
        cv::LUT(image, cv::Mat(1, 256, CV_8U, m_shadowLUT.data()), result);
    }
    else {
        // Fallback to original implementation
        cv::Scalar mean, stddev;
        cv::meanStdDev(image, mean, stddev);
        double avgBrightness = mean[0];
        double adaptiveThreshold = std::min(SHADOW_THRESHOLD, static_cast<int>(avgBrightness * 0.7));

        // Parallel processing using TBB
        tbb::parallel_for(tbb::blocked_range<int>(0, image.rows),
            [&](const tbb::blocked_range<int>& range) {
                for (int y = range.begin(); y != range.end(); ++y) {
                    const uchar* src = image.ptr<uchar>(y);
                    uchar* dst = result.ptr<uchar>(y);

                    // Process 4 pixels at a time using SIMD when possible
                    int x = 0;
                    for (; x <= image.cols - 4; x += 4) {
                        for (int i = 0; i < 4; ++i) {
                            if (src[x + i] < adaptiveThreshold) {
                                float enhancementRatio = 1.0f - (src[x + i] / static_cast<float>(adaptiveThreshold));
                                float enhancementFactor = SHADOW_ENHANCEMENT_BASE + factor * enhancementRatio;
                                dst[x + i] = cv::saturate_cast<uchar>(src[x + i] * enhancementFactor);
                            }
                            else {
                                dst[x + i] = src[x + i];
                            }
                        }
                    }

                    // Handle remaining pixels
                    for (; x < image.cols; ++x) {
                        if (src[x] < adaptiveThreshold) {
                            float enhancementRatio = 1.0f - (src[x] / static_cast<float>(adaptiveThreshold));
                            float enhancementFactor = SHADOW_ENHANCEMENT_BASE + factor * enhancementRatio;
                            dst[x] = cv::saturate_cast<uchar>(src[x] * enhancementFactor);
                        }
                    }
                }
            }
        );
    }

    if (m_paramsPtr && m_paramsPtr->saveIntermediateImages) {
        m_lastShadowEnhanced = result.clone();
    }

    return result;
}

cv::Mat BallDetector::Impl::computeEdgeMapOptimized(const cv::Mat& image, int method) {
    cv::Mat edges;

    if (method == 0) {
        // Optimized Canny edge detection
        cv::Canny(image, edges, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD, SOBEL_KERNEL_SIZE, true);
    }
    else {
        // Optimized Sobel
        cv::Mat gradX, gradY;
        cv::Sobel(image, gradX, CV_32F, 1, 0, SOBEL_KERNEL_SIZE, 1, 0, cv::BORDER_REPLICATE);
        cv::Sobel(image, gradY, CV_32F, 0, 1, SOBEL_KERNEL_SIZE, 1, 0, cv::BORDER_REPLICATE);

        // Magnitude calculation
        cv::Mat mag;
        cv::magnitude(gradX, gradY, mag);
        cv::convertScaleAbs(mag, edges);
    }

    if (m_paramsPtr && m_paramsPtr->saveIntermediateImages) {
        m_lastEdgeImage = edges.clone();
    }

    return edges;
}

std::vector<cv::Vec3f> BallDetector::Impl::detectCirclesHoughOptimized(const cv::Mat& image, const DetectionParams& params) {
    std::vector<cv::Vec3f> circles;
    circles.reserve(params.maxCandidates);

    int minRadius = params.minRadius;
    int maxRadius = params.maxRadius;
    if (params.downscaleFactor > 1) {
        minRadius = std::max(3, minRadius / params.downscaleFactor);
        maxRadius = maxRadius / params.downscaleFactor;
    }

    // Use HoughCircles with optimized parameters
    cv::HoughCircles(image, circles, cv::HOUGH_GRADIENT,
                     params.dp,
                     params.minDist,
                     params.param1,
                     params.param2,
                     minRadius,
                     maxRadius);

    // Limit candidates
    if (params.maxCandidates > 0 && circles.size() > static_cast<size_t>(params.maxCandidates)) {
        circles.resize(params.maxCandidates);
    }

    return circles;
}

bool BallDetector::Impl::quickValidateCircleOptimized(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    int cx = cvRound(circle[0]), cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Basic boundary check
    if (cx < 0 || cy < 0 || cx >= image.cols || cy >= image.rows) {
        return false;
    }

    // Color filter with boundary check
    if (params.useColorFilter) {
        if (image.at<uchar>(cy, cx) < params.brightnessThreshold * 0.7) {
            return false;
        }
    }

    // Contrast check - proper boundary checking for all sample points
    if (params.contrastThreshold > 0) {
        int innerR = radius / 2;
        int outerR = radius * 3 / 2;

        // Check if all points are within bounds
        if (cx - outerR < 0 || cx + outerR >= image.cols || cy - outerR < 0 || cy + outerR >= image.rows) {
            return true;
        }

        // Safe sampling with verified bounds
        int innerSum = 0, outerSum = 0;
        int innerCount = 0, outerCount = 0;

        // Inner samples
        if (cx - innerR >= 0) {
            innerSum += image.at<uchar>(cy, cx - innerR);
            innerCount++;
        }
        if (cx + innerR < image.cols) {
            innerSum += image.at<uchar>(cy, cx + innerR);
            innerCount++;
        }
        if (cy - innerR >= 0) {
            innerSum += image.at<uchar>(cy - innerR, cx);
            innerCount++;
        }
        if (cy + innerR < image.rows) {
            innerSum += image.at<uchar>(cy + innerR, cx);
            innerCount++;
        }

        // Outer samples
        if (cx - outerR >= 0) {
            outerSum += image.at<uchar>(cy, cx - outerR);
            outerCount++;
        }
        if (cx + outerR < image.cols) {
            outerSum += image.at<uchar>(cy, cx + outerR);
            outerCount++;
        }
        if (cy - outerR >= 0) {
            outerSum += image.at<uchar>(cy - outerR, cx);
            outerCount++;
        }
        if (cy + outerR < image.rows) {
            outerSum += image.at<uchar>(cy + outerR, cx);
            outerCount++;
        }

        if (innerCount > 0 && outerCount > 0) {
            float avgInner = static_cast<float>(innerSum) / innerCount;
            float avgOuter = static_cast<float>(outerSum) / outerCount;
            float avgDiff = std::abs(avgInner - avgOuter);

            if (avgDiff < params.contrastThreshold) {
                return false;
            }
        }
    }

    return true;
}

float BallDetector::Impl::calculateConfidenceOptimized(const cv::Mat& image, const cv::Vec3f& circle,
    const DetectionParams& params, const cv::Mat& edgeMap) {

    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Initialize scores
    float scores[6] = { 0.0f };  // size, contrast, brightness, edge, symmetry, shape
    const float weights[6] = { 0.10f, 0.30f, 0.15f, 0.25f, 0.10f, 0.10f };

    // 1. Size score
    float sizeRange = static_cast<float>(params.maxRadius - params.minRadius);
    float optimalRadius = params.minRadius + sizeRange * 0.5f;
    scores[0] = 1.0f - std::min(1.0f, std::abs(radius - optimalRadius) / (sizeRange * 0.5f));

    // 2 & 3. Contrast and brightness scores - combined ROI calculation
    cv::Rect innerRect(cx - radius / 3, cy - radius / 3, 2 * radius / 3, 2 * radius / 3);
    innerRect &= cv::Rect(0, 0, image.cols, image.rows);

    if (innerRect.area() > 0) {
        cv::Mat innerRegion = image(innerRect);
        cv::Scalar innerMean, innerStdDev;
        cv::meanStdDev(innerRegion, innerMean, innerStdDev);

        // Contrast calculation - simplified with safe boundary checking
        cv::Rect outerRect(cx - radius, cy - radius, 2 * radius, 2 * radius);
        outerRect &= cv::Rect(0, 0, image.cols, image.rows);

        if (outerRect.area() > innerRect.area()) {
            std::vector<int> samples;

            // Safe sampling with boundary checks
            auto addSample = [&](int y, int x) {
                if (y >= 0 && y < image.rows && x >= 0 && x < image.cols) {
                    samples.push_back(image.at<uchar>(y, x));
                }
                };

            addSample(cy - radius, cx);
            addSample(cy + radius, cx);
            addSample(cy, cx - radius);
            addSample(cy, cx + radius);
            addSample(cy - radius / 2, cx - radius / 2);
            addSample(cy - radius / 2, cx + radius / 2);
            addSample(cy + radius / 2, cx - radius / 2);
            addSample(cy + radius / 2, cx + radius / 2);

            if (!samples.empty()) {
                double outerMean = 0;
                for (int val : samples) outerMean += val;
                outerMean /= samples.size();

                double contrastDiff = std::abs(innerMean[0] - outerMean);
                scores[1] = contrastDiff < 20.0f ? static_cast<float>(contrastDiff / 40.0f) : std::min(1.0f, static_cast<float>(contrastDiff / 100.0f));
            }

            // Brightness consistency
            double brightnessVariance = innerStdDev[0];
            scores[2] = brightnessVariance > 40.0 ?
                (1.0f - std::min(1.0f, static_cast<float>(brightnessVariance / 60.0))) :
                (1.0f - std::min(1.0f, static_cast<float>(brightnessVariance / 50.0)));
        }
    }

    // 4. Edge score - optimized
    if (!edgeMap.empty()) {
        scores[3] = calculateEdgeStrengthOptimized(edgeMap, circle);
        if (scores[3] > 0.6f) {
            scores[3] = std::min(1.0f, scores[3] * 1.2f);
        }
    }

    // 5. Symmetry score - Fixed: proper boundary checking
    const int numSamples = 4;
    float radialSum = 0;
    float radialSqSum = 0;
    int validSamples = 0;

    for (int i = 0; i < numSamples; ++i) {
        double angle = 2 * CV_PI * i / numSamples;
        int sx = cx + static_cast<int>(radius * 0.7 * cos(angle));
        int sy = cy + static_cast<int>(radius * 0.7 * sin(angle));

        if (sx >= 0 && sx < image.cols && sy >= 0 && sy < image.rows) {
            float val = static_cast<float>(image.at<uchar>(sy, sx));
            radialSum += val;
            radialSqSum += val * val;
            validSamples++;
        }
    }

    if (validSamples > 0) {
        float radialMean = radialSum / validSamples;
        float variance = (radialSqSum / validSamples) - (radialMean * radialMean);
        float stdDev = sqrt(variance);

        scores[4] = stdDev > 30.0f ?
            (1.0f - std::min(1.0f, stdDev / 40.0f)) :
            (1.0f - std::min(1.0f, stdDev / 50.0f));
    }

    // 6. Shape score
    scores[5] = 0.8f;

    // Calculate weighted sum
    float weightedSum = 0.0f;
    float totalWeight = 0.0f;
    for (int i = 0; i < 6; ++i) {
        if (scores[i] > 0) {
            weightedSum += scores[i] * weights[i];
            totalWeight += weights[i];
        }
    }

    float finalConfidence = totalWeight > 0 ? (weightedSum / totalWeight) : 0.3f;

    // Apply penalties
    if (scores[1] < 0.3f || scores[3] < 0.2f) {
        finalConfidence *= 0.8f;
    }

    return finalConfidence;
}

float BallDetector::Impl::calculateEdgeStrengthOptimized(const cv::Mat& edgeMap, const cv::Vec3f& circle) {
    // Check if edgeMap is valid
    if (edgeMap.empty()) {
        return 0.0f;
    }

    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Sample fewer points for efficiency
    const int numSamples = 16;  // Reduced from 36
    int edgeCount = 0;

    for (int i = 0; i < numSamples; ++i) {
        double angle = 2 * CV_PI * i / numSamples;
        int x = cx + static_cast<int>(radius * cos(angle));
        int y = cy + static_cast<int>(radius * sin(angle));

        // Fixed: More robust boundary checking
        if (x >= 1 && x < edgeMap.cols - 1 && y >= 1 && y < edgeMap.rows - 1) {
            // Check 3x3 area
            bool edgeFound = false;
            for (int dy = -1; dy <= 1 && !edgeFound; ++dy) {
                for (int dx = -1; dx <= 1 && !edgeFound; ++dx) {
                    if (edgeMap.at<uchar>(y + dy, x + dx) > 0) {
                        edgeFound = true;
                        edgeCount++;
                    }
                }
            }
        }
    }

    return static_cast<float>(edgeCount) / numSamples;
}

std::vector<BallInfo> BallDetector::Impl::evaluateCandidatesOptimized(const cv::Mat& image,
    const std::vector<cv::Vec3f>& candidates,
    const DetectionParams& params,
    float scaleFactor,
    const cv::Rect& roiRect,
    int frameIndex,
    const cv::Mat& edgeMap) {

    // Pre-allocate result vector
    tbb::concurrent_vector<BallInfo> concurrentValidBalls;
    concurrentValidBalls.reserve(candidates.size() / 2);

    // Process candidates in parallel
    tbb::parallel_for(tbb::blocked_range<size_t>(0, candidates.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                const cv::Vec3f& circle = candidates[i];

                // Quick validation
                if (!quickValidateCircleOptimized(image, circle, params)) {
                    continue;
                }

                BallInfo info;
                info.center.x = circle[0] * scaleFactor;
                info.center.y = circle[1] * scaleFactor;
                if (params.useROI) {
                    info.center.x += roiRect.x;
                    info.center.y += roiRect.y;
                }
                info.radius = circle[2] * scaleFactor;
                info.frameIndex = frameIndex;

                // Calculate confidence
                info.confidence = calculateConfidenceOptimized(image, circle, params, edgeMap);

                // Brightness calculation with proper bounds checking
                int cx = cvRound(circle[0]);
                int cy = cvRound(circle[1]);
                int r = cvRound(circle[2]);

                cv::Rect ballRect(cx - r / 2, cy - r / 2, r, r);
                ballRect &= cv::Rect(0, 0, image.cols, image.rows);

                if (ballRect.area() > 0) {
                    info.brightness = static_cast<float>(cv::mean(image(ballRect))[0]);

                    // Apply brightness penalty
                    if (info.brightness < 50.0f) {
                        info.confidence *= 0.7f;
                    }
                    else if (info.brightness < 80.0f) {
                        info.confidence *= 0.85f;
                    }
                }

                // Circularity check if needed
                if (params.useCircularityCheck && !params.fastMode) {
                    info.circularity = calculateCircularityOptimized(image, circle, params);
                    if (info.circularity < params.minCircularity) {
                        continue;
                    }

                    if (info.circularity > 0.85f) {
                        info.confidence *= 1.1f;
                    }
                }
                else {
                    info.circularity = 1.0f;
                }

                if (!edgeMap.empty()) {
                    info.edgeStrength = calculateEdgeStrengthOptimized(edgeMap, circle);

                    if (info.edgeStrength < 0.3f) {
                        info.confidence *= 0.8f;
                    }
                }

                // Ensure confidence doesn't exceed 1.0
                info.confidence = std::min(1.0f, info.confidence);

                // Only accept high confidence candidates
                if (info.confidence >= MIN_CONFIDENCE_THRESHOLD) {
                    concurrentValidBalls.push_back(info);
                }
            }
        }
    );

    // Convert to regular vector
    std::vector<BallInfo> validBalls(concurrentValidBalls.begin(), concurrentValidBalls.end());
    return validBalls;
}

float BallDetector::Impl::calculateCircularityOptimized(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Define ROI
    const int margin = 3;
    cv::Rect roi(cx - radius - margin, cy - radius - margin, (radius + margin) * 2, (radius + margin) * 2);
    roi &= cv::Rect(0, 0, image.cols, image.rows);

    if (roi.area() == 0) return 0.0f;

    cv::Mat roiImage = image(roi);

    // Quick threshold
    cv::Mat binary;
    cv::threshold(roiImage, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Find largest contour
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return 0.0f;

    // Find contour closest to expected center
    cv::Point roiCenter(cx - roi.x, cy - roi.y);
    size_t bestIdx = 0;
    double minDist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Moments M = cv::moments(contours[i]);
        if (M.m00 > 0) {
            cv::Point centroid(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
            double dist = cv::norm(centroid - roiCenter);
            if (dist < minDist) {
                minDist = dist;
                bestIdx = i;
            }
        }
    }

    // Calculate circularity
    double area = cv::contourArea(contours[bestIdx]);
    double perimeter = cv::arcLength(contours[bestIdx], true);

    if (perimeter <= 0) return 0.0f;

    return static_cast<float>(std::min(1.0, 4.0 * CV_PI * area / (perimeter * perimeter)));
}

cv::Mat BallDetector::Impl::extractROIOptimized(const cv::Mat& image, float scale) {
    int roiWidth = static_cast<int>(image.cols * scale);
    int roiHeight = static_cast<int>(image.rows * scale);

    // Ensure even dimensions
    roiWidth = (roiWidth / 2) * 2;
    roiHeight = (roiHeight / 2) * 2;

    int x = (image.cols - roiWidth) / 2;
    int y = (image.rows - roiHeight) / 2;

    cv::Rect roiRect(x, y, roiWidth, roiHeight);
    roiRect &= cv::Rect(0, 0, image.cols, image.rows);

    // Return ROI without copying
    return image(roiRect);
}

void BallDetector::Impl::saveIntermediateImagesAsync(const std::string& basePath, int frameIndex) {
    if (basePath.empty()) return;

    std::string prefix = basePath + "/frame_" + std::to_string(frameIndex);

    std::lock_guard<std::mutex> lock(m_saveQueueMutex);

    // Queue images for saving
    if (!m_lastProcessedImage.empty()) {
        m_saveQueue.push({ m_lastProcessedImage.clone(), prefix + "_01_processed.png" });
    }
    if (!m_lastBinaryImage.empty()) {
        m_saveQueue.push({ m_lastBinaryImage.clone(), prefix + "_02_binary.png" });
    }
    if (!m_lastShadowEnhanced.empty()) {
        m_saveQueue.push({ m_lastShadowEnhanced.clone(), prefix + "_03_shadow_enhanced.png" });
    }
    if (!m_lastCLAHE.empty()) {
        m_saveQueue.push({ m_lastCLAHE.clone(), prefix + "_04_clahe.png" });
    }
    if (!m_lastEdgeImage.empty()) {
        m_saveQueue.push({ m_lastEdgeImage.clone(), prefix + "_05_edges.png" });
    }

    m_saveCV.notify_one();
}

cv::Mat BallDetector::Impl::getMatFromPoolOptimized(int rows, int cols, int type) {
    std::lock_guard<std::mutex> lock(m_poolMutex);

    // Clean up old entries periodically
    static auto lastCleanup = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (now - lastCleanup > POOL_CLEANUP_INTERVAL) {
        cleanupMatPool();
        lastCleanup = now;
    }

    // Search for matching Mat
    for (auto it = m_matPool.begin(); it != m_matPool.end(); ++it) {
        if (it->mat.rows == rows && it->mat.cols == cols && it->mat.type() == type) {
            cv::Mat mat = it->mat;
            m_matPool.erase(it);
            return mat;
        }
    }

    // Create new Mat if not found
    return cv::Mat(rows, cols, type);
}

void BallDetector::Impl::returnMatToPool(cv::Mat& mat) {
    if (!mat.empty() && mat.isContinuous()) {
        std::lock_guard<std::mutex> lock(m_poolMutex);
        if (m_matPool.size() < MAX_POOL_SIZE) {
            m_matPool.push_back({ mat, std::chrono::steady_clock::now() });
        }
    }
}

void BallDetector::Impl::cleanupMatPool() {
    auto now = std::chrono::steady_clock::now();
    m_matPool.erase(
        std::remove_if(m_matPool.begin(), m_matPool.end(),
            [&now](const MatPoolEntry& entry) {
                return (now - entry.lastUsed) > std::chrono::seconds(60);
            }),
        m_matPool.end()
    );
}

// Tracking functions
float BallDetector::calculateMotionConsistency(const BallInfo& ball, const TrackingInfo& track) {
    if (track.history.empty()) {
        return 0.0f;
    }

    float distance = static_cast<float>(cv::norm(ball.center - track.predictedCenter));
    float maxDist = m_params.maxTrackingDistance;
    float distanceScore = 1.0f - std::min(1.0f, distance / maxDist);

    float radiusDiff = std::abs(ball.radius - track.predictedRadius);
    float maxRadiusDiff = track.predictedRadius * 0.3f;
    float sizeScore = 1.0f - std::min(1.0f, radiusDiff / maxRadiusDiff);

    float velocityScore = 1.0f;
    if (track.history.size() >= 3) {
        const auto& h1 = track.history[track.history.size() - 1];
        const auto& h2 = track.history[track.history.size() - 2];
        const auto& h3 = track.history[track.history.size() - 3];

        cv::Point2f v1 = h1.center - h2.center;
        cv::Point2f v2 = h2.center - h3.center;
        cv::Point2f currentVel = ball.center - h1.center;

        float velChange1 = static_cast<float>(cv::norm(currentVel - v1));
        float velChange2 = static_cast<float>(cv::norm(v1 - v2));
        float avgVelChange = (velChange1 + velChange2) / 2.0f;

        velocityScore = 1.0f - std::min(1.0f, avgVelChange / 20.0f);
    }

    return distanceScore * 0.5f + sizeScore * 0.3f + velocityScore * 0.2f;
}

void BallDetector::updateTracking(const BallDetectionResult& result) {
    if (!result.found || result.balls.empty()) {
        for (auto it = m_tracks.begin(); it != m_tracks.end(); ) {
            it->second.consecutiveDetections = 0;
            if (it->second.history.size() > m_params.trackingHistorySize) {
                it = m_tracks.erase(it);
            }
            else {
                ++it;
            }
        }
        return;
    }

    std::vector<bool> ballAssigned(result.balls.size(), false);

    for (auto& [trackId, track] : m_tracks) {
        float bestScore = 0.0f;
        int bestBallIdx = -1;

        for (size_t i = 0; i < result.balls.size(); ++i) {
            if (!ballAssigned[i]) {
                float score = calculateMotionConsistency(result.balls[i], track);
                if (score > bestScore && score > 0.5f) {
                    bestScore = score;
                    bestBallIdx = static_cast<int>(i);
                }
            }
        }

        if (bestBallIdx >= 0) {
            ballAssigned[bestBallIdx] = true;
            track.history.push_back(result.balls[bestBallIdx]);
            if (track.history.size() > m_params.trackingHistorySize) {
                track.history.erase(track.history.begin());
            }
            track.consecutiveDetections++;
            predictNextPosition(track);
        }
        else {
            track.consecutiveDetections = 0;
        }
    }

    for (size_t i = 0; i < result.balls.size(); ++i) {
        if (!ballAssigned[i]) {
            TrackingInfo newTrack;
            newTrack.trackId = m_nextTrackId++;
            newTrack.history.push_back(result.balls[i]);
            newTrack.consecutiveDetections = 1;
            newTrack.predictedCenter = result.balls[i].center;
            newTrack.predictedRadius = result.balls[i].radius;

            m_tracks[newTrack.trackId] = newTrack;
        }
    }

    for (auto it = m_tracks.begin(); it != m_tracks.end(); ) {
        if (it->second.consecutiveDetections == 0 &&
            it->second.history.size() < 3) {
            it = m_tracks.erase(it);
        }
        else {
            ++it;
        }
    }
}

void BallDetector::predictNextPosition(TrackingInfo& track) {
    if (track.history.size() < 2) {
        if (!track.history.empty()) {
            track.predictedCenter = track.history.back().center;
            track.predictedRadius = track.history.back().radius;
        }
        return;
    }

    const auto& current = track.history.back();
    const auto& previous = track.history[track.history.size() - 2];

    cv::Point2f velocity = current.center - previous.center;
    track.predictedCenter = current.center + velocity;

    float radiusChange = current.radius - previous.radius;
    track.predictedRadius = current.radius + radiusChange * 0.5f;

    track.predictedRadius = std::max(static_cast<float>(m_params.minRadius),
        std::min(static_cast<float>(m_params.maxRadius),
            track.predictedRadius));
}

// Template functions
bool BallDetector::InitializeTemplate(const cv::Mat& templateImage) {
    if (templateImage.empty()) {
        LOG_ERROR("Template image is empty");
        return false;
    }

    if (templateImage.channels() > 1) {
        cv::cvtColor(templateImage, m_ballTemplate, cv::COLOR_BGR2GRAY);
    }
    else {
        m_ballTemplate = templateImage.clone();
    }

    cv::GaussianBlur(m_ballTemplate, m_ballTemplate, cv::Size(3, 3), 0.75);

    m_templateInitialized = true;
    LOG_INFO("Ball template initialized: " + std::to_string(m_ballTemplate.cols) + "x" +
        std::to_string(m_ballTemplate.rows));

    return true;
}

void BallDetector::ClearTemplate() {
    m_ballTemplate.release();
    m_templateInitialized = false;
    LOG_INFO("Ball template cleared");
}

void BallDetector::ResetTracking() {
    m_tracks.clear();
    m_nextTrackId = 0;
    LOG_INFO("Tracking state reset");
}

std::vector<BallDetector::TrackingInfo> BallDetector::GetActiveTracks() const {
    std::vector<TrackingInfo> activeTracks;
    for (const auto& [trackId, track] : m_tracks) {
        if (track.consecutiveDetections > 0) {
            activeTracks.push_back(track);
        }
    }
    return activeTracks;
}

// Calibration function
bool BallDetector::CalibrateForBallSize(const std::vector<cv::Mat>& sampleImages,
    float knownBallDiameter_mm) {
    if (sampleImages.empty()) {
        LOG_ERROR("No sample images provided for calibration");
        return false;
    }

    std::vector<float> detectedRadii;

    for (size_t i = 0; i < sampleImages.size(); ++i) {
        cv::Mat gray = sampleImages[i];
        if (sampleImages[i].channels() > 1) {
            cv::cvtColor(sampleImages[i], gray, cv::COLOR_BGR2GRAY);
        }

        BallDetectionResult result = DetectBall(gray.data, gray.cols, gray.rows, static_cast<int>(i));
        if (result.found && !result.balls.empty()) {
            detectedRadii.push_back(result.balls[0].radius);
        }
    }

    if (detectedRadii.empty()) {
        LOG_ERROR("No balls detected in calibration images");
        return false;
    }

    float avgRadius = std::accumulate(detectedRadii.begin(), detectedRadii.end(), 0.0f) / detectedRadii.size();

    float radiusRange = avgRadius * 0.4f;
    m_params.minRadius = static_cast<int>(avgRadius - radiusRange);
    m_params.maxRadius = static_cast<int>(avgRadius + radiusRange);

    LOG_INFO("Calibration complete: Ball radius range set to [" +
        std::to_string(m_params.minRadius) + ", " +
        std::to_string(m_params.maxRadius) + "] pixels");
    LOG_INFO("Estimated pixel size: " +
        std::to_string(knownBallDiameter_mm / (avgRadius * 2)) + " mm/pixel");

    return true;
}


// Visualization
bool BallDetector::SaveDetectionImage(const unsigned char* originalImage, int width, int height,
    const BallDetectionResult& result, const std::string& outputPath, bool saveAsColor) {
    if (!originalImage || width <= 0 || height <= 0) {
        LOG_ERROR("Invalid parameters for SaveDetectionImage");
        return false;
    }

    try {
        cv::Mat grayImg(height, width, CV_8UC1, const_cast<unsigned char*>(originalImage));
        cv::Mat colorImg;
        cv::cvtColor(grayImg, colorImg, cv::COLOR_GRAY2BGR);

        // Draw header
        cv::rectangle(colorImg, cv::Point(0, 0), cv::Point(width, 50), cv::Scalar(40, 40, 40), cv::FILLED);
        cv::putText(colorImg, "Ball Detection Result", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

        if (result.found && !result.balls.empty()) {
            int infoY = 70;

            for (size_t i = 0; i < result.balls.size() && i < MAX_DISPLAY_BALLS; ++i) {
                const BallInfo& ball = result.balls[i];
                cv::Point center(cvRound(ball.center.x), cvRound(ball.center.y));
                int radius = cvRound(ball.radius);

                // Draw semi-transparent circle
                cv::Mat overlay = colorImg.clone();
                cv::circle(overlay, center, radius, cv::Scalar(0, 0, 255), -1);
                cv::addWeighted(colorImg, 0.7, overlay, 0.3, 0, colorImg);

                // Draw circle outline
                cv::circle(colorImg, center, radius, cv::Scalar(0, 0, 255), 3);
                cv::drawMarker(colorImg, center, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 20, 2);

                // Draw info panel
                cv::Rect infoBg(10, infoY - 20, INFO_PANEL_WIDTH, INFO_PANEL_HEIGHT);
                infoBg &= cv::Rect(0, 0, width, height);
                cv::rectangle(colorImg, infoBg, cv::Scalar(20, 20, 20), cv::FILLED);
                cv::rectangle(colorImg, infoBg, cv::Scalar(100, 100, 100), 1);

                std::ostringstream info;
                info << "Ball " << (i + 1) << ":";
                cv::putText(colorImg, info.str(), cv::Point(15, infoY), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

                info.str("");
                info << "  Center: (" << cvRound(ball.center.x) << ", " << cvRound(ball.center.y) << ")";
                cv::putText(colorImg, info.str(), cv::Point(15, infoY + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                info.str("");
                info << "  Radius: " << radius << " pixels";
                cv::putText(colorImg, info.str(), cv::Point(15, infoY + 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                info.str("");
                info << "  Confidence: " << std::fixed << std::setprecision(1) << (ball.confidence * 100) << "%";
                cv::Scalar confColor = (ball.confidence > 0.8) ? cv::Scalar(0, 255, 0) :
                                       (ball.confidence > 0.6) ? cv::Scalar(0, 255, 255) :
                                                                 cv::Scalar(0, 165, 255);
                cv::putText(colorImg, info.str(), cv::Point(15, infoY + 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, confColor, 1);

                infoY += 100;
            }

            // Draw summary footer
            cv::Rect summaryBg(0, height - 30, width, 30);
            cv::rectangle(colorImg, summaryBg, cv::Scalar(40, 40, 40), cv::FILLED);

            std::ostringstream summary;
            summary << "Total balls: " << result.balls.size() << " | Frame: " << (result.balls.empty() ? 0 : result.balls[0].frameIndex);
            cv::putText(colorImg, summary.str(), cv::Point(10, height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
        else {
            // No ball detected
            cv::Rect msgBg(10, 60, 200, 40);
            cv::rectangle(colorImg, msgBg, cv::Scalar(0, 0, 100), cv::FILLED);
            cv::putText(colorImg, "No ball detected", cv::Point(15, 85), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        // Add timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        struct tm localTime;
        localtime_s(&localTime, &t);

        std::ostringstream timestamp;
        timestamp << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S");
        cv::putText(colorImg, timestamp.str(), cv::Point(width - 180, height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        // Save image
        bool success = false;
        if (saveAsColor) {
            success = cv::imwrite(outputPath, colorImg);
        }
        else {
            cv::Mat finalGray;
            cv::cvtColor(colorImg, finalGray, cv::COLOR_BGR2GRAY);
            success = cv::imwrite(outputPath, finalGray);
        }

        if (success) {
            LOG_DEBUG("Detection result image saved: " + outputPath);
        }
        else {
            LOG_ERROR("Failed to save detection image: " + outputPath);
        }

        return success;
    }
    catch (const cv::Exception& e) {
        LOG_ERROR("OpenCV error in SaveDetectionImage: " + std::string(e.what()));
        return false;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Error in SaveDetectionImage: " + std::string(e.what()));
        return false;
    }
}