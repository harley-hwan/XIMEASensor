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
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>

namespace fs = std::filesystem;

// === Constants ===
namespace {
    // Image processing constraints
    constexpr int MIN_DOWNSCALE_WIDTH = 160;
    constexpr int MIN_DOWNSCALE_HEIGHT = 120;
    constexpr float MIN_CONFIDENCE_THRESHOLD = 0.4f;

    // Shadow enhancement parameters
    constexpr int SHADOW_THRESHOLD = 100;
    constexpr float SHADOW_ENHANCEMENT_BASE = 1.0f;

    // Performance and display
    constexpr int PERFORMANCE_LOG_INTERVAL = 100;
    constexpr int MAX_DISPLAY_BALLS = 5;
    constexpr int INFO_PANEL_HEIGHT = 90;
    constexpr int INFO_PANEL_WIDTH = 280;

    // Edge detection parameters
    constexpr double CANNY_LOW_THRESHOLD = 50.0;
    constexpr double CANNY_HIGH_THRESHOLD = 150.0;
    constexpr int SOBEL_KERNEL_SIZE = 3;

    // Template matching parameters
    constexpr float TEMPLATE_MATCH_THRESHOLD = 0.7f;
    constexpr int TEMPLATE_SCALES = 5;
    constexpr float TEMPLATE_SCALE_STEP = 0.1f;
}

// === Performance Profiling Macro ===
#ifdef ENABLE_PERFORMANCE_PROFILING
#define MEASURE_TIME(name, code, metricsVar) \
    { \
        auto start = std::chrono::high_resolution_clock::now(); \
        code \
        auto end = std::chrono::high_resolution_clock::now(); \
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count(); \
        metricsVar = duration / 1000.0; \
        if (m_performanceProfilingEnabled) { \
            LOG_DEBUG(std::string(name) + " took " + std::to_string(duration) + " us (" + \
                      std::to_string(duration / 1000.0) + " ms)"); \
        } \
    }
#else
#define MEASURE_TIME(name, code, metricsVar) { code }
#endif

// Runtime-controlled debug image saving macro
#define DEBUG_IMAGE_TIME_MEASURE(name, code, metricsVar) \
    if (m_params.saveIntermediateImages) { \
        MEASURE_TIME(name, code, metricsVar) \
    } else { \
        metricsVar = 0.0; \
    }

// === Constructor Implementation ===
BallDetector::DetectionParams::DetectionParams() {
    // Circle detection parameters
    minRadius = 20;
    maxRadius = 30;
    minCircularity = 0.70f;

    // Hough parameters
    dp = 2.0;
    minDist = 40.0;
    param1 = 130.0;
    param2 = 0.87;

    // Thresholding parameters
    brightnessThreshold = 120;
    useAdaptiveThreshold = false;
    adaptiveBlockSize = 51;
    adaptiveConstant = 5.0;

    // Validation parameters
    useColorFilter = false;
    useCircularityCheck = false;
    contrastThreshold = 20.0f;
    detectMultiple = false;
    edgeThreshold = 30.0f;

    // Preprocessing parameters
    skipPreprocessing = false;
    useEnhanceShadows = true;
    shadowEnhanceFactor = 0.7f;
    useMorphology = false;
    useNormalization = true;
    useCLAHE = false;   // 더 잡다한게 잡힘
    claheClipLimit = 2.0;

    // Detection method control
    useContourDetection = true;
    useHoughDetection = true;
    useThresholding = true;
    useTemplateMatching = false;

    // Performance parameters
    fastMode = true;
    useROI = true;
    roiScale = 0.75f;
    downscaleFactor = 2;
    useParallel = true;
    maxCandidates = 15;
    processingThreads = std::max(2u, std::thread::hardware_concurrency() / 2);
    useGPU = false;

    // Debug parameters
    saveIntermediateImages = false;
    debugOutputDir = "";
    enableProfiling = false;

    // Tracking parameters
    enableTracking = false;
    maxTrackingDistance = 50.0f;
    trackingHistorySize = 10;
}

// === PerformanceMetrics Reset ===
void BallDetector::PerformanceMetrics::Reset() {
    totalDetectionTime_ms = 0;
    roiExtractionTime_ms = 0;
    downscaleTime_ms = 0;
    preprocessingTime_ms = 0;
    claheTime_ms = 0;
    thresholdingTime_ms = 0;
    morphologyTime_ms = 0;
    contourDetectionTime_ms = 0;
    houghDetectionTime_ms = 0;
    templateMatchingTime_ms = 0;
    candidateEvaluationTime_ms = 0;
    trackingTime_ms = 0;
    imagesSavingTime_ms = 0;
    candidatesFound = 0;
    candidatesEvaluated = 0;
    candidatesRejected = 0;
    ballDetected = false;
    averageConfidence = 0.0f;
}

// === BallDetector::Impl Class Implementation ===
class BallDetector::Impl {
public:
    // Debug image buffers
    cv::Mat m_lastProcessedImage;
    cv::Mat m_lastBinaryImage;
    cv::Mat m_lastShadowEnhanced;
    cv::Mat m_lastCLAHE;
    cv::Mat m_lastEdgeImage;
    cv::Mat m_downscaledImage;

    std::string m_currentCaptureFolder;
    const DetectionParams* m_paramsPtr;

    // Memory pool for efficient allocation
    std::vector<cv::Mat> m_matPool;
    std::mutex m_poolMutex;

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

    Impl() : m_currentCaptureFolder(""), m_paramsPtr(nullptr) {
        m_saveThreadRunning = true;
        m_saveThread = std::thread(&Impl::saveWorker, this);
        m_matPool.reserve(10);
    }

    ~Impl() {
        m_saveThreadRunning = false;
        m_saveCV.notify_all();
        if (m_saveThread.joinable()) {
            m_saveThread.join();
        }
    }

    void setCurrentCaptureFolder(const std::string& folder) { m_currentCaptureFolder = folder; }
    void setParamsReference(const DetectionParams* params) { m_paramsPtr = params; }

    cv::Mat preprocessImage(const cv::Mat& grayImage, const DetectionParams& params);
    cv::Mat applyCLAHE(const cv::Mat& image, double clipLimit);
    cv::Mat enhanceShadowRegions(const cv::Mat& image, float factor);
    cv::Mat computeEdgeMap(const cv::Mat& image, int method = 0);
    std::vector<cv::Vec3f> detectCirclesHough(const cv::Mat& image, const DetectionParams& params);
    bool quickValidateCircle(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
    float calculateConfidence(const cv::Mat& image, const cv::Vec3f& circle,
        const DetectionParams& params, const cv::Mat& edgeMap = cv::Mat());
    float calculateCircularity(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
    float calculateEdgeStrength(const cv::Mat& edgeMap, const cv::Vec3f& circle);
    cv::Mat extractROI(const cv::Mat& image, float scale);
    std::vector<BallInfo> evaluateCandidatesTBB(const cv::Mat& image,
        const std::vector<cv::Vec3f>& candidates,
        const DetectionParams& params,
        float scaleFactor,
        const cv::Rect& roiRect,
        int frameIndex,
        const cv::Mat& edgeMap = cv::Mat());
    void saveIntermediateImages(const std::string& basePath, int frameIndex);
    cv::Mat getMatFromPool(int rows, int cols, int type);
    void returnMatToPool(cv::Mat& mat);

private:
    void saveWorker() {
        while (m_saveThreadRunning.load()) {
            std::unique_lock<std::mutex> lock(m_saveQueueMutex);
            m_saveCV.wait(lock, [this] {
                return !m_saveQueue.empty() || !m_saveThreadRunning.load();
                });

            while (!m_saveQueue.empty()) {
                SaveTask task = std::move(m_saveQueue.front());
                m_saveQueue.pop();
                lock.unlock();

                try {
                    cv::imwrite(task.path, task.image);
                    LOG_DEBUG("Saved image: " + task.path);
                }
                catch (const cv::Exception& e) {
                    LOG_ERROR("Failed to save image: " + std::string(e.what()));
                }

                lock.lock();
            }
        }
    }
};

// === BallDetector Constructor ===
BallDetector::BallDetector()
    : m_params(),
    pImpl(std::make_unique<Impl>()),
    m_performanceProfilingEnabled(false),
    m_nextTrackId(0),
    m_templateInitialized(false) {

    cv::setNumThreads(0);
    cv::setUseOptimized(true);

    LOG_INFO("BallDetector initialized with TBB optimization");
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
    LOG_INFO("BallDetector parameters reset to default values");
}

void BallDetector::EnablePerformanceProfiling(bool enable) {
    m_performanceProfilingEnabled = enable;
    LOG_INFO("BallDetector performance profiling " + std::string(enable ? "ENABLED" : "DISABLED"));
}

void BallDetector::SetCurrentCaptureFolder(const std::string& folder) {
    if (pImpl) {
        pImpl->setCurrentCaptureFolder(folder);
        LOG_DEBUG("Current capture folder set to: " + folder);
    }
}

// === Main Detection Function ===
BallDetectionResult BallDetector::DetectBall(const unsigned char* imageData, int width, int height, int frameIndex) {
    auto totalStartTime = std::chrono::high_resolution_clock::now();
    m_lastMetrics.Reset();
    pImpl->setParamsReference(&m_params);

    BallDetectionResult result;
    result.found = false;
    result.balls.clear();

    // Input validation
    if (!imageData || width <= 0 || height <= 0) {
        result.errorMessage = "Invalid input parameters";
        LOG_ERROR("DetectBall: " + result.errorMessage);
        return result;
    }

    try {
        // Wrap input image without copying
        cv::Mat grayImage(height, width, CV_8UC1, const_cast<unsigned char*>(imageData));

        cv::Mat workingImage = grayImage;
        cv::Rect roiRect(0, 0, width, height);
        float scaleFactor = 1.0f;

        // === Stage 1: ROI Extraction ===
        if (m_params.useROI && m_params.roiScale < 1.0f) {
            MEASURE_TIME("ROI extraction",
                workingImage = pImpl->extractROI(grayImage, m_params.roiScale);
            int offsetX = (width - workingImage.cols) / 2;
            int offsetY = (height - workingImage.rows) / 2;
            roiRect = cv::Rect(offsetX, offsetY, workingImage.cols, workingImage.rows);
            , m_lastMetrics.roiExtractionTime_ms);
        }

        // === Stage 2: Downscaling ===
        if (m_params.fastMode && m_params.downscaleFactor > 1) {
            int newWidth = workingImage.cols / m_params.downscaleFactor;
            int newHeight = workingImage.rows / m_params.downscaleFactor;

            if (newWidth >= MIN_DOWNSCALE_WIDTH && newHeight >= MIN_DOWNSCALE_HEIGHT) {
                MEASURE_TIME("Image downscaling",
                    cv::Mat downscaled;
                cv::resize(workingImage, downscaled, cv::Size(newWidth, newHeight), 0, 0, cv::INTER_AREA);
                workingImage = downscaled;
                pImpl->m_downscaledImage = downscaled;
                scaleFactor = static_cast<float>(m_params.downscaleFactor);
                , m_lastMetrics.downscaleTime_ms);

                LOG_DEBUG("Image downscaled to " + std::to_string(newWidth) + "x" + std::to_string(newHeight));
            }
        }

        // === Stage 3: Preprocessing ===
        cv::Mat processed = workingImage;
        if (!m_params.skipPreprocessing) {
            MEASURE_TIME("Preprocessing",
                processed = pImpl->preprocessImage(workingImage, m_params);
            , m_lastMetrics.preprocessingTime_ms);
        }

        // === Stage 4: Edge Detection ===  // 여기도 MEASURE_TIME 매크로 넣어야함.
        cv::Mat edgeMap;
        if (m_params.edgeThreshold > 0) {
            edgeMap = pImpl->computeEdgeMap(processed, 0);
        }

        // === Stage 5: Detection ===
        std::vector<cv::Vec3f> candidates;

        // 5.1 Contour-based detection
        if (m_params.useContourDetection && m_params.useThresholding) {
            cv::Mat binary;

            MEASURE_TIME("Thresholding",
                if (m_params.useAdaptiveThreshold) {
                    cv::adaptiveThreshold(processed, binary, 255,
                        cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY,
                        m_params.adaptiveBlockSize, m_params.adaptiveConstant);
                }
                else {
                    double threshVal = m_params.brightnessThreshold > 0 ?
                        m_params.brightnessThreshold : 0;
                    int threshType = m_params.brightnessThreshold > 0 ?
                        cv::THRESH_BINARY : (cv::THRESH_BINARY | cv::THRESH_OTSU);
                    cv::threshold(processed, binary, threshVal, 255, threshType);
                }
            , m_lastMetrics.thresholdingTime_ms);

            if (m_params.useMorphology) {
                MEASURE_TIME("Morphology operations",
                    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
                cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
                cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
                , m_lastMetrics.morphologyTime_ms);
            }

            if (m_params.saveIntermediateImages) {
                pImpl->m_lastBinaryImage = binary.clone();
            }

            MEASURE_TIME("Contour detection",
                candidates = detectByContours(binary, processed, scaleFactor);
            , m_lastMetrics.contourDetectionTime_ms);

            LOG_DEBUG("Contour detection found " + std::to_string(candidates.size()) + " circle candidates");
        }

        // 5.2 Hough-based detection
        if (m_params.useHoughDetection && (candidates.empty() || !m_params.useContourDetection)) {
            MEASURE_TIME("Hough circle detection",
                auto houghCandidates = pImpl->detectCirclesHough(processed, m_params);
            candidates.insert(candidates.end(), houghCandidates.begin(), houghCandidates.end());
            , m_lastMetrics.houghDetectionTime_ms);

            if (!candidates.empty()) {
                LOG_DEBUG("Hough detection found " + std::to_string(candidates.size()) + " candidates");
            }
        }

        // 5.3 Template matching
        if (m_params.useTemplateMatching && m_templateInitialized) {
            MEASURE_TIME("Template matching",
                auto templateCandidates = detectByTemplate(processed, scaleFactor);
            candidates.insert(candidates.end(), templateCandidates.begin(), templateCandidates.end());
            , m_lastMetrics.templateMatchingTime_ms);
        }

        m_lastMetrics.candidatesFound = static_cast<int>(candidates.size());

        // === Stage 6: Candidate Evaluation ===
        std::vector<BallInfo> validBalls;
        if (!candidates.empty()) {
            MEASURE_TIME("Candidate evaluation",
                validBalls = pImpl->evaluateCandidatesTBB(
                    processed, candidates, m_params, scaleFactor, roiRect, frameIndex, edgeMap);
            , m_lastMetrics.candidateEvaluationTime_ms);
        }
        m_lastMetrics.candidatesEvaluated = static_cast<int>(candidates.size());
        m_lastMetrics.candidatesRejected = static_cast<int>(candidates.size() - validBalls.size());

        // === Stage 7: Tracking Update ===
        if (m_params.enableTracking && !validBalls.empty()) {
            MEASURE_TIME("Tracking update",
                for (auto& ball : validBalls) {
                    for (auto& [trackId, track] : m_tracks) {
                        float motionScore = calculateMotionConsistency(ball, track);
                        if (motionScore > ball.motionScore) {
                            ball.motionScore = motionScore;
                        }
                    }
                    ball.confidence = ball.confidence * 0.8f + ball.motionScore * 0.2f;
                }
            , m_lastMetrics.trackingTime_ms);
        }

        // === Stage 8: Final Selection ===
        if (!validBalls.empty()) {
            selectBestBalls(validBalls, result);
            m_lastMetrics.ballDetected = true;

            if (m_params.enableTracking) {
                updateTracking(result);
            }

            float totalConfidence = 0.0f;
            for (const auto& ball : result.balls) {
                totalConfidence += ball.confidence;
            }
            m_lastMetrics.averageConfidence = totalConfidence / result.balls.size();

            const BallInfo& ball = result.balls[0];
            LOG_INFO("Ball detected at (" + std::to_string(static_cast<int>(ball.center.x)) + ", " +
                std::to_string(static_cast<int>(ball.center.y)) + "), radius=" +
                std::to_string(static_cast<int>(ball.radius)) + ", confidence=" +
                std::to_string(ball.confidence));
        }

        // === Stage 9: Debug Output ===
        if (m_params.saveIntermediateImages) {
            saveDebugImages(imageData, width, height, frameIndex, result);
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
    m_lastMetrics.totalDetectionTime_ms =
        std::chrono::duration_cast<std::chrono::microseconds>(totalEndTime - totalStartTime).count() / 1000.0;

    if (m_performanceProfilingEnabled && (frameIndex % PERFORMANCE_LOG_INTERVAL == 0)) {
        logPerformanceMetrics(frameIndex);
    }

    return result;
}

// === Contour-based Detection ===
std::vector<cv::Vec3f> BallDetector::detectByContours(const cv::Mat& binary,
    const cv::Mat& grayImage,
    float downscaleFactor) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    float minR = static_cast<float>(m_params.minRadius);
    float maxR = static_cast<float>(m_params.maxRadius);
    if (downscaleFactor > 1) {
        minR = std::max(3.0f, minR / downscaleFactor);
        maxR = maxR / downscaleFactor;
    }

    float minArea = static_cast<float>(CV_PI * minR * minR);
    float maxArea = static_cast<float>(CV_PI * maxR * maxR);

    tbb::concurrent_vector<cv::Vec3f> circleList;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, contours.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
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

                double perimeter = cv::arcLength(contours[i], true);
                if (perimeter <= 0) {
                    continue;
                }

                double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
                if (circularity >= m_params.minCircularity) {
                    if (m_params.useColorFilter) {
                        cv::Rect checkRect(
                            static_cast<int>(center.x - 2),
                            static_cast<int>(center.y - 2),
                            5, 5
                        );
                        checkRect &= cv::Rect(0, 0, grayImage.cols, grayImage.rows);

                        if (checkRect.area() > 0) {
                            double meanBrightness = cv::mean(grayImage(checkRect))[0];
                            if (meanBrightness < m_params.brightnessThreshold * 0.5) {
                                continue;
                            }
                        }
                    }

                    circleList.push_back(cv::Vec3f(center.x, center.y, radius));
                }
            }
        }
    );

    return std::vector<cv::Vec3f>(circleList.begin(), circleList.end());
}

// === Template Matching ===
std::vector<cv::Vec3f> BallDetector::detectByTemplate(const cv::Mat& image, float downscaleFactor) {
    std::vector<cv::Vec3f> candidates;

    if (m_ballTemplate.empty()) {
        return candidates;
    }

    for (int s = 0; s < TEMPLATE_SCALES; ++s) {
        float scale = 1.0f - s * TEMPLATE_SCALE_STEP;
        cv::Mat scaledTemplate;
        cv::resize(m_ballTemplate, scaledTemplate,
            cv::Size(static_cast<int>(m_ballTemplate.cols * scale),
                static_cast<int>(m_ballTemplate.rows * scale)));

        cv::Mat result;
        cv::matchTemplate(image, scaledTemplate, result, cv::TM_CCOEFF_NORMED);

        cv::Mat mask;
        cv::threshold(result, mask, TEMPLATE_MATCH_THRESHOLD, 1, cv::THRESH_BINARY);
        mask.convertTo(mask, CV_8U);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            cv::Rect bbox = cv::boundingRect(contour);
            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(result(bbox), &minVal, &maxVal, &minLoc, &maxLoc);

            if (maxVal > TEMPLATE_MATCH_THRESHOLD) {
                float cx = bbox.x + maxLoc.x + scaledTemplate.cols / 2.0f;
                float cy = bbox.y + maxLoc.y + scaledTemplate.rows / 2.0f;
                float radius = (scaledTemplate.cols + scaledTemplate.rows) / 4.0f;

                candidates.push_back(cv::Vec3f(cx, cy, radius));
            }
        }
    }

    return candidates;
}

// === Ball Selection ===
void BallDetector::selectBestBalls(const std::vector<BallInfo>& validBalls,
    BallDetectionResult& result) {
    if (!m_params.detectMultiple) {
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
            auto bestIt = std::max_element(validBalls.begin(), validBalls.end(),
                [](const BallInfo& a, const BallInfo& b) {
                    return a.confidence < b.confidence;
                });
            result.balls.push_back(*bestIt);
        }
    }
    else {
        std::vector<BallInfo> sortedBalls = validBalls;

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

        std::vector<BallInfo> finalBalls;
        for (const auto& ball : sortedBalls) {
            bool overlap = false;
            for (const auto& selected : finalBalls) {
                float dist = cv::norm(ball.center - selected.center);
                if (dist < (ball.radius + selected.radius) * 0.5f) {
                    overlap = true;
                    break;
                }
            }

            if (!overlap) {
                finalBalls.push_back(ball);
                if (finalBalls.size() >= MAX_DISPLAY_BALLS) {
                    break;
                }
            }
        }

        result.balls = std::move(finalBalls);
    }

    result.found = true;
}

// === Debug Image Saving ===
void BallDetector::saveDebugImages(const unsigned char* imageData, int width, int height,
    int frameIndex, const BallDetectionResult& result) {
    DEBUG_IMAGE_TIME_MEASURE("Debug image saving",
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
            LOG_DEBUG("Created debug directory: " + debugPath);
        }

        pImpl->saveIntermediateImages(debugPath, frameIndex);

        if (result.found) {
            std::string resultPath = debugPath + "/frame_" +
                std::to_string(frameIndex) + "_result.png";
            SaveDetectionImage(imageData, width, height, result, resultPath, true);
        }
    }
    catch (const fs::filesystem_error& e) {
        LOG_ERROR("Failed to save debug images: " + std::string(e.what()));
    }
    , m_lastMetrics.imagesSavingTime_ms);
}

// === Performance Logging ===
void BallDetector::logPerformanceMetrics(int frameIndex) const {
    std::stringstream ss;
    ss << "Frame " << frameIndex << " Performance Metrics:\n";
    ss << "  Total: " << m_lastMetrics.totalDetectionTime_ms << " ms\n";
    ss << "  - ROI: " << m_lastMetrics.roiExtractionTime_ms << " ms\n";
    ss << "  - Downscale: " << m_lastMetrics.downscaleTime_ms << " ms\n";
    ss << "  - Preprocess: " << m_lastMetrics.preprocessingTime_ms << " ms\n";
    ss << "  - CLAHE: " << m_lastMetrics.claheTime_ms << " ms\n";
    ss << "  - Threshold: " << m_lastMetrics.thresholdingTime_ms << " ms\n";
    ss << "  - Morphology: " << m_lastMetrics.morphologyTime_ms << " ms\n";
    ss << "  - Contour: " << m_lastMetrics.contourDetectionTime_ms << " ms\n";
    ss << "  - Hough: " << m_lastMetrics.houghDetectionTime_ms << " ms\n";
    ss << "  - Template: " << m_lastMetrics.templateMatchingTime_ms << " ms\n";
    ss << "  - Evaluation: " << m_lastMetrics.candidateEvaluationTime_ms << " ms\n";
    ss << "  - Tracking: " << m_lastMetrics.trackingTime_ms << " ms\n";
    ss << "  Candidates: " << m_lastMetrics.candidatesFound << " found, "
        << m_lastMetrics.candidatesEvaluated << " evaluated, "
        << m_lastMetrics.candidatesRejected << " rejected";
    ss << "  Avg Confidence: " << m_lastMetrics.averageConfidence;

    LOG_INFO(ss.str());
}

// === Impl Methods ===
cv::Mat BallDetector::Impl::preprocessImage(const cv::Mat& grayImage, const DetectionParams& params) {
    cv::Mat processed = grayImage;

    // Step 1: Apply bilateral filter for edge-preserving smoothing
    if (!params.fastMode) {
        cv::Mat bilateral;
        cv::bilateralFilter(processed, bilateral, 5, 50, 50);
        processed = bilateral;
    }
    else {
        cv::Size kernelSize = cv::Size(3, 3);
        cv::GaussianBlur(processed, processed, kernelSize, 0.75, 0.75, cv::BORDER_REPLICATE);
    }

    // Step 2: Apply CLAHE for local contrast enhancement
    if (params.useCLAHE) {
        processed = applyCLAHE(processed, params.claheClipLimit);
        if (params.saveIntermediateImages) {
            m_lastCLAHE = processed.clone();
        }
    }

    // Step 3: Enhance shadow regions
    if (params.useEnhanceShadows && !params.fastMode) {
        processed = enhanceShadowRegions(processed, params.shadowEnhanceFactor);
    }

    // Step 4: Apply sharpening filter (에러 수정됨)
    if (!params.fastMode && params.contrastThreshold > 0) {
        cv::Mat kernel = (cv::Mat_<float>(3, 3) <<
            0, -1, 0,
            -1, 5, -1,
            0, -1, 0);
        cv::Mat sharpened;
        cv::filter2D(processed, sharpened, -1, kernel);
        cv::addWeighted(processed, 0.7, sharpened, 0.3, 0, processed);
    }

    // Step 5: Normalize if requested
    if (params.useNormalization) {
        cv::normalize(processed, processed, 0, 255, cv::NORM_MINMAX);
    }

    if (params.saveIntermediateImages) {
        m_lastProcessedImage = processed.clone();
    }

    return processed;
}

cv::Mat BallDetector::Impl::applyCLAHE(const cv::Mat& image, double clipLimit) {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clipLimit, cv::Size(8, 8));
    cv::Mat enhanced;
    clahe->apply(image, enhanced);
    return enhanced;
}

cv::Mat BallDetector::Impl::enhanceShadowRegions(const cv::Mat& image, float factor) {
    cv::Mat result = image.clone();

    cv::Scalar mean, stddev;
    cv::meanStdDev(image, mean, stddev);
    double avgBrightness = mean[0];
    double adaptiveThreshold = std::min(SHADOW_THRESHOLD, static_cast<int>(avgBrightness * 0.7));

    tbb::parallel_for(tbb::blocked_range<int>(0, image.rows),
        [&](const tbb::blocked_range<int>& range) {
            for (int y = range.begin(); y != range.end(); ++y) {
                const uchar* src = image.ptr<uchar>(y);
                uchar* dst = result.ptr<uchar>(y);
                for (int x = 0; x < image.cols; ++x) {
                    if (src[x] < adaptiveThreshold) {
                        float enhancementRatio = 1.0f - (src[x] / static_cast<float>(adaptiveThreshold));
                        float enhancementFactor = SHADOW_ENHANCEMENT_BASE + factor * enhancementRatio;
                        dst[x] = cv::saturate_cast<uchar>(src[x] * enhancementFactor);
                    }
                }
            }
        }
    );

    if (m_paramsPtr && m_paramsPtr->saveIntermediateImages) {
        m_lastShadowEnhanced = result.clone();
    }

    return result;
}

cv::Mat BallDetector::Impl::computeEdgeMap(const cv::Mat& image, int method) {
    cv::Mat edges;

    if (method == 0) {
        cv::Canny(image, edges, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD, SOBEL_KERNEL_SIZE);
    }
    else {
        cv::Mat gradX, gradY;
        cv::Sobel(image, gradX, CV_16S, 1, 0, SOBEL_KERNEL_SIZE);
        cv::Sobel(image, gradY, CV_16S, 0, 1, SOBEL_KERNEL_SIZE);

        cv::Mat absGradX, absGradY;
        cv::convertScaleAbs(gradX, absGradX);
        cv::convertScaleAbs(gradY, absGradY);

        cv::addWeighted(absGradX, 0.5, absGradY, 0.5, 0, edges);
    }

    if (m_paramsPtr && m_paramsPtr->saveIntermediateImages) {
        m_lastEdgeImage = edges.clone();
    }

    return edges;
}

std::vector<cv::Vec3f> BallDetector::Impl::detectCirclesHough(const cv::Mat& image, const DetectionParams& params) {
    std::vector<cv::Vec3f> circles;

    int minRadius = params.minRadius;
    int maxRadius = params.maxRadius;
    if (params.downscaleFactor > 1) {
        minRadius = std::max(3, minRadius / params.downscaleFactor);
        maxRadius = maxRadius / params.downscaleFactor;
    }

    cv::HoughCircles(image, circles, cv::HOUGH_GRADIENT,
        params.dp,
        params.minDist,
        params.param1,
        params.param2,
        minRadius,
        maxRadius);

    if (params.maxCandidates > 0 && circles.size() > static_cast<size_t>(params.maxCandidates)) {
        circles.resize(params.maxCandidates);
        LOG_DEBUG("Limited circles to " + std::to_string(params.maxCandidates) + " candidates");
    }

    return circles;
}

bool BallDetector::Impl::quickValidateCircle(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    int cx = cvRound(circle[0]), cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);
    const int margin = 2;

    if (cx - radius - margin < 0 || cy - radius - margin < 0 ||
        cx + radius + margin >= image.cols || cy + radius + margin >= image.rows) {
        return false;
    }

    if (params.useColorFilter) {
        cv::Rect sampleRect(cx - 2, cy - 2, 5, 5);
        sampleRect &= cv::Rect(0, 0, image.cols, image.rows);
        if (sampleRect.area() > 0) {
            double meanBrightness = cv::mean(image(sampleRect))[0];
            if (!params.useAdaptiveThreshold) {
                if (meanBrightness < params.brightnessThreshold * 0.7) {
                    return false;
                }
            }
        }
    }

    if (params.contrastThreshold > 0) {
        int innerSum = 0, outerSum = 0, count = 0;
        int sampleRadius = radius / 2;
        int offsets[4][2] = { {0,-1},{0,1},{-1,0},{1,0} };

        for (auto& off : offsets) {
            int ix = cx + off[0] * sampleRadius;
            int iy = cy + off[1] * sampleRadius;
            int ox = cx + off[0] * radius * 3 / 2;
            int oy = cy + off[1] * radius * 3 / 2;

            if (ix >= 0 && ix < image.cols && iy >= 0 && iy < image.rows &&
                ox >= 0 && ox < image.cols && oy >= 0 && oy < image.rows) {
                innerSum += image.at<uchar>(iy, ix);
                outerSum += image.at<uchar>(oy, ox);
                count++;
            }
        }

        if (count > 0) {
            float avgDiff = std::fabs(innerSum - outerSum) / count;
            if (avgDiff < params.contrastThreshold) {
                return false;
            }
        }
    }

    return true;
}

float BallDetector::Impl::calculateConfidence(const cv::Mat& image, const cv::Vec3f& circle,
    const DetectionParams& params, const cv::Mat& edgeMap) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    float sizeScore = 0.0f;
    float contrastScore = 0.0f;
    float brightnessScore = 0.0f;
    float edgeScore = 0.0f;
    float symmetryScore = 0.0f;

    // 1. Size score
    float sizeRange = static_cast<float>(params.maxRadius - params.minRadius);
    float optimalRadius = params.minRadius + sizeRange * 0.5f;
    sizeScore = 1.0f - std::min(1.0f, std::abs(radius - optimalRadius) / (sizeRange * 0.5f));

    // 2. Contrast score
    cv::Rect innerRect(cx - radius / 3, cy - radius / 3, 2 * radius / 3, 2 * radius / 3);
    cv::Rect outerRect(cx - radius * 3 / 2, cy - radius * 3 / 2, radius * 3, radius * 3);

    innerRect &= cv::Rect(0, 0, image.cols, image.rows);
    outerRect &= cv::Rect(0, 0, image.cols, image.rows);

    if (innerRect.area() > 0 && outerRect.area() > innerRect.area()) {
        cv::Mat innerRegion = image(innerRect);
        cv::Mat outerRegion = image(outerRect);

        cv::Scalar innerMean, innerStdDev;
        cv::Scalar outerMean, outerStdDev;
        cv::meanStdDev(innerRegion, innerMean, innerStdDev);
        cv::meanStdDev(outerRegion, outerMean, outerStdDev);

        double contrastDiff = std::abs(innerMean[0] - outerMean[0]);
        contrastScore = std::min(1.0f, static_cast<float>(contrastDiff / 127.5));

        // 3. Brightness consistency
        double brightnessVariance = innerStdDev[0];
        brightnessScore = 1.0f - std::min(1.0f, static_cast<float>(brightnessVariance / 50.0));
    }

    // 4. Edge score
    if (!edgeMap.empty()) {
        edgeScore = calculateEdgeStrength(edgeMap, circle);
    }

    // 5. Symmetry score
    const int numSamples = 8;
    std::vector<double> radialSamples;
    for (int i = 0; i < numSamples; ++i) {
        double angle = 2 * CV_PI * i / numSamples;
        int sx = cx + static_cast<int>(radius * 0.7 * cos(angle));
        int sy = cy + static_cast<int>(radius * 0.7 * sin(angle));

        if (sx >= 0 && sx < image.cols && sy >= 0 && sy < image.rows) {
            radialSamples.push_back(image.at<uchar>(sy, sx));
        }
    }

    if (radialSamples.size() == numSamples) {
        double sum = std::accumulate(radialSamples.begin(), radialSamples.end(), 0.0);
        double mean = sum / radialSamples.size();
        double variance = 0.0;
        for (double val : radialSamples) {
            variance += (val - mean) * (val - mean);
        }
        variance /= radialSamples.size();
        symmetryScore = 1.0f - std::min(1.0f, static_cast<float>(sqrt(variance) / 50.0));
    }

    float weights[] = { 0.15f, 0.25f, 0.20f, 0.25f, 0.15f };
    float scores[] = { sizeScore, contrastScore, brightnessScore, edgeScore, symmetryScore };

    float totalWeight = 0.0f;
    float weightedSum = 0.0f;
    for (int i = 0; i < 5; ++i) {
        if (scores[i] > 0) {
            weightedSum += scores[i] * weights[i];
            totalWeight += weights[i];
        }
    }

    return totalWeight > 0 ? (weightedSum / totalWeight) : 0.5f;
}

float BallDetector::Impl::calculateCircularity(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    const int margin = 5;
    cv::Rect roi(cx - radius - margin, cy - radius - margin, (radius + margin) * 2, (radius + margin) * 2);
    roi &= cv::Rect(0, 0, image.cols, image.rows);

    if (roi.area() == 0) return 0.0f;

    cv::Mat roiImage = image(roi);
    cv::Mat binary;
    cv::threshold(roiImage, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return 0.0f;

    cv::Point roiCenter(cx - roi.x, cy - roi.y);
    size_t bestIdx = 0;
    double minDist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Moments M = cv::moments(contours[i]);
        if (M.m00 > 0) {
            cv::Point centroid(static_cast<int>(M.m10 / M.m00),
                static_cast<int>(M.m01 / M.m00));
            double dist = cv::norm(centroid - roiCenter);
            if (dist < minDist) {
                minDist = dist;
                bestIdx = i;
            }
        }
    }

    double area = cv::contourArea(contours[bestIdx]);
    double perimeter = cv::arcLength(contours[bestIdx], true);

    if (perimeter <= 0) return 0.0f;

    return static_cast<float>(std::min(1.0, 4.0 * CV_PI * area / (perimeter * perimeter)));
}

float BallDetector::Impl::calculateEdgeStrength(const cv::Mat& edgeMap, const cv::Vec3f& circle) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    const int numSamples = 36;
    int edgeCount = 0;
    int validSamples = 0;

    for (int i = 0; i < numSamples; ++i) {
        double angle = 2 * CV_PI * i / numSamples;
        int x = cx + static_cast<int>(radius * cos(angle));
        int y = cy + static_cast<int>(radius * sin(angle));

        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int nx = x + dx;
                int ny = y + dy;
                if (nx >= 0 && nx < edgeMap.cols && ny >= 0 && ny < edgeMap.rows) {
                    if (edgeMap.at<uchar>(ny, nx) > 0) {
                        edgeCount++;
                    }
                    validSamples++;
                }
            }
        }
    }

    return validSamples > 0 ? static_cast<float>(edgeCount) / validSamples : 0.0f;
}

std::vector<BallInfo> BallDetector::Impl::evaluateCandidatesTBB(const cv::Mat& image,
    const std::vector<cv::Vec3f>& candidates,
    const DetectionParams& params,
    float scaleFactor,
    const cv::Rect& roiRect,
    int frameIndex,
    const cv::Mat& edgeMap) {

    tbb::concurrent_vector<BallInfo> concurrentValidBalls;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, candidates.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                const cv::Vec3f& circle = candidates[i];

                if (quickValidateCircle(image, circle, params)) {
                    BallInfo info;
                    info.center.x = circle[0] * scaleFactor;
                    info.center.y = circle[1] * scaleFactor;
                    if (params.useROI) {
                        info.center.x += roiRect.x;
                        info.center.y += roiRect.y;
                    }
                    info.radius = circle[2] * scaleFactor;
                    info.frameIndex = frameIndex;

                    info.confidence = calculateConfidence(image, circle, params, edgeMap);

                    if (params.useCircularityCheck && !params.fastMode) {
                        info.circularity = calculateCircularity(image, circle, params);
                        if (info.circularity < params.minCircularity) {
                            continue;
                        }
                    }
                    else {
                        info.circularity = 1.0f;
                    }

                    cv::Rect ballRect(cvRound(circle[0] - circle[2] / 2),
                        cvRound(circle[1] - circle[2] / 2),
                        cvRound(circle[2]), cvRound(circle[2]));

                    ballRect &= cv::Rect(0, 0, image.cols, image.rows);
                    if (ballRect.area() > 0) {
                        info.brightness = static_cast<float>(cv::mean(image(ballRect))[0]);
                    }

                    if (!edgeMap.empty()) {
                        info.edgeStrength = calculateEdgeStrength(edgeMap, circle);
                    }

                    if (info.confidence >= MIN_CONFIDENCE_THRESHOLD) {
                        concurrentValidBalls.push_back(info);
                    }
                }
            }
        }
    );

    std::vector<BallInfo> validBalls(concurrentValidBalls.begin(), concurrentValidBalls.end());

    LOG_DEBUG("Evaluated " + std::to_string(candidates.size()) + " candidates, " +
        std::to_string(validBalls.size()) + " passed validation");

    return validBalls;
}

cv::Mat BallDetector::Impl::extractROI(const cv::Mat& image, float scale) {
    int roiWidth = static_cast<int>(image.cols * scale);
    int roiHeight = static_cast<int>(image.rows * scale);

    roiWidth = (roiWidth / 2) * 2;
    roiHeight = (roiHeight / 2) * 2;

    int x = (image.cols - roiWidth) / 2;
    int y = (image.rows - roiHeight) / 2;

    cv::Rect roiRect(x, y, roiWidth, roiHeight);
    roiRect &= cv::Rect(0, 0, image.cols, image.rows);

    return image(roiRect);
}

void BallDetector::Impl::saveIntermediateImages(const std::string& basePath, int frameIndex) {
    if (basePath.empty()) {
        LOG_ERROR("Debug output path is empty");
        return;
    }

    if (!std::filesystem::exists(basePath)) {
        LOG_WARNING("Debug directory does not exist: " + basePath);
        try {
            std::filesystem::create_directories(basePath);
            LOG_INFO("Created debug directory: " + basePath);
        }
        catch (const std::exception& e) {
            LOG_ERROR("Failed to create debug directory: " + std::string(e.what()));
            return;
        }
    }

    std::string prefix = basePath + "/frame_" + std::to_string(frameIndex);

    std::lock_guard<std::mutex> lock(m_saveQueueMutex);

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

cv::Mat BallDetector::Impl::getMatFromPool(int rows, int cols, int type) {
    std::lock_guard<std::mutex> lock(m_poolMutex);

    for (auto it = m_matPool.begin(); it != m_matPool.end(); ++it) {
        if (it->rows == rows && it->cols == cols && it->type() == type) {
            cv::Mat mat = *it;
            m_matPool.erase(it);
            return mat;
        }
    }

    return cv::Mat(rows, cols, type);
}

void BallDetector::Impl::returnMatToPool(cv::Mat& mat) {
    if (!mat.empty()) {
        std::lock_guard<std::mutex> lock(m_poolMutex);
        if (m_matPool.size() < 10) {
            m_matPool.push_back(mat);
        }
    }
}

// === Tracking Functions ===
float BallDetector::calculateMotionConsistency(const BallInfo& ball, const TrackingInfo& track) {
    if (track.history.empty()) {
        return 0.0f;
    }

    float distance = cv::norm(ball.center - track.predictedCenter);
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

        float velChange1 = cv::norm(currentVel - v1);
        float velChange2 = cv::norm(v1 - v2);
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

// === Template Functions ===
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

// === Calibration Function ===
bool BallDetector::CalibrateForBallSize(const std::vector<cv::Mat>& sampleImages,
    float knownBallDiameter_mm) {
    if (sampleImages.empty()) {
        LOG_ERROR("No sample images provided for calibration");
        return false;
    }

    std::vector<float> detectedRadii;

    for (size_t i = 0; i < sampleImages.size(); ++i) {
        if (sampleImages[i].channels() > 1) {
            cv::Mat gray;
            cv::cvtColor(sampleImages[i], gray, cv::COLOR_BGR2GRAY);

            BallDetectionResult result = DetectBall(gray.data, gray.cols, gray.rows, static_cast<int>(i));
            if (result.found && !result.balls.empty()) {
                detectedRadii.push_back(result.balls[0].radius);
            }
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

// === Utility Functions ===
double BallDetector::EstimateProcessingTime(int width, int height) const {
    double pixels = width * height;
    double scaledPixels = pixels;

    if (m_params.useROI) { scaledPixels *= (m_params.roiScale * m_params.roiScale); }
    if (m_params.downscaleFactor > 1) { scaledPixels /= (m_params.downscaleFactor * m_params.downscaleFactor); }

    double baseTime = scaledPixels / 50000.0;

    if (m_params.fastMode) { baseTime *= 0.5; }
    if (m_params.useParallel) { baseTime /= std::min(4.0, static_cast<double>(m_params.processingThreads)); }

    return baseTime;
}

// === Visualization Function ===
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

        cv::rectangle(colorImg, cv::Point(0, 0), cv::Point(width, 50), cv::Scalar(40, 40, 40), cv::FILLED);
        cv::putText(colorImg, "Ball Detection Result", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

        if (result.found && !result.balls.empty()) {
            int infoY = 70;

            for (size_t i = 0; i < result.balls.size() && i < MAX_DISPLAY_BALLS; ++i) {
                const BallInfo& ball = result.balls[i];
                cv::Point center(cvRound(ball.center.x), cvRound(ball.center.y));
                int radius = cvRound(ball.radius);

                cv::Mat overlay = colorImg.clone();
                cv::circle(overlay, center, radius, cv::Scalar(0, 0, 255), -1);
                cv::addWeighted(colorImg, 0.7, overlay, 0.3, 0, colorImg);

                cv::circle(colorImg, center, radius, cv::Scalar(0, 0, 255), 3);
                cv::drawMarker(colorImg, center, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 20, 2);

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

            cv::Rect summaryBg(0, height - 30, width, 30);
            cv::rectangle(colorImg, summaryBg, cv::Scalar(40, 40, 40), cv::FILLED);

            std::ostringstream summary;
            summary << "Total balls: " << result.balls.size() << " | Frame: " << (result.balls.empty() ? 0 : result.balls[0].frameIndex);
            cv::putText(colorImg, summary.str(), cv::Point(10, height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
        else {
            cv::Rect msgBg(10, 60, 200, 40);
            cv::rectangle(colorImg, msgBg, cv::Scalar(0, 0, 100), cv::FILLED);
            cv::putText(colorImg, "No ball detected", cv::Point(15, 85), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        struct tm localTime;
        localtime_s(&localTime, &t);

        std::ostringstream timestamp;
        timestamp << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S");
        cv::putText(colorImg, timestamp.str(), cv::Point(width - 180, height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

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