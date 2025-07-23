#include "pch.h"
#include "BallDetector.h"
#include "Logger.h"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <numeric>
#include <execution>
#include <future>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>

namespace fs = std::filesystem;

// Constants
namespace {
    constexpr int DEFAULT_BUFFER_WIDTH = 1280;
    constexpr int DEFAULT_BUFFER_HEIGHT = 960;
    constexpr int MIN_DOWNSCALE_WIDTH = 160;
    constexpr int MIN_DOWNSCALE_HEIGHT = 120;
    constexpr float MIN_CONFIDENCE_THRESHOLD = 0.4f;
    constexpr int SHADOW_THRESHOLD = 100;
    constexpr float SHADOW_ENHANCEMENT_BASE = 1.0f;
    constexpr int LUT_SIZE = 256 * 256;
    constexpr int PERFORMANCE_LOG_INTERVAL = 100;
    constexpr int MAX_DISPLAY_BALLS = 5;
    constexpr int INFO_PANEL_HEIGHT = 90;
    constexpr int INFO_PANEL_WIDTH = 280;
}

// Performance profiling macro
#ifdef ENABLE_PERFORMANCE_PROFILING
#define MEASURE_TIME(name, code, metricsVar) \
        { \
            auto start = std::chrono::high_resolution_clock::now(); \
            code \
            auto end = std::chrono::high_resolution_clock::now(); \
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count(); \
            if (m_performanceProfilingEnabled) { \
                metricsVar = duration / 1000.0; \
                LOG_DEBUG(std::string(name) + " took " + std::to_string(duration) + " us (" + \
                          std::to_string(duration / 1000.0) + " ms)"); \
            } \
        }
#else
#define MEASURE_TIME(name, code, metricsVar) { code }
#endif

// Constructor 
BallDetector::DetectionParams::DetectionParams() {
    dp = 2.0;
    minDist = 40.0;
    param1 = 130.0;
    param2 = 0.87;
    minRadius = 20;
    maxRadius = 30;
    brightnessThreshold = 120;
    minCircularity = 0.70f;
    contrastThreshold = 20.0f;

    useColorFilter = true;
    useCircularityCheck = false;  // Skip for speed
    useHoughGradientAlt = false;
    detectMultiple = false;
    useMorphology = false;
    useAdaptiveThreshold = false;
    correctPerspective = false;
    enhanceShadows = false;
    shadowEnhanceFactor = 0.7f;
    saveIntermediateImages = true;
    debugOutputDir = "";

    // Performance optimize
    fastMode = true;
    useROI = true;
    roiScale = 0.75f;
    downscaleFactor = 2;
    useParallel = true;
    maxCandidates = 15;
    skipPreprocessing = false;
    edgeThreshold = 100;
    useCache = false;
    processingThreads = std::max(2u, std::thread::hardware_concurrency() / 2);
}

// BallDetector implementation
class BallDetector::Impl {
public:
    // Cache
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;
    cv::Mat m_lastProcessedImage;
    cv::Mat m_lastBinaryImage;
    cv::Mat m_lastEdgeImage;
    cv::Mat m_lastShadowEnhanced;

    cv::Mat m_tempBuffer1;
    cv::Mat m_tempBuffer2;
    cv::Mat m_downscaledImage;
    cv::Mat m_roiMask;
    cv::Mat m_blurKernel;

    // LUT
    std::vector<uint8_t> m_sqrtLUT;
    std::vector<float> m_normalizeLUT;

    std::string m_currentCaptureFolder;

    Impl() : m_currentCaptureFolder("") {
        initializeBuffers();
        initializeLUTs();
    }

    void setCurrentCaptureFolder(const std::string& folder) {
        m_currentCaptureFolder = folder;
    }

    cv::Mat preprocessImageFast(const cv::Mat& grayImage, const DetectionParams& params);
    cv::Mat enhanceShadowRegionsFast(const cv::Mat& image, float factor);

    std::vector<cv::Vec3f> detectCirclesHoughFast(const cv::Mat& image, const DetectionParams& params);
    std::vector<cv::Vec3f> detectCirclesByContourFast(const cv::Mat& binaryImage, const DetectionParams& params);

    // TBB-optimized evaluation
    bool quickValidateCircle(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
    float calculateConfidenceFast(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
    float calculateCircularityFast(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);

    // ROI and coordinate transformation
    cv::Mat extractROI(const cv::Mat& image, float scale);
    cv::Point2f transformToOriginalCoords(const cv::Point2f& roiPoint, const cv::Size& roiSize, const cv::Size& origSize, float scale);

    // TBB parallel processing
    std::vector<BallInfo> evaluateCandidatesTBB(const cv::Mat& image,
        const std::vector<cv::Vec3f>& candidates,
        const DetectionParams& params,
        float scaleFactor,
        const cv::Rect& roiRect,
        int frameIndex);

    // Utility functions
    void saveIntermediateImages(const std::string& basePath, int frameIndex);
    inline float fastDistance(float x1, float y1, float x2, float y2) {
        float dx = x2 - x1;
        float dy = y2 - y1;
        int idx = static_cast<int>(dx * dx + dy * dy);
        return (idx < LUT_SIZE) ? m_sqrtLUT[idx] : std::sqrt(dx * dx + dy * dy);
    }

private:
    void initializeBuffers() {
        // Pre-allocate common buffers with default size
        m_tempBuffer1.create(DEFAULT_BUFFER_HEIGHT, DEFAULT_BUFFER_WIDTH, CV_8UC1);
        m_tempBuffer2.create(DEFAULT_BUFFER_HEIGHT, DEFAULT_BUFFER_WIDTH, CV_8UC1);

        // Pre-create Gaussian kernel for fast blur
        m_blurKernel = cv::getGaussianKernel(3, 1.0);
    }

    void initializeLUTs() {
        // Initialize square root LUT for fast distance calculations
        m_sqrtLUT.resize(LUT_SIZE);
        for (int i = 0; i < LUT_SIZE; ++i) {
            m_sqrtLUT[i] = static_cast<uint8_t>(std::sqrt(i));
        }

        // Initialize normalization LUT
        m_normalizeLUT.resize(256);
        for (int i = 0; i < 256; ++i) {
            m_normalizeLUT[i] = i / 255.0f;
        }
    }
};

// BallDetector Constructor
BallDetector::BallDetector()
    : m_params(),
    pImpl(std::make_unique<Impl>()),
    m_performanceProfilingEnabled(false) {

    cv::setNumThreads(0);  // Use all available threads
    cv::setUseOptimized(true);  // Enable optimized code

    LOG_INFO("BallDetector initialized with TBB optimization");
    InitializeDefaultParams();
    m_lastMetrics.Reset();
}

BallDetector::~BallDetector() = default;

void BallDetector::InitializeDefaultParams() {
    m_params = DetectionParams();
    LOG_INFO("BallDetector parameters initialized for optimized performance");
}

void BallDetector::ResetToDefaults() {
    InitializeDefaultParams();
    LOG_INFO("BallDetector parameters reset to default values");
}

void BallDetector::EnablePerformanceProfiling(bool enable) {
    m_performanceProfilingEnabled = enable;
    LOG_INFO("Performance profiling " + std::string(enable ? "enabled" : "disabled"));
}

void BallDetector::SetCurrentCaptureFolder(const std::string& folder) {
    if (pImpl) {
        pImpl->setCurrentCaptureFolder(folder);
        LOG_DEBUG("Current capture folder set to: " + folder);
    }
}

// Main detection function
BallDetectionResult BallDetector::DetectBall(const unsigned char* imageData, int width, int height, int frameIndex) {
    auto totalStartTime = std::chrono::high_resolution_clock::now();
    m_lastMetrics.Reset();

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
        // 1. Create OpenCV Mat from input (zero-copy)
        cv::Mat grayImage(height, width, CV_8UC1, const_cast<unsigned char*>(imageData));

        // 2. Apply ROI if enabled (significant speedup)
        cv::Mat processImage = grayImage;
        cv::Rect roiRect(0, 0, width, height);
        if (m_params.useROI && m_params.roiScale < 1.0f) {
            processImage = pImpl->extractROI(grayImage, m_params.roiScale);
            roiRect = cv::Rect(
                (width - processImage.cols) / 2,
                (height - processImage.rows) / 2,
                processImage.cols,
                processImage.rows
            );
        }

        // 3. Downscale if enabled (major speedup)
        cv::Mat workingImage = processImage;
        float scaleFactor = 1.0f;
        if (m_params.fastMode && m_params.downscaleFactor > 1) {
            int newWidth = processImage.cols / m_params.downscaleFactor;
            int newHeight = processImage.rows / m_params.downscaleFactor;

            if (newWidth >= MIN_DOWNSCALE_WIDTH && newHeight >= MIN_DOWNSCALE_HEIGHT) {
                cv::resize(processImage, pImpl->m_downscaledImage,
                    cv::Size(newWidth, newHeight),
                    0, 0, cv::INTER_AREA);
                workingImage = pImpl->m_downscaledImage;
                scaleFactor = static_cast<float>(m_params.downscaleFactor);
                LOG_DEBUG("Image downscaled to " + std::to_string(newWidth) + "x" + std::to_string(newHeight));
            }
        }

        // 4. Minimal preprocessing
        cv::Mat processed;
        if (!m_params.skipPreprocessing) {
            MEASURE_TIME("Fast preprocessing",
                processed = pImpl->preprocessImageFast(workingImage, m_params); ,
                m_lastMetrics.preprocessingTime_ms
            )
        }
        else {
            processed = workingImage;
            m_lastMetrics.preprocessingTime_ms = 0;
        }

        // 5. Fast circle detection
        std::vector<cv::Vec3f> candidates;
        MEASURE_TIME("Hough circle detection",
            candidates = pImpl->detectCirclesHoughFast(processed, m_params); ,
            m_lastMetrics.houghDetectionTime_ms
        )

            // 6. Backup detection if no circles found
            if (candidates.empty() && !m_params.fastMode) {
                MEASURE_TIME("Contour-based detection",
                    cv::Mat binary;
                cv::threshold(processed, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
                auto contourCircles = pImpl->detectCirclesByContourFast(binary, m_params);
                candidates.insert(candidates.end(), contourCircles.begin(), contourCircles.end()); ,
                    m_lastMetrics.contourDetectionTime_ms
                    )
            }

        m_lastMetrics.candidatesFound = static_cast<int>(candidates.size());

        // 7. TBB-optimized candidate evaluation
        std::vector<BallInfo> validBalls;
        if (!candidates.empty()) {
            MEASURE_TIME("TBB candidate evaluation",
                validBalls = pImpl->evaluateCandidatesTBB(
                    workingImage, candidates, m_params, scaleFactor, roiRect, frameIndex); ,
                m_lastMetrics.candidateEvaluationTime_ms
            )
        }

        m_lastMetrics.candidatesEvaluated = static_cast<int>(candidates.size());

        // 8. Select best ball(s)
        if (!validBalls.empty()) {
            if (!m_params.detectMultiple) {
                // Find single best ball using TBB reduction
                auto bestBall = tbb::parallel_reduce(
                    tbb::blocked_range<size_t>(0, validBalls.size()),
                    validBalls[0],
                    [&validBalls](const tbb::blocked_range<size_t>& r, BallInfo best) {
                        for (size_t i = r.begin(); i != r.end(); ++i) {
                            if (validBalls[i].confidence > best.confidence) {
                                best = validBalls[i];
                            }
                        }
                        return best;
                    },
                    [](const BallInfo& a, const BallInfo& b) {
                        return a.confidence > b.confidence ? a : b;
                    }
                );
                result.balls.push_back(bestBall);
            }
            else {
                // Sort and return top candidates
                tbb::parallel_sort(validBalls.begin(), validBalls.end(),
                    [](const BallInfo& a, const BallInfo& b) { return a.confidence > b.confidence; });

                int maxBalls = std::min(MAX_DISPLAY_BALLS, static_cast<int>(validBalls.size()));
                result.balls.assign(validBalls.begin(), validBalls.begin() + maxBalls);
            }
            result.found = true;
            m_lastMetrics.ballDetected = true;

            const BallInfo& ball = result.balls[0];
            LOG_INFO("Ball detected at (" + std::to_string(cvRound(ball.center.x)) + ", " +
                std::to_string(cvRound(ball.center.y)) + "), radius=" +
                std::to_string(cvRound(ball.radius)) +
                ", confidence=" + std::to_string(ball.confidence));
        }

        // 9. Save debug images with proper path handling
        if (m_params.saveIntermediateImages) {
            std::string debugPath;

            // Use capture folder from ContinuousCaptureManager if available
            if (!pImpl->m_currentCaptureFolder.empty()) {
                debugPath = pImpl->m_currentCaptureFolder + "/debug_images";
            }
            // Fallback to debugOutputDir if set
            else if (!m_params.debugOutputDir.empty()) {
                debugPath = m_params.debugOutputDir;
            }
            // Last resort: use current directory
            else {
                debugPath = "./debug_images";
                LOG_WARNING("No debug output directory specified, using ./debug_images");
            }

            MEASURE_TIME("Debug image saving",
                try {
                // Create directory if it doesn't exist
                if (!fs::exists(debugPath)) {
                    fs::create_directories(debugPath);
                    LOG_DEBUG("Created debug directory: " + debugPath);
                }

                // Save intermediate images
                pImpl->saveIntermediateImages(debugPath, frameIndex);

                // Save detection result
                if (result.found) {
                    std::string resultPath = debugPath + "/frame_" +
                        std::to_string(frameIndex) + "_result.png";
                    SaveDetectionImage(imageData, width, height, result, resultPath, true);
                }
            }
            catch (const fs::filesystem_error& e) {
                LOG_ERROR("Failed to create debug directory: " + std::string(e.what()));
            },
                m_lastMetrics.imagesSavingTime_ms
                )
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

    // Calculate total processing time
    auto totalEndTime = std::chrono::high_resolution_clock::now();
    m_lastMetrics.totalDetectionTime_ms =
        std::chrono::duration_cast<std::chrono::microseconds>(totalEndTime - totalStartTime).count() / 1000.0;

    // Log performance periodically
    if (m_performanceProfilingEnabled && (frameIndex % PERFORMANCE_LOG_INTERVAL == 0)) {
        LOG_INFO("Frame " + std::to_string(frameIndex) + " - Total detection time: " +
            std::to_string(m_lastMetrics.totalDetectionTime_ms) + " ms");
    }

    return result;
}

// Optimized preprocessing
cv::Mat BallDetector::Impl::preprocessImageFast(const cv::Mat& grayImage, const DetectionParams& params) {
    cv::Mat processed = grayImage.clone();

    // Fast Gaussian blur using separable filter
    cv::Size kernelSize = params.fastMode ? cv::Size(3, 3) : cv::Size(5, 5);
    double sigma = params.fastMode ? 0.75 : 1.0;
    cv::GaussianBlur(processed, processed, kernelSize, sigma, sigma, cv::BORDER_REPLICATE);

    // Optional shadow enhancement (only if really needed)
    if (params.enhanceShadows && !params.fastMode) {
        processed = enhanceShadowRegionsFast(processed, params.shadowEnhanceFactor);
    }

    // Simple contrast normalization (faster than CLAHE)
    if (!params.fastMode && params.contrastThreshold > 0) {
        cv::normalize(processed, processed, 0, 255, cv::NORM_MINMAX);
    }

    // Cache processed image for debugging
    m_lastProcessedImage = processed.clone();

    return processed;
}

// Fast shadow enhancement with TBB
cv::Mat BallDetector::Impl::enhanceShadowRegionsFast(const cv::Mat& image, float factor) {
    cv::Mat result = image.clone();

    // TBB parallel processing for shadow enhancement
    tbb::parallel_for(tbb::blocked_range<int>(0, image.rows),
        [&](const tbb::blocked_range<int>& range) {
            for (int y = range.begin(); y != range.end(); ++y) {
                const uchar* src = image.ptr<uchar>(y);
                uchar* dst = result.ptr<uchar>(y);
                for (int x = 0; x < image.cols; ++x) {
                    if (src[x] < SHADOW_THRESHOLD) {
                        dst[x] = cv::saturate_cast<uchar>(src[x] * (SHADOW_ENHANCEMENT_BASE + factor));
                    }
                }
            }
        }
    );

    m_lastShadowEnhanced = result.clone();
    return result;
}

// Fast Hough circle detection
std::vector<cv::Vec3f> BallDetector::Impl::detectCirclesHoughFast(const cv::Mat& image, const DetectionParams& params) {
    std::vector<cv::Vec3f> circles;

    // Adjust parameters for downscaled image
    int minRadius = params.minRadius;
    int maxRadius = params.maxRadius;
    if (params.downscaleFactor > 1) {
        minRadius = std::max(3, minRadius / params.downscaleFactor);
        maxRadius = maxRadius / params.downscaleFactor;
    }

    // Use standard Hough gradient
    cv::HoughCircles(image, circles, cv::HOUGH_GRADIENT,
        params.dp,
        params.minDist,
        params.param1,
        params.param2,
        minRadius,
        maxRadius);

    // Limit candidates early
    if (params.maxCandidates > 0 && circles.size() > static_cast<size_t>(params.maxCandidates)) {
        circles.resize(params.maxCandidates);
        LOG_DEBUG("Limited circles to " + std::to_string(params.maxCandidates) + " candidates");
    }

    return circles;
}

// Fast contour-based circle detection
std::vector<cv::Vec3f> BallDetector::Impl::detectCirclesByContourFast(const cv::Mat& binaryImage, const DetectionParams& params) {
    std::vector<cv::Vec3f> circles;
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    m_lastBinaryImage = binaryImage.clone();

    // Pre-calculate area limits
    const float minArea = static_cast<float>(CV_PI * params.minRadius * params.minRadius);
    const float maxArea = static_cast<float>(CV_PI * params.maxRadius * params.maxRadius);

    // TBB concurrent vector for thread-safe insertion
    tbb::concurrent_vector<cv::Vec3f> concurrentCircles;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, contours.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                double area = cv::contourArea(contours[i]);
                if (area >= minArea && area <= maxArea) {
                    cv::Point2f center;
                    float radius;
                    cv::minEnclosingCircle(contours[i], center, radius);

                    if (radius >= params.minRadius && radius <= params.maxRadius) {
                        double perimeter = cv::arcLength(contours[i], true);
                        if (perimeter > 0) {
                            double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
                            if (circularity >= params.minCircularity * 0.8f) {
                                concurrentCircles.push_back(cv::Vec3f(center.x, center.y, radius));
                            }
                        }
                    }
                }
            }
        }
    );

    // Convert to regular vector
    circles.assign(concurrentCircles.begin(), concurrentCircles.end());
    LOG_DEBUG("Found " + std::to_string(circles.size()) + " circles by contour");

    return circles;
}

bool BallDetector::Impl::quickValidateCircle(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
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
            double brightness = cv::mean(image(sampleRect))[0];
            if (brightness < params.brightnessThreshold * 0.7) {
                return false;
            }
        }
    }

    if (params.contrastThreshold > 0) {
        // 4 cardinal points
        int sampleRadius = radius / 2;
        int innerSum = 0, outerSum = 0;
        int validSamples = 0;

        const int offsets[4][2] = { {0, -1}, {0, 1}, {-1, 0}, {1, 0} };
        for (int i = 0; i < 4; ++i) {
            int ix = cx + offsets[i][0] * sampleRadius;
            int iy = cy + offsets[i][1] * sampleRadius;
            int ox = cx + offsets[i][0] * radius * 3 / 2;
            int oy = cy + offsets[i][1] * radius * 3 / 2;

            if (ix >= 0 && ix < image.cols && iy >= 0 && iy < image.rows &&
                ox >= 0 && ox < image.cols && oy >= 0 && oy < image.rows) {
                innerSum += image.at<uchar>(iy, ix);
                outerSum += image.at<uchar>(oy, ox);
                validSamples++;
            }
        }

        if (validSamples > 0) {
            float contrast = std::abs(innerSum - outerSum) / static_cast<float>(validSamples);
            if (contrast < params.contrastThreshold) {
                return false;
            }
        }
    }

    return true;
}

float BallDetector::Impl::calculateConfidenceFast(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    float confidence = 0.5f;

    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Size score
    float sizeRange = static_cast<float>(params.maxRadius - params.minRadius);
    float optimalRadius = params.minRadius + sizeRange * 0.5f;
    float sizeScore = 1.0f - std::min(1.0f, std::abs(radius - optimalRadius) / (sizeRange * 0.5f));
    confidence += sizeScore * 0.3f;

    // Contrast score
    cv::Rect innerRect(cx - radius / 2, cy - radius / 2, radius, radius);
    cv::Rect outerRect(cx - radius, cy - radius, radius * 2, radius * 2);

    innerRect &= cv::Rect(0, 0, image.cols, image.rows);
    outerRect &= cv::Rect(0, 0, image.cols, image.rows);

    if (innerRect.area() > 0 && outerRect.area() > innerRect.area()) {
        double innerMean = cv::mean(image(innerRect))[0];
        double outerMean = cv::mean(image(outerRect))[0];
        float contrastScore = std::min(1.0f, static_cast<float>(std::abs(innerMean - outerMean) / 100.0));
        confidence += contrastScore * 0.2f;
    }

    return std::min(1.0f, confidence);
}

float BallDetector::Impl::calculateCircularityFast(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Create a small ROI around the circle
    const int margin = 5;
    cv::Rect roi(cx - radius - margin, cy - radius - margin,
        (radius + margin) * 2, (radius + margin) * 2);
    roi &= cv::Rect(0, 0, image.cols, image.rows);

    if (roi.area() == 0) return 0.0f;

    cv::Mat roiImage = image(roi);
    cv::Mat binary;
    cv::threshold(roiImage, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // find contour
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return 0.0f;

    // Find contour closest to expected circle center
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

    // circularity
    double area = cv::contourArea(contours[bestIdx]);
    double perimeter = cv::arcLength(contours[bestIdx], true);

    if (perimeter <= 0) return 0.0f;

    return static_cast<float>(std::min(1.0, 4.0 * CV_PI * area / (perimeter * perimeter)));
}

std::vector<BallInfo> BallDetector::Impl::evaluateCandidatesTBB(
    const cv::Mat& image,
    const std::vector<cv::Vec3f>& candidates,
    const DetectionParams& params,
    float scaleFactor,
    const cv::Rect& roiRect,
    int frameIndex) {

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

                    // Calculate scores
                    info.confidence = calculateConfidenceFast(image, circle, params);

                    if (params.useCircularityCheck && !params.fastMode) {
                        info.circularity = calculateCircularityFast(image, circle, params);
                        if (info.circularity < params.minCircularity) {
                            continue;
                        }
                    }
                    else {
                        info.circularity = 1.0f;
                    }

                    // brightness
                    cv::Rect ballRect(cvRound(circle[0] - circle[2] / 2),
                        cvRound(circle[1] - circle[2] / 2),
                        cvRound(circle[2]), cvRound(circle[2]));
                    ballRect &= cv::Rect(0, 0, image.cols, image.rows);
                    if (ballRect.area() > 0) {
                        info.brightness = static_cast<float>(cv::mean(image(ballRect))[0]);
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

cv::Point2f BallDetector::Impl::transformToOriginalCoords(const cv::Point2f& roiPoint,
    const cv::Size& roiSize,
    const cv::Size& origSize,
    float scale) {
    cv::Point2f originalPoint;
    originalPoint.x = roiPoint.x + (origSize.width - roiSize.width) / 2.0f;
    originalPoint.y = roiPoint.y + (origSize.height - roiSize.height) / 2.0f;
    return originalPoint;
}

void BallDetector::Impl::saveIntermediateImages(const std::string& basePath, int frameIndex) {
    std::string prefix = basePath + "/frame_" + std::to_string(frameIndex);

    try {
        if (!m_lastProcessedImage.empty()) {
            std::string path = prefix + "_01_processed.png";
            if (cv::imwrite(path, m_lastProcessedImage)) {
                LOG_DEBUG("Saved processed image: " + path);
            }
        }
        if (!m_lastEdgeImage.empty()) {
            std::string path = prefix + "_02_edges.png";
            if (cv::imwrite(path, m_lastEdgeImage)) {
                LOG_DEBUG("Saved edge image: " + path);
            }
        }
        if (!m_lastBinaryImage.empty()) {
            std::string path = prefix + "_03_binary.png";
            if (cv::imwrite(path, m_lastBinaryImage)) {
                LOG_DEBUG("Saved binary image: " + path);
            }
        }
        if (!m_lastShadowEnhanced.empty()) {
            std::string path = prefix + "_04_shadow_enhanced.png";
            if (cv::imwrite(path, m_lastShadowEnhanced)) {
                LOG_DEBUG("Saved shadow enhanced image: " + path);
            }
        }
    }
    catch (const cv::Exception& e) {
        LOG_ERROR("Failed to save intermediate images: " + std::string(e.what()));
    }
}

double BallDetector::EstimateProcessingTime(int width, int height) const {
    double pixels = width * height;
    double scaledPixels = pixels;

    if (m_params.useROI) {
        scaledPixels *= (m_params.roiScale * m_params.roiScale);
    }

    if (m_params.downscaleFactor > 1) {
        scaledPixels /= (m_params.downscaleFactor * m_params.downscaleFactor);
    }

    // Base time estimation
    double baseTime = scaledPixels / 50000.0;  // ~20ms for 1MP

    if (m_params.fastMode) {
        baseTime *= 0.5;
    }

    if (m_params.useParallel) {
        baseTime /= std::min(4.0, static_cast<double>(m_params.processingThreads));
    }

    return baseTime;
}

bool BallDetector::SaveDetectionImage(const unsigned char* originalImage, int width, int height,
    const BallDetectionResult& result,
    const std::string& outputPath,
    bool saveAsColor) {
    if (!originalImage || width <= 0 || height <= 0) {
        LOG_ERROR("Invalid parameters for SaveDetectionImage");
        return false;
    }

    try {
        cv::Mat grayImg(height, width, CV_8UC1, const_cast<unsigned char*>(originalImage));
        cv::Mat colorImg;
        cv::cvtColor(grayImg, colorImg, cv::COLOR_GRAY2BGR);

        // title
        cv::rectangle(colorImg, cv::Point(0, 0), cv::Point(width, 50),
            cv::Scalar(40, 40, 40), cv::FILLED);
        cv::putText(colorImg, "Ball Detection Result",
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
            cv::Scalar(0, 255, 255), 2);

        if (result.found && !result.balls.empty()) {
            int infoY = 70;

            for (size_t i = 0; i < result.balls.size() && i < MAX_DISPLAY_BALLS; ++i) {
                const BallInfo& ball = result.balls[i];
                cv::Point center(cvRound(ball.center.x), cvRound(ball.center.y));
                int radius = cvRound(ball.radius);

                // filled circle
                cv::Mat overlay = colorImg.clone();
                cv::circle(overlay, center, radius, cv::Scalar(0, 0, 255), -1);
                cv::addWeighted(colorImg, 0.7, overlay, 0.3, 0, colorImg);

                // Circle outline
                cv::circle(colorImg, center, radius, cv::Scalar(0, 0, 255), 3);

                // Center crosshair
                cv::drawMarker(colorImg, center,
                    cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 20, 2);

                // Ball ID
                std::string ballNum = "B" + std::to_string(i + 1);
                cv::putText(colorImg, ballNum,
                    cv::Point(center.x - 15, center.y + 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

                // Info
                cv::Rect infoBg(10, infoY - 20, INFO_PANEL_WIDTH, INFO_PANEL_HEIGHT);
                infoBg &= cv::Rect(0, 0, width, height);
                cv::rectangle(colorImg, infoBg, cv::Scalar(20, 20, 20), cv::FILLED);
                cv::rectangle(colorImg, infoBg, cv::Scalar(100, 100, 100), 1);

                // Ball info
                std::ostringstream info;
                info << "Ball " << (i + 1) << ":";
                cv::putText(colorImg, info.str(),
                    cv::Point(15, infoY),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

                info.str("");
                info << "  Center: (" << cvRound(ball.center.x) << ", " << cvRound(ball.center.y) << ")";
                cv::putText(colorImg, info.str(),
                    cv::Point(15, infoY + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                info.str("");
                info << "  Radius: " << radius << " pixels";
                cv::putText(colorImg, info.str(),
                    cv::Point(15, infoY + 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                info.str("");
                info << "  Confidence: " << std::fixed << std::setprecision(1) << (ball.confidence * 100) << "%";
                cv::Scalar confColor = (ball.confidence > 0.8) ? cv::Scalar(0, 255, 0) :
                    (ball.confidence > 0.6) ? cv::Scalar(0, 255, 255) :
                    cv::Scalar(0, 165, 255);
                cv::putText(colorImg, info.str(),
                    cv::Point(15, infoY + 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, confColor, 1);

                infoY += 100;
            }

            // Summary
            cv::Rect summaryBg(0, height - 30, width, 30);
            cv::rectangle(colorImg, summaryBg, cv::Scalar(40, 40, 40), cv::FILLED);

            std::ostringstream summary;
            summary << "Total balls: " << result.balls.size()
                << " | Frame: " << (result.balls.empty() ? 0 : result.balls[0].frameIndex);
            cv::putText(colorImg, summary.str(),
                cv::Point(10, height - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
        else {
            // No ball
            cv::Rect msgBg(10, 60, 200, 40);
            cv::rectangle(colorImg, msgBg, cv::Scalar(0, 0, 100), cv::FILLED);
            cv::putText(colorImg, "No ball detected",
                cv::Point(15, 85), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 0, 255), 2);
        }

        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        struct tm localTime;
        localtime_s(&localTime, &t);

        std::ostringstream timestamp;
        timestamp << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S");
        cv::putText(colorImg, timestamp.str(),
            cv::Point(width - 180, height - 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

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