// BallDetector.cpp - Complete Optimized Version with TBB
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

// Performance profiling control
#define ENABLE_PERFORMANCE_PROFILING

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

// Constructor for DetectionParams struct
BallDetector::DetectionParams::DetectionParams() {
    // Optimized parameters for faster detection
    dp = 2.0;
    minDist = 40.0;
    param1 = 120.0;
    param2 = 0.85;
    minRadius = 20;
    maxRadius = 40;
    brightnessThreshold = 120;
    minCircularity = 0.65f;
    contrastThreshold = 15.0f;
    useColorFilter = true;
    useCircularityCheck = true;
    useHoughGradientAlt = false;
    detectMultiple = false;
    useMorphology = false;
    useAdaptiveThreshold = false;
    correctPerspective = false;
    enhanceShadows = false;
    shadowEnhanceFactor = 0.7f;
    saveIntermediateImages = false;
    debugOutputDir = "";

    // New optimization parameters
    useROI = true;
    roiScale = 0.8f;
    downscaleFactor = 2;
    useParallel = true;
    maxCandidates = 20;
    fastMode = true;
    skipPreprocessing = false;
    edgeThreshold = 100;
    useCache = false;
    processingThreads = std::max(2u, std::thread::hardware_concurrency() / 2);
}

// BallDetector implementation class
class BallDetector::Impl {
public:
    // Cache for frequently used data
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;
    cv::Mat m_lastProcessedImage;
    cv::Mat m_lastBinaryImage;
    cv::Mat m_lastEdgeImage;
    cv::Mat m_lastShadowEnhanced;

    // Pre-allocated buffers
    cv::Mat m_tempBuffer1;
    cv::Mat m_tempBuffer2;
    cv::Mat m_downscaledImage;
    cv::Mat m_roiMask;
    cv::Mat m_blurKernel;

    // LUT for fast calculations
    std::vector<uint8_t> m_sqrtLUT;
    std::vector<float> m_normalizeLUT;

    Impl() {
        // Pre-allocate common buffers
        m_tempBuffer1.create(960, 1280, CV_8UC1);
        m_tempBuffer2.create(960, 1280, CV_8UC1);

        // Pre-create Gaussian kernel
        m_blurKernel = cv::getGaussianKernel(3, 1.0);

        // Initialize LUTs
        m_sqrtLUT.resize(256 * 256);
        for (int i = 0; i < 256 * 256; ++i) {
            m_sqrtLUT[i] = static_cast<uint8_t>(std::sqrt(i));
        }

        m_normalizeLUT.resize(256);
        for (int i = 0; i < 256; ++i) {
            m_normalizeLUT[i] = i / 255.0f;
        }
    }

    // Optimized preprocessing
    cv::Mat preprocessImageFast(const cv::Mat& grayImage, const DetectionParams& params);
    cv::Mat enhanceShadowRegionsFast(const cv::Mat& image, float factor);

    // Fast detection methods
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
    std::vector<BallInfo> evaluateCandidatesTBB(const cv::Mat& image, const std::vector<cv::Vec3f>& candidates, const DetectionParams& params, float scaleFactor, const cv::Rect& roiRect, int frameIndex);

    // Utility functions
    void saveIntermediateImages(const std::string& basePath, int frameIndex);
    inline float fastDistance(float x1, float y1, float x2, float y2) {
        float dx = x2 - x1;
        float dy = y2 - y1;
        return m_sqrtLUT[static_cast<int>(dx * dx + dy * dy)];
    }
};

// BallDetector Constructor
BallDetector::BallDetector() : m_params(), pImpl(std::make_unique<Impl>()), m_performanceProfilingEnabled(false) {
    cv::setNumThreads(0);
    cv::setUseOptimized(true);

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

void BallDetector::SetCalibrationData(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
    pImpl->m_cameraMatrix = cameraMatrix.clone();
    pImpl->m_distCoeffs = distCoeffs.clone();
    m_params.correctPerspective = true;
    LOG_INFO("Camera calibration data set. Perspective correction enabled.");
}

void BallDetector::EnablePerformanceProfiling(bool enable) {
    m_performanceProfilingEnabled = enable;
    LOG_INFO("Performance profiling " + std::string(enable ? "enabled" : "disabled"));
}

bool BallDetector::IsPerformanceProfilingEnabled() const {
    return m_performanceProfilingEnabled;
}

// Main detection function - heavily optimized with TBB
BallDetectionResult BallDetector::DetectBall(const unsigned char* imageData, int width, int height, int frameIndex) {
    auto totalStartTime = std::chrono::high_resolution_clock::now();
    m_lastMetrics.Reset();

    BallDetectionResult result;
    result.found = false;
    result.balls.clear();

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

            if (newWidth >= 160 && newHeight >= 120) {
                cv::resize(processImage, pImpl->m_downscaledImage,
                    cv::Size(newWidth, newHeight),
                    0, 0, cv::INTER_AREA);  // INTER_AREA for downsampling
                workingImage = pImpl->m_downscaledImage;
                scaleFactor = static_cast<float>(m_params.downscaleFactor);
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

                int maxBalls = std::min(5, static_cast<int>(validBalls.size()));
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

        // 9. Save debug images only if necessary
        if (m_params.saveIntermediateImages && !m_params.debugOutputDir.empty()) {
            MEASURE_TIME("Debug image saving",
                fs::create_directories(m_params.debugOutputDir);
            pImpl->saveIntermediateImages(m_params.debugOutputDir, frameIndex);
            if (result.found) {
                SaveDetectionImage(imageData, width, height, result,
                    m_params.debugOutputDir + "/frame_" + std::to_string(frameIndex) + "_result.png");
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

    auto totalEndTime = std::chrono::high_resolution_clock::now();
    m_lastMetrics.totalDetectionTime_ms =
        std::chrono::duration_cast<std::chrono::microseconds>(totalEndTime - totalStartTime).count() / 1000.0;

    if (m_performanceProfilingEnabled && (frameIndex % 100 == 0)) {
        LOG_INFO("Frame " + std::to_string(frameIndex) + " - Total detection time: " +
            std::to_string(m_lastMetrics.totalDetectionTime_ms) + " ms");
    }

    return result;
}

// Optimized preprocessing
cv::Mat BallDetector::Impl::preprocessImageFast(const cv::Mat& grayImage, const DetectionParams& params) {
    cv::Mat processed = grayImage;

    // Fast Gaussian blur using separable filter
    if (params.fastMode) {
        cv::GaussianBlur(processed, processed, cv::Size(3, 3), 0.75, 0.75, cv::BORDER_REPLICATE);
    }
    else {
        cv::GaussianBlur(processed, processed, cv::Size(5, 5), 1.0, 1.0, cv::BORDER_REPLICATE);
    }

    // Optional shadow enhancement (only if really needed)
    if (params.enhanceShadows && !params.fastMode) {
        processed = enhanceShadowRegionsFast(processed, params.shadowEnhanceFactor);
    }

    // Simple contrast normalization (faster than CLAHE)
    if (!params.fastMode && params.contrastThreshold > 0) {
        cv::normalize(processed, processed, 0, 255, cv::NORM_MINMAX);
    }

    return processed;
}

// Fast shadow enhancement
cv::Mat BallDetector::Impl::enhanceShadowRegionsFast(const cv::Mat& image, float factor) {
    cv::Mat result;
    image.copyTo(result);

    // TBB parallel processing for shadow enhancement
    tbb::parallel_for(tbb::blocked_range<int>(0, image.rows),
        [&](const tbb::blocked_range<int>& range) {
            for (int y = range.begin(); y != range.end(); ++y) {
                const uchar* src = image.ptr<uchar>(y);
                uchar* dst = result.ptr<uchar>(y);
                for (int x = 0; x < image.cols; ++x) {
                    if (src[x] < 100) {
                        dst[x] = cv::saturate_cast<uchar>(src[x] * (1.0f + factor));
                    }
                }
            }
        }
    );

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

    // Use standard Hough gradient (faster than ALT)
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
    }

    return circles;
}

// Fast contour-based circle detection
std::vector<cv::Vec3f> BallDetector::Impl::detectCirclesByContourFast(const cv::Mat& binaryImage, const DetectionParams& params) {
    std::vector<cv::Vec3f> circles;
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Pre-calculate area limits
    float minArea = static_cast<float>(CV_PI * params.minRadius * params.minRadius);
    float maxArea = static_cast<float>(CV_PI * params.maxRadius * params.maxRadius);

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

    return circles;
}

// Quick circle validation
bool BallDetector::Impl::quickValidateCircle(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Boundary check
    if (cx - radius < 0 || cy - radius < 0 ||
        cx + radius >= image.cols || cy + radius >= image.rows) {
        return false;
    }

    // Quick brightness check
    if (params.useColorFilter) {
        cv::Rect sampleRect(cx - 2, cy - 2, 5, 5);
        sampleRect &= cv::Rect(0, 0, image.cols, image.rows);
        double brightness = cv::mean(image(sampleRect))[0];
        if (brightness < params.brightnessThreshold * 0.7) {
            return false;
        }
    }

    // Quick contrast check
    if (params.contrastThreshold > 0) {
        // Sample 4 cardinal points
        int sampleRadius = radius / 2;
        int innerSum = 0, outerSum = 0;

        const int offsets[4][2] = { {0, -1}, {0, 1}, {-1, 0}, {1, 0} };
        for (int i = 0; i < 4; ++i) {
            int ix = cx + offsets[i][0] * sampleRadius;
            int iy = cy + offsets[i][1] * sampleRadius;
            int ox = cx + offsets[i][0] * radius * 3 / 2;
            int oy = cy + offsets[i][1] * radius * 3 / 2;

            if (ix >= 0 && ix < image.cols && iy >= 0 && iy < image.rows) {
                innerSum += image.at<uchar>(iy, ix);
            }
            if (ox >= 0 && ox < image.cols && oy >= 0 && oy < image.rows) {
                outerSum += image.at<uchar>(oy, ox);
            }
        }

        float contrast = std::abs(innerSum - outerSum) / 4.0f;
        if (contrast < params.contrastThreshold) {
            return false;
        }
    }

    return true;
}

// Fast confidence calculation
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

    // Contrast score using integral image (if available)
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

// Fast circularity calculation
float BallDetector::Impl::calculateCircularityFast(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Create a small ROI around the circle
    int margin = 5;
    cv::Rect roi(cx - radius - margin, cy - radius - margin,
        (radius + margin) * 2, (radius + margin) * 2);
    roi &= cv::Rect(0, 0, image.cols, image.rows);

    if (roi.area() == 0) return 0.0f;

    cv::Mat roiImage = image(roi);
    cv::Mat binary;
    cv::threshold(roiImage, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Find the largest contour
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

    // Calculate circularity
    double area = cv::contourArea(contours[bestIdx]);
    double perimeter = cv::arcLength(contours[bestIdx], true);

    if (perimeter <= 0) return 0.0f;

    return static_cast<float>(std::min(1.0, 4.0 * CV_PI * area / (perimeter * perimeter)));
}

// TBB-optimized candidate evaluation
std::vector<BallInfo> BallDetector::Impl::evaluateCandidatesTBB(const cv::Mat& image, const std::vector<cv::Vec3f>& candidates, const DetectionParams& params, float scaleFactor, const cv::Rect& roiRect, int frameIndex) {

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

                    // brightness calc
                    cv::Rect ballRect(cvRound(circle[0] - circle[2] / 2),
                        cvRound(circle[1] - circle[2] / 2),
                        cvRound(circle[2]), cvRound(circle[2]));
                    ballRect &= cv::Rect(0, 0, image.cols, image.rows);
                    if (ballRect.area() > 0) {
                        info.brightness = static_cast<float>(cv::mean(image(ballRect))[0]);
                    }

                    if (info.confidence >= 0.4f) {
                        concurrentValidBalls.push_back(info);
                    }
                }
            }
        }
    );

    std::vector<BallInfo> validBalls(concurrentValidBalls.begin(), concurrentValidBalls.end());

    return validBalls;
}

// Extract ROI
cv::Mat BallDetector::Impl::extractROI(const cv::Mat& image, float scale) {
    int roiWidth = static_cast<int>(image.cols * scale);
    int roiHeight = static_cast<int>(image.rows * scale);

    // Ensure even dimensions
    roiWidth = (roiWidth / 2) * 2;
    roiHeight = (roiHeight / 2) * 2;

    int x = (image.cols - roiWidth) / 2;
    int y = (image.rows - roiHeight) / 2;

    return image(cv::Rect(x, y, roiWidth, roiHeight));
}

// Transform ROI coordinates to original
cv::Point2f BallDetector::Impl::transformToOriginalCoords(const cv::Point2f& roiPoint,
    const cv::Size& roiSize,
    const cv::Size& origSize,
    float scale) {
    cv::Point2f originalPoint;
    originalPoint.x = roiPoint.x + (origSize.width - roiSize.width) / 2.0f;
    originalPoint.y = roiPoint.y + (origSize.height - roiSize.height) / 2.0f;
    return originalPoint;
}

// Save intermediate images for debugging
void BallDetector::Impl::saveIntermediateImages(const std::string& basePath, int frameIndex) {
    std::string prefix = basePath + "/frame_" + std::to_string(frameIndex);

    if (!m_lastProcessedImage.empty()) {
        cv::imwrite(prefix + "_01_processed.png", m_lastProcessedImage);
    }
    if (!m_lastEdgeImage.empty()) {
        cv::imwrite(prefix + "_02_edges.png", m_lastEdgeImage);
    }
    if (!m_lastBinaryImage.empty()) {
        cv::imwrite(prefix + "_03_binary.png", m_lastBinaryImage);
    }
}

// Public configuration methods
void BallDetector::EnableFastMode(bool enable) {
    m_params.fastMode = enable;
    m_params.useROI = enable;
    m_params.downscaleFactor = enable ? 2 : 1;
    m_params.useParallel = enable;
    m_params.enhanceShadows = !enable;
    m_params.useMorphology = !enable;
    m_params.useAdaptiveThreshold = !enable;
    m_params.maxCandidates = enable ? 10 : 20;
    LOG_INFO("Fast mode " + std::string(enable ? "enabled" : "disabled"));
}

void BallDetector::SetDownscaleFactor(int factor) {
    m_params.downscaleFactor = std::max(1, std::min(4, factor));
    LOG_INFO("Downscale factor set to " + std::to_string(m_params.downscaleFactor));
}

void BallDetector::SetROIScale(float scale) {
    m_params.roiScale = std::max(0.5f, std::min(1.0f, scale));
    LOG_INFO("ROI scale set to " + std::to_string(m_params.roiScale));
}

void BallDetector::SetMaxCandidates(int maxCandidates) {
    m_params.maxCandidates = std::max(1, std::min(50, maxCandidates));
    LOG_INFO("Max candidates set to " + std::to_string(m_params.maxCandidates));
}

void BallDetector::SetProcessingThreads(int threads) {
    m_params.processingThreads = std::max(1, std::min(32, threads));
    LOG_INFO("Processing threads set to " + std::to_string(m_params.processingThreads));
}

// Auto-tune parameters
void BallDetector::AutoTuneParameters(const std::vector<unsigned char*>& sampleImages, int width, int height) {
    LOG_INFO("Starting auto-tune with " + std::to_string(sampleImages.size()) + " sample images");

    struct ParameterSet {
        float roiScale;
        int downscaleFactor;
        float param2;
        float score;
    };

    std::vector<ParameterSet> paramSets;

    // Generate parameter combinations
    for (float roi = 0.7f; roi <= 1.0f; roi += 0.1f) {
        for (int ds = 1; ds <= 3; ++ds) {
            for (float p2 = 0.8f; p2 <= 1.0f; p2 += 0.05f) {
                paramSets.push_back({ roi, ds, p2, 0.0f });
            }
        }
    }

    // Test each parameter set in parallel
    tbb::parallel_for(tbb::blocked_range<size_t>(0, paramSets.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                auto& paramSet = paramSets[i];

                // Create temporary detector with test parameters
                DetectionParams testParams = m_params;
                testParams.roiScale = paramSet.roiScale;
                testParams.downscaleFactor = paramSet.downscaleFactor;
                testParams.param2 = paramSet.param2;

                float totalConfidence = 0;
                int detectionCount = 0;

                for (const auto& imageData : sampleImages) {
                    auto tempParams = m_params;
                    m_params = testParams;

                    auto result = DetectBall(imageData, width, height, 0);

                    m_params = tempParams;

                    if (result.found && !result.balls.empty()) {
                        totalConfidence += result.balls[0].confidence;
                        detectionCount++;
                    }
                }

                if (detectionCount > 0) {
                    paramSet.score = (totalConfidence / detectionCount) *
                        (static_cast<float>(detectionCount) / sampleImages.size());
                }
            }
        }
    );

    // Find best parameters
    auto best = std::max_element(paramSets.begin(), paramSets.end(),
        [](const ParameterSet& a, const ParameterSet& b) { return a.score < b.score; });

    if (best != paramSets.end() && best->score > 0) {
        m_params.roiScale = best->roiScale;
        m_params.downscaleFactor = best->downscaleFactor;
        m_params.param2 = best->param2;

        LOG_INFO("Auto-tune complete. Best parameters: ROI=" + std::to_string(best->roiScale) +
            ", Downscale=" + std::to_string(best->downscaleFactor) +
            ", Param2=" + std::to_string(best->param2) +
            ", Score=" + std::to_string(best->score));
    }
}

// Generate performance report
std::string BallDetector::GeneratePerformanceReport() const {
    std::ostringstream report;
    report << std::fixed << std::setprecision(2);
    report << "\n========== Ball Detection Performance Report ==========\n";

    if (!m_performanceProfilingEnabled) {
        report << "Performance profiling is disabled.\n";
        report << "Call EnablePerformanceProfiling(true) to enable timing measurements.\n";
        report << "======================================================\n";
        return report.str();
    }

    report << "Total Detection Time: " << m_lastMetrics.totalDetectionTime_ms << " ms\n";

    if (m_lastMetrics.totalDetectionTime_ms > 0) {
        report << "Breakdown:\n";
        report << "  - Preprocessing: " << m_lastMetrics.preprocessingTime_ms << " ms ("
            << (m_lastMetrics.preprocessingTime_ms / m_lastMetrics.totalDetectionTime_ms * 100) << "%)\n";
        report << "  - Hough Detection: " << m_lastMetrics.houghDetectionTime_ms << " ms ("
            << (m_lastMetrics.houghDetectionTime_ms / m_lastMetrics.totalDetectionTime_ms * 100) << "%)\n";

        if (m_lastMetrics.contourDetectionTime_ms > 0) {
            report << "  - Contour Detection: " << m_lastMetrics.contourDetectionTime_ms << " ms ("
                << (m_lastMetrics.contourDetectionTime_ms / m_lastMetrics.totalDetectionTime_ms * 100) << "%)\n";
        }

        report << "  - Candidate Evaluation: " << m_lastMetrics.candidateEvaluationTime_ms << " ms ("
            << (m_lastMetrics.candidateEvaluationTime_ms / m_lastMetrics.totalDetectionTime_ms * 100) << "%)\n";

        if (m_lastMetrics.imagesSavingTime_ms > 0) {
            report << "  - Image Saving: " << m_lastMetrics.imagesSavingTime_ms << " ms ("
                << (m_lastMetrics.imagesSavingTime_ms / m_lastMetrics.totalDetectionTime_ms * 100) << "%)\n";
        }
    }

    report << "\nDetection Statistics:\n";
    report << "  - Candidates found: " << m_lastMetrics.candidatesFound << "\n";
    report << "  - Candidates evaluated: " << m_lastMetrics.candidatesEvaluated << "\n";
    report << "  - Result: " << (m_lastMetrics.ballDetected ? "Ball detected" : "No ball detected") << "\n";

    report << "\nCurrent Settings:\n";
    report << "  - Fast Mode: " << (m_params.fastMode ? "ON" : "OFF") << "\n";
    report << "  - ROI Scale: " << m_params.roiScale << "\n";
    report << "  - Downscale Factor: " << m_params.downscaleFactor << "\n";
    report << "  - Max Candidates: " << m_params.maxCandidates << "\n";
    report << "  - Parallel Processing: " << (m_params.useParallel ? "ON" : "OFF") << "\n";
    report << "  - Processing Threads: " << m_params.processingThreads << "\n";

    // Performance analysis
    report << "\nPerformance Analysis:\n";
    if (m_lastMetrics.totalDetectionTime_ms < 10) {
        report << "  ✓ Excellent: Processing under 10ms\n";
    }
    else if (m_lastMetrics.totalDetectionTime_ms < 20) {
        report << "  ✓ Good: Processing under 20ms (suitable for 50 FPS)\n";
    }
    else if (m_lastMetrics.totalDetectionTime_ms < 33) {
        report << "  ⚠ Fair: Processing under 33ms (suitable for 30 FPS)\n";
    }
    else {
        report << "  ✗ Poor: Processing over 33ms (may impact real-time performance)\n";
    }

    report << "======================================================\n";

    return report.str();
}

// Estimate processing time
double BallDetector::EstimateProcessingTime(int width, int height) const {
    double pixels = width * height;
    double scaledPixels = pixels;

    if (m_params.useROI) {
        scaledPixels *= (m_params.roiScale * m_params.roiScale);
    }

    if (m_params.downscaleFactor > 1) {
        scaledPixels /= (m_params.downscaleFactor * m_params.downscaleFactor);
    }

    // Base time estimation (empirical values)
    double baseTime = scaledPixels / 50000.0;  // ~20ms for 1MP

    if (m_params.fastMode) {
        baseTime *= 0.5;
    }

    if (m_params.useParallel) {
        baseTime /= std::min(4.0, static_cast<double>(m_params.processingThreads));
    }

    return baseTime;
}

// Check if can achieve target FPS
bool BallDetector::CanAchieveTargetFPS(int targetFPS, int imageWidth, int imageHeight) const {
    double estimatedTime = EstimateProcessingTime(imageWidth, imageHeight);
    double targetFrameTime = 1000.0 / targetFPS;
    return estimatedTime <= targetFrameTime;
}

// Get recommended settings
BallDetector::DetectionParams BallDetector::GetRecommendedSettings(int targetFPS, int imageWidth, int imageHeight) const {
    DetectionParams recommended = m_params;
    double targetFrameTime = 1000.0 / targetFPS;

    if (targetFrameTime < 10) {
        // Ultra-fast settings for > 100 FPS
        recommended.fastMode = true;
        recommended.useROI = true;
        recommended.roiScale = 0.6f;
        recommended.downscaleFactor = 3;
        recommended.maxCandidates = 5;
        recommended.skipPreprocessing = true;
    }
    else if (targetFrameTime < 20) {
        // Fast settings for 50-100 FPS
        recommended.fastMode = true;
        recommended.useROI = true;
        recommended.roiScale = 0.7f;
        recommended.downscaleFactor = 2;
        recommended.maxCandidates = 10;
    }
    else if (targetFrameTime < 33) {
        // Balanced settings for 30-50 FPS
        recommended.fastMode = false;
        recommended.useROI = true;
        recommended.roiScale = 0.8f;
        recommended.downscaleFactor = 1;
        recommended.maxCandidates = 20;
    }
    else {
        // Quality settings for < 30 FPS
        recommended.fastMode = false;
        recommended.useROI = false;
        recommended.roiScale = 1.0f;
        recommended.downscaleFactor = 1;
        recommended.maxCandidates = 30;
    }

    return recommended;
}

// Draw detection result
bool BallDetector::DrawDetectionResult(unsigned char* imageData, int width, int height,
    const BallDetectionResult& result,
    cv::Scalar color, int thickness) {
    if (!imageData || width <= 0 || height <= 0) {
        LOG_ERROR("Invalid parameters for DrawDetectionResult");
        return false;
    }

    try {
        cv::Mat grayImg(height, width, CV_8UC1, imageData);
        cv::Mat colorImg;
        cv::cvtColor(grayImg, colorImg, cv::COLOR_GRAY2BGR);

        // Draw title
        cv::putText(colorImg, "Ball Detection - Optimized",
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
            cv::Scalar(0, 255, 0), 2);

        if (result.found && !result.balls.empty()) {
            for (const auto& ball : result.balls) {
                // Draw circle
                cv::circle(colorImg, cv::Point(cvRound(ball.center.x), cvRound(ball.center.y)),
                    cvRound(ball.radius), color, thickness);

                // Draw center marker
                cv::drawMarker(colorImg, cv::Point(cvRound(ball.center.x), cvRound(ball.center.y)),
                    cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 2);

                // Draw confidence text
                std::ostringstream conf;
                conf << std::fixed << std::setprecision(0) << (ball.confidence * 100) << "%";
                cv::putText(colorImg, conf.str(),
                    cv::Point(cvRound(ball.center.x) - 20, cvRound(ball.center.y) - ball.radius - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            }
        }
        else {
            cv::putText(colorImg, "No ball detected",
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 0, 255), 1);
        }

        // Convert back to grayscale
        cv::cvtColor(colorImg, grayImg, cv::COLOR_BGR2GRAY);

        return true;
    }
    catch (const cv::Exception& e) {
        LOG_ERROR("OpenCV error in DrawDetectionResult: " + std::string(e.what()));
        return false;
    }
}

// Save detection image
bool BallDetector::SaveDetectionImage(const unsigned char* originalImage, int width, int height,
    const BallDetectionResult& result,
    const std::string& outputPath) {
    if (!originalImage || width <= 0 || height <= 0) {
        LOG_ERROR("Invalid parameters for SaveDetectionImage");
        return false;
    }

    try {
        cv::Mat grayImg(height, width, CV_8UC1, const_cast<unsigned char*>(originalImage));
        cv::Mat colorImg;
        cv::cvtColor(grayImg, colorImg, cv::COLOR_GRAY2BGR);

        // Add timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        struct tm localTime;
        localtime_s(&localTime, &t);

        std::ostringstream timestamp;
        timestamp << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S");
        cv::putText(colorImg, timestamp.str(), cv::Point(10, height - 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        // Title
        cv::putText(colorImg, "Ball Detection Result - TBB Optimized",
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
            cv::Scalar(0, 255, 0), 2);

        if (result.found && !result.balls.empty()) {
            for (size_t i = 0; i < result.balls.size(); ++i) {
                const BallInfo& ball = result.balls[i];

                // Draw circle
                cv::circle(colorImg, cv::Point(cvRound(ball.center.x), cvRound(ball.center.y)),
                    cvRound(ball.radius), cv::Scalar(0, 255, 0), 2);

                // Draw center
                cv::drawMarker(colorImg, cv::Point(cvRound(ball.center.x), cvRound(ball.center.y)),
                    cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 15, 2);

                // Draw info box
                int boxX = 10;
                int boxY = 50 + static_cast<int>(i * 25);

                std::ostringstream info;
                info << "Ball " << (i + 1) << ": ("
                    << cvRound(ball.center.x) << ", " << cvRound(ball.center.y) << ") "
                    << "R=" << cvRound(ball.radius) << " "
                    << "Conf=" << std::fixed << std::setprecision(1) << (ball.confidence * 100) << "%";

                cv::putText(colorImg, info.str(), cv::Point(boxX, boxY),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }
        }
        else {
            cv::putText(colorImg, "No ball detected",
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 0, 255), 2);
        }

        // Save image
        bool success = cv::imwrite(outputPath, colorImg);

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
}