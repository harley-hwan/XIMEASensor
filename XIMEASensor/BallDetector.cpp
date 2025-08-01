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
#include <thread>
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
    constexpr float MIN_CONFIDENCE_THRESHOLD = 0.4f;
    constexpr int SHADOW_THRESHOLD = 100;
    constexpr float SHADOW_ENHANCEMENT_BASE = 1.0f;
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

// Constructor 
BallDetector::DetectionParams::DetectionParams() {
    // Circle detection
    minRadius = 20;
    maxRadius = 30;
    minCircularity = 0.70f;

    // Hough params
    dp = 2.0;
    minDist = 40.0;
    param1 = 130.0;
    param2 = 0.87;

    // Thresholding
    brightnessThreshold = 120;
    useAdaptiveThreshold = false;

    // Validation
    useColorFilter = false;
    useCircularityCheck = false;
    contrastThreshold = 20.0f;
    detectMultiple = false;

    // Preprocessing
    skipPreprocessing = false;
    enhanceShadows = false;
    shadowEnhanceFactor = 0.7f;
    useMorphology = false;

    // Performance
    fastMode = true;
    useROI = true;
    roiScale = 0.75f;
    downscaleFactor = 2;
    useParallel = true;
    maxCandidates = 15;
    processingThreads = std::max(2u, std::thread::hardware_concurrency() / 2);

    // Debug
    saveIntermediateImages = false;
    debugOutputDir = "";
}

// PerformanceMetrics Reset
void BallDetector::PerformanceMetrics::Reset() {
    totalDetectionTime_ms = 0;
    roiExtractionTime_ms = 0;
    downscaleTime_ms = 0;
    preprocessingTime_ms = 0;
    thresholdingTime_ms = 0;
    morphologyTime_ms = 0;
    contourDetectionTime_ms = 0;
    houghDetectionTime_ms = 0;
    candidateEvaluationTime_ms = 0;
    imagesSavingTime_ms = 0;
    candidatesFound = 0;
    candidatesEvaluated = 0;
    ballDetected = false;
}

// BallDetector implementation
class BallDetector::Impl {
public:
    // Debug image buffers
    cv::Mat m_lastProcessedImage;
    cv::Mat m_lastBinaryImage;
    cv::Mat m_lastShadowEnhanced;
    cv::Mat m_downscaledImage;

    std::string m_currentCaptureFolder;
    const DetectionParams* m_paramsPtr;

    Impl() : m_currentCaptureFolder(""), m_paramsPtr(nullptr) {}

    void setCurrentCaptureFolder(const std::string& folder) { m_currentCaptureFolder = folder; }
    void setParamsReference(const DetectionParams* params) { m_paramsPtr = params; }

    cv::Mat preprocessImage(const cv::Mat& grayImage, const DetectionParams& params);
    cv::Mat enhanceShadowRegions(const cv::Mat& image, float factor);
    std::vector<cv::Vec3f> detectCirclesHough(const cv::Mat& image, const DetectionParams& params);
    bool quickValidateCircle(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
    float calculateConfidence(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
    float calculateCircularity(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
    cv::Mat extractROI(const cv::Mat& image, float scale);
    std::vector<BallInfo> evaluateCandidatesTBB(const cv::Mat& image, const std::vector<cv::Vec3f>& candidates,
        const DetectionParams& params, float scaleFactor,
        const cv::Rect& roiRect, int frameIndex);
    void saveIntermediateImages(const std::string& basePath, int frameIndex);
};

// BallDetector Constructor
BallDetector::BallDetector()
    : m_params(),
    pImpl(std::make_unique<Impl>()),
    m_performanceProfilingEnabled(false) {

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

BallDetectionResult BallDetector::DetectBall(const unsigned char* imageData, int width, int height, int frameIndex) {
    auto totalStartTime = std::chrono::high_resolution_clock::now();
    m_lastMetrics.Reset();

    pImpl->setParamsReference(&m_params);

    BallDetectionResult result;
    result.found = false;
    result.balls.clear();

    if (!imageData || width <= 0 || height <= 0) {
        result.errorMessage = "Invalid input parameters";
        LOG_ERROR("DetectBall: " + result.errorMessage);
        return result;
    }

    try {
        cv::Mat grayImage(height, width, CV_8UC1, const_cast<unsigned char*>(imageData));

        // 1. ROI extraction
        cv::Mat processImage = grayImage;
        cv::Rect roiRect(0, 0, width, height);

        MEASURE_TIME("ROI extraction",
            if (m_params.useROI && m_params.roiScale < 1.0f) {
                processImage = pImpl->extractROI(grayImage, m_params.roiScale);
                roiRect = cv::Rect((width - processImage.cols) / 2,
                    (height - processImage.rows) / 2,
                    processImage.cols, processImage.rows);
            },
                m_lastMetrics.roiExtractionTime_ms
                );

        // 2. Downscaling
        cv::Mat workingImage = processImage;
        float scaleFactor = 1.0f;

        MEASURE_TIME("Image downscaling",
            if (m_params.fastMode && m_params.downscaleFactor > 1) {
                int newWidth = processImage.cols / m_params.downscaleFactor;
                int newHeight = processImage.rows / m_params.downscaleFactor;
                if (newWidth >= MIN_DOWNSCALE_WIDTH && newHeight >= MIN_DOWNSCALE_HEIGHT) {
                    cv::resize(processImage, pImpl->m_downscaledImage, cv::Size(newWidth, newHeight), 0, 0, cv::INTER_AREA);

                    workingImage = pImpl->m_downscaledImage;
                    scaleFactor = static_cast<float>(m_params.downscaleFactor);
                    LOG_DEBUG("Image downscaled to " + std::to_string(newWidth) + "x" + std::to_string(newHeight));
                }
            },
                m_lastMetrics.downscaleTime_ms
                );

        // 3. Preprocessing
        cv::Mat processed;
        if (!m_params.skipPreprocessing) {
            MEASURE_TIME("Preprocessing",
                processed = pImpl->preprocessImage(workingImage, m_params); ,
                m_lastMetrics.preprocessingTime_ms
            );
        }
        else {
            processed = workingImage;
            m_lastMetrics.preprocessingTime_ms = 0.0;
        }

        // 4. Thresholding - not use
        cv::Mat binary;
        MEASURE_TIME("Thresholding",
            if (m_params.useAdaptiveThreshold) {
                cv::adaptiveThreshold(processed, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 51, 5);
            }
            else {
                double threshVal = m_params.brightnessThreshold > 0 ? m_params.brightnessThreshold : 0;
                int threshType = m_params.brightnessThreshold > 0 ? cv::THRESH_BINARY : (cv::THRESH_BINARY | cv::THRESH_OTSU);
                cv::threshold(processed, binary, threshVal, 255, threshType);
            }, 
                m_lastMetrics.thresholdingTime_ms
                );

        // 5. Morphology - not use
        MEASURE_TIME("Morphology operations",
            if (m_params.useMorphology) {
                cv::Mat morphKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
                cv::morphologyEx(binary, binary, cv::MORPH_OPEN, morphKernel);
                cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, morphKernel);
            },
                m_lastMetrics.morphologyTime_ms
                );

        if (m_params.saveIntermediateImages) {
            pImpl->m_lastBinaryImage = binary.clone();
        }

        // 6. Contour detection
        std::vector<cv::Vec3f> candidates;
        std::vector<std::vector<cv::Point>> contours;

        MEASURE_TIME("Contour detection",
            {
                cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                float minR = static_cast<float>(m_params.minRadius);
                float maxR = static_cast<float>(m_params.maxRadius);
                if (m_params.downscaleFactor > 1) {
                    float factor = static_cast<float>(m_params.downscaleFactor);
                    minR = std::max(3.0f, minR / factor);
                    maxR = maxR / factor;
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
                                circleList.push_back(cv::Vec3f(center.x, center.y, radius));
                            }
                        }
                    }
                );
                candidates.assign(circleList.begin(), circleList.end());
                LOG_DEBUG("Contour detection found " + std::to_string(candidates.size()) + " circle candidates");
            },
            m_lastMetrics.contourDetectionTime_ms
        );

        // 7. Hough fallback
        if (candidates.empty() && !m_params.fastMode) {
            MEASURE_TIME("Hough circle detection (fallback)",
                candidates = pImpl->detectCirclesHough(processed, m_params); ,
                m_lastMetrics.houghDetectionTime_ms
            );
            if (!candidates.empty()) {
                LOG_DEBUG("Hough fallback found " + std::to_string(candidates.size()) + " candidates");
            }
        }
        else {
            m_lastMetrics.houghDetectionTime_ms = 0.0;
        }

        m_lastMetrics.candidatesFound = static_cast<int>(candidates.size());

        // 8. Evaluate candidates
        std::vector<BallInfo> validBalls;
        if (!candidates.empty()) {
            MEASURE_TIME("Candidate evaluation",
                validBalls = pImpl->evaluateCandidatesTBB(workingImage, candidates, m_params, scaleFactor, roiRect, frameIndex); ,
                m_lastMetrics.candidateEvaluationTime_ms
            );
        }
        else {
            m_lastMetrics.candidateEvaluationTime_ms = 0.0;
        }
        m_lastMetrics.candidatesEvaluated = static_cast<int>(candidates.size());

        // 9. Select best ball(s)
        if (!validBalls.empty()) {
            if (!m_params.detectMultiple) {
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
                tbb::parallel_sort(validBalls.begin(), validBalls.end(),
                    [](const BallInfo& a, const BallInfo& b) {
                        return a.confidence > b.confidence;
                    });
                int maxBalls = std::min(MAX_DISPLAY_BALLS, static_cast<int>(validBalls.size()));
                result.balls.assign(validBalls.begin(), validBalls.begin() + maxBalls);
            }
            result.found = true;
            m_lastMetrics.ballDetected = true;

            const BallInfo& ball = result.balls[0];
            LOG_INFO("Ball detected at (" + std::to_string(cvRound(ball.center.x)) + ", " +
                std::to_string(cvRound(ball.center.y)) + "), radius=" +
                std::to_string(cvRound(ball.radius)) + ", confidence=" +
                std::to_string(ball.confidence));
        }

        // 10. Save debug images
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
                LOG_WARNING("No debug output directory specified, using ./debug_images");
            }
            try {
                if (!fs::exists(debugPath)) {
                    fs::create_directories(debugPath);
                    LOG_DEBUG("Created debug directory: " + debugPath);
                }
                pImpl->saveIntermediateImages(debugPath, frameIndex);
                if (result.found) {
                    std::string resultPath = debugPath + "/frame_" + std::to_string(frameIndex) + "_result.png";
                    SaveDetectionImage(imageData, width, height, result, resultPath, true);
                }
            }
            catch (const fs::filesystem_error& e) {
                LOG_ERROR("Failed to create/save debug images: " + std::string(e.what()));
            },
            m_lastMetrics.imagesSavingTime_ms
            );

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
    m_lastMetrics.totalDetectionTime_ms = std::chrono::duration_cast<std::chrono::microseconds>(totalEndTime - totalStartTime).count() / 1000.0;

    if (m_performanceProfilingEnabled && (frameIndex % PERFORMANCE_LOG_INTERVAL == 0)) {
        LOG_INFO("Frame " + std::to_string(frameIndex) + " - Total detection time: " + std::to_string(m_lastMetrics.totalDetectionTime_ms) + " ms");
    }

    return result;
}

cv::Mat BallDetector::Impl::preprocessImage(const cv::Mat& grayImage, const DetectionParams& params) {
    cv::Mat processed = grayImage.clone();

    // Gaussian blur
    cv::Size kernelSize = params.fastMode ? cv::Size(3, 3) : cv::Size(5, 5);
    double sigma = params.fastMode ? 0.75 : 1.0;
    cv::GaussianBlur(processed, processed, kernelSize, sigma, sigma, cv::BORDER_REPLICATE);

    // Shadow enhancement
    if (params.enhanceShadows && !params.fastMode) {
        processed = enhanceShadowRegions(processed, params.shadowEnhanceFactor);
    }

    // Normalization
    if (!params.fastMode && params.contrastThreshold > 0) {
        cv::normalize(processed, processed, 0, 255, cv::NORM_MINMAX);
    }

    if (params.saveIntermediateImages) {
        m_lastProcessedImage = processed.clone();
    }

    return processed;
}

cv::Mat BallDetector::Impl::enhanceShadowRegions(const cv::Mat& image, float factor) {
    cv::Mat result = image.clone();

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

    if (m_paramsPtr && m_paramsPtr->saveIntermediateImages) {
        m_lastShadowEnhanced = result.clone();
    }

    return result;
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

    // Boundary check
    if (cx - radius - margin < 0 || cy - radius - margin < 0 || cx + radius + margin >= image.cols || cy + radius + margin >= image.rows) {
        return false;
    }

    // Brightness check
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

    // Contrast check
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

float BallDetector::Impl::calculateConfidence(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
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

std::vector<BallInfo> BallDetector::Impl::evaluateCandidatesTBB(const cv::Mat& image,
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

                    info.confidence = calculateConfidence(image, circle, params);

                    if (params.useCircularityCheck && !params.fastMode) {
                        info.circularity = calculateCircularity(image, circle, params);
                        if (info.circularity < params.minCircularity) {
                            continue;
                        }
                    }
                    else {
                        info.circularity = 1.0f;
                    }

                    // Calculate brightness
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

    try {
        int savedCount = 0;

        if (!m_lastProcessedImage.empty()) {
            std::string path = prefix + "_01_processed.png";
            if (cv::imwrite(path, m_lastProcessedImage)) {
                LOG_DEBUG("Saved processed image: " + path);
                savedCount++;
            }
            else {
                LOG_ERROR("Failed to save processed image: " + path);
            }
        }

        if (!m_lastBinaryImage.empty()) {
            std::string path = prefix + "_02_binary.png";
            if (cv::imwrite(path, m_lastBinaryImage)) {
                LOG_DEBUG("Saved binary image: " + path);
                savedCount++;
            }
        }

        if (!m_lastShadowEnhanced.empty()) {
            std::string path = prefix + "_03_shadow_enhanced.png";
            if (cv::imwrite(path, m_lastShadowEnhanced)) {
                LOG_DEBUG("Saved shadow enhanced image: " + path);
                savedCount++;
            }
        }

        if (savedCount > 0) {
            LOG_DEBUG("Saved " + std::to_string(savedCount) + " debug images for frame " +
                std::to_string(frameIndex));
        }
        else {
            LOG_WARNING("No debug images saved for frame " + std::to_string(frameIndex));
        }
    }
    catch (const cv::Exception& e) {
        LOG_ERROR("OpenCV error saving debug images: " + std::string(e.what()));
    }
    catch (const std::exception& e) {
        LOG_ERROR("Error saving debug images: " + std::string(e.what()));
    }
}

double BallDetector::EstimateProcessingTime(int width, int height) const {
    double pixels = width * height;
    double scaledPixels = pixels;

    if (m_params.useROI)                { scaledPixels *= (m_params.roiScale * m_params.roiScale); }
    if (m_params.downscaleFactor > 1)   { scaledPixels /= (m_params.downscaleFactor * m_params.downscaleFactor); }

    double baseTime = scaledPixels / 50000.0;  // ~20ms for 1MP

    if (m_params.fastMode)              { baseTime *= 0.5; }
    if (m_params.useParallel)           { baseTime /= std::min(4.0, static_cast<double>(m_params.processingThreads)); }

    return baseTime;
}

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

        // Title
        cv::rectangle(colorImg, cv::Point(0, 0), cv::Point(width, 50), cv::Scalar(40, 40, 40), cv::FILLED);
        cv::putText(colorImg, "Ball Detection Result", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

        if (result.found && !result.balls.empty()) {
            int infoY = 70;

            for (size_t i = 0; i < result.balls.size() && i < MAX_DISPLAY_BALLS; ++i) {
                const BallInfo& ball = result.balls[i];
                cv::Point center(cvRound(ball.center.x), cvRound(ball.center.y));
                int radius = cvRound(ball.radius);

                // Filled circle with transparency
                cv::Mat overlay = colorImg.clone();
                cv::circle(overlay, center, radius, cv::Scalar(0, 0, 255), -1);
                cv::addWeighted(colorImg, 0.7, overlay, 0.3, 0, colorImg);

                // Circle outline
                cv::circle(colorImg, center, radius, cv::Scalar(0, 0, 255), 3);

                // Center crosshair
                cv::drawMarker(colorImg, center, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 20, 2);

                // Info panel
                cv::Rect infoBg(10, infoY - 20, INFO_PANEL_WIDTH, INFO_PANEL_HEIGHT);
                infoBg &= cv::Rect(0, 0, width, height);
                cv::rectangle(colorImg, infoBg, cv::Scalar(20, 20, 20), cv::FILLED);
                cv::rectangle(colorImg, infoBg, cv::Scalar(100, 100, 100), 1);

                // Ball info
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

            // Summary
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

        // Timestamp
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