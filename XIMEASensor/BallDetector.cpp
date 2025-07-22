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
#include <atomic>

namespace fs = std::filesystem;

// Performance timing macro - only active when profiling is enabled
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

// Debug output macro - only active when debug is enabled
#ifdef ENABLE_DEBUG_OUTPUT
#define DEBUG_SAVE_IMAGE(condition, code) if (condition) { code }
#define DEBUG_LOG(message) LOG_DEBUG(message)
#else
#define DEBUG_SAVE_IMAGE(condition, code)
#define DEBUG_LOG(message)
#endif

// Constructor for DetectionParams struct: set all default values
BallDetector::DetectionParams::DetectionParams() {
    // Default parameters optimized for overhead (ceiling) camera detecting a golf ball on the floor
    dp = 1.5;
    minDist = 30.0;
    param1 = 100.0;
    param2 = 0.9;
    minRadius = 10;
    maxRadius = 80;
    brightnessThreshold = 100;
    minCircularity = 0.7f;
    contrastThreshold = 10.0f;
    useColorFilter = true;
    useCircularityCheck = true;
    useHoughGradientAlt = true;
    detectMultiple = false;     // find only one ball
    useMorphology = true;
    useAdaptiveThreshold = true;
    correctPerspective = false;
    enhanceShadows = true;
    shadowEnhanceFactor = 0.7f;
    saveIntermediateImages = false;
    debugOutputDir = "";

    // Performance optimization parameters
    numThreads = 0;  // 0 = auto-detect
    useParallelDetection = true;
    maxCandidates = 20;  // Limit candidates for faster evaluation
}

// BallDetector implementation (PIMPL idiom for internal details)
class BallDetector::Impl {
public:
    // Calibration data for perspective correction
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;

    // Pre-allocated buffers to avoid repeated allocations
    cv::Mat m_workBuffer1;
    cv::Mat m_workBuffer2;
    cv::Mat m_maskBuffer;
    cv::Mat m_tempBuffer;

    // Thread-local storage for parallel processing
    static thread_local cv::Mat tls_workBuffer;
    static thread_local cv::Mat tls_maskBuffer;

#ifdef ENABLE_DEBUG_OUTPUT
    // Debugging images (stored for optional saving)
    cv::Mat m_lastProcessedImage;
    cv::Mat m_lastBinaryImage;
    cv::Mat m_lastEdgeImage;
    cv::Mat m_lastShadowEnhanced;
#endif

    // Preprocessing helper functions
    cv::Mat preprocessImage(const cv::Mat& grayImage, const DetectionParams& params);
    cv::Mat enhanceShadowRegions(const cv::Mat& image, float factor);
    cv::Mat correctPerspective(const cv::Mat& image);

    // Detection helper functions
    std::vector<cv::Vec3f> detectCirclesHough(const cv::Mat& image, const DetectionParams& params);
    std::vector<cv::Vec3f> detectByAdaptiveThreshold(const cv::Mat& image, const DetectionParams& params);
    std::vector<cv::Vec3f> detectCircleByContour(const cv::Mat& binaryImage, const DetectionParams& params);

    // Parallel detection wrapper
    std::vector<cv::Vec3f> detectCirclesParallel(const cv::Mat& image, const DetectionParams& params);

    // Evaluation helper functions
    float calculateCircularity(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
    bool  passesColorFilter(const cv::Mat& grayImage, const cv::Vec3f& circle, const DetectionParams& params);
    float calculateContrastScore(const cv::Mat& image, const cv::Vec3f& circle);
    float calculateConfidence(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);

    // Optimized candidate evaluation
    std::vector<BallInfo> evaluateCandidatesParallel(const cv::Mat& image,
        const std::vector<cv::Vec3f>& candidates,
        const DetectionParams& params,
        int frameIndex);

    // Utility functions
#ifdef ENABLE_DEBUG_OUTPUT
    void saveIntermediateImages(const std::string& basePath, int frameIndex);
#endif

    // Pre-allocate buffers based on expected image size
    void preallocateBuffers(int width, int height);
};

// Thread-local storage definitions
thread_local cv::Mat BallDetector::Impl::tls_workBuffer;
thread_local cv::Mat BallDetector::Impl::tls_maskBuffer;

// BallDetector Constructor
BallDetector::BallDetector() : m_params(), pImpl(std::make_unique<Impl>()), m_performanceProfilingEnabled(false) {
    // Ensure OpenCV is initialized
    cv::Mat testMat(10, 10, CV_8UC1);
    if (testMat.empty()) {
        LOG_ERROR("OpenCV initialization failed");
    }
    else {
        LOG_INFO("BallDetector initialized successfully for ceiling camera");
    }
    InitializeDefaultParams();
    InitializeThreadPool();
    m_lastMetrics.Reset();
}

BallDetector::~BallDetector() = default;

void BallDetector::InitializeDefaultParams() {
    m_params = DetectionParams();

#ifdef ENABLE_DEBUG_OUTPUT
    m_params.saveIntermediateImages = false;
    m_params.debugOutputDir = "";
#endif

    LOG_INFO("BallDetector parameters initialized for ceiling camera:");
    LOG_INFO("  - Radius range: " + std::to_string(m_params.minRadius) + " - " + std::to_string(m_params.maxRadius));
    LOG_INFO("  - Shadow enhancement: " + std::string(m_params.enhanceShadows ? "ON" : "OFF"));
    LOG_INFO("  - Adaptive thresholding: " + std::string(m_params.useAdaptiveThreshold ? "ON" : "OFF"));
    LOG_INFO("  - Mode: " + std::string(m_params.detectMultiple ? "Multi-ball" : "Single-ball") + " detection");
    LOG_INFO("  - Parallel detection: " + std::string(m_params.useParallelDetection ? "ON" : "OFF"));
}

void BallDetector::InitializeThreadPool() {
    m_numThreads = m_params.numThreads;
    if (m_numThreads == 0) {
        m_numThreads = std::thread::hardware_concurrency();
        if (m_numThreads == 0) m_numThreads = 4;  // Fallback
    }
    LOG_INFO("Thread pool initialized with " + std::to_string(m_numThreads) + " threads");
}

void BallDetector::SetParameters(const DetectionParams& params) {
    m_params = params;
    if (m_params.numThreads != m_numThreads) {
        InitializeThreadPool();
    }
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
#ifdef ENABLE_PERFORMANCE_PROFILING
    LOG_INFO("Performance profiling " + std::string(enable ? "enabled" : "disabled"));
#else
    if (enable) {
        LOG_WARNING("Performance profiling requested but ENABLE_PERFORMANCE_PROFILING is not defined");
    }
#endif
}

bool BallDetector::IsPerformanceProfilingEnabled() const {
    return m_performanceProfilingEnabled;
}

BallDetectionResult BallDetector::DetectBall(const unsigned char* imageData, int width, int height, int frameIndex) {
    auto totalStartTime = std::chrono::high_resolution_clock::now();

    // Reset metrics for this detection
    m_lastMetrics.Reset();

    BallDetectionResult result;
    result.found = false;
    result.balls.clear();

    if (!imageData || width <= 0 || height <= 0) {
        result.errorMessage = "Invalid input image parameters";
        LOG_ERROR("DetectBall: " + result.errorMessage);
        return result;
    }

    try {
        DEBUG_LOG("Starting ball detection for frame " + std::to_string(frameIndex));

        // Pre-allocate buffers on first use or size change
        pImpl->preallocateBuffers(width, height);

        // 1. Construct grayscale image from input buffer (no copy, just wrapper)
        cv::Mat grayImage(height, width, CV_8UC1, const_cast<unsigned char*>(imageData));

        // 2. Correct lens distortion/perspective if calibration data is available
        cv::Mat correctedImage;
        if (m_params.correctPerspective && !pImpl->m_cameraMatrix.empty()) {
            MEASURE_TIME("Perspective correction",
                correctedImage = pImpl->correctPerspective(grayImage); ,
                m_lastMetrics.preprocessingTime_ms
            )
        }
        else {
            correctedImage = grayImage;  // No copy, just reference
        }

        // 3. Preprocess the image
        cv::Mat processedImage;
        MEASURE_TIME("Image preprocessing",
            processedImage = pImpl->preprocessImage(correctedImage, m_params);
        DEBUG_SAVE_IMAGE(m_params.saveIntermediateImages,
            pImpl->m_lastProcessedImage = processedImage.clone();
        ),
            m_lastMetrics.preprocessingTime_ms
            )

            // 4. Detect candidates using parallel or sequential methods
            std::vector<cv::Vec3f> allCandidates;

        if (m_params.useParallelDetection) {
            MEASURE_TIME("Parallel detection",
                allCandidates = pImpl->detectCirclesParallel(processedImage, m_params); ,
                m_lastMetrics.houghDetectionTime_ms
            )
        }
        else {
            // Sequential detection (original code path)
            std::vector<cv::Vec3f> houghCircles;
            MEASURE_TIME("Hough circle detection",
                houghCircles = pImpl->detectCirclesHough(processedImage, m_params); ,
                m_lastMetrics.houghDetectionTime_ms
            )
                allCandidates = std::move(houghCircles);

            if (m_params.useAdaptiveThreshold && allCandidates.size() < 5) {
                std::vector<cv::Vec3f> adaptiveCircles;
                MEASURE_TIME("Adaptive threshold detection",
                    adaptiveCircles = pImpl->detectByAdaptiveThreshold(processedImage, m_params); ,
                    m_lastMetrics.adaptiveThresholdTime_ms
                )
                    allCandidates.insert(allCandidates.end(), adaptiveCircles.begin(), adaptiveCircles.end());
            }
        }

        // 5. Remove duplicate/overlapping candidates efficiently
        if (allCandidates.size() > 1) {
            std::sort(allCandidates.begin(), allCandidates.end(),
                [](const cv::Vec3f& a, const cv::Vec3f& b) {
                    return a[2] > b[2];  // Sort by radius descending
                });

            std::vector<cv::Vec3f> uniqueCandidates;
            uniqueCandidates.reserve(allCandidates.size());

            for (const auto& cand : allCandidates) {
                bool duplicate = false;
                for (const auto& uc : uniqueCandidates) {
                    float dist = std::hypot(cand[0] - uc[0], cand[1] - uc[1]);
                    if (dist < m_params.minDist) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) {
                    uniqueCandidates.push_back(cand);
                    if (uniqueCandidates.size() >= m_params.maxCandidates) {
                        break;  // Early termination
                    }
                }
            }
            allCandidates = std::move(uniqueCandidates);
        }

        m_lastMetrics.candidatesFound = static_cast<int>(allCandidates.size());
        DEBUG_LOG("After de-duplication: " + std::to_string(allCandidates.size()) + " unique candidate(s)");

        // 6. Evaluate candidates (parallel if many candidates)
        std::vector<BallInfo> validBalls;
        MEASURE_TIME("Candidate evaluation",
            if (allCandidates.size() > 5 && m_params.useParallelDetection) {
                validBalls = pImpl->evaluateCandidatesParallel(correctedImage, allCandidates, m_params, frameIndex);
            }
            else {
                // Sequential evaluation for small number of candidates
                for (const auto& circle : allCandidates) {
                    BallInfo info;
                    info.center = cv::Point2f(circle[0], circle[1]);
                    info.radius = circle[2];
                    info.frameIndex = frameIndex;

                    // Quick reject based on color filter
                    if (m_params.useColorFilter && !pImpl->passesColorFilter(correctedImage, circle, m_params)) {
                        continue;
                    }

                    // Calculate metrics
                    info.circularity = m_params.useCircularityCheck ?
                        pImpl->calculateCircularity(processedImage, circle, m_params) : 1.0f;

                    if (info.circularity < m_params.minCircularity) {
                        continue;
                    }

                    info.contrast = pImpl->calculateContrastScore(correctedImage, circle);
                    if (info.contrast < m_params.contrastThreshold) {
                        continue;
                    }

                    // Calculate brightness
                    cv::Mat mask = cv::Mat::zeros(correctedImage.size(), CV_8UC1);
                    cv::circle(mask, cv::Point(cvRound(circle[0]), cvRound(circle[1])),
                        cvRound(circle[2] * 0.9), cv::Scalar(255), -1);
                    info.brightness = cv::mean(correctedImage, mask)[0];

                    // Calculate confidence
                    info.confidence = pImpl->calculateConfidence(processedImage, circle, m_params);

                    if (info.confidence >= 0.4f) {
                        validBalls.push_back(info);
                        if (!m_params.detectMultiple) {
                            break;  // Early termination for single ball mode
                        }
                    }
                }
            }
        m_lastMetrics.candidatesEvaluated = static_cast<int>(allCandidates.size()); ,
            m_lastMetrics.candidateEvaluationTime_ms
            )

            // 7. Select the best ball(s)
            if (!validBalls.empty()) {
                if (!m_params.detectMultiple) {
                    // Single ball mode - find best
                    auto bestBall = std::max_element(validBalls.begin(), validBalls.end(),
                        [](const BallInfo& a, const BallInfo& b) { return a.confidence < b.confidence; });
                    result.balls.push_back(*bestBall);
                }
                else {
                    // Multi-ball mode - sort by confidence
                    std::sort(validBalls.begin(), validBalls.end(),
                        [](const BallInfo& a, const BallInfo& b) { return a.confidence > b.confidence; });
                    result.balls = std::move(validBalls);
                }

                result.found = true;
                m_lastMetrics.ballDetected = true;

                const BallInfo& ball = result.balls[0];
                LOG_INFO("Ball detected at (" + std::to_string(cvRound(ball.center.x)) + ", " +
                    std::to_string(cvRound(ball.center.y)) + "), radius=" + std::to_string(cvRound(ball.radius)) +
                    ", confidence=" + std::to_string(ball.confidence));
            }

#ifdef ENABLE_DEBUG_OUTPUT
        // 8. Save debug images if enabled
        if (m_params.saveIntermediateImages && !m_params.debugOutputDir.empty()) {
            MEASURE_TIME("Saving intermediate images",
                fs::create_directories(m_params.debugOutputDir);
            pImpl->saveIntermediateImages(m_params.debugOutputDir, frameIndex);
            SaveDetectionImage(imageData, width, height, result,
                m_params.debugOutputDir + "/frame_" + std::to_string(frameIndex) + "_result.png"); ,
                m_lastMetrics.imagesSavingTime_ms
                )
        }
#endif
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
    auto totalDuration = std::chrono::duration_cast<std::chrono::microseconds>(totalEndTime - totalStartTime).count();
    m_lastMetrics.totalDetectionTime_ms = totalDuration / 1000.0;

#ifdef ENABLE_PERFORMANCE_PROFILING
    if (m_performanceProfilingEnabled) {
        LOG_INFO("Total ball detection time for frame " + std::to_string(frameIndex) + ": " +
            std::to_string(totalDuration) + " us (" + std::to_string(totalDuration / 1000.0) + " ms)");
    }
#endif

    return result;
}

void BallDetector::Impl::preallocateBuffers(int width, int height) {
    // Pre-allocate work buffers if size changed
    if (m_workBuffer1.empty() || m_workBuffer1.rows != height || m_workBuffer1.cols != width) {
        m_workBuffer1 = cv::Mat(height, width, CV_8UC1);
        m_workBuffer2 = cv::Mat(height, width, CV_8UC1);
        m_maskBuffer = cv::Mat(height, width, CV_8UC1);
        m_tempBuffer = cv::Mat(height, width, CV_8UC1);
    }
}

cv::Mat BallDetector::Impl::preprocessImage(const cv::Mat& grayImage, const DetectionParams& params) {
    // Use pre-allocated buffer
    cv::Mat processed = m_workBuffer1;
    grayImage.copyTo(processed);

    // 1. Noise reduction with Gaussian blur - use optimized in-place operation
    cv::GaussianBlur(processed, processed, cv::Size(5, 5), 1.5);

    // 2. Shadow enhancement (optimized)
    if (params.enhanceShadows) {
        processed = enhanceShadowRegions(processed, params.shadowEnhanceFactor);
#ifdef ENABLE_DEBUG_OUTPUT
        m_lastShadowEnhanced = processed.clone();
#endif
    }

    // 3. Local contrast enhancement using CLAHE
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        cv::Mat localContrast = m_workBuffer2;
        clahe->apply(processed, localContrast);
        cv::addWeighted(processed, 0.6, localContrast, 0.4, 0, processed);
    }

    // 4. Sharpen image using Unsharp Mask (optimized with pre-allocated buffer)
    {
        cv::Mat blurred = m_tempBuffer;
        cv::GaussianBlur(processed, blurred, cv::Size(0, 0), 2.0);
        cv::addWeighted(processed, 1.5, blurred, -0.5, 0, processed);
    }

    return processed;
}

cv::Mat BallDetector::Impl::enhanceShadowRegions(const cv::Mat& image, float factor) {
    cv::Mat darkMask = m_maskBuffer;
    cv::threshold(image, darkMask, 100, 255, cv::THRESH_BINARY_INV);

    cv::Mat result = image.clone();

    // Optimized shadow enhancement using parallel execution
    cv::parallel_for_(cv::Range(0, image.rows), [&](const cv::Range& range) {
        for (int y = range.start; y < range.end; ++y) {
            const uchar* maskRow = darkMask.ptr<uchar>(y);
            const uchar* imgRow = image.ptr<uchar>(y);
            uchar* resRow = result.ptr<uchar>(y);

            for (int x = 0; x < image.cols; ++x) {
                if (maskRow[x] > 0) {
                    int enhanced = static_cast<int>(imgRow[x] * (1.0f + factor));
                    resRow[x] = static_cast<uchar>(std::min(enhanced, 255));
                }
            }
        }
        });

    return result;
}

cv::Mat BallDetector::Impl::correctPerspective(const cv::Mat& image) {
    if (m_cameraMatrix.empty() || m_distCoeffs.empty()) {
        return image;
    }

    cv::Mat undistorted;
    cv::undistort(image, undistorted, m_cameraMatrix, m_distCoeffs);
    return undistorted;
}

std::vector<cv::Vec3f> BallDetector::Impl::detectCirclesParallel(const cv::Mat& image, const DetectionParams& params) {
    std::vector<std::future<std::vector<cv::Vec3f>>> futures;

    // Launch parallel detection tasks
    futures.push_back(std::async(std::launch::async, [this, &image, &params]() {
        return detectCirclesHough(image, params);
        }));

    if (params.useAdaptiveThreshold) {
        futures.push_back(std::async(std::launch::async, [this, &image, &params]() {
            return detectByAdaptiveThreshold(image, params);
            }));
    }

    // Collect results
    std::vector<cv::Vec3f> allCandidates;
    for (auto& future : futures) {
        auto candidates = future.get();
        allCandidates.insert(allCandidates.end(), candidates.begin(), candidates.end());
    }

    return allCandidates;
}

std::vector<cv::Vec3f> BallDetector::Impl::detectCirclesHough(const cv::Mat& image, const DetectionParams& params) {
    std::vector<cv::Vec3f> circles;

#ifdef ENABLE_DEBUG_OUTPUT
    // Generate edge image for debug
    cv::Mat edges;
    cv::Canny(image, edges, params.param1 / 2.0, params.param1);
    m_lastEdgeImage = edges.clone();
#endif

    // Perform Hough circle detection
    int method = params.useHoughGradientAlt ? cv::HOUGH_GRADIENT_ALT : cv::HOUGH_GRADIENT;
    cv::HoughCircles(image, circles, method,
        params.dp,
        params.minDist,
        params.param1,
        params.param2,
        params.minRadius,
        params.maxRadius);

    DEBUG_LOG("HoughCircles found " + std::to_string(circles.size()) + " circles");
    return circles;
}

std::vector<cv::Vec3f> BallDetector::Impl::detectByAdaptiveThreshold(const cv::Mat& image, const DetectionParams& params) {
    // Use thread-local buffer if available
    cv::Mat binary = tls_workBuffer.empty() ? cv::Mat() : tls_workBuffer;
    if (binary.empty() || binary.rows != image.rows || binary.cols != image.cols) {
        binary = cv::Mat(image.rows, image.cols, CV_8UC1);
        tls_workBuffer = binary;
    }

    cv::adaptiveThreshold(image, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
        cv::THRESH_BINARY, 21, -5);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

    return detectCircleByContour(binary, params);
}

std::vector<cv::Vec3f> BallDetector::Impl::detectCircleByContour(const cv::Mat& binaryImage, const DetectionParams& params) {
    std::vector<cv::Vec3f> circles;
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Pre-calculate area limits
    const double minArea = CV_PI * params.minRadius * params.minRadius;
    const double maxArea = CV_PI * params.maxRadius * params.maxRadius;
    const float minCircularityThreshold = params.minCircularity * 0.8f;

    circles.reserve(contours.size());  // Pre-allocate

    // Process contours efficiently
    for (const auto& contour : contours) {
        // Quick area check
        double area = cv::contourArea(contour);
        if (area < minArea || area > maxArea) {
            continue;
        }

        // Get enclosing circle
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);

        // Quick radius check
        if (radius < params.minRadius || radius > params.maxRadius) {
            continue;
        }

        // Circularity check
        double perimeter = cv::arcLength(contour, true);
        if (perimeter > 0) {
            double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
            if (circularity >= minCircularityThreshold) {
                circles.emplace_back(center.x, center.y, radius);
            }
        }
    }

    return circles;
}

float BallDetector::Impl::calculateCircularity(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Define ROI with bounds checking
    int margin = 10;
    int x = std::max(0, cx - radius - margin);
    int y = std::max(0, cy - radius - margin);
    int w = std::min(image.cols - x, (radius + margin) * 2);
    int h = std::min(image.rows - y, (radius + margin) * 2);

    if (w <= 0 || h <= 0) {
        return 0.0f;
    }

    // Use pre-allocated thread-local buffer for ROI processing
    cv::Mat roiBinary = tls_maskBuffer;
    if (roiBinary.empty() || roiBinary.rows < h || roiBinary.cols < w) {
        roiBinary = cv::Mat(h, w, CV_8UC1);
        tls_maskBuffer = roiBinary;
    }
    else {
        roiBinary = roiBinary(cv::Rect(0, 0, w, h));
    }

    cv::Mat roi = image(cv::Rect(x, y, w, h));
    cv::threshold(roi, roiBinary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roiBinary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return 0.0f;
    }

    // Find best matching contour (closest to expected center)
    cv::Point roiCenter(cx - x, cy - y);
    int bestIdx = -1;
    double minDist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Moments M = cv::moments(contours[i]);
        if (M.m00 > 0) {
            cv::Point centroid(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
            double dist = cv::norm(centroid - roiCenter);
            if (dist < minDist) {
                minDist = dist;
                bestIdx = static_cast<int>(i);
            }
        }
    }

    if (bestIdx < 0) {
        return 0.0f;
    }

    // Calculate circularity
    double area = cv::contourArea(contours[bestIdx]);
    double perimeter = cv::arcLength(contours[bestIdx], true);
    if (perimeter <= 0) {
        return 0.0f;
    }

    double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
    return static_cast<float>(std::min(circularity, 1.0));
}

bool BallDetector::Impl::passesColorFilter(const cv::Mat& grayImage, const cv::Vec3f& circle, const DetectionParams& params) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Use pre-allocated mask buffer
    cv::Mat mask = m_maskBuffer;
    mask.setTo(0);
    cv::circle(mask, cv::Point(cx, cy), radius, cv::Scalar(255), -1);

    // Calculate mean brightness
    double ballMean = cv::mean(grayImage, mask)[0];

    // Quick brightness check
    if (ballMean >= params.brightnessThreshold) {
        return true;
    }

    // Calculate contrast with background
    cv::Mat ringMask = m_tempBuffer;
    ringMask.setTo(0);
    cv::circle(ringMask, cv::Point(cx, cy), std::min(radius + 10, std::min(grayImage.rows, grayImage.cols) / 2),
        cv::Scalar(255), -1);
    cv::circle(ringMask, cv::Point(cx, cy), radius, cv::Scalar(0), -1);

    double bgMean = cv::mean(grayImage, ringMask)[0];
    double contrast = std::abs(ballMean - bgMean);

    return contrast >= params.contrastThreshold;
}

float BallDetector::Impl::calculateContrastScore(const cv::Mat& image, const cv::Vec3f& circle) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Create masks efficiently
    cv::Mat ballMask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::Mat bgMask = cv::Mat::zeros(image.size(), CV_8UC1);

    cv::circle(ballMask, cv::Point(cx, cy), static_cast<int>(radius * 0.9), cv::Scalar(255), -1);
    cv::circle(bgMask, cv::Point(cx, cy), static_cast<int>(radius * 1.5), cv::Scalar(255), -1);
    cv::circle(bgMask, cv::Point(cx, cy), radius, cv::Scalar(0), -1);

    double ballMean = cv::mean(image, ballMask)[0];
    double bgMean = cv::mean(image, bgMask)[0];

    return static_cast<float>(std::abs(ballMean - bgMean));
}

float BallDetector::Impl::calculateConfidence(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    // Pre-calculate components
    float circScore = calculateCircularity(image, circle, params);
    float contrast = calculateContrastScore(image, circle);

    // Normalize contrast
    float contrastNorm = std::min(1.0f, contrast / 50.0f);

    // Size suitability
    float optimalRadius = (params.minRadius + params.maxRadius) / 2.0f;
    float radiusRange = (params.maxRadius - params.minRadius) / 2.0f;
    float sizeScore = 1.0f;
    if (radiusRange > 0) {
        float radiusDiff = std::fabs(circle[2] - optimalRadius);
        sizeScore = 1.0f - std::min(1.0f, radiusDiff / radiusRange);
    }

    // Calculate uniformity
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::circle(mask, cv::Point(cvRound(circle[0]), cvRound(circle[1])),
        cvRound(circle[2] * 0.8), cv::Scalar(255), -1);

    cv::Scalar meanVal, stdDevVal;
    cv::meanStdDev(image, meanVal, stdDevVal, mask);
    float uniformity = 1.0f - std::min(1.0f, static_cast<float>(stdDevVal[0]) / 40.0f);

    // Weighted combination
    float confidence = circScore * 0.3f + contrastNorm * 0.3f +
        sizeScore * 0.2f + uniformity * 0.2f;

    return confidence;
}

std::vector<BallInfo> BallDetector::Impl::evaluateCandidatesParallel(
    const cv::Mat& image,
    const std::vector<cv::Vec3f>& candidates,
    const DetectionParams& params,
    int frameIndex) {

    std::vector<BallInfo> results(candidates.size());
    std::atomic<int> validCount(0);

    // Parallel evaluation of candidates
    std::for_each(std::execution::par_unseq,
        candidates.begin(), candidates.end(),
        [&, this](const cv::Vec3f& circle) {
            size_t idx = &circle - &candidates[0];
            BallInfo& info = results[idx];

            info.center = cv::Point2f(circle[0], circle[1]);
            info.radius = circle[2];
            info.frameIndex = frameIndex;
            info.confidence = 0.0f;

            // Quick reject based on color filter
            if (params.useColorFilter && !passesColorFilter(image, circle, params)) {
                return;
            }

            // Calculate metrics
            if (params.useCircularityCheck) {
                info.circularity = calculateCircularity(image, circle, params);
                if (info.circularity < params.minCircularity) {
                    return;
                }
            }
            else {
                info.circularity = 1.0f;
            }

            info.contrast = calculateContrastScore(image, circle);
            if (info.contrast < params.contrastThreshold) {
                return;
            }

            // Calculate brightness
            cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
            cv::circle(mask, cv::Point(cvRound(circle[0]), cvRound(circle[1])),
                cvRound(circle[2] * 0.9), cv::Scalar(255), -1);
            info.brightness = cv::mean(image, mask)[0];

            // Calculate confidence
            info.confidence = calculateConfidence(image, circle, params);

            if (info.confidence >= 0.4f) {
                validCount++;
            }
        });

    // Filter out invalid results
    std::vector<BallInfo> validBalls;
    validBalls.reserve(validCount);

    for (const auto& ball : results) {
        if (ball.confidence >= 0.4f) {
            validBalls.push_back(ball);
        }
    }

    return validBalls;
}

#ifdef ENABLE_DEBUG_OUTPUT
void BallDetector::Impl::saveIntermediateImages(const std::string& basePath, int frameIndex) {
    std::string prefix = basePath + "/frame_" + std::to_string(frameIndex);

    if (!m_lastProcessedImage.empty()) {
        cv::imwrite(prefix + "_01_processed.png", m_lastProcessedImage);
    }
    if (!m_lastShadowEnhanced.empty()) {
        cv::imwrite(prefix + "_02_shadow_enhanced.png", m_lastShadowEnhanced);
    }
    if (!m_lastEdgeImage.empty()) {
        cv::imwrite(prefix + "_03_edges.png", m_lastEdgeImage);
    }
    if (!m_lastBinaryImage.empty()) {
        cv::imwrite(prefix + "_04_binary.png", m_lastBinaryImage);
    }
}
#endif

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
        cv::putText(colorImg, "Ball Detection - Top Camera",
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
            cv::Scalar(0, 255, 0), 2);

        if (result.found && !result.balls.empty()) {
            const BallInfo& ball = result.balls[0];
            // Draw circle
            cv::circle(colorImg, cv::Point(cvRound(ball.center.x), cvRound(ball.center.y)),
                cvRound(ball.radius), color, thickness);
            // Draw center marker
            cv::drawMarker(colorImg, cv::Point(cvRound(ball.center.x), cvRound(ball.center.y)),
                cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 15, 2);

            // Draw information
            std::ostringstream info;
            info << "Position: (" << cvRound(ball.center.x) << ", " << cvRound(ball.center.y) << ")";
            cv::putText(colorImg, info.str(), cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);

            info.str("");
            info << "Radius: " << cvRound(ball.radius) << " px";
            cv::putText(colorImg, info.str(), cv::Point(10, 80),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);

            info.str("");
            info << "Confidence: " << std::fixed << std::setprecision(2) << (ball.confidence * 100) << "%";
            cv::putText(colorImg, info.str(), cv::Point(10, 100),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
        }
        else {
            cv::putText(colorImg, "No ball detected",
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 0, 255), 1);
        }

        cv::cvtColor(colorImg, grayImg, cv::COLOR_BGR2GRAY);
        return true;
    }
    catch (const cv::Exception& e) {
        LOG_ERROR("OpenCV error in DrawDetectionResult: " + std::string(e.what()));
        return false;
    }
}

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
#ifdef _MSC_VER
        localtime_s(&localTime, &t);
#else
        localtime_r(&t, &localTime);
#endif
        std::ostringstream timestamp;
        timestamp << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S");
        cv::putText(colorImg, timestamp.str(), cv::Point(10, height - 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        // Title
        cv::putText(colorImg, "Ball Detection Result - Ceiling Camera",
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
            cv::Scalar(0, 255, 0), 2);

        if (result.found && !result.balls.empty()) {
            const BallInfo& ball = result.balls[0];

            // Draw circle
            cv::circle(colorImg, cv::Point(cvRound(ball.center.x), cvRound(ball.center.y)),
                cvRound(ball.radius), cv::Scalar(0, 255, 0), 2);
            // Draw center
            cv::drawMarker(colorImg, cv::Point(cvRound(ball.center.x), cvRound(ball.center.y)),
                cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 15, 2);

            // Draw info box
            int boxX = 10, boxY = 50;
            cv::rectangle(colorImg, cv::Point(boxX, boxY), cv::Point(boxX + 250, boxY + 100),
                cv::Scalar(0, 0, 0), cv::FILLED);
            cv::rectangle(colorImg, cv::Point(boxX, boxY), cv::Point(boxX + 250, boxY + 100),
                cv::Scalar(0, 255, 0), 1);

            // Write details
            std::ostringstream info;
            info << "Position: (" << cvRound(ball.center.x) << ", " << cvRound(ball.center.y) << ")";
            cv::putText(colorImg, info.str(), cv::Point(boxX + 10, boxY + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

            info.str("");
            info << "Radius: " << cvRound(ball.radius) << " px";
            cv::putText(colorImg, info.str(), cv::Point(boxX + 10, boxY + 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

            info.str("");
            info << "Confidence: " << std::fixed << std::setprecision(1) << (ball.confidence * 100) << "%";
            cv::putText(colorImg, info.str(), cv::Point(boxX + 10, boxY + 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

            info.str("");
            info << "Circularity: " << std::fixed << std::setprecision(2) << ball.circularity;
            cv::putText(colorImg, info.str(), cv::Point(boxX + 10, boxY + 80),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
        else {
            cv::putText(colorImg, "No ball detected",
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 0, 255), 2);
        }

        bool success = cv::imwrite(outputPath, colorImg);

        if (success) {
            DEBUG_LOG("Detection result image saved: " + outputPath);
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

void BallDetector::AutoTuneParameters(const std::vector<unsigned char*>& sampleImages, int width, int height) {
    LOG_INFO("Starting auto-tune with " + std::to_string(sampleImages.size()) + " sample images");

    DetectionParams baseParams = m_params;
    std::vector<DetectionParams> paramCandidates;

    // Generate parameter combinations
    for (float circularity = 0.5f; circularity <= 0.8f; circularity += 0.1f) {
        for (int brightness = 120; brightness <= 180; brightness += 20) {
            for (float contrast = 10.0f; contrast <= 25.0f; contrast += 5.0f) {
                DetectionParams candidate = baseParams;
                candidate.minCircularity = circularity;
                candidate.brightnessThreshold = brightness;
                candidate.contrastThreshold = contrast;
                paramCandidates.push_back(candidate);
            }
        }
    }

    LOG_INFO("Generated " + std::to_string(paramCandidates.size()) + " parameter combinations to test");

    float bestScore = 0.0f;
    DetectionParams bestParams = baseParams;

    // Use parallel evaluation for parameter tuning
    std::vector<std::pair<float, DetectionParams>> results(paramCandidates.size());

    std::transform(std::execution::par_unseq,
        paramCandidates.begin(), paramCandidates.end(),
        results.begin(),
        [&](const DetectionParams& params) {
            // Temporarily set parameters
            BallDetector tempDetector;
            tempDetector.SetParameters(params);

            float totalScore = 0.0f;
            int detectCount = 0;

            // Test on each sample image
            for (size_t i = 0; i < sampleImages.size(); ++i) {
                BallDetectionResult res = tempDetector.DetectBall(
                    sampleImages[i], width, height, static_cast<int>(i));
                if (res.found && !res.balls.empty()) {
                    totalScore += res.balls[0].confidence;
                    detectCount++;
                }
            }

            float avgScore = detectCount > 0 ? totalScore / detectCount : 0.0f;
            float detectionRate = static_cast<float>(detectCount) / sampleImages.size();
            float combinedScore = avgScore * detectionRate;

            return std::make_pair(combinedScore, params);
        });

    // Find best parameters
    auto bestResult = std::max_element(results.begin(), results.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    if (bestResult != results.end() && bestResult->first > bestScore) {
        bestScore = bestResult->first;
        bestParams = bestResult->second;
    }

    // Apply best parameters
    m_params = bestParams;

    LOG_INFO("Auto-tune completed");
    LOG_INFO("Best score: " + std::to_string(bestScore));
    LOG_INFO("Optimal parameters: circularity=" + std::to_string(bestParams.minCircularity) +
        ", brightnessThreshold=" + std::to_string(bestParams.brightnessThreshold) +
        ", contrastThreshold=" + std::to_string(bestParams.contrastThreshold));
}

std::string BallDetector::GeneratePerformanceReport() const {
    std::ostringstream report;
    report << std::fixed << std::setprecision(2);
    report << "\n========== Ball Detection Performance Report ==========\n";

#ifndef ENABLE_PERFORMANCE_PROFILING
    report << "Performance profiling is disabled at compile time.\n";
    report << "Define ENABLE_PERFORMANCE_PROFILING to enable timing measurements.\n";
    report << "======================================================\n";
    return report.str();
#endif

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

        if (m_lastMetrics.adaptiveThresholdTime_ms > 0) {
            report << "  - Adaptive Threshold: " << m_lastMetrics.adaptiveThresholdTime_ms << " ms ("
                << (m_lastMetrics.adaptiveThresholdTime_ms / m_lastMetrics.totalDetectionTime_ms * 100) << "%)\n";
        }

        if (m_lastMetrics.contourDetectionTime_ms > 0) {
            report << "  - Contour Detection: " << m_lastMetrics.contourDetectionTime_ms << " ms ("
                << (m_lastMetrics.contourDetectionTime_ms / m_lastMetrics.totalDetectionTime_ms * 100) << "%)\n";
        }

        report << "  - Candidate Evaluation: " << m_lastMetrics.candidateEvaluationTime_ms << " ms ("
            << (m_lastMetrics.candidateEvaluationTime_ms / m_lastMetrics.totalDetectionTime_ms * 100) << "%)\n";

#ifdef ENABLE_DEBUG_OUTPUT
        if (m_lastMetrics.imagesSavingTime_ms > 0) {
            report << "  - Image Saving: " << m_lastMetrics.imagesSavingTime_ms << " ms ("
                << (m_lastMetrics.imagesSavingTime_ms / m_lastMetrics.totalDetectionTime_ms * 100) << "%)\n";
        }
#endif

        // Calculate unaccounted time
        double accountedTime = m_lastMetrics.preprocessingTime_ms + m_lastMetrics.houghDetectionTime_ms +
            m_lastMetrics.adaptiveThresholdTime_ms + m_lastMetrics.contourDetectionTime_ms +
            m_lastMetrics.candidateEvaluationTime_ms;
#ifdef ENABLE_DEBUG_OUTPUT
        accountedTime += m_lastMetrics.imagesSavingTime_ms;
#endif
        double unaccountedTime = m_lastMetrics.totalDetectionTime_ms - accountedTime;
        if (unaccountedTime > 0.1) {
            report << "  - Other operations: " << unaccountedTime << " ms ("
                << (unaccountedTime / m_lastMetrics.totalDetectionTime_ms * 100) << "%)\n";
        }
    }

    report << "\nDetection Statistics:\n";
    report << "  - Candidates found: " << m_lastMetrics.candidatesFound << "\n";
    report << "  - Candidates evaluated: " << m_lastMetrics.candidatesEvaluated << "\n";
    report << "  - Result: " << (m_lastMetrics.ballDetected ? "Ball detected" : "No ball detected") << "\n";

    report << "\nConfiguration:\n";
    report << "  - Parallel detection: " << (m_params.useParallelDetection ? "ENABLED" : "DISABLED") << "\n";
    report << "  - Thread count: " << m_numThreads << "\n";
    report << "  - Max candidates: " << m_params.maxCandidates << "\n";

    // Performance recommendations
    report << "\nPerformance Recommendations:\n";
    if (m_lastMetrics.totalDetectionTime_ms > 100) {
        report << "  - Total time exceeds 100ms target. Consider:\n";
        if (!m_params.useParallelDetection) {
            report << "    * Enable parallel detection (useParallelDetection = true)\n";
        }
        if (m_lastMetrics.preprocessingTime_ms > 30) {
            report << "    * Reduce preprocessing steps or use lower resolution\n";
        }
        if (m_lastMetrics.candidatesFound > m_params.maxCandidates) {
            report << "    * Tighten detection parameters to reduce false positives\n";
        }
        if (m_params.useAdaptiveThreshold && m_lastMetrics.adaptiveThresholdTime_ms > 20) {
            report << "    * Disable adaptive threshold if not necessary\n";
        }
#ifdef ENABLE_DEBUG_OUTPUT
        if (m_params.saveIntermediateImages) {
            report << "    * Disable intermediate image saving for production\n";
        }
#endif
    }
    else if (m_lastMetrics.totalDetectionTime_ms < 20) {
        report << "  - Excellent performance! Detection completed in under 20ms.\n";
    }
    else {
        report << "  - Good performance. Detection time is within acceptable range.\n";
    }

    report << "======================================================\n";

    return report.str();
}