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

namespace fs = std::filesystem;

// Performance profiling enable/disable
// Comment out this line to disable performance profiling
#define ENABLE_PERFORMANCE_PROFILING

// Enhanced timing helper macro with metrics recording
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
#define MEASURE_TIME(name, code, metricsVar) \
        { \
            code \
        }
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
}

// BallDetector implementation (PIMPL idiom for internal details)
class BallDetector::Impl {
public:
    // Calibration data for perspective correction
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;

    // Debugging images (stored for optional saving)
    cv::Mat m_lastProcessedImage;
    cv::Mat m_lastBinaryImage;
    cv::Mat m_lastEdgeImage;
    cv::Mat m_lastShadowEnhanced;

    // Preprocessing helper functions
    cv::Mat preprocessImage(const cv::Mat& grayImage, const DetectionParams& params);
    cv::Mat enhanceShadowRegions(const cv::Mat& image, float factor);
    cv::Mat correctPerspective(const cv::Mat& image);

    // Detection helper functions
    std::vector<cv::Vec3f> detectCirclesHough(const cv::Mat& image, const DetectionParams& params);
    std::vector<cv::Vec3f> detectByAdaptiveThreshold(const cv::Mat& image, const DetectionParams& params);
    std::vector<cv::Vec3f> detectCircleByContour(const cv::Mat& binaryImage, const DetectionParams& params);

    // Evaluation helper functions
    float calculateCircularity(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
    bool  passesColorFilter(const cv::Mat& grayImage, const cv::Vec3f& circle, const DetectionParams& params);
    float calculateContrastScore(const cv::Mat& image, const cv::Vec3f& circle);
    float calculateConfidence(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);

    // Utility functions
    void saveIntermediateImages(const std::string& basePath, int frameIndex);
};

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
    m_lastMetrics.Reset();
}

BallDetector::~BallDetector() = default;

void BallDetector::InitializeDefaultParams() {
    m_params = DetectionParams();

    m_params.saveIntermediateImages = false;
    m_params.debugOutputDir = "";

    LOG_INFO("BallDetector parameters initialized for ceiling camera:");
    LOG_INFO("  - Radius range: " + std::to_string(m_params.minRadius) + " - " + std::to_string(m_params.maxRadius));
    LOG_INFO("  - Shadow enhancement: " + std::string(m_params.enhanceShadows ? "ON" : "OFF"));
    LOG_INFO("  - Adaptive thresholding: " + std::string(m_params.useAdaptiveThreshold ? "ON" : "OFF"));
    LOG_INFO("  - Mode: " + std::string(m_params.detectMultiple ? "Multi-ball" : "Single-ball") + " detection");
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
        LOG_DEBUG("Starting ball detection for frame " + std::to_string(frameIndex));

        // 1. Construct grayscale image from input buffer
        cv::Mat grayImage(height, width, CV_8UC1, const_cast<unsigned char*>(imageData));

        // 2. Correct lens distortion/perspective if calibration data is available
        cv::Mat correctedImage = grayImage;
        if (m_params.correctPerspective && !pImpl->m_cameraMatrix.empty()) {
            double perspectiveTime = 0;
            MEASURE_TIME("Perspective correction",
                correctedImage = pImpl->correctPerspective(grayImage); ,
                perspectiveTime
            )
        }

        // 3. Preprocess the image
        cv::Mat processedImage;
        MEASURE_TIME("Image preprocessing",
            processedImage = pImpl->preprocessImage(correctedImage, m_params);
        pImpl->m_lastProcessedImage = processedImage.clone(); ,
            m_lastMetrics.preprocessingTime_ms
            )

            // 4. Apply multiple detection methods to gather candidates
            std::vector<cv::Vec3f> allCandidates;

        // 4-1. Hough Circle Transform detection
        std::vector<cv::Vec3f> houghCircles;
        MEASURE_TIME("Hough circle detection",
            houghCircles = pImpl->detectCirclesHough(processedImage, m_params); ,
            m_lastMetrics.houghDetectionTime_ms
        )
            allCandidates.insert(allCandidates.end(), houghCircles.begin(), houghCircles.end());
        LOG_DEBUG("HoughCircles found " + std::to_string(houghCircles.size()) + " candidate(s)");

        // 4-2. Adaptive threshold + contour-based detection
        if (m_params.useAdaptiveThreshold) {
            std::vector<cv::Vec3f> adaptiveCircles;
            MEASURE_TIME("Adaptive threshold detection",
                adaptiveCircles = pImpl->detectByAdaptiveThreshold(processedImage, m_params); ,
                m_lastMetrics.adaptiveThresholdTime_ms
            )
                allCandidates.insert(allCandidates.end(), adaptiveCircles.begin(), adaptiveCircles.end());
            LOG_DEBUG("Adaptive thresholding found " + std::to_string(adaptiveCircles.size()) + " candidate(s)");
        }

        // 4-3. Otsu threshold + contour detection (backup method)
        if (allCandidates.empty()) {
            MEASURE_TIME("Backup contour detection",
                cv::Mat binary;
            cv::threshold(processedImage, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

            if (m_params.useMorphology) {
                cv::Mat kernelClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
                cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernelClose);
                cv::Mat kernelOpen = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
                cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernelOpen);
            }

            pImpl->m_lastBinaryImage = binary.clone();
            std::vector<cv::Vec3f> contourCircles = pImpl->detectCircleByContour(binary, m_params);
            allCandidates.insert(allCandidates.end(), contourCircles.begin(), contourCircles.end());
            LOG_DEBUG("Contour-based detection (backup) found " + std::to_string(contourCircles.size()) + " candidate(s)"); ,
                m_lastMetrics.contourDetectionTime_ms
                )
        }

        // 5. Remove duplicate/overlapping candidates
        double duplicateRemovalTime = 0;
        MEASURE_TIME("Duplicate removal",
            std::vector<cv::Vec3f> uniqueCandidates;
        for (const auto& cand : allCandidates) {
            bool duplicate = false;
            for (const auto& uc : uniqueCandidates) {
                float dist = std::sqrt(std::pow(cand[0] - uc[0], 2) + std::pow(cand[1] - uc[1], 2));
                if (dist < m_params.minDist) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                uniqueCandidates.push_back(cand);
            }
        }
        allCandidates = std::move(uniqueCandidates); ,
            duplicateRemovalTime
            )
            LOG_DEBUG("After de-duplication: " + std::to_string(allCandidates.size()) + " unique candidate(s)");
        m_lastMetrics.candidatesFound = static_cast<int>(allCandidates.size());

        // 6. Evaluate each candidate circle
        std::vector<BallInfo> validBalls;
        MEASURE_TIME("Candidate evaluation",
            for (const auto& circle : allCandidates) {
                BallInfo info;
                info.center = cv::Point2f(circle[0], circle[1]);
                info.radius = circle[2];
                info.frameIndex = frameIndex;
                info.confidence = 0.0f;
                info.circularity = 1.0f;
                info.brightness = 0.0f;
                info.contrast = 0.0f;

                // 6-1. Color/Brightness filter check
                if (m_params.useColorFilter) {
                    auto filterStart = std::chrono::high_resolution_clock::now();
                    bool passed = pImpl->passesColorFilter(correctedImage, circle, m_params);
                    auto filterEnd = std::chrono::high_resolution_clock::now();
                    auto filterDuration = std::chrono::duration_cast<std::chrono::microseconds>(filterEnd - filterStart).count();

                    if (!passed) {
                        if (m_performanceProfilingEnabled) {
                            LOG_DEBUG("Color filter rejected candidate at (" + std::to_string(cvRound(circle[0])) + "," +
                                std::to_string(cvRound(circle[1])) + ") - took " + std::to_string(filterDuration) + " us");
                        }
                        continue;
                    }
                }

                // 6-2. Circularity check
                if (m_params.useCircularityCheck) {
                    auto circStart = std::chrono::high_resolution_clock::now();
                    info.circularity = pImpl->calculateCircularity(processedImage, circle, m_params);
                    auto circEnd = std::chrono::high_resolution_clock::now();
                    auto circDuration = std::chrono::duration_cast<std::chrono::microseconds>(circEnd - circStart).count();

                    if (info.circularity < m_params.minCircularity) {
                        if (m_performanceProfilingEnabled) {
                            LOG_DEBUG("Circularity check rejected candidate at (" + std::to_string(cvRound(circle[0])) + "," +
                                std::to_string(cvRound(circle[1])) + ") - circularity: " + std::to_string(info.circularity) +
                                " - took " + std::to_string(circDuration) + " us");
                        }
                        continue;
                    }
                }

                // 6-3. Contrast check
                auto contrastStart = std::chrono::high_resolution_clock::now();
                info.contrast = pImpl->calculateContrastScore(correctedImage, circle);
                auto contrastEnd = std::chrono::high_resolution_clock::now();
                auto contrastDuration = std::chrono::duration_cast<std::chrono::microseconds>(contrastEnd - contrastStart).count();

                if (info.contrast < m_params.contrastThreshold) {
                    if (m_performanceProfilingEnabled) {
                        LOG_DEBUG("Contrast check rejected candidate at (" + std::to_string(cvRound(circle[0])) + "," +
                            std::to_string(cvRound(circle[1])) + ") - contrast: " + std::to_string(info.contrast) +
                            " - took " + std::to_string(contrastDuration) + " us");
                    }
                    continue;
                }

                // 6-4. Brightness calculation
                cv::Mat mask = cv::Mat::zeros(correctedImage.size(), CV_8UC1);
                cv::circle(mask, cv::Point(cvRound(circle[0]), cvRound(circle[1])),
                    cvRound(circle[2] * 0.9), cv::Scalar(255), -1);
                info.brightness = cv::mean(correctedImage, mask)[0];

                // 6-5. Compute overall confidence score
                auto confStart = std::chrono::high_resolution_clock::now();
                info.confidence = pImpl->calculateConfidence(processedImage, circle, m_params);
                auto confEnd = std::chrono::high_resolution_clock::now();
                auto confDuration = std::chrono::duration_cast<std::chrono::microseconds>(confEnd - confStart).count();
                if (m_performanceProfilingEnabled) {
                    LOG_DEBUG("Confidence calculation took " + std::to_string(confDuration) + " us");
                }

                if (info.confidence >= 0.4f) {
                    validBalls.push_back(info);
                }
            }
        m_lastMetrics.candidatesEvaluated = static_cast<int>(allCandidates.size()); ,
            m_lastMetrics.candidateEvaluationTime_ms
            )

            // 7. Select the best ball
            if (!validBalls.empty()) {
                std::sort(validBalls.begin(), validBalls.end(),
                    [](const BallInfo& a, const BallInfo& b) { return a.confidence > b.confidence; });
                result.balls.push_back(validBalls[0]);
                result.found = true;
                m_lastMetrics.ballDetected = true;

                const BallInfo& ball = result.balls[0];
                LOG_INFO("Ball detected at (" + std::to_string(cvRound(ball.center.x)) + ", " +
                    std::to_string(cvRound(ball.center.y)) + "), radius=" + std::to_string(cvRound(ball.radius)) +
                    ", confidence=" + std::to_string(ball.confidence) +
                    ", circularity=" + std::to_string(ball.circularity) +
                    ", contrast=" + std::to_string(ball.contrast));
            }

        // 8. Save debug images if enabled
        if (m_params.saveIntermediateImages) {
            MEASURE_TIME("Saving intermediate images",
                const std::string & dir = m_params.debugOutputDir;
            if (!dir.empty()) {
                fs::create_directories(dir);
                pImpl->saveIntermediateImages(dir, frameIndex);
                SaveDetectionImage(imageData, width, height, result,
                    dir + "/frame_" + std::to_string(frameIndex) + "_result.png");
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
    auto totalDuration = std::chrono::duration_cast<std::chrono::microseconds>(totalEndTime - totalStartTime).count();
    m_lastMetrics.totalDetectionTime_ms = totalDuration / 1000.0;

    LOG_INFO("Total ball detection time for frame " + std::to_string(frameIndex) + ": " +
        std::to_string(totalDuration) + " us (" + std::to_string(totalDuration / 1000.0) + " ms)");

    return result;
}

cv::Mat BallDetector::Impl::preprocessImage(const cv::Mat& grayImage, const DetectionParams& params) {
    auto startTime = std::chrono::high_resolution_clock::now();

    cv::Mat processed = grayImage.clone();

    // 1. Noise reduction with Gaussian blur
    {
        auto blurStart = std::chrono::high_resolution_clock::now();
        cv::GaussianBlur(processed, processed, cv::Size(5, 5), 1.5);
        auto blurEnd = std::chrono::high_resolution_clock::now();
        auto blurDuration = std::chrono::duration_cast<std::chrono::microseconds>(blurEnd - blurStart).count();
        LOG_DEBUG(std::string("Gaussian blur") + " took " + std::to_string(blurDuration) + " us (" +
            std::to_string(blurDuration / 1000.0) + " ms)");
    }

    // 2. Shadow enhancement
    if (params.enhanceShadows) {
        auto shadowStart = std::chrono::high_resolution_clock::now();
        processed = enhanceShadowRegions(processed, params.shadowEnhanceFactor);
        m_lastShadowEnhanced = processed.clone();
        auto shadowEnd = std::chrono::high_resolution_clock::now();
        auto shadowDuration = std::chrono::duration_cast<std::chrono::microseconds>(shadowEnd - shadowStart).count();
        LOG_DEBUG(std::string("Shadow enhancement") + " took " + std::to_string(shadowDuration) + " us (" +
            std::to_string(shadowDuration / 1000.0) + " ms)");
    }

    // 3. Local contrast enhancement using CLAHE
    {
        auto claheStart = std::chrono::high_resolution_clock::now();
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        cv::Mat localContrast;
        clahe->apply(processed, localContrast);
        cv::addWeighted(processed, 0.6, localContrast, 0.4, 0, processed);
        auto claheEnd = std::chrono::high_resolution_clock::now();
        auto claheDuration = std::chrono::duration_cast<std::chrono::microseconds>(claheEnd - claheStart).count();
        LOG_DEBUG(std::string("CLAHE contrast enhancement") + " took " + std::to_string(claheDuration) + " us (" +
            std::to_string(claheDuration / 1000.0) + " ms)");
    }

    // 4. Sharpen image using Unsharp Mask
    {
        auto sharpStart = std::chrono::high_resolution_clock::now();
        cv::Mat blurred;
        cv::GaussianBlur(processed, blurred, cv::Size(0, 0), 2.0);
        cv::addWeighted(processed, 1.5, blurred, -0.5, 0, processed);
        auto sharpEnd = std::chrono::high_resolution_clock::now();
        auto sharpDuration = std::chrono::duration_cast<std::chrono::microseconds>(sharpEnd - sharpStart).count();
        LOG_DEBUG(std::string("Unsharp mask sharpening") + " took " + std::to_string(sharpDuration) + " us (" +
            std::to_string(sharpDuration / 1000.0) + " ms)");
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    LOG_DEBUG("Total preprocessing time: " + std::to_string(duration) + " us (" +
        std::to_string(duration / 1000.0) + " ms)");

    return processed;
}

// Brighten dark (shadow) regions to enhance ball visibility
cv::Mat BallDetector::Impl::enhanceShadowRegions(const cv::Mat& image, float factor) {
    auto startTime = std::chrono::high_resolution_clock::now();

    cv::Mat darkMask;
    cv::threshold(image, darkMask, 100, 255, cv::THRESH_BINARY_INV);

    cv::Mat shadows = image.clone();
    shadows.setTo(cv::Scalar(0), darkMask == 0);
    shadows *= factor;

    cv::Mat result;
    cv::add(image, shadows, result);
    cv::threshold(result, result, 255, 255, cv::THRESH_TRUNC);

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    LOG_DEBUG("Shadow enhancement took " + std::to_string(duration) + " us");

    return result;
}

// Correct lens distortion/perspective using calibration matrix
cv::Mat BallDetector::Impl::correctPerspective(const cv::Mat& image) {
    if (m_cameraMatrix.empty() || m_distCoeffs.empty()) {
        return image;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    cv::Mat undistorted;
    cv::undistort(image, undistorted, m_cameraMatrix, m_distCoeffs);

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    LOG_DEBUG("Lens undistortion took " + std::to_string(duration) + " us");

    return undistorted;
}

// Detect circles using Hough Circle Transform
std::vector<cv::Vec3f> BallDetector::Impl::detectCirclesHough(const cv::Mat& image, const DetectionParams& params) {
    auto startTime = std::chrono::high_resolution_clock::now();

    std::vector<cv::Vec3f> circles;

    // Generate edge image for debug
    cv::Mat edges;
    {
        auto cannyStart = std::chrono::high_resolution_clock::now();
        cv::Canny(image, edges, params.param1 / 2.0, params.param1);
        m_lastEdgeImage = edges.clone();
        auto cannyEnd = std::chrono::high_resolution_clock::now();
        auto cannyDuration = std::chrono::duration_cast<std::chrono::microseconds>(cannyEnd - cannyStart).count();
        LOG_DEBUG(std::string("Canny edge detection") + " took " + std::to_string(cannyDuration) + " us (" +
            std::to_string(cannyDuration / 1000.0) + " ms)");
    }

    // Perform Hough circle detection
    int method = params.useHoughGradientAlt ? cv::HOUGH_GRADIENT_ALT : cv::HOUGH_GRADIENT;
    {
        auto houghStart = std::chrono::high_resolution_clock::now();
        cv::HoughCircles(image, circles, method,
            params.dp,
            params.minDist,
            params.param1,
            params.param2,
            params.minRadius,
            params.maxRadius);
        auto houghEnd = std::chrono::high_resolution_clock::now();
        auto houghDuration = std::chrono::duration_cast<std::chrono::microseconds>(houghEnd - houghStart).count();
        LOG_DEBUG(std::string("HoughCircles transform") + " took " + std::to_string(houghDuration) + " us (" +
            std::to_string(houghDuration / 1000.0) + " ms)");
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    LOG_DEBUG("Total Hough detection time: " + std::to_string(duration) + " us (" +
        std::to_string(duration / 1000.0) + " ms) - found " + std::to_string(circles.size()) + " circles");

    return circles;
}

std::vector<cv::Vec3f> BallDetector::Impl::detectByAdaptiveThreshold(const cv::Mat& image, const DetectionParams& params) {
    auto startTime = std::chrono::high_resolution_clock::now();

    cv::Mat binary;
    {
        auto adaptiveStart = std::chrono::high_resolution_clock::now();
        cv::adaptiveThreshold(image, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
            cv::THRESH_BINARY, 21, -5);
        auto adaptiveEnd = std::chrono::high_resolution_clock::now();
        auto adaptiveDuration = std::chrono::duration_cast<std::chrono::microseconds>(adaptiveEnd - adaptiveStart).count();
        LOG_DEBUG(std::string("Adaptive threshold") + " took " + std::to_string(adaptiveDuration) + " us (" +
            std::to_string(adaptiveDuration / 1000.0) + " ms)");
    }

    {
        auto morphStart = std::chrono::high_resolution_clock::now();
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
        auto morphEnd = std::chrono::high_resolution_clock::now();
        auto morphDuration = std::chrono::duration_cast<std::chrono::microseconds>(morphEnd - morphStart).count();
        LOG_DEBUG(std::string("Morphological operations") + " took " + std::to_string(morphDuration) + " us (" +
            std::to_string(morphDuration / 1000.0) + " ms)");
    }

    auto circles = detectCircleByContour(binary, params);

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    LOG_DEBUG("Total adaptive threshold detection time: " + std::to_string(duration) + " us (" +
        std::to_string(duration / 1000.0) + " ms)");

    return circles;
}

// Find circles by analyzing contours in a binary image
std::vector<cv::Vec3f> BallDetector::Impl::detectCircleByContour(const cv::Mat& binaryImage, const DetectionParams& params) {
    auto startTime = std::chrono::high_resolution_clock::now();

    std::vector<cv::Vec3f> circles;
    std::vector<std::vector<cv::Point>> contours;

    {
        auto findStart = std::chrono::high_resolution_clock::now();
        cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        auto findEnd = std::chrono::high_resolution_clock::now();
        auto findDuration = std::chrono::duration_cast<std::chrono::microseconds>(findEnd - findStart).count();
        LOG_DEBUG(std::string("Find contours") + " took " + std::to_string(findDuration) + " us (" +
            std::to_string(findDuration / 1000.0) + " ms)");
    }
    LOG_DEBUG("Found " + std::to_string(contours.size()) + " contours");

    int validContours = 0;
    {
        auto processStart = std::chrono::high_resolution_clock::now();
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < CV_PI * params.minRadius * params.minRadius ||
                area > CV_PI * params.maxRadius * params.maxRadius) {
                continue;
            }

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);
            if (radius < params.minRadius || radius > params.maxRadius) {
                continue;
            }

            double perimeter = cv::arcLength(contour, true);
            if (perimeter <= 0) continue;
            double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);

            if (circularity >= params.minCircularity * 0.8f) {
                circles.emplace_back(center.x, center.y, radius);
                validContours++;
            }
        }
        auto processEnd = std::chrono::high_resolution_clock::now();
        auto processDuration = std::chrono::duration_cast<std::chrono::microseconds>(processEnd - processStart).count();
        LOG_DEBUG(std::string("Process contours") + " took " + std::to_string(processDuration) + " us (" +
            std::to_string(processDuration / 1000.0) + " ms)");
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    LOG_DEBUG("Total contour detection time: " + std::to_string(duration) + " us (" +
        std::to_string(duration / 1000.0) + " ms) - " + std::to_string(validContours) +
        " valid circles from " + std::to_string(contours.size()) + " contours");

    return circles;
}

// Calculate how circular the region around the detected circle is
float BallDetector::Impl::calculateCircularity(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    auto startTime = std::chrono::high_resolution_clock::now();

    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Define ROI
    int margin = 10;
    int x = std::max(0, cx - radius - margin);
    int y = std::max(0, cy - radius - margin);
    int w = std::min(image.cols - x, (radius + margin) * 2);
    int h = std::min(image.rows - y, (radius + margin) * 2);
    if (w <= 0 || h <= 0) {
        return 0.0f;
    }
    cv::Mat roi = image(cv::Rect(x, y, w, h)).clone();

    // Binarize ROI
    cv::Mat roiBinary;
    cv::threshold(roi, roiBinary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roiBinary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        return 0.0f;
    }

    // Find best matching contour
    cv::Point roiCenter(cx - x, cy - y);
    double minDist = std::numeric_limits<double>::max();
    int bestIdx = -1;
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Moments M = cv::moments(contours[i]);
        if (M.m00 == 0) continue;
        cv::Point centroid(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
        double dist = cv::norm(centroid - roiCenter);
        if (dist < minDist) {
            minDist = dist;
            bestIdx = static_cast<int>(i);
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

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    LOG_DEBUG("Circularity calculation took " + std::to_string(duration) + " us");

    return static_cast<float>(std::min(circularity, 1.0));
}

// Check if the detected circle region passes basic brightness/contrast filters
bool BallDetector::Impl::passesColorFilter(const cv::Mat& grayImage, const cv::Vec3f& circle, const DetectionParams& params) {
    auto startTime = std::chrono::high_resolution_clock::now();

    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Create masks
    cv::Mat mask = cv::Mat::zeros(grayImage.size(), CV_8UC1);
    cv::circle(mask, cv::Point(cx, cy), radius, cv::Scalar(255), -1);

    cv::Mat ringMask = cv::Mat::zeros(grayImage.size(), CV_8UC1);
    cv::circle(ringMask, cv::Point(cx, cy), radius + 10, cv::Scalar(255), -1);
    cv::circle(ringMask, cv::Point(cx, cy), radius, cv::Scalar(0), -1);

    // Calculate statistics
    cv::Scalar meanVal, stdDevVal;
    cv::meanStdDev(grayImage, meanVal, stdDevVal, mask);
    cv::Scalar bgMeanVal = cv::mean(grayImage, ringMask);

    double ballMean = meanVal[0];
    double ballStdDev = stdDevVal[0];
    double bgMean = bgMeanVal[0];
    double contrast = std::abs(ballMean - bgMean);

    bool passed = (ballMean >= params.brightnessThreshold) ||
        (contrast >= params.contrastThreshold) ||
        (ballStdDev < 20.0);

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    LOG_DEBUG("Color filter check took " + std::to_string(duration) + " us - " +
        (passed ? "PASSED" : "FAILED"));

    return passed;
}

// Calculate contrast score between the ball region and its immediate background
float BallDetector::Impl::calculateContrastScore(const cv::Mat& image, const cv::Vec3f& circle) {
    auto startTime = std::chrono::high_resolution_clock::now();

    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Create masks
    cv::Mat ballMask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::circle(ballMask, cv::Point(cx, cy), static_cast<int>(radius * 0.9), cv::Scalar(255), -1);

    cv::Mat bgMask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::circle(bgMask, cv::Point(cx, cy), static_cast<int>(radius * 1.5), cv::Scalar(255), -1);
    cv::circle(bgMask, cv::Point(cx, cy), radius, cv::Scalar(0), -1);

    double ballMean = cv::mean(image, ballMask)[0];
    double bgMean = cv::mean(image, bgMask)[0];
    float contrast = static_cast<float>(std::abs(ballMean - bgMean));

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    LOG_DEBUG("Contrast calculation took " + std::to_string(duration) + " us - contrast: " +
        std::to_string(contrast));

    return contrast;
}

// Calculate an overall confidence score for a detected circle
float BallDetector::Impl::calculateConfidence(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    auto startTime = std::chrono::high_resolution_clock::now();

    // 1. Circularity component (30%)
    float circScore = calculateCircularity(image, circle, params);
    float confidence = circScore * 0.3f;

    // 2. Contrast component (30%)
    float contrast = calculateContrastScore(image, circle);
    float contrastNorm = std::min(1.0f, contrast / 50.0f);
    confidence += contrastNorm * 0.3f;

    // 3. Size suitability component (20%)
    float optimalRadius = (params.minRadius + params.maxRadius) / 2.0f;
    float radiusRange = (params.maxRadius - params.minRadius) / 2.0f;
    float sizeScore = 1.0f;
    if (radiusRange > 0) {
        float radiusDiff = std::fabs(circle[2] - optimalRadius);
        sizeScore = 1.0f - std::min(1.0f, radiusDiff / radiusRange);
    }
    confidence += sizeScore * 0.2f;

    // 4. Color uniformity component (20%)
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::circle(mask, cv::Point(cvRound(circle[0]), cvRound(circle[1])),
        cvRound(circle[2] * 0.8), cv::Scalar(255), -1);
    cv::Scalar meanVal, stdDevVal;
    cv::meanStdDev(image, meanVal, stdDevVal, mask);
    float stdDevIntensity = static_cast<float>(stdDevVal[0]);
    float uniformity = 1.0f - std::min(1.0f, stdDevIntensity / 40.0f);
    confidence += uniformity * 0.2f;

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    LOG_DEBUG("Confidence calculation took " + std::to_string(duration) + " us - confidence: " +
        std::to_string(confidence));

    return confidence;
}

// Save intermediate images (for debugging/analysis) to disk
void BallDetector::Impl::saveIntermediateImages(const std::string& basePath, int frameIndex) {
    auto startTime = std::chrono::high_resolution_clock::now();

    std::string prefix = basePath + "/frame_" + std::to_string(frameIndex);
    int savedCount = 0;

    if (!m_lastProcessedImage.empty()) {
        cv::imwrite(prefix + "_01_processed.png", m_lastProcessedImage);
        savedCount++;
    }
    if (!m_lastShadowEnhanced.empty()) {
        cv::imwrite(prefix + "_02_shadow_enhanced.png", m_lastShadowEnhanced);
        savedCount++;
    }
    if (!m_lastEdgeImage.empty()) {
        cv::imwrite(prefix + "_03_edges.png", m_lastEdgeImage);
        savedCount++;
    }
    if (!m_lastBinaryImage.empty()) {
        cv::imwrite(prefix + "_04_binary.png", m_lastBinaryImage);
        savedCount++;
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    LOG_DEBUG("Saved " + std::to_string(savedCount) + " intermediate images in " +
        std::to_string(duration) + " us (" + std::to_string(duration / 1000.0) + " ms)");
}

// Draw detection results on a grayscale image buffer
bool BallDetector::DrawDetectionResult(unsigned char* imageData, int width, int height,
    const BallDetectionResult& result,
    cv::Scalar color, int thickness) {
    if (!imageData || width <= 0 || height <= 0) {
        LOG_ERROR("Invalid parameters for DrawDetectionResult");
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    bool success = false;

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

        success = true;
    }
    catch (const cv::Exception& e) {
        LOG_ERROR("OpenCV error in DrawDetectionResult: " + std::string(e.what()));
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    if (m_performanceProfilingEnabled) {
        LOG_DEBUG("DrawDetectionResult took " + std::to_string(duration) + " us (" +
            std::to_string(duration / 1000.0) + " ms)");
    }

    return success;
}

// Save a detection result image to disk with annotations
bool BallDetector::SaveDetectionImage(const unsigned char* originalImage, int width, int height,
    const BallDetectionResult& result,
    const std::string& outputPath) {
    if (!originalImage || width <= 0 || height <= 0) {
        LOG_ERROR("Invalid parameters for SaveDetectionImage");
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    bool success = false;

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

        success = cv::imwrite(outputPath, colorImg);

        if (success) {
            LOG_DEBUG("Detection result image saved: " + outputPath);
        }
        else {
            LOG_ERROR("Failed to save detection image: " + outputPath);
        }
    }
    catch (const cv::Exception& e) {
        LOG_ERROR("OpenCV error in SaveDetectionImage: " + std::string(e.what()));
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    if (m_performanceProfilingEnabled) {
        LOG_DEBUG("SaveDetectionImage took " + std::to_string(duration) + " us (" +
            std::to_string(duration / 1000.0) + " ms)");
    }

    return success;
}

// Auto-tune parameters based on sample images
void BallDetector::AutoTuneParameters(const std::vector<unsigned char*>& sampleImages, int width, int height) {
    auto totalStartTime = std::chrono::high_resolution_clock::now();

    LOG_INFO("Starting auto-tune with " + std::to_string(sampleImages.size()) + " sample images");

    DetectionParams baseParams = m_params;
    std::vector<DetectionParams> paramCandidates;

    // Generate parameter combinations
    double candidateGenTime = 0;
    MEASURE_TIME("Generate parameter candidates",
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
        },
            candidateGenTime
            )
        LOG_INFO("Generated " + std::to_string(paramCandidates.size()) + " parameter combinations to test");

    float bestScore = 0.0f;
    DetectionParams bestParams = baseParams;

    // Evaluate each parameter combination
    double evaluationTime = 0;
    MEASURE_TIME("Evaluate all parameter combinations",
        for (size_t paramIdx = 0; paramIdx < paramCandidates.size(); ++paramIdx) {
            auto paramStartTime = std::chrono::high_resolution_clock::now();

            const auto& params = paramCandidates[paramIdx];
            m_params = params;
            float totalScore = 0.0f;
            int detectCount = 0;

            // Test on each sample image
            for (size_t i = 0; i < sampleImages.size(); ++i) {
                BallDetectionResult res = DetectBall(sampleImages[i], width, height, static_cast<int>(i));
                if (res.found && !res.balls.empty()) {
                    totalScore += res.balls[0].confidence;
                    detectCount++;
                }
            }

            if (detectCount > 0) {
                float avgScore = totalScore / detectCount;
                float detectionRate = static_cast<float>(detectCount) / sampleImages.size();
                float combinedScore = avgScore * detectionRate;

                if (combinedScore > bestScore) {
                    bestScore = combinedScore;
                    bestParams = params;
                }
            }

            auto paramEndTime = std::chrono::high_resolution_clock::now();
            auto paramDuration = std::chrono::duration_cast<std::chrono::milliseconds>(paramEndTime - paramStartTime).count();

            if ((paramIdx + 1) % 10 == 0) {
                LOG_DEBUG("Tested " + std::to_string(paramIdx + 1) + "/" +
                    std::to_string(paramCandidates.size()) + " combinations (" +
                    std::to_string(paramDuration) + " ms for this set)");
            }
        },
            evaluationTime
            )

        // Apply best parameters
        m_params = bestParams;

    auto totalEndTime = std::chrono::high_resolution_clock::now();
    auto totalDuration = std::chrono::duration_cast<std::chrono::seconds>(totalEndTime - totalStartTime).count();

    LOG_INFO("Auto-tune completed in " + std::to_string(totalDuration) + " seconds");
    LOG_INFO("Best score: " + std::to_string(bestScore));
    LOG_INFO("Optimal parameters: circularity=" + std::to_string(bestParams.minCircularity) +
        ", brightnessThreshold=" + std::to_string(bestParams.brightnessThreshold) +
        ", contrastThreshold=" + std::to_string(bestParams.contrastThreshold));
}

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

        if (m_lastMetrics.imagesSavingTime_ms > 0) {
            report << "  - Image Saving: " << m_lastMetrics.imagesSavingTime_ms << " ms ("
                << (m_lastMetrics.imagesSavingTime_ms / m_lastMetrics.totalDetectionTime_ms * 100) << "%)\n";
        }

        // Calculate unaccounted time
        double accountedTime = m_lastMetrics.preprocessingTime_ms + m_lastMetrics.houghDetectionTime_ms +
            m_lastMetrics.adaptiveThresholdTime_ms + m_lastMetrics.contourDetectionTime_ms +
            m_lastMetrics.candidateEvaluationTime_ms + m_lastMetrics.imagesSavingTime_ms;
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

    // Performance recommendations
    report << "\nPerformance Recommendations:\n";
    if (m_lastMetrics.totalDetectionTime_ms > 50) {
        report << "  - Total time exceeds 50ms. Consider:\n";
        if (m_lastMetrics.preprocessingTime_ms > 20) {
            report << "    * Reducing preprocessing steps\n";
        }
        if (m_lastMetrics.candidatesFound > 10) {
            report << "    * Tightening detection parameters to reduce false positives\n";
        }
        if (m_params.useAdaptiveThreshold && m_lastMetrics.adaptiveThresholdTime_ms > 10) {
            report << "    * Disabling adaptive threshold if not necessary\n";
        }
    }
    else if (m_lastMetrics.totalDetectionTime_ms < 10) {
        report << "  - Excellent performance! Detection completed in under 10ms.\n";
    }
    else {
        report << "  - Good performance. Detection time is within acceptable range.\n";
    }

    report << "======================================================\n";

    return report.str();
}