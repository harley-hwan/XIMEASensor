#include "pch.h"                 // Precompiled header (if used in the project)
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

// Constructor
BallDetector::BallDetector() : pImpl(std::make_unique<Impl>()) {
    // Simple OpenCV functionality test (to ensure OpenCV is initialized)
    cv::Mat testMat(10, 10, CV_8UC1);
    if (testMat.empty()) {
        LOG_ERROR("OpenCV initialization failed");
    }
    else {
        LOG_INFO("BallDetector initialized successfully for ceiling camera");
    }
    InitializeDefaultParams();
}

BallDetector::~BallDetector() = default;

void BallDetector::InitializeDefaultParams() {
    // Default parameters optimized for overhead (ceiling) camera detecting a golf ball on the floor
    m_params.minRadius = 10;         // Detect slightly smaller balls
    m_params.maxRadius = 80;        // Adjust based on camera height (3m ceiling approx.)
    m_params.dp = 1.5;
    m_params.minDist = 30.0;
    m_params.param1 = 100.0;         // Lower Canny high threshold (account for shadows)
    m_params.param2 = 0.9;         // Lower accumulator threshold to detect more candidates
    m_params.brightnessThreshold = 150;  // Brightness threshold for white balls (consider shadows)
    m_params.minCircularity = 0.7f;      // Allow some shape distortion (e.g., motion blur)
    m_params.contrastThreshold = 10.0f;  // Minimum background contrast
    m_params.useColorFilter = true;
    m_params.useCircularityCheck = true;
    m_params.useHoughGradientAlt = true;
    m_params.detectMultiple = false;     // Only one ball expected in scene
    m_params.useMorphology = true;
    m_params.useAdaptiveThreshold = true;
    m_params.correctPerspective = false;
    m_params.enhanceShadows = true;
    m_params.shadowEnhanceFactor = 0.7f;
    m_params.saveIntermediateImages = true;
    m_params.debugOutputDir = "detect_outputs";

    LOG_INFO("BallDetector parameters initialized for ceiling camera:");
    LOG_INFO("  - Radius range: " + std::to_string(m_params.minRadius) + " - " + std::to_string(m_params.maxRadius));
    LOG_INFO("  - Shadow enhancement: " + std::string(m_params.enhanceShadows ? "ON" : "OFF"));
    LOG_INFO("  - Adaptive thresholding: " + std::string(m_params.useAdaptiveThreshold ? "ON" : "OFF"));
    LOG_INFO("  - Mode: Single-ball detection");
}

void BallDetector::ResetToDefaults() {
    InitializeDefaultParams();
    LOG_INFO("BallDetector parameters reset to default values");
}

void BallDetector::SetCalibrationData(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
    // Store camera calibration for later undistortion
    pImpl->m_cameraMatrix = cameraMatrix.clone();
    pImpl->m_distCoeffs = distCoeffs.clone();
    m_params.correctPerspective = true;
    LOG_INFO("Camera calibration data set. Perspective correction enabled.");
}

BallDetectionResult BallDetector::DetectBall(const unsigned char* imageData, int width, int height, int frameIndex) {
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

        // 1. Construct grayscale image from input buffer (8-bit, single channel)
        cv::Mat grayImage(height, width, CV_8UC1, const_cast<unsigned char*>(imageData));

        // 2. Correct lens distortion/perspective if calibration data is available
        cv::Mat correctedImage = grayImage;
        if (m_params.correctPerspective && !pImpl->m_cameraMatrix.empty()) {
            correctedImage = pImpl->correctPerspective(grayImage);
        }

        // 3. Preprocess the image (noise reduction, shadow enhancement, contrast improvement)
        cv::Mat processedImage = pImpl->preprocessImage(correctedImage, m_params);
        pImpl->m_lastProcessedImage = processedImage.clone();  // save for debugging

        // 4. Apply multiple detection methods to gather candidates
        std::vector<cv::Vec3f> allCandidates;

        // 4-1. Hough Circle Transform detection
        std::vector<cv::Vec3f> houghCircles = pImpl->detectCirclesHough(processedImage, m_params);
        allCandidates.insert(allCandidates.end(), houghCircles.begin(), houghCircles.end());
        LOG_DEBUG("HoughCircles found " + std::to_string(houghCircles.size()) + " candidate(s)");

        // 4-2. Adaptive threshold + contour-based detection
        if (m_params.useAdaptiveThreshold) {
            std::vector<cv::Vec3f> adaptiveCircles = pImpl->detectByAdaptiveThreshold(processedImage, m_params);
            allCandidates.insert(allCandidates.end(), adaptiveCircles.begin(), adaptiveCircles.end());
            LOG_DEBUG("Adaptive thresholding found " + std::to_string(adaptiveCircles.size()) + " candidate(s)");
        }

        // 4-3. Otsu threshold + contour detection (backup method, if no candidates yet)
        if (allCandidates.empty()) {
            cv::Mat binary;
            cv::threshold(processedImage, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

            // Use morphology to remove noise and fill gaps in the binary image
            if (m_params.useMorphology) {
                cv::Mat kernelClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
                cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernelClose);
                cv::Mat kernelOpen = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
                cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernelOpen);
            }

            pImpl->m_lastBinaryImage = binary.clone();  // save binary image for debugging

            std::vector<cv::Vec3f> contourCircles = pImpl->detectCircleByContour(binary, m_params);
            allCandidates.insert(allCandidates.end(), contourCircles.begin(), contourCircles.end());
            LOG_DEBUG("Contour-based detection (backup) found " + std::to_string(contourCircles.size()) + " candidate(s)");
        }

        // 5. Remove duplicate/overlapping candidates (merge close circle detections)
        std::vector<cv::Vec3f> uniqueCandidates;
        for (const auto& cand : allCandidates) {
            bool duplicate = false;
            for (const auto& uc : uniqueCandidates) {
                float dist = std::sqrt(std::pow(cand[0] - uc[0], 2) + std::pow(cand[1] - uc[1], 2));
                if (dist < m_params.minDist) {  // if centers are too close, consider them the same ball
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                uniqueCandidates.push_back(cand);
            }
        }
        LOG_DEBUG("After de-duplication: " + std::to_string(uniqueCandidates.size()) + " unique candidate(s)");

        // 6. Evaluate each candidate circle
        std::vector<BallInfo> validBalls;
        for (const auto& circle : uniqueCandidates) {
            BallInfo info;
            info.center = cv::Point2f(circle[0], circle[1]);
            info.radius = circle[2];
            info.frameIndex = frameIndex;
            info.confidence = 0.0f;
            info.circularity = 1.0f;
            info.brightness = 0.0f;
            info.contrast = 0.0f;

            // 6-1. Color/Brightness filter check
            if (m_params.useColorFilter && !pImpl->passesColorFilter(correctedImage, circle, m_params)) {
                LOG_DEBUG("Candidate at (" + std::to_string(cvRound(circle[0])) + "," + std::to_string(cvRound(circle[1])) +
                    ") rejected by brightness/color filter");
                continue;
            }

            // 6-2. Circularity check
            if (m_params.useCircularityCheck) {
                info.circularity = pImpl->calculateCircularity(processedImage, circle, m_params);
                if (info.circularity < m_params.minCircularity) {
                    LOG_DEBUG("Candidate at (" + std::to_string(cvRound(circle[0])) + "," + std::to_string(cvRound(circle[1])) +
                        ") rejected by low circularity: " + std::to_string(info.circularity));
                    continue;
                }
            }

            // 6-3. Contrast check
            info.contrast = pImpl->calculateContrastScore(correctedImage, circle);
            if (info.contrast < m_params.contrastThreshold) {
                LOG_DEBUG("Candidate at (" + std::to_string(cvRound(circle[0])) + "," + std::to_string(cvRound(circle[1])) +
                    ") rejected by low contrast: " + std::to_string(info.contrast));
                continue;
            }

            // 6-4. Brightness (mean intensity) of the ball region
            cv::Mat mask = cv::Mat::zeros(correctedImage.size(), CV_8UC1);
            cv::circle(mask, cv::Point(cvRound(circle[0]), cvRound(circle[1])),
                cvRound(circle[2] * 0.9), cv::Scalar(255), -1);
            info.brightness = cv::mean(correctedImage, mask)[0];

            // 6-5. Compute overall confidence score for this detection
            info.confidence = pImpl->calculateConfidence(processedImage, circle, m_params);

            // If confidence above a threshold, accept this detection
            if (info.confidence >= 0.4f) {
                validBalls.push_back(info);
            }
        }

        // 7. Select the best ball (highest confidence) if any valid detections
        if (!validBalls.empty()) {
            std::sort(validBalls.begin(), validBalls.end(),
                [](const BallInfo& a, const BallInfo& b) { return a.confidence > b.confidence; });
            // We only care about the top result for single-ball detection
            result.balls.push_back(validBalls[0]);
            result.found = true;

            const BallInfo& ball = result.balls[0];
            LOG_INFO("Ball detected at (" + std::to_string(cvRound(ball.center.x)) + ", " +
                std::to_string(cvRound(ball.center.y)) + "), radius=" + std::to_string(cvRound(ball.radius)) +
                ", confidence=" + std::to_string(ball.confidence) +
                ", circularity=" + std::to_string(ball.circularity) +
                ", contrast=" + std::to_string(ball.contrast));
        }

        // 8. Save debug images if enabled
        if (m_params.saveIntermediateImages) {
            const std::string& dir = m_params.debugOutputDir;
            if (!dir.empty()) {
                fs::create_directories(dir);
                pImpl->saveIntermediateImages(dir, frameIndex);
                // Save an output image with the detection result marked (as a color image)
                SaveDetectionImage(imageData, width, height, result,
                    dir + "/frame_" + std::to_string(frameIndex) + "_result.png");
            }
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

    return result;
}

// ---------------- Implementation of helper functions ----------------

cv::Mat BallDetector::Impl::preprocessImage(const cv::Mat& grayImage, const DetectionParams& params) {
    cv::Mat processed = grayImage.clone();

    // 1. Noise reduction with Gaussian blur
    cv::GaussianBlur(processed, processed, cv::Size(5, 5), 1.5);

    // 2. Shadow enhancement (brighten dark regions if enabled)
    if (params.enhanceShadows) {
        processed = enhanceShadowRegions(processed, params.shadowEnhanceFactor);
        m_lastShadowEnhanced = processed.clone();
    }

    // 3. Local contrast enhancement using CLAHE
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    cv::Mat localContrast;
    clahe->apply(processed, localContrast);

    // 4. Blend original and enhanced images to avoid over-amplification
    cv::addWeighted(processed, 0.6, localContrast, 0.4, 0, processed);

    // 5. Sharpen image using Unsharp Mask technique
    cv::Mat blurred;
    cv::GaussianBlur(processed, blurred, cv::Size(0, 0), 2.0);
    cv::addWeighted(processed, 1.5, blurred, -0.5, 0, processed);

    return processed;
}

// Enhance shadow regions by brightening dark areas
cv::Mat BallDetector::Impl::enhanceShadowRegions(const cv::Mat& image, float factor) {
    // Create a mask of dark regions (thresholding inverse to get shadows)
    cv::Mat darkMask;
    cv::threshold(image, darkMask, 100, 255, cv::THRESH_BINARY_INV);

    // Create an image that is zero everywhere except in dark regions
    cv::Mat shadows = image.clone();
    shadows.setTo(cv::Scalar(0), darkMask == 0);
    // Brighten the shadow regions
    shadows *= factor;

    // Add the brightened shadows back to the original image
    cv::Mat result;
    cv::add(image, shadows, result);
    // Clip values to 255 (since adding might exceed 255)
    cv::threshold(result, result, 255, 255, cv::THRESH_TRUNC);
    return result;
}

// Correct lens distortion/perspective using calibration matrix (if available)
cv::Mat BallDetector::Impl::correctPerspective(const cv::Mat& image) {
    if (m_cameraMatrix.empty() || m_distCoeffs.empty()) {
        return image;  // nothing to do if no calibration data
    }
    cv::Mat undistorted;
    cv::undistort(image, undistorted, m_cameraMatrix, m_distCoeffs);
    return undistorted;
}

// Detect circles using Hough Circle Transform
std::vector<cv::Vec3f> BallDetector::Impl::detectCirclesHough(const cv::Mat& image, const DetectionParams& params) {
    std::vector<cv::Vec3f> circles;
    // Use Canny to find edges (for debugging or to assist HOUGH_GRADIENT; not directly used by HOUGH_GRADIENT_ALT)
    cv::Mat edges;
    cv::Canny(image, edges, params.param1 / 2.0, params.param1);
    m_lastEdgeImage = edges.clone();

    int method = params.useHoughGradientAlt ? cv::HOUGH_GRADIENT_ALT : cv::HOUGH_GRADIENT;
    cv::HoughCircles(image, circles, method,
        params.dp,
        params.minDist,
        params.param1,
        params.param2,
        params.minRadius,
        params.maxRadius);
    return circles;
}

// Detect circles by adaptive thresholding and contour analysis
std::vector<cv::Vec3f> BallDetector::Impl::detectByAdaptiveThreshold(const cv::Mat& image, const DetectionParams& params) {
    // Apply adaptive threshold (Gaussian) to create binary image
    cv::Mat binary;
    cv::adaptiveThreshold(image, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
        cv::THRESH_BINARY, 21, -5);
    // Morphological close to fill small holes and smooth the binary mask
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    // Use contour-based circle detection on this binary image
    return detectCircleByContour(binary, params);
}

// Find circles by analyzing contours in a binary image
std::vector<cv::Vec3f> BallDetector::Impl::detectCircleByContour(const cv::Mat& binaryImage, const DetectionParams& params) {
    std::vector<cv::Vec3f> circles;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        // Filter out contours that are too small or too large based on expected ball size
        if (area < CV_PI * params.minRadius * params.minRadius || area > CV_PI * params.maxRadius * params.maxRadius) {
            continue;
        }

        // Fit a minimum enclosing circle to the contour
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);
        if (radius < params.minRadius || radius > params.maxRadius) {
            continue;
        }

        // Calculate contour circularity (4π * area / perimeter^2)
        double perimeter = cv::arcLength(contour, true);
        if (perimeter <= 0) continue;
        double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
        // Allow some leniency (80% of minCircularity) because this is an initial filter
        if (circularity >= params.minCircularity * 0.8f) {
            circles.emplace_back(center.x, center.y, radius);
        }
    }
    return circles;
}

// Calculate how circular the region around the detected circle is
float BallDetector::Impl::calculateCircularity(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Define a ROI around the circle (with some margin to include edges)
    int margin = 10;
    int x = std::max(0, cx - radius - margin);
    int y = std::max(0, cy - radius - margin);
    int w = std::min(image.cols - x, (radius + margin) * 2);
    int h = std::min(image.rows - y, (radius + margin) * 2);
    if (w <= 0 || h <= 0) {
        return 0.0f;
    }
    cv::Mat roi = image(cv::Rect(x, y, w, h)).clone();

    // Binarize the ROI using Otsu's method to separate potential ball area
    cv::Mat roiBinary;
    cv::threshold(roi, roiBinary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Find contours in the ROI
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roiBinary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        return 0.0f;
    }

    // Find the contour with centroid closest to the expected center of the ball within the ROI
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

    // Calculate circularity of the best matching contour
    double area = cv::contourArea(contours[bestIdx]);
    double perimeter = cv::arcLength(contours[bestIdx], true);
    if (perimeter <= 0) {
        return 0.0f;
    }
    double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
    // Cap circularity at 1.0 (perfect circle) and return as float
    return static_cast<float>(std::min(circularity, 1.0));
}

// Check if the detected circle region passes basic brightness/contrast filters 
bool BallDetector::Impl::passesColorFilter(const cv::Mat& grayImage, const cv::Vec3f& circle, const DetectionParams& params) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Create a mask for the ball region
    cv::Mat mask = cv::Mat::zeros(grayImage.size(), CV_8UC1);
    cv::circle(mask, cv::Point(cx, cy), radius, cv::Scalar(255), -1);

    // Compute mean and stddev of the ball region
    cv::Scalar meanVal, stdDevVal;
    cv::meanStdDev(grayImage, meanVal, stdDevVal, mask);

    // Create a ring (donut) mask for background immediately around the ball
    cv::Mat ringMask = cv::Mat::zeros(grayImage.size(), CV_8UC1);
    cv::circle(ringMask, cv::Point(cx, cy), radius + 10, cv::Scalar(255), -1);
    cv::circle(ringMask, cv::Point(cx, cy), radius, cv::Scalar(0), -1);
    // Compute background mean intensity
    cv::Scalar bgMeanVal = cv::mean(grayImage, ringMask);

    double ballMean = meanVal[0];
    double ballStdDev = stdDevVal[0];
    double bgMean = bgMeanVal[0];
    double contrast = std::abs(ballMean - bgMean);

    // Filter conditions:
    // 1. Very bright ball (likely white) 
    if (ballMean >= params.brightnessThreshold) {
        return true;
    }
    // 2. Sufficient contrast with background
    if (contrast >= params.contrastThreshold) {
        return true;
    }
    // 3. Uniform color ball (low standard deviation in intensity)
    if (ballStdDev < 20.0) {
        return true;
    }
    // If none of the conditions met, reject this candidate
    return false;
}

// Calculate contrast score between the ball region and its immediate background
float BallDetector::Impl::calculateContrastScore(const cv::Mat& image, const cv::Vec3f& circle) {
    int cx = cvRound(circle[0]);
    int cy = cvRound(circle[1]);
    int radius = cvRound(circle[2]);

    // Mask for the ball (slightly inside the ball to avoid edges)
    cv::Mat ballMask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::circle(ballMask, cv::Point(cx, cy), static_cast<int>(radius * 0.9), cv::Scalar(255), -1);
    // Mask for a ring around the ball (background)
    cv::Mat bgMask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::circle(bgMask, cv::Point(cx, cy), static_cast<int>(radius * 1.5), cv::Scalar(255), -1);
    cv::circle(bgMask, cv::Point(cx, cy), radius, cv::Scalar(0), -1);

    double ballMean = cv::mean(image, ballMask)[0];
    double bgMean = cv::mean(image, bgMask)[0];
    return static_cast<float>(std::abs(ballMean - bgMean));
}

// Calculate an overall confidence score for a detected circle
float BallDetector::Impl::calculateConfidence(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    // 1. Circularity component (30%)
    float circScore = calculateCircularity(image, circle, params);
    float confidence = circScore * 0.3f;

    // 2. Contrast component (30%)
    float contrast = calculateContrastScore(image, circle);
    float contrastNorm = std::min(1.0f, contrast / 50.0f);  // normalize contrast: 50 intensity difference = full score
    confidence += contrastNorm * 0.3f;

    // 3. Size suitability component (20%) – how close the radius is to expected optimal size
    float optimalRadius = (params.minRadius + params.maxRadius) / 2.0f;
    float radiusRange = (params.maxRadius - params.minRadius) / 2.0f;
    float sizeScore = 1.0f;
    if (radiusRange > 0) {
        float radiusDiff = std::fabs(circle[2] - optimalRadius);
        sizeScore = 1.0f - std::min(1.0f, radiusDiff / radiusRange);
    }
    confidence += sizeScore * 0.2f;

    // 4. Color uniformity component (20%) – reward low intensity variance within the ball
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::circle(mask, cv::Point(cvRound(circle[0]), cvRound(circle[1])), cvRound(circle[2] * 0.8), cv::Scalar(255), -1);
    cv::Scalar meanVal, stdDevVal;
    cv::meanStdDev(image, meanVal, stdDevVal, mask);
    // Compute uniformity (1 - normalized stddev, capped at 1)
    float stdDevIntensity = static_cast<float>(stdDevVal[0]);
    float uniformity = 1.0f - std::min(1.0f, stdDevIntensity / 40.0f);
    confidence += uniformity * 0.2f;

    return confidence;
}

// Save intermediate images (for debugging/analysis) to disk
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
    LOG_DEBUG("Intermediate images saved for frame " + std::to_string(frameIndex));
}

// Draw detection results on a grayscale image buffer (overlay in-place)
bool BallDetector::DrawDetectionResult(unsigned char* imageData, int width, int height,
    const BallDetectionResult& result,
    cv::Scalar color, int thickness) {
    if (!imageData || width <= 0 || height <= 0) {
        LOG_ERROR("Invalid parameters for DrawDetectionResult");
        return false;
    }
    try {
        // Convert input gray buffer to a 3-channel BGR image for drawing
        cv::Mat grayImg(height, width, CV_8UC1, imageData);
        cv::Mat colorImg;
        cv::cvtColor(grayImg, colorImg, cv::COLOR_GRAY2BGR);

        // Title text
        cv::putText(colorImg, "Ball Detection - Ceiling Camera",
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
            cv::Scalar(0, 255, 0), 2);
        if (result.found && !result.balls.empty()) {
            // Draw the detected ball (only the first ball since single detection mode)
            const BallInfo& ball = result.balls[0];
            cv::circle(colorImg, cv::Point(cvRound(ball.center.x), cvRound(ball.center.y)),
                cvRound(ball.radius), color, thickness);
            // Mark the center of the ball
            cv::drawMarker(colorImg, cv::Point(cvRound(ball.center.x), cvRound(ball.center.y)),
                cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 15, 2);
            // Display position
            std::ostringstream info;
            info << "Position: (" << cvRound(ball.center.x) << ", " << cvRound(ball.center.y) << ")";
            cv::putText(colorImg, info.str(), cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
            // Display radius
            info.str("");
            info << "Radius: " << cvRound(ball.radius) << " px";
            cv::putText(colorImg, info.str(), cv::Point(10, 80),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
            // Display confidence
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

        // Convert back to grayscale (the drawing is colored, but we write into original buffer in grayscale form)
        cv::cvtColor(colorImg, grayImg, cv::COLOR_BGR2GRAY);
        return true;
    }
    catch (const cv::Exception& e) {
        LOG_ERROR("OpenCV error in DrawDetectionResult: " + std::string(e.what()));
        return false;
    }
}

// Save a detection result image to disk with annotations (for debugging or analysis)
bool BallDetector::SaveDetectionImage(const unsigned char* originalImage, int width, int height,
    const BallDetectionResult& result,
    const std::string& outputPath) {
    if (!originalImage || width <= 0 || height <= 0) {
        LOG_ERROR("Invalid parameters for SaveDetectionImage");
        return false;
    }
    try {
        // Create a color image from the original grayscale for annotation
        cv::Mat grayImg(height, width, CV_8UC1, const_cast<unsigned char*>(originalImage));
        cv::Mat colorImg;
        cv::cvtColor(grayImg, colorImg, cv::COLOR_GRAY2BGR);

        // Add timestamp at bottom-left corner
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        struct tm localTime;
#ifdef _MSC_VER
        localtime_s(&localTime, &t); // secure version for MSVC
#else
        localtime_r(&t, &localTime); // POSIX
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
            // Draw detected ball
            cv::circle(colorImg, cv::Point(cvRound(ball.center.x), cvRound(ball.center.y)),
                cvRound(ball.radius), cv::Scalar(0, 255, 0), 2);
            // Mark center
            cv::drawMarker(colorImg, cv::Point(cvRound(ball.center.x), cvRound(ball.center.y)),
                cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 15, 2);
            // Draw info box background
            int boxX = 10, boxY = 50;
            cv::rectangle(colorImg, cv::Point(boxX, boxY), cv::Point(boxX + 250, boxY + 100),
                cv::Scalar(0, 0, 0), cv::FILLED);
            cv::rectangle(colorImg, cv::Point(boxX, boxY), cv::Point(boxX + 250, boxY + 100),
                cv::Scalar(0, 255, 0), 1);
            // Write details in the info box
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

        if (cv::imwrite(outputPath, colorImg)) {
            LOG_DEBUG("Detection result image saved: " + outputPath);
            return true;
        }
        else {
            LOG_ERROR("Failed to save detection image: " + outputPath);
            return false;
        }
    }
    catch (const cv::Exception& e) {
        LOG_ERROR("OpenCV error in SaveDetectionImage: " + std::string(e.what()));
        return false;
    }
}

void BallDetector::AutoTuneParameters(const std::vector<unsigned char*>& sampleImages, int width, int height) {
    LOG_INFO("Starting auto-tune with " + std::to_string(sampleImages.size()) + " sample images");
    // Base parameters to start with
    DetectionParams baseParams = m_params;
    std::vector<DetectionParams> paramCandidates;

    // Generate a range of parameter combinations to test
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

    float bestScore = 0.0f;
    DetectionParams bestParams = baseParams;
    // Evaluate each parameter combination on the sample images
    for (const auto& params : paramCandidates) {
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
            // Average confidence score * detection rate
            float avgScore = totalScore / detectCount;
            float detectionRate = static_cast<float>(detectCount) / sampleImages.size();
            float combinedScore = avgScore * detectionRate;
            if (combinedScore > bestScore) {
                bestScore = combinedScore;
                bestParams = params;
            }
        }
    }

    // Apply the best found parameters
    m_params = bestParams;
    LOG_INFO("Auto-tune completed. Best score: " + std::to_string(bestScore));
    LOG_INFO("Optimal parameters: circularity=" + std::to_string(bestParams.minCircularity) +
        ", brightnessThreshold=" + std::to_string(bestParams.brightnessThreshold) +
        ", contrastThreshold=" + std::to_string(bestParams.contrastThreshold));
}
