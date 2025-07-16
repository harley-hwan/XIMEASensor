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

class BallDetector::Impl {
public:
    cv::Mat m_lastProcessedImage;

    cv::Mat preprocessImage(const cv::Mat& grayImage);
    std::vector<cv::Vec3f> detectCircles(const cv::Mat& processedImage, const DetectionParams& params);
    float calculateCircularity(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
    bool isWhiteBall(const cv::Mat& grayImage, const cv::Vec3f& circle, const DetectionParams& params);
    float calculateConfidence(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
};

BallDetector::BallDetector() : pImpl(std::make_unique<Impl>()) {
    cv::Mat testMat(10, 10, CV_8UC1);
    if (testMat.empty()) {
        LOG_ERROR("OpenCV initialization failed");
    }
    else {
        LOG_INFO("BallDetector initialized successfully");
    }
}

BallDetector::~BallDetector() = default;

BallDetectionResult BallDetector::DetectBall(const unsigned char* imageData,
    int width, int height,
    int frameIndex) {
    BallDetectionResult result;
    result.found = false;
    result.balls.clear();

    if (!imageData || width <= 0 || height <= 0) {
        result.errorMessage = "Invalid input parameters";
        LOG_ERROR("DetectBall: " + result.errorMessage);
        return result;
    }

    try {
        // �׷��̽����� �̹��� ����
        cv::Mat grayImage(height, width, CV_8UC1, const_cast<unsigned char*>(imageData));

        // �̹��� ��ó��
        cv::Mat processedImage = pImpl->preprocessImage(grayImage);

        // �� ����
        std::vector<cv::Vec3f> circles = pImpl->detectCircles(processedImage, m_params);

        LOG_DEBUG("Frame " + std::to_string(frameIndex) + ": Found " +
            std::to_string(circles.size()) + " candidate circles");

        if (circles.empty()) {
            LOG_DEBUG("No circles detected in frame " + std::to_string(frameIndex));
            return result;
        }

        // ����� ������ ��
        for (const auto& circle : circles) {
            BallInfo ballInfo;
            ballInfo.center = cv::Point2f(circle[0], circle[1]);
            ballInfo.radius = circle[2];
            ballInfo.frameIndex = frameIndex;

            // ���� ���� ����
            if (m_params.useColorFilter && !pImpl->isWhiteBall(grayImage, circle, m_params)) {
                LOG_DEBUG("Circle rejected by color filter at (" +
                    std::to_string((int)circle[0]) + "," +
                    std::to_string((int)circle[1]) + ")");
                continue;
            }

            // ������ �˻�
            float circularity = 1.0f;
            if (m_params.useCircularityCheck) {
                circularity = pImpl->calculateCircularity(grayImage, circle, m_params);
                if (circularity < m_params.minCircularity) {
                    LOG_DEBUG("Circle rejected by circularity check: " +
                        std::to_string(circularity));
                    continue;
                }
            }

            // �ŷڵ� ���
            ballInfo.confidence = pImpl->calculateConfidence(grayImage, circle, m_params);

            // �Ӱ谪 �̻��� �ŷڵ��� ���� ��츸 �߰�
            if (ballInfo.confidence >= 0.5f) {
                result.balls.push_back(ballInfo);
                result.found = true;

                LOG_INFO("Ball detected in frame " + std::to_string(frameIndex) +
                    " at (" + std::to_string((int)ballInfo.center.x) + "," +
                    std::to_string((int)ballInfo.center.y) + ") with confidence " +
                    std::to_string(ballInfo.confidence));

                if (!m_params.detectMultiple && result.balls.size() >= 1) {
                    break;
                }
            }
        }

        // �ŷڵ� ������ ����
        if (result.found) {
            std::sort(result.balls.begin(), result.balls.end(),
                [](const BallInfo& a, const BallInfo& b) {
                    return a.confidence > b.confidence;
                });

            LOG_INFO("Total " + std::to_string(result.balls.size()) +
                " ball(s) detected in frame " + std::to_string(frameIndex));
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

cv::Mat BallDetector::Impl::preprocessImage(const cv::Mat& grayImage) {
    cv::Mat processed;

    // ����þ� ���� ������ ����
    cv::GaussianBlur(grayImage, processed, cv::Size(5, 5), 1.5);

    // ������׷� �յ�ȭ�� ��� ���
    cv::Mat equalized;
    cv::equalizeHist(processed, equalized);

    // �� �̹����� ���� ������� ����
    cv::addWeighted(processed, 0.7, equalized, 0.3, 0, processed);

    return processed;
}

std::vector<cv::Vec3f> BallDetector::Impl::detectCircles(const cv::Mat& processedImage, const DetectionParams& params) {
    std::vector<cv::Vec3f> circles;

    // HoughCircles�� �� ����
    cv::HoughCircles(processedImage, circles,
        cv::HOUGH_GRADIENT,
        params.dp,
        params.minDist,
        params.param1,
        params.param2,
        params.minRadius,
        params.maxRadius);

    return circles;
}

float BallDetector::Impl::calculateCircularity(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    int centerX = static_cast<int>(circle[0]);
    int centerY = static_cast<int>(circle[1]);
    int radius = static_cast<int>(circle[2]);

    // ROI ����
    int roiSize = radius * 2 + 10;
    int x = std::max(0, centerX - roiSize / 2);
    int y = std::max(0, centerY - roiSize / 2);
    int width = std::min(roiSize, image.cols - x);
    int height = std::min(roiSize, image.rows - y);

    if (width <= 0 || height <= 0) return 0.0f;

    cv::Mat roi = image(cv::Rect(x, y, width, height));

    // ����ȭ
    cv::Mat binary;
    cv::threshold(roi, binary, params.brightnessThreshold, 255, cv::THRESH_BINARY);

    // ������ ã��
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return 0.0f;

    // ���� ū ������ ����
    double maxArea = 0;
    int maxIdx = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            maxIdx = static_cast<int>(i);
        }
    }

    if (maxIdx < 0) return 0.0f;

    // ������ ���
    double perimeter = cv::arcLength(contours[maxIdx], true);
    if (perimeter == 0) return 0.0f;

    float circularity = static_cast<float>(4 * CV_PI * maxArea / (perimeter * perimeter));

    return std::min(1.0f, circularity);
}

bool BallDetector::Impl::isWhiteBall(const cv::Mat& grayImage, const cv::Vec3f& circle, const DetectionParams& params) {
    int centerX = static_cast<int>(circle[0]);
    int centerY = static_cast<int>(circle[1]);
    int radius = static_cast<int>(circle[2]);

    // �� ������ ��� ��� ���
    double sumBrightness = 0;
    int pixelCount = 0;

    for (int y = centerY - radius; y <= centerY + radius; y++) {
        for (int x = centerX - radius; x <= centerX + radius; x++) {
            if (x >= 0 && x < grayImage.cols && y >= 0 && y < grayImage.rows) {
                double distance = std::sqrt(std::pow(x - centerX, 2) + std::pow(y - centerY, 2));
                if (distance <= radius) {
                    sumBrightness += grayImage.at<unsigned char>(y, x);
                    pixelCount++;
                }
            }
        }
    }

    if (pixelCount == 0) return false;

    double avgBrightness = sumBrightness / pixelCount;

    // ��Ⱑ �Ӱ谪 �̻��̸� ������� �Ǵ�
    return avgBrightness >= params.brightnessThreshold;
}

float BallDetector::Impl::calculateConfidence(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    float confidence = 0.0f;

    // 1. ������ ���� (40%)
    float circularity = calculateCircularity(image, circle, params);
    confidence += circularity * 0.4f;

    // 2. ��� ���ϼ� ���� (30%)
    int centerX = static_cast<int>(circle[0]);
    int centerY = static_cast<int>(circle[1]);
    int radius = static_cast<int>(circle[2]);

    std::vector<int> pixelValues;
    for (int y = centerY - radius; y <= centerY + radius; y++) {
        for (int x = centerX - radius; x <= centerX + radius; x++) {
            if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                double distance = std::sqrt(std::pow(x - centerX, 2) + std::pow(y - centerY, 2));
                if (distance <= radius * 0.8) {
                    pixelValues.push_back(image.at<unsigned char>(y, x));
                }
            }
        }
    }

    if (!pixelValues.empty()) {
        double mean = 0;
        for (int val : pixelValues) mean += val;
        mean /= pixelValues.size();

        double variance = 0;
        for (int val : pixelValues) {
            variance += std::pow(val - mean, 2);
        }
        variance /= pixelValues.size();
        double stdDev = std::sqrt(variance);

        float uniformity = 1.0f - std::min(1.0f, static_cast<float>(stdDev / 50.0));
        confidence += uniformity * 0.3f;
    }

    // 3. ũ�� ���ռ� ���� (30%)
    float optimalRadius = (params.minRadius + params.maxRadius) / 2.0f;
    float radiusDiff = std::abs(circle[2] - optimalRadius);
    float radiusRange = (params.maxRadius - params.minRadius) / 2.0f;
    float sizeScore = 1.0f - std::min(1.0f, radiusDiff / radiusRange);
    confidence += sizeScore * 0.3f;

    return confidence;
}

bool BallDetector::DrawDetectionResult(unsigned char* imageData, int width, int height,
    const BallDetectionResult& result,
    cv::Scalar color, int thickness) {
    if (!imageData || width <= 0 || height <= 0) {
        LOG_ERROR("Invalid parameters for drawing detection result");
        return false;
    }

    try {
        cv::Mat grayImage(height, width, CV_8UC1, imageData);
        cv::Mat colorImage;
        cv::cvtColor(grayImage, colorImage, cv::COLOR_GRAY2BGR);

        // ������ ���� ǥ��
        cv::putText(colorImage, "Ball Detection Result",
            cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        if (result.found) {
            std::string detectionInfo = "Detected: " + std::to_string(result.balls.size()) + " ball(s)";
            cv::putText(colorImage, detectionInfo,
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
        }
        else {
            cv::putText(colorImage, "No ball detected",
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1);
        }

        // �� ����� ���� ����
        int ballIndex = 0;
        for (const auto& ball : result.balls) {
            ballIndex++;

            // �� �׸���
            cv::circle(colorImage,
                cv::Point(static_cast<int>(ball.center.x), static_cast<int>(ball.center.y)),
                static_cast<int>(ball.radius),
                cv::Scalar(0, 255, 0), thickness);

            // �߽��� ǥ��
            cv::drawMarker(colorImage,
                cv::Point(static_cast<int>(ball.center.x), static_cast<int>(ball.center.y)),
                cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 2);

            // ���� �ؽ�Ʈ
            std::stringstream ss;
            ss << "Ball #" << ballIndex;
            cv::putText(colorImage, ss.str(),
                cv::Point(static_cast<int>(ball.center.x) - 30,
                    static_cast<int>(ball.center.y) - static_cast<int>(ball.radius) - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
        }

        // �ٽ� �׷��̽����Ϸ� ��ȯ
        cv::cvtColor(colorImage, grayImage, cv::COLOR_BGR2GRAY);

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
        LOG_ERROR("Invalid parameters for saving detection image");
        return false;
    }

    try {
        cv::Mat grayImage(height, width, CV_8UC1, const_cast<unsigned char*>(originalImage));
        cv::Mat colorImage;
        cv::cvtColor(grayImage, colorImage, cv::COLOR_GRAY2BGR);

        // Ÿ�ӽ����� �߰�
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        struct tm localTime;
        localtime_s(&localTime, &time_t);

        std::stringstream timestamp;
        timestamp << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S");
        cv::putText(colorImage, timestamp.str(),
            cv::Point(10, height - 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        // ���� ��� ǥ��
        cv::putText(colorImage, "Ball Detection Result",
            cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        if (result.found) {
            std::string detectionInfo = "Detected: " + std::to_string(result.balls.size()) + " ball(s)";
            cv::putText(colorImage, detectionInfo,
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);

            // �� ����� ���� ����
            int ballIndex = 0;
            for (const auto& ball : result.balls) {
                ballIndex++;

                // �� �׸���
                cv::circle(colorImage,
                    cv::Point(static_cast<int>(ball.center.x), static_cast<int>(ball.center.y)),
                    static_cast<int>(ball.radius),
                    cv::Scalar(0, 255, 0), 2);

                // �߽��� ǥ��
                cv::drawMarker(colorImage,
                    cv::Point(static_cast<int>(ball.center.x), static_cast<int>(ball.center.y)),
                    cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 2);

                // ���� �ڽ�
                int boxX = static_cast<int>(ball.center.x) - 50;
                int boxY = static_cast<int>(ball.center.y) - static_cast<int>(ball.radius) - 45;

                cv::rectangle(colorImage,
                    cv::Point(boxX, boxY),
                    cv::Point(boxX + 100, boxY + 40),
                    cv::Scalar(0, 0, 0), cv::FILLED);

                // ���� �ؽ�Ʈ
                std::stringstream ss;
                ss << "Ball #" << ballIndex;
                cv::putText(colorImage, ss.str(),
                    cv::Point(boxX + 5, boxY + 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);

                std::stringstream coords;
                coords << "(" << static_cast<int>(ball.center.x) << "," << static_cast<int>(ball.center.y) << ")";
                cv::putText(colorImage, coords.str(),
                    cv::Point(boxX + 5, boxY + 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);
            }
        }
        else {
            cv::putText(colorImage, "No ball detected",
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1);
        }

        // �̹��� ����
        bool saveResult = cv::imwrite(outputPath, colorImage);

        if (saveResult) {
            LOG_DEBUG("Detection image saved: " + outputPath);
        }
        else {
            LOG_ERROR("Failed to save detection image: " + outputPath);
        }

        return saveResult;

    }
    catch (const cv::Exception& e) {
        LOG_ERROR("OpenCV error in SaveDetectionImage: " + std::string(e.what()));
        return false;
    }
}

void BallDetector::AutoTuneParameters(const std::vector<unsigned char*>& sampleImages,
    int width, int height) {
    LOG_INFO("Auto-tuning parameters with " + std::to_string(sampleImages.size()) + " sample images");
    // TODO: ���� ����
}