#include "pch.h"
#include "GolfBallDetector.h"
#include "Logger.h"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>

// Private implementation class
class GolfBallDetector::Impl {
public:
    cv::Mat m_lastProcessedImage;

    cv::Mat preprocessImage(const cv::Mat& grayImage);
    std::vector<cv::Vec3f> detectCircles(const cv::Mat& processedImage, const DetectionParams& params);
    float calculateCircularity(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
    bool isWhiteBall(const cv::Mat& grayImage, const cv::Vec3f& circle, const DetectionParams& params);
    float calculateConfidence(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params);
};

GolfBallDetector::GolfBallDetector() : pImpl(std::make_unique<Impl>()) {
    // OpenCV �ʱ�ȭ Ȯ��
    cv::Mat testMat(10, 10, CV_8UC1);
    if (testMat.empty()) {
        LOG_ERROR("OpenCV initialization failed");
    }
}

GolfBallDetector::~GolfBallDetector() = default;

GolfBallDetectionResult GolfBallDetector::DetectGolfBall(const unsigned char* imageData,
    int width, int height,
    int frameIndex) {
    GolfBallDetectionResult result;
    result.found = false;
    result.balls.clear();

    if (!imageData || width <= 0 || height <= 0) {
        result.errorMessage = "Invalid input parameters";
        return result;
    }

    try {
        // �׷��̽����� �̹��� ����
        cv::Mat grayImage(height, width, CV_8UC1, const_cast<unsigned char*>(imageData));

        // �̹��� ��ó��
        cv::Mat processedImage = pImpl->preprocessImage(grayImage);

        // �� ����
        std::vector<cv::Vec3f> circles = pImpl->detectCircles(processedImage, m_params);

        if (circles.empty()) {
            LOG_DEBUG("No circles detected in frame " + std::to_string(frameIndex));
            return result;
        }

        // ����� ������ ��
        for (const auto& circle : circles) {
            GolfBallInfo ballInfo;
            ballInfo.center = cv::Point2f(circle[0], circle[1]);
            ballInfo.radius = circle[2];
            ballInfo.frameIndex = frameIndex;

            // ���� ���� ����
            if (m_params.useColorFilter && !pImpl->isWhiteBall(grayImage, circle, m_params)) {
                continue;
            }

            // ������ �˻�
            if (m_params.useCircularityCheck) {
                float circularity = pImpl->calculateCircularity(grayImage, circle, m_params);
                if (circularity < m_params.minCircularity) {
                    continue;
                }
            }

            // �ŷڵ� ���
            ballInfo.confidence = pImpl->calculateConfidence(grayImage, circle, m_params);

            // �Ӱ谪 �̻��� �ŷڵ��� ���� ��츸 �߰�
            if (ballInfo.confidence >= 0.5f) {
                result.balls.push_back(ballInfo);
                result.found = true;

                if (!m_params.detectMultiple && result.balls.size() >= 1) {
                    break;
                }
            }
        }

        // �ŷڵ� ������ ����
        if (result.found) {
            std::sort(result.balls.begin(), result.balls.end(),
                [](const GolfBallInfo& a, const GolfBallInfo& b) {
                    return a.confidence > b.confidence;
                });

            LOG_INFO("Detected " + std::to_string(result.balls.size()) +
                " golf ball(s) in frame " + std::to_string(frameIndex));
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

cv::Mat GolfBallDetector::Impl::preprocessImage(const cv::Mat& grayImage) {
    cv::Mat processed;

    // ����þ� ���� ������ ����
    cv::GaussianBlur(grayImage, processed, cv::Size(5, 5), 1.5);

    // ������׷� �յ�ȭ�� ��� ���
    cv::Mat equalized;
    cv::equalizeHist(processed, equalized);

    // �� �̹����� ���� ������� ���� (������ ������ ����)
    cv::addWeighted(processed, 0.7, equalized, 0.3, 0, processed);

    return processed;
}

std::vector<cv::Vec3f> GolfBallDetector::Impl::detectCircles(const cv::Mat& processedImage, const DetectionParams& params) {
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

float GolfBallDetector::Impl::calculateCircularity(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
    int centerX = static_cast<int>(circle[0]);
    int centerY = static_cast<int>(circle[1]);
    int radius = static_cast<int>(circle[2]);

    // ROI ���� (�� �ֺ� ����)
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

    // ������ ��� (4�� �� area / perimeter��)
    double perimeter = cv::arcLength(contours[maxIdx], true);
    if (perimeter == 0) return 0.0f;

    float circularity = static_cast<float>(4 * CV_PI * maxArea / (perimeter * perimeter));

    return std::min(1.0f, circularity);
}

bool GolfBallDetector::Impl::isWhiteBall(const cv::Mat& grayImage, const cv::Vec3f& circle, const DetectionParams& params) {
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

float GolfBallDetector::Impl::calculateConfidence(const cv::Mat& image, const cv::Vec3f& circle, const DetectionParams& params) {
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
                if (distance <= radius * 0.8) {  // �߽ɺ� 80% ������ ���
                    pixelValues.push_back(image.at<unsigned char>(y, x));
                }
            }
        }
    }

    if (!pixelValues.empty()) {
        // ǥ������ ���
        double mean = 0;
        for (int val : pixelValues) mean += val;
        mean /= pixelValues.size();

        double variance = 0;
        for (int val : pixelValues) {
            variance += std::pow(val - mean, 2);
        }
        variance /= pixelValues.size();
        double stdDev = std::sqrt(variance);

        // ǥ�������� �������� ������ (���� ����)
        float uniformity = 1.0f - std::min(1.0f, static_cast<float>(stdDev / 50.0));
        confidence += uniformity * 0.3f;
    }

    // 3. ũ�� ���ռ� ���� (30%)
    // �������� �Ϲ����� ũ�� ������ �������� ���� ����
    float optimalRadius = (params.minRadius + params.maxRadius) / 2.0f;
    float radiusDiff = std::abs(circle[2] - optimalRadius);
    float radiusRange = (params.maxRadius - params.minRadius) / 2.0f;
    float sizeScore = 1.0f - std::min(1.0f, radiusDiff / radiusRange);
    confidence += sizeScore * 0.3f;

    return confidence;
}

bool GolfBallDetector::DrawDetectionResult(unsigned char* imageData, int width, int height,
    const GolfBallDetectionResult& result,
    cv::Scalar color, int thickness) {
    if (!imageData || width <= 0 || height <= 0) {
        LOG_ERROR("Invalid parameters for drawing detection result");
        return false;
    }

    try {
        // �׷��̽������� BGR�� ��ȯ (�÷� �׸��⸦ ����)
        cv::Mat grayImage(height, width, CV_8UC1, imageData);
        cv::Mat colorImage;
        cv::cvtColor(grayImage, colorImage, cv::COLOR_GRAY2BGR);

        // ������ ���� ǥ��
        cv::putText(colorImage, "Golf Ball Detection Result",
            cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        if (result.found) {
            std::string detectionInfo = "Detected: " + std::to_string(result.balls.size()) + " ball(s)";
            cv::putText(colorImage, detectionInfo,
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
        }
        else {
            cv::putText(colorImage, "No golf ball detected",
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1);
        }

        // �� ����� �������� ����
        int ballIndex = 0;
        for (const auto& ball : result.balls) {
            ballIndex++;

            // �� �׸��� (�ʷϻ�)
            cv::circle(colorImage,
                cv::Point(static_cast<int>(ball.center.x), static_cast<int>(ball.center.y)),
                static_cast<int>(ball.radius),
                cv::Scalar(0, 255, 0), thickness);

            // �߽��� ǥ�� (������ ����)
            cv::drawMarker(colorImage,
                cv::Point(static_cast<int>(ball.center.x), static_cast<int>(ball.center.y)),
                cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 2);

            // ��ǥ �� ���� �ؽ�Ʈ ǥ��
            std::stringstream ss;
            ss << "Ball #" << ballIndex;
            cv::putText(colorImage, ss.str(),
                cv::Point(static_cast<int>(ball.center.x) - 30,
                    static_cast<int>(ball.center.y) - static_cast<int>(ball.radius) - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

            // ��ǥ ǥ��
            std::stringstream coords;
            coords << "(" << static_cast<int>(ball.center.x) << ", "
                << static_cast<int>(ball.center.y) << ")";
            cv::putText(colorImage, coords.str(),
                cv::Point(static_cast<int>(ball.center.x) - 30,
                    static_cast<int>(ball.center.y) - static_cast<int>(ball.radius) - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

            // �ŷڵ� ǥ��
            std::stringstream conf;
            conf << "Conf: " << std::fixed << std::setprecision(1) << ball.confidence * 100 << "%";
            cv::putText(colorImage, conf.str(),
                cv::Point(static_cast<int>(ball.center.x) - 30,
                    static_cast<int>(ball.center.y) + static_cast<int>(ball.radius) + 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

            // ������ ǥ��
            std::stringstream rad;
            rad << "R: " << static_cast<int>(ball.radius) << "px";
            cv::putText(colorImage, rad.str(),
                cv::Point(static_cast<int>(ball.center.x) - 30,
                    static_cast<int>(ball.center.y) + static_cast<int>(ball.radius) + 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
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

bool GolfBallDetector::SaveDetectionImage(const unsigned char* originalImage, int width, int height,
    const GolfBallDetectionResult& result,
    const std::string& outputPath) {
    if (!originalImage || width <= 0 || height <= 0) {
        LOG_ERROR("Invalid parameters for saving detection image");
        return false;
    }

    try {
        // ���� �̹����� �÷��� ��ȯ
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

        // ������ ���� ǥ��
        cv::putText(colorImage, "Golf Ball Detection Result",
            cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        if (result.found) {
            std::string detectionInfo = "Detected: " + std::to_string(result.balls.size()) + " ball(s)";
            cv::putText(colorImage, detectionInfo,
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);

            // �� ����� �������� ����
            int ballIndex = 0;
            for (const auto& ball : result.balls) {
                ballIndex++;

                // �� �׸��� (�ʷϻ�)
                cv::circle(colorImage,
                    cv::Point(static_cast<int>(ball.center.x), static_cast<int>(ball.center.y)),
                    static_cast<int>(ball.radius),
                    cv::Scalar(0, 255, 0), 2);

                // �߽��� ǥ�� (������ ����)
                cv::drawMarker(colorImage,
                    cv::Point(static_cast<int>(ball.center.x), static_cast<int>(ball.center.y)),
                    cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 2);

                // ���� �ڽ� ���
                cv::rectangle(colorImage,
                    cv::Point(static_cast<int>(ball.center.x) - 40,
                        static_cast<int>(ball.center.y) - static_cast<int>(ball.radius) - 40),
                    cv::Point(static_cast<int>(ball.center.x) + 40,
                        static_cast<int>(ball.center.y) - static_cast<int>(ball.radius) - 5),
                    cv::Scalar(0, 0, 0), cv::FILLED);

                // ��ǥ �� ���� �ؽ�Ʈ ǥ��
                std::stringstream ss;
                ss << "Ball #" << ballIndex;
                cv::putText(colorImage, ss.str(),
                    cv::Point(static_cast<int>(ball.center.x) - 30,
                        static_cast<int>(ball.center.y) - static_cast<int>(ball.radius) - 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);

                // ��ǥ ǥ��
                std::stringstream coords;
                coords << "X:" << static_cast<int>(ball.center.x) << " Y:" << static_cast<int>(ball.center.y);
                cv::putText(colorImage, coords.str(),
                    cv::Point(static_cast<int>(ball.center.x) - 35,
                        static_cast<int>(ball.center.y) - static_cast<int>(ball.radius) - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);

                // �ŷڵ��� ������ ���� �ڽ�
                cv::rectangle(colorImage,
                    cv::Point(static_cast<int>(ball.center.x) - 40,
                        static_cast<int>(ball.center.y) + static_cast<int>(ball.radius) + 5),
                    cv::Point(static_cast<int>(ball.center.x) + 40,
                        static_cast<int>(ball.center.y) + static_cast<int>(ball.radius) + 35),
                    cv::Scalar(0, 0, 0), cv::FILLED);

                // �ŷڵ� ǥ��
                std::stringstream conf;
                conf << "Conf: " << std::fixed << std::setprecision(1) << ball.confidence * 100 << "%";
                cv::putText(colorImage, conf.str(),
                    cv::Point(static_cast<int>(ball.center.x) - 35,
                        static_cast<int>(ball.center.y) + static_cast<int>(ball.radius) + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 255, 255), 1);

                // ������ ǥ��
                std::stringstream rad;
                rad << "R: " << static_cast<int>(ball.radius) << "px";
                cv::putText(colorImage, rad.str(),
                    cv::Point(static_cast<int>(ball.center.x) - 35,
                        static_cast<int>(ball.center.y) + static_cast<int>(ball.radius) + 32),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);
            }
        }
        else {
            cv::putText(colorImage, "No golf ball detected",
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1);
        }

        // �̹��� ����
        bool saveResult = cv::imwrite(outputPath, colorImage);

        if (saveResult) {
            LOG_DEBUG("Detection image saved: " + outputPath +
                " (Found: " + std::to_string(result.found) +
                ", Balls: " + std::to_string(result.balls.size()) + ")");
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

void GolfBallDetector::AutoTuneParameters(const std::vector<unsigned char*>& sampleImages,
    int width, int height) {
    // �ڵ� �Ķ���� ���� (���� ���� ����)
    LOG_INFO("Auto-tuning parameters with " + std::to_string(sampleImages.size()) + " sample images");

    // TODO: ����
    // 1. �پ��� �Ķ���� ���� �õ�
    // 2. �� ���տ� ���� ���� ���� ��
    // 3. ���� �Ķ���� ����
}