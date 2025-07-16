#pragma once
#include <vector>
#include <string>
#include <opencv2/core/types.hpp>

namespace cv {
    class Mat;
}

struct BallInfo {
    cv::Point2f center;      // �� �߽� ��ǥ
    float radius;            // �� ������
    float confidence;        // ���� �ŷڵ� (0.0 ~ 1.0)
    int frameIndex;          // ������ �ε���
};

struct BallDetectionResult {
    bool found;              // �� �߰� ����
    std::vector<BallInfo> balls;  // ����� ����
    std::string errorMessage;
};

class BallDetector {
public:
    struct DetectionParams {
        // Hough Circle ���� �Ķ����
        double dp = 1.2;              // �ػ� ����
        double minDist = 30.0;        // ����� �� ������ �ּ� �Ÿ�
        double param1 = 100.0;        // Canny edge detector�� ���� �Ӱ谪
        double param2 = 25.0;         // �� ���� �Ӱ谪
        int minRadius = 40;           // �ּ� ������ (�ȼ�)
        int maxRadius = 100;           // �ִ� ������ (�ȼ�)

        // ���� ���͸� �Ķ����
        int brightnessThreshold = 180;  // ��� �Ӱ谪 (0-255)
        float minCircularity = 0.7f;    // �ּ� ������ (0.0-1.0)

        // ���� �ɼ�
        bool useColorFilter = true;     // ���� ���� ��� ����
        bool useCircularityCheck = true;// ������ �˻� ��� ����
        bool detectMultiple = true;     // ���� �� ���� ���
    };

private:
    DetectionParams m_params;

    // Private implementation (PIMPL pattern)
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    BallDetector();
    ~BallDetector();

    // �Ķ���� ����
    void SetParameters(const DetectionParams& params) { m_params = params; }
    DetectionParams GetParameters() const { return m_params; }

    // ���� �̹������� �� ����
    BallDetectionResult DetectBall(const unsigned char* imageData,
        int width, int height,
        int frameIndex = 0);

    // ���� ����� �̹����� �׸���
    bool DrawDetectionResult(unsigned char* imageData, int width, int height,
        const BallDetectionResult& result,
        cv::Scalar color = cv::Scalar(0, 255, 0),
        int thickness = 2);

    // ���� ����� ���� �̹����� ����
    bool SaveDetectionImage(const unsigned char* originalImage, int width, int height,
        const BallDetectionResult& result,
        const std::string& outputPath);

    // �Ķ���� �ڵ� ���� (�ɼ�)
    void AutoTuneParameters(const std::vector<unsigned char*>& sampleImages,
        int width, int height);
};