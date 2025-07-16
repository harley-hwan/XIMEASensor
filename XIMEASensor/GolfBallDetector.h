#pragma once
#include <vector>
#include <string>
#include <opencv2/core/types.hpp>  // OpenCV Point2f Ÿ���� ���� �ʿ�

// Forward declaration for cv::Mat to avoid full OpenCV dependency
namespace cv {
    class Mat;
}

struct GolfBallInfo {
    cv::Point2f center;      // ������ �߽� ��ǥ
    float radius;            // ������ ������
    float confidence;        // ���� �ŷڵ� (0.0 ~ 1.0)
    int frameIndex;          // ������ �ε���
};

struct GolfBallDetectionResult {
    bool found;              // ������ �߰� ����
    std::vector<GolfBallInfo> balls;  // ����� ��������
    std::string errorMessage;
};

class GolfBallDetector {
public:
    struct DetectionParams {
        // Hough Circle ���� �Ķ����
        double dp = 1.2;              // �ػ� ����
        double minDist = 30.0;        // ����� �� ������ �ּ� �Ÿ�
        double param1 = 100.0;        // Canny edge detector�� ���� �Ӱ谪
        double param2 = 30.0;         // �� ���� �Ӱ谪
        int minRadius = 5;            // �ּ� ������ (�ȼ�)
        int maxRadius = 50;           // �ִ� ������ (�ȼ�)

        // ���� ���͸� �Ķ����
        int brightnessThreshold = 200;  // ��� �Ӱ谪 (0-255)
        float minCircularity = 0.8f;    // �ּ� ������ (0.0-1.0)

        // ���� �ɼ�
        bool useColorFilter = true;     // ���� ���� ��� ����
        bool useCircularityCheck = true;// ������ �˻� ��� ����
        bool detectMultiple = false;    // ���� �� ���� ���
    };

private:
    DetectionParams m_params;

    // Private implementation (PIMPL pattern)
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    GolfBallDetector();
    ~GolfBallDetector();

    // �Ķ���� ����
    void SetParameters(const DetectionParams& params) { m_params = params; }
    DetectionParams GetParameters() const { return m_params; }

    // ���� �̹������� ������ ����
    GolfBallDetectionResult DetectGolfBall(const unsigned char* imageData,
        int width, int height,
        int frameIndex = 0);

    // ���� ����� �̹����� �׸���
    bool DrawDetectionResult(unsigned char* imageData, int width, int height,
        const GolfBallDetectionResult& result,
        cv::Scalar color = cv::Scalar(0, 255, 0),
        int thickness = 2);

    // ���� ����� ���� �̹����� ����
    bool SaveDetectionImage(const unsigned char* originalImage, int width, int height,
        const GolfBallDetectionResult& result,
        const std::string& outputPath);

    // �Ķ���� �ڵ� ���� (�ɼ�)
    void AutoTuneParameters(const std::vector<unsigned char*>& sampleImages,
        int width, int height);
};