#pragma once
#include <vector>
#include <string>
#include <opencv2/core/types.hpp>

namespace cv {
    class Mat;
}

struct BallInfo {
    cv::Point2f center;      // 공 중심 좌표
    float radius;            // 공 반지름
    float confidence;        // 검출 신뢰도 (0.0 ~ 1.0)
    int frameIndex;          // 프레임 인덱스
};

struct BallDetectionResult {
    bool found;              // 공 발견 여부
    std::vector<BallInfo> balls;  // 검출된 공들
    std::string errorMessage;
};

class BallDetector {
public:
    struct DetectionParams {
        // Hough Circle 검출 파라미터
        double dp = 1.2;              // 해상도 비율
        double minDist = 30.0;        // 검출된 원 사이의 최소 거리
        double param1 = 100.0;        // Canny edge detector의 높은 임계값
        double param2 = 25.0;         // 원 검출 임계값
        int minRadius = 40;           // 최소 반지름 (픽셀)
        int maxRadius = 100;           // 최대 반지름 (픽셀)

        // 색상 필터링 파라미터
        int brightnessThreshold = 180;  // 밝기 임계값 (0-255)
        float minCircularity = 0.7f;    // 최소 원형도 (0.0-1.0)

        // 검출 옵션
        bool useColorFilter = true;     // 색상 필터 사용 여부
        bool useCircularityCheck = true;// 원형도 검사 사용 여부
        bool detectMultiple = true;     // 여러 개 검출 허용
    };

private:
    DetectionParams m_params;

    // Private implementation (PIMPL pattern)
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    BallDetector();
    ~BallDetector();

    // 파라미터 설정
    void SetParameters(const DetectionParams& params) { m_params = params; }
    DetectionParams GetParameters() const { return m_params; }

    // 단일 이미지에서 공 검출
    BallDetectionResult DetectBall(const unsigned char* imageData,
        int width, int height,
        int frameIndex = 0);

    // 검출 결과를 이미지에 그리기
    bool DrawDetectionResult(unsigned char* imageData, int width, int height,
        const BallDetectionResult& result,
        cv::Scalar color = cv::Scalar(0, 255, 0),
        int thickness = 2);

    // 검출 결과를 별도 이미지로 저장
    bool SaveDetectionImage(const unsigned char* originalImage, int width, int height,
        const BallDetectionResult& result,
        const std::string& outputPath);

    // 파라미터 자동 조정 (옵션)
    void AutoTuneParameters(const std::vector<unsigned char*>& sampleImages,
        int width, int height);
};