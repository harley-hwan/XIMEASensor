#pragma once
#include <vector>
#include <string>
#include <opencv2/core/types.hpp>
#include <memory>

#define ENABLE_PERFORMANCE_PROFILING

namespace cv {
    class Mat;
}

struct BallInfo {
    cv::Point2f center;
    float radius;
    float confidence;     // Detection confidence (0.0 ~ 1.0)
    int frameIndex;
    float circularity;    // 0.0 ~ 1.0
    float brightness;     // Average brightness (0 ~ 255)

    BallInfo() : radius(0.0f), confidence(0.0f), frameIndex(0),
        circularity(1.0f), brightness(0.0f) {
    }
};

struct BallDetectionResult {
    bool found;
    std::vector<BallInfo> balls;
    std::string errorMessage;

    BallDetectionResult() : found(false) {}
};

class BallDetector {
public:
    struct DetectionParams {
        // Circle detection params
        int minRadius;                      // 최소 반지름 (픽셀)
        int maxRadius;                      // 최대 반지름 (픽셀)
        float minCircularity;               // 최소 원형도 (0.0 ~ 1.0)

        // Hough Circle params (fallback용)
        double dp;                          // 누산기 해상도 역비율
        double minDist;                     // 검출된 중심점 간 최소 거리
        double param1;                      // Canny 엣지 상위 임계값
        double param2;                      // 원 검출 임계값

        // Thresholding params
        int brightnessThreshold;            // 전역 임계값 (0 = Otsu)
        bool useAdaptiveThreshold;          // 적응형 임계값 사용

        // Validation params
        bool useColorFilter;                // 밝기 필터 사용
        bool useCircularityCheck;           // 원형도 재검증
        float contrastThreshold;            // 대비 임계값
        bool detectMultiple;                // 다중 검출 모드

        // Preprocessing params
        bool skipPreprocessing;             // 전처리 스킵
        bool enhanceShadows;                // 그림자 영역 향상
        float shadowEnhanceFactor;          // 그림자 향상 팩터
        bool useMorphology;                 // 형태학적 연산 사용

        // Performance params
        bool fastMode;                      // 고속 모드
        bool useROI;                        // ROI 사용
        float roiScale;                     // ROI 스케일 (0.5 ~ 1.0)
        int downscaleFactor;                // 다운스케일 팩터 (1 ~ 4)
        bool useParallel;                   // TBB 병렬 처리
        int maxCandidates;                  // 최대 후보 개수
        int processingThreads;              // 처리 스레드 수

        // Debug params
        bool saveIntermediateImages;        // 중간 이미지 저장
        std::string debugOutputDir;         // 디버그 출력 디렉토리

        DetectionParams();
    };

    struct PerformanceMetrics {
        double totalDetectionTime_ms;

        // 이미지 전처리 단계
        double roiExtractionTime_ms;
        double downscaleTime_ms;
        double preprocessingTime_ms;

        // 이진화 단계
        double thresholdingTime_ms;
        double morphologyTime_ms;

        // 검출 단계
        double contourDetectionTime_ms;
        double houghDetectionTime_ms;
        double candidateEvaluationTime_ms;

        // 저장 단계
        double imagesSavingTime_ms;

        // 통계
        int candidatesFound;
        int candidatesEvaluated;
        bool ballDetected;

        void Reset();
    };

private:
    DetectionParams m_params;
    class Impl;
    std::unique_ptr<Impl> pImpl;
    mutable PerformanceMetrics m_lastMetrics;
    bool m_performanceProfilingEnabled;

    void InitializeDefaultParams();

public:
    BallDetector();
    ~BallDetector();

    // 복사 및 이동 금지
    BallDetector(const BallDetector&) = delete;
    BallDetector& operator=(const BallDetector&) = delete;
    BallDetector(BallDetector&&) = delete;
    BallDetector& operator=(BallDetector&&) = delete;

    void SetParameters(const DetectionParams& params) { m_params = params; }
    DetectionParams GetParameters() const { return m_params; }
    void ResetToDefaults();

    void EnablePerformanceProfiling(bool enable);
    bool IsPerformanceProfilingEnabled() const { return m_performanceProfilingEnabled; }

    void SetCurrentCaptureFolder(const std::string& folder);

    BallDetectionResult DetectBall(const unsigned char* imageData, int width, int height, int frameIndex = 0);

    bool SaveDetectionImage(const unsigned char* originalImage, int width, int height, const BallDetectionResult& result, const std::string& outputPath, bool saveAsColor = false);

    PerformanceMetrics GetLastPerformanceMetrics() const { return m_lastMetrics; }

    double EstimateProcessingTime(int width, int height) const;
};