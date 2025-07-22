#pragma once
#include <vector>
#include <string>
#include <opencv2/core/types.hpp>
#include <memory>
#include <thread>
#include <future>

// Performance optimization flags
#define ENABLE_DEBUG_OUTPUT      // Comment out for production builds
#define ENABLE_PERFORMANCE_PROFILING  // Comment out for production builds

namespace cv {
    class Mat;
}

// Structure to hold information about a detected ball
struct BallInfo {
    cv::Point2f center;   // Ball center coordinates (in image pixels)
    float radius;         // Ball radius (in pixels)
    float confidence;     // Detection confidence (0.0 ~ 1.0)
    int frameIndex;       // Frame index of detection

    // Additional diagnostic information
    float circularity;    // Circularity metric of the detected shape
    float brightness;     // Average brightness of the ball region
    float contrast;       // Contrast between ball and background
};

struct BallDetectionResult {
    bool found;                   // Whether a ball was detected
    std::vector<BallInfo> balls;  // Detected ball information (usually 1 ball for ceiling camera)
    std::string errorMessage;     // Error message if detection failed
};

class BallDetector {
public:
    struct DetectionParams {
        // Hough Circle detection parameters
        double dp;             // Inverse ratio of the accumulator resolution to image resolution
        double minDist;        // Minimum distance between detected centers
        double param1;         // Higher threshold for Canny (lower is half of this)
        double param2;         // Circle detection threshold (smaller -> more false circles possible)
        int minRadius;         // Minimum circle radius to detect (pixels)
        int maxRadius;         // Maximum circle radius to detect

        // Color/Brightness filtering and shape validation
        int brightnessThreshold;   // Brightness threshold for definitely bright (white) balls
        float minCircularity;      // Minimum acceptable circularity
        float contrastThreshold;   // Minimum intensity difference from background

        // Detection options
        bool useColorFilter;        // Use brightness/color filtering
        bool useCircularityCheck;   // Use circularity check on candidates
        bool useHoughGradientAlt;   // Use HOUGH_GRADIENT_ALT (more accurate circle detection)
        bool detectMultiple;        // Detect multiple balls vs single ball (false = single-ball mode)
        bool useMorphology;         // Use morphological operations to clean binary images
        bool useAdaptiveThreshold;  // Use adaptive threshold + contour detection method

        // Ceiling camera specific options
        bool correctPerspective;    // Apply lens distortion correction if calibration provided
        bool enhanceShadows;        // Enhance dark (shadow) regions to help detection
        float shadowEnhanceFactor;  // Brightening factor for shadow regions

        // Debug options
        bool saveIntermediateImages;        // Save intermediate processing images for debugging
        std::string debugOutputDir;         // Directory to save debug images

        // Performance optimization parameters
        int numThreads;             // Number of threads for parallel processing (0 = auto)
        bool useParallelDetection;  // Enable parallel detection methods
        int maxCandidates;          // Maximum candidates to evaluate (early termination)

        // Constructor: initialize with default values
        DetectionParams();
    };

    struct PerformanceMetrics {
        double totalDetectionTime_ms;
        double preprocessingTime_ms;
        double houghDetectionTime_ms;
        double adaptiveThresholdTime_ms;
        double contourDetectionTime_ms;
        double candidateEvaluationTime_ms;
        double imagesSavingTime_ms;
        int candidatesFound;
        int candidatesEvaluated;
        bool ballDetected;

        void Reset() {
            totalDetectionTime_ms = 0;
            preprocessingTime_ms = 0;
            houghDetectionTime_ms = 0;
            adaptiveThresholdTime_ms = 0;
            contourDetectionTime_ms = 0;
            candidateEvaluationTime_ms = 0;
            imagesSavingTime_ms = 0;
            candidatesFound = 0;
            candidatesEvaluated = 0;
            ballDetected = false;
        }
    };

private:
    DetectionParams m_params;

    // PIMPL: internal implementation details
    class Impl;
    std::unique_ptr<Impl> pImpl;

    mutable PerformanceMetrics m_lastMetrics;

    // Performance profiling control
    bool m_performanceProfilingEnabled;

    // Thread pool for parallel processing
    std::unique_ptr<std::thread[]> m_threadPool;
    int m_numThreads;

    // Initialize default parameters optimized for ceiling camera
    void InitializeDefaultParams();

    // Initialize thread pool
    void InitializeThreadPool();

public:
    BallDetector();
    ~BallDetector();

    // Disable copy constructor and assignment operator
    BallDetector(const BallDetector&) = delete;
    BallDetector& operator=(const BallDetector&) = delete;

    // Set or get current detection parameters
    void SetParameters(const DetectionParams& params);
    DetectionParams GetParameters() const { return m_params; }

    // Reset parameters to their default values
    void ResetToDefaults();

    // Set camera calibration data (for perspective correction)
    void SetCalibrationData(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

    // Enable/disable performance profiling
    void EnablePerformanceProfiling(bool enable);
    bool IsPerformanceProfilingEnabled() const;

    // Main function: detect ball from a grayscale image frame
    BallDetectionResult DetectBall(const unsigned char* imageData, int width, int height, int frameIndex = 0);

    // Utility: draw detection result on a grayscale image buffer (in-place)
    bool DrawDetectionResult(unsigned char* imageData, int width, int height,
        const BallDetectionResult& result,
        cv::Scalar color = cv::Scalar(0, 255, 0),
        int thickness = 2);

    // Utility: save detection result as a color image with annotations (for debugging)
    bool SaveDetectionImage(const unsigned char* originalImage, int width, int height,
        const BallDetectionResult& result,
        const std::string& outputPath);

    // Auto-tune parameters based on a set of sample images (experimental)
    void AutoTuneParameters(const std::vector<unsigned char*>& sampleImages, int width, int height);

    // Get performance metrics from the last detection
    PerformanceMetrics GetLastPerformanceMetrics() const { return m_lastMetrics; }

    // Generate a detailed performance report
    std::string GeneratePerformanceReport() const;
};