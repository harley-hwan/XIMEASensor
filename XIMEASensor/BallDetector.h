#pragma once
#include <vector>
#include <string>
#include <opencv2/core/types.hpp>

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
        double dp = 1.5;             // Inverse ratio of the accumulator resolution to image resolution
        double minDist = 30.0;       // Minimum distance between detected centers
        double param1 = 100.0;        // Higher threshold for Canny (lower is half of this)
        double param2 = 0.9;        // Circle detection threshold (smaller -> more false circles possible)
        int minRadius = 10;          // Minimum circle radius to detect (pixels)
        int maxRadius = 80;         // Maximum circle radius to detect

        // Color/Brightness filtering and shape validation
        int brightnessThreshold = 130;   // Brightness threshold for definitely bright (white) balls
        float minCircularity = 0.7f;     // Minimum acceptable circularity
        float contrastThreshold = 10.0f; // Minimum intensity difference from background

        // Detection options
        bool useColorFilter = true;        // Use brightness/color filtering
        bool useCircularityCheck = true;   // Use circularity check on candidates
        bool useHoughGradientAlt = true;   // Use HOUGH_GRADIENT_ALT (more accurate circle detection)
        bool detectMultiple = false;       // Detect only a single ball (most likely scenario for ceiling cam)
        bool useMorphology = true;         // Use morphological operations to clean binary images
        bool useAdaptiveThreshold = true;  // Use adaptive threshold + contour detection method

        // Ceiling camera specific options
        bool correctPerspective = false;   // Apply lens distortion correction if calibration provided
        bool enhanceShadows = true;        // Enhance dark (shadow) regions to help detection
        float shadowEnhanceFactor = 0.7f;  // Brightening factor for shadow regions

        // Debug options
        bool saveIntermediateImages = true;        // Save intermediate processing images for debugging
        std::string debugOutputDir = "detect_outputs"; // Directory to save debug images
    };

private:
    DetectionParams m_params;

    // PIMPL: internal implementation details
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Initialize default parameters optimized for ceiling camera
    void InitializeDefaultParams();

public:
    BallDetector();
    ~BallDetector();

    // Set or get current detection parameters
    void SetParameters(const DetectionParams& params) { m_params = params; }
    DetectionParams GetParameters() const { return m_params; }

    // Reset parameters to their default values
    void ResetToDefaults();

    // Set camera calibration data (for perspective correction)
    void SetCalibrationData(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

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
};
