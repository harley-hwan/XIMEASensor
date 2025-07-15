#pragma once
#include <string>

enum class ImageFormat {
    PNG = 0,
    JPG = 1,
    BMP = 2,
    TGA = 3
};

class ImageSaver {
public:
    static bool SaveGrayscaleImage(const unsigned char* data, int width, int height,
        const std::string& filename, ImageFormat format = ImageFormat::PNG,
        int jpgQuality = 90);

    static bool EncodeToMemory(const unsigned char* data, int width, int height,
        unsigned char** output, int* outputSize,
        ImageFormat format = ImageFormat::PNG, int jpgQuality = 90);

    static bool SaveImageAuto(const unsigned char* data, int width, int height,
        const std::string& filename, int jpgQuality = 90);

private:
    static ImageFormat GetFormatFromExtension(const std::string& filename);
};