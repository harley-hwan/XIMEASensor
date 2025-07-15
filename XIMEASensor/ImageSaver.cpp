#include "pch.h"
#include "ImageSaver.h"
#include "Logger.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <algorithm>
#include <cctype>

bool ImageSaver::SaveGrayscaleImage(const unsigned char* data, int width, int height,
    const std::string& filename, ImageFormat format, int jpgQuality) {
    if (!data || width <= 0 || height <= 0 || filename.empty()) {
        LOG_ERROR("Invalid parameters for image saving");
        return false;
    }

    int result = 0;

    try {
        switch (format) {
        case ImageFormat::PNG:
            result = stbi_write_png(filename.c_str(), width, height, 1, data, width);
            break;

        case ImageFormat::JPG:
            result = stbi_write_jpg(filename.c_str(), width, height, 1, data, jpgQuality);
            break;

        case ImageFormat::BMP:
            result = stbi_write_bmp(filename.c_str(), width, height, 1, data);
            break;

        case ImageFormat::TGA:
            result = stbi_write_tga(filename.c_str(), width, height, 1, data);
            break;

        default:
            LOG_ERROR("Unknown image format");
            return false;
        }

        if (result) {
            LOG_INFO("Image saved successfully: " + filename);
            return true;
        }
        else {
            LOG_ERROR("Failed to save image: " + filename);
            return false;
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception while saving image: " + std::string(e.what()));
        return false;
    }
}

bool ImageSaver::EncodeToMemory(const unsigned char* data, int width, int height,
    unsigned char** output, int* outputSize,
    ImageFormat format, int jpgQuality) {
    if (!data || width <= 0 || height <= 0 || !output || !outputSize) {
        LOG_ERROR("Invalid parameters for image encoding");
        return false;
    }

	// PNG encoding callback func
    auto write_func = [](void* context, void* data, int size) {
        std::vector<unsigned char>* buffer = (std::vector<unsigned char>*)context;
        unsigned char* bytes = (unsigned char*)data;
        buffer->insert(buffer->end(), bytes, bytes + size);
        };

    std::vector<unsigned char> buffer;
    int result = 0;

    try {
        switch (format) {
        case ImageFormat::PNG:
            result = stbi_write_png_to_func(write_func, &buffer, width, height, 1, data, width);
            break;

        case ImageFormat::JPG:
            result = stbi_write_jpg_to_func(write_func, &buffer, width, height, 1, data, jpgQuality);
            break;

        default:
            LOG_ERROR("Memory encoding only supports PNG and JPG formats");
            return false;
        }

        if (result && !buffer.empty()) {
            *outputSize = (int)buffer.size();
            *output = new unsigned char[*outputSize];
            memcpy(*output, buffer.data(), *outputSize);
            return true;
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception while encoding image: " + std::string(e.what()));
    }

    return false;
}

bool ImageSaver::SaveImageAuto(const unsigned char* data, int width, int height,
    const std::string& filename, int jpgQuality) {
    ImageFormat format = GetFormatFromExtension(filename);
    return SaveGrayscaleImage(data, width, height, filename, format, jpgQuality);
}

ImageFormat ImageSaver::GetFormatFromExtension(const std::string& filename) {
    size_t dotPos = filename.find_last_of(".");
    if (dotPos == std::string::npos) {
        return ImageFormat::PNG;
    }

    std::string ext = filename.substr(dotPos + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if (ext == "png") return ImageFormat::PNG;
    if (ext == "jpg" || ext == "jpeg") return ImageFormat::JPG;
    if (ext == "bmp") return ImageFormat::BMP;
    if (ext == "tga") return ImageFormat::TGA;

    return ImageFormat::PNG;
}