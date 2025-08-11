#include "pch.h"
#include "XIMEACameraImpl.h"
#include <stdexcept>

namespace Camera {

    std::unique_ptr<ICameraInterface> CameraFactory::CreateCamera(CameraType type) {
        switch (type) {
        case CameraType::XIMEA:
            return std::make_unique<XIMEACameraImpl>();

        case CameraType::HIKVISION:
            // return std::make_unique<HIKVISIONCameraImpl>();
            throw std::runtime_error("HIKVISION camera not implemented yet");

        default:
            throw std::runtime_error("Unknown camera type");
        }
    }

} // namespace Camera