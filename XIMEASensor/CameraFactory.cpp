#include "pch.h"
#include "ICameraInterface.h"
#include "XIMEACameraImpl.h"
#include <stdexcept>

namespace Camera {

    std::unique_ptr<ICameraInterface> CameraFactory::CreateCamera(CameraType type) {
        switch (type) {
        case CameraType::XIMEA:
            return std::make_unique<XIMEACameraImpl>();

        case CameraType::MOCK:
            // Return mock implementation for testing
            // return std::make_unique<MockCameraImpl>();
            throw std::runtime_error("Mock camera not implemented yet");

        default:
            throw std::runtime_error("Unknown camera type");
        }
    }

} // namespace Camera