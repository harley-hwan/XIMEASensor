#pragma once
#include "../XIMEASensor/IXIMEACallback.h"
#include <functional>

class CameraCallback : public IXIMEACallback {
private:
    std::function<void(const FrameInfo&)> onFrameReceived;
    std::function<void(CameraState, CameraState)> onStateChanged;
    std::function<void(CameraError, const std::string&)> onError;
    std::function<void(const std::string&, const std::string&)> onPropertyChanged;

public:
    void SetFrameCallback(std::function<void(const FrameInfo&)> callback) {
        onFrameReceived = callback;
    }

    void SetStateCallback(std::function<void(CameraState, CameraState)> callback) {
        onStateChanged = callback;
    }

    void SetErrorCallback(std::function<void(CameraError, const std::string&)> callback) {
        onError = callback;
    }

    void SetPropertyCallback(std::function<void(const std::string&, const std::string&)> callback) {
        onPropertyChanged = callback;
    }

    virtual void OnFrameReceived(const FrameInfo& frameInfo) override {
        if (onFrameReceived) {
            onFrameReceived(frameInfo);
        }
    }

    virtual void OnCameraStateChanged(CameraState newState, CameraState oldState) override {
        if (onStateChanged) {
            onStateChanged(newState, oldState);
        }
    }
    
    virtual void OnError(CameraError error, const std::string& errorMessage) override {
        if (onError) {
            onError(error, errorMessage);
        }
    }

    virtual void OnPropertyChanged(const std::string& propertyName, const std::string& value) override {
        if (onPropertyChanged) {
            onPropertyChanged(propertyName, value);
        }
    }
};