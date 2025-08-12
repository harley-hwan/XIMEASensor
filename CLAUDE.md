# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

XIMEASensor is a Windows C++ project that provides a DLL for interfacing with XIMEA cameras and performing real-time ball detection. The project consists of two main components:

- **XIMEASensor**: Core DLL library for camera control and ball detection
- **XIMEASensorDiag**: MFC-based diagnostic application for testing and configuration

## Build System

This project uses Visual Studio MSBuild with two solution files:

### Building the Project
```bash
# Build the main library (from project root)
msbuild XIMEASensor.sln /p:Configuration=Debug /p:Platform=x64
msbuild XIMEASensor.sln /p:Configuration=Release /p:Platform=x64

# Build diagnostic application
msbuild XIMEASensorDiag.sln /p:Configuration=Debug /p:Platform=x64
msbuild XIMEASensorDiag.sln /p:Configuration=Release /p:Platform=x64
```

### Output Locations
- Debug builds: `x64/Debug/`
- Release builds: `x64/Release/`
- Library automatically copies to diagnostic project: `XIMEASensorDiag/`

## Architecture

### Core Components

1. **CameraController** (`CameraController.h/.cpp`)
   - Singleton class managing camera lifecycle
   - Handles frame capture and threading
   - Manages callbacks and statistics
   - Provides real-time detection integration

2. **BallDetector** (`BallDetector.h/.cpp`)
   - Computer vision module for golf ball detection
   - Uses OpenCV for image processing
   - Supports multiple detection algorithms (Hough circles, contour detection, template matching)
   - Thread-safe with performance profiling

3. **ContinuousCaptureManager** (`ContinuousCaptureManager.h/.cpp`)
   - Manages batch capture sessions
   - Async image saving with buffer pooling
   - Performance metrics and reporting
   - Optional ball detection during capture

4. **XIMEACameraImpl** (`XIMEACameraImpl.h/.cpp`)
   - XIMEA camera hardware interface
   - Wraps XIMEA API calls
   - Handles camera-specific operations

### Key Features

- **Real-time Ball Detection**: Processing frames as they arrive from camera
- **Ball State Tracking**: State machine tracking ball movement (NOT_DETECTED → MOVING → STABILIZING → READY → STOPPED)
- **Dynamic ROI**: Automatic region-of-interest adjustment for performance optimization
- **Continuous Capture**: Batch frame capture with configurable duration and formats
- **Performance Profiling**: Detailed timing metrics for all operations

### Dependencies

- **XIMEA API**: Camera hardware interface (`xiapi64.dll`)
- **OpenCV 4.12**: Computer vision processing (`opencv_world4120.dll`)
- **Intel TBB**: Threading Building Blocks (`tbb12.dll`)

## API Structure

The main API is exposed through C functions in `XIMEASensor.h`:

- **System**: `Camera_Initialize()`, `Camera_Shutdown()`
- **Device Management**: `Camera_GetDeviceCount()`, `Camera_Open()`
- **Capture Control**: `Camera_Start()`, `Camera_Stop()`, `Camera_GetFrame()`
- **Parameters**: `Camera_SetExposure()`, `Camera_SetGain()`, `Camera_SetROI()`
- **Ball Detection**: `Camera_EnableRealtimeDetection()`, `Camera_GetLastDetectionResult()`
- **State Tracking**: `Camera_EnableBallStateTracking()`, `Camera_GetBallState()`

## Configuration

### Camera Defaults
- Exposure: Defined in `CameraDefaults::EXPOSURE_US`
- Gain: Defined in `CameraDefaults::GAIN_DB`
- Frame Rate: Defined in `CameraDefaults::FRAMERATE_FPS`
- Max FPS for PYTHON1300: 210 FPS

### Ball Detection Parameters
- Configurable through `BallDetector::DetectionParams`
- Includes circle detection, preprocessing, validation, and performance settings
- Debug image output configurable via `saveIntermediateImages`

## Conditional Compilation

- `ENABLE_CONTINUOUS_CAPTURE`: Controls availability of batch capture functionality
- `ENABLE_PERFORMANCE_PROFILING`: Enables detailed timing measurements

## Directory Structure

- `XIMEASensor/`: Core library source files
- `XIMEASensorDiag/`: Diagnostic application
- `opencv/`: OpenCV library distribution
- `ref/flow chart/`: Architecture diagrams and flowcharts
- Capture results stored in timestamped folders under `XIMEASensorDiag/Capture_*`

## Memory Management

- Double-buffered frame acquisition for thread safety
- Buffer pooling in continuous capture to reduce allocations
- RAII patterns throughout for resource management
- Thread-local storage for detection contexts

## Threading Model

- Main capture thread in `CameraController::CaptureLoop()`
- Separate detection worker thread for real-time processing
- Async save thread for continuous capture
- All threads coordinate via atomic variables and mutexes