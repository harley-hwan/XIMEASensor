#pragma once
#include <cstdint>
#include <xiApi.h>
// Camera-independent type definitions
namespace Camera {

    // Image format enum (replaces XI_IMG_FORMAT)
    enum class ImageFormat {
        MONO8 = 0,
        MONO16 = 1,
        RGB24 = 2,
        RGB32 = 3,
        RAW8 = 4,
        RAW16 = 5,
        FRM_TRANSPORT_DATA = 6
    };

    // Return codes (replaces XI_RETURN)
    enum class ReturnCode {
        OK = 0,
        INVALID_HANDLE = 1,
        READREG = 2,
        WRITEREG = 3,
        FREE_RESOURCES = 4,
        FREE_CHANNEL = 5,
        FREE_BANDWIDTH = 6,
        READBLK = 7,
        WRITEBLK = 8,
        NO_IMAGE = 9,
        TIMEOUT = 10,
        INVALID_ARG = 11,
        NOT_SUPPORTED = 12,
        ISOCH_ATTACH_BUFFERS = 13,
        GET_OVERLAPPED_RESULT = 14,
        MEMORY_ALLOCATION = 15,
        DLLCONTEXTISNULL = 16,
        DLLCONTEXTISNONZERO = 17,
        DLLCONTEXTEXIST = 18,
        TOOMANYDEVICES = 19,
        ERRORCAMCONTEXT = 20,
        UNKNOWN_HARDWARE = 21,
        INVALID_TM_FILE = 22,
        INVALID_TM_TAG = 23,
        INCOMPLETE_TM = 24,
        BUS_RESET_FAILED = 25,
        NOT_IMPLEMENTED = 26,
        SHADING_TOOBRIGHT = 27,
        SHADING_TOODARK = 28,
        TOO_LOW_GAIN = 29,
        INVALID_BPL = 30,
        BPL_REALLOC = 31,
        INVALID_PIXEL_LIST = 32,
        INVALID_FFS = 33,
        INVALID_PROFILE = 34,
        INVALID_CALIBRATION = 35,
        INVALID_BUFFER = 36,
        INVALID_DATA = 37,
        TGBUSY = 38,
        IO_WRONG = 39,
        ACQUISITION_ALREADY_UP = 40,
        OLD_DRIVER_VERSION = 41,
        GET_LAST_ERROR = 42,
        CANT_PROCESS = 43,
        ACQUISITION_STOPED = 44,
        ACQUISITION_STOPED_WERR = 45,
        INVALID_INPUT_ICC_PROFILE = 46,
        INVALID_OUTPUT_ICC_PROFILE = 47,
        DEVICE_NOT_READY = 48,
        SHADING_TOOCONTRAST = 49,
        ALREADY_INITIALIZED = 50,
        NOT_ENOUGH_PRIVILEGES = 51,
        NOT_COMPATIBLE_DRIVER = 52,
        TM_INVALID_RESOURCE = 53,
        DEVICE_HAS_BEEN_RESETED = 54,
        NO_DEVICES_FOUND = 55,
        RESOURCE_OR_FUNCTION_LOCKED = 56,
        BUFFER_SIZE_TOO_SMALL = 57,
        COULDNT_INIT_PROCESSOR = 58,
        NOT_INITIALIZED = 59,
        RESOURCE_NOT_FOUND = 60,
        UNKNOWN_PARAM = 61,
        WRONG_PARAM_VALUE = 62,
        WRONG_PARAM_TYPE = 63,
        WRONG_PARAM_SIZE = 64,
        BUFFER_TOO_SMALL = 65,
        NOT_SUPPORTED_PARAM = 66,
        NOT_SUPPORTED_PARAM_INFO = 67,
        NOT_SUPPORTED_DATA_FORMAT = 68,
        READ_ONLY_PARAM = 69,
        BANDWIDTH_NOT_SUPPORTED = 70,
        INVALID_FFS_FILE_NAME = 71,
        FFS_FILE_NOT_FOUND = 72,
        PARAM_NOT_SETTABLE = 73,
        SAFE_POLICY_NOT_SUPPORTED = 74,
        GPUDIRECT_NOT_AVAILABLE = 75,
        INCORRECT_SENS_ID_CHECK = 76,
        INCORRECT_FPGA_TYPE = 77,
        PARAM_CONDITIONALLY_NOT_AVAILABLE = 78,
        ERR_FRAME_BUFFER_RAM_INIT = 79,
        PROC_OTHER_ERROR = 100,
        PROC_PROCESSING_ERROR = 101,
        PROC_INPUT_FORMAT_UNSUPPORTED = 102,
        PROC_OUTPUT_FORMAT_UNSUPPORTED = 103,
        OUT_OF_RANGE = 104
    };

    // Image data structure (replaces XI_IMG)
    struct ImageData {
        uint32_t size;                      // Size of this structure
        void* bp;                           // Pointer to data buffer
        uint32_t bp_size;                   // Data buffer size
        ImageFormat frm;                    // Image data format
        uint32_t width;                     // Image width
        uint32_t height;                    // Image height
        uint32_t nframe;                    // Frame number
        uint32_t tsSec;                     // Timestamp (seconds)
        uint32_t tsUSec;                    // Timestamp (microseconds)
        uint32_t GPI_level;                 // GPI level
        uint32_t black_level;               // Black level
        uint32_t padding_x;                 // Horizontal padding
        uint32_t AbsoluteOffsetX;           // Absolute offset X
        uint32_t AbsoluteOffsetY;           // Absolute offset Y
        uint32_t transport_frm;             // Transport format
        XI_IMG_DESC img_desc;                     // Image descriptor
        uint32_t DownsamplingX;             // Downsampling X
        uint32_t DownsamplingY;             // Downsampling Y
        uint32_t flags;                     // Flags
        uint32_t exposure_time_us;          // Exposure time (microseconds)
        float gain_db;                      // Gain (dB)
        uint32_t acq_nframe;                // Acquisition frame number
        uint32_t image_user_data;           // User data
        uint32_t exposure_sub_times_us[5];  // Sub exposure times

        // Initialize structure
        ImageData() {
            memset(this, 0, sizeof(ImageData));
            size = sizeof(ImageData);
        }
    };

    // Parameter types
    enum class ParamType {
        // Device parameters
        DEVICE_NAME,
        DEVICE_SN,

        // Image format parameters
        IMAGE_DATA_FORMAT,
        OUTPUT_DATA_BIT_DEPTH,
        SENSOR_DATA_BIT_DEPTH,
        WIDTH,
        HEIGHT,
        OFFSET_X,
        OFFSET_Y,

        // Acquisition parameters
        EXPOSURE,
        GAIN,
        FRAMERATE,
        ACQ_TIMING_MODE,
        BUFFER_POLICY,
        AUTO_BANDWIDTH_CALCULATION,
        TRG_SOURCE,
        DOWNSAMPLING,
        DOWNSAMPLING_TYPE,

        // Advanced parameters
        SENSOR_TAPS,
        BUFFERS_QUEUE_SIZE,
        RECENT_FRAME,
        GAMMAY,
        SHARPNESS,
        HDR,
        AUTO_WB,
        MANUAL_WB,
        SENS_DEFECTS_CORR,
        COLOR_FILTER_ARRAY,
        TRANSPORT_PIXEL_FORMAT
    };

    // Parameter values
    enum class AcqTimingMode {
        FREE_RUN = 0,
        FRAME_RATE = 1
    };

    enum class BufferPolicy {
        UNSAFE = 0,
        SAFE = 1
    };

    enum class TriggerSource {
        OFF = 0,
        EDGE_RISING = 1,
        EDGE_FALLING = 2,
        SOFTWARE = 3,
        LEVEL_HIGH = 4,
        LEVEL_LOW = 5
    };

    enum class Downsampling {
        DWN_1x1 = 1,
        DWN_2x2 = 2,
        DWN_3x3 = 3,
        DWN_4x4 = 4,
        DWN_5x5 = 5,
        DWN_6x6 = 6,
        DWN_7x7 = 7,
        DWN_8x8 = 8,
        DWN_9x9 = 9,
        DWN_10x10 = 10,
        DWN_16x16 = 16
    };

    enum class DownsamplingType {
        BINNING = 0,
        SKIPPING = 1
    };

    enum class SensorTaps {
        TAP_CNT_1 = 1,
        TAP_CNT_2 = 2,
        TAP_CNT_4 = 4
    };

    enum class ColorFilterArray {
        CFA_NONE = 0,
        CFA_BAYER_RGGB = 1,
        CFA_BAYER_GRBG = 2,
        CFA_BAYER_GBRG = 3,
        CFA_BAYER_BGGR = 4
    };

    enum class GenTLImageFormat {
        Mono8 = 0x01080001,
        Mono10 = 0x01100003,
        Mono12 = 0x01100005,
        Mono14 = 0x01100025,
        Mono16 = 0x01100007
    };

    enum class OnOff {
        OFF = 0,
        ON = 1
    };

} // namespace Camera