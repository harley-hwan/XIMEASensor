#pragma once
#include "XIMEASensorCommon.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <memory>
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

namespace XimeaSensor {

    // 네트워크 프로토콜 정의
    enum class CommandType : uint16_t {
        // 카메라 제어
        OpenCamera = 0x0100,
        CloseCamera = 0x0101,
        StartCapture = 0x0102,
        StopCapture = 0x0103,

        // 설정
        SetExposure = 0x0200,
        SetGain = 0x0201,
        SetROI = 0x0202,
        SetSettings = 0x0203,

        // 조회
        GetCameraInfo = 0x0300,
        GetSettings = 0x0301,
        GetStats = 0x0302,

        // 프레임 스트리밍
        StartStreaming = 0x0400,
        StopStreaming = 0x0401,

        // 시스템
        Ping = 0x0500,
        Shutdown = 0x0501
    };

    // 프로토콜 헤더
#pragma pack(push, 1)
    struct ProtocolHeader {
        uint32_t magic;      // 0x58494D45 ('XIME')
        uint16_t version;    // 프로토콜 버전
        uint16_t command;    // CommandType
        uint32_t payloadSize;
        uint32_t sequenceId;
    };

    struct ResponseHeader {
        uint32_t magic;
        uint16_t version;
        uint16_t command;
        int32_t errorCode;   // ErrorCode
        uint32_t payloadSize;
        uint32_t sequenceId;
    };
#pragma pack(pop)

    class NetworkServer : public IFrameCallback {
    private:
        struct ClientSession {
            SOCKET socket;
            std::string address;
            uint16_t port;
            std::atomic<bool> isStreaming;
            std::atomic<bool> connected;
            std::unique_ptr<std::thread> thread;
            std::mutex sendMutex;
            uint32_t sequenceId;
        };

        // 서버 상태
        std::atomic<bool> m_running;
        uint16_t m_port;
        SOCKET m_listenSocket;
        std::unique_ptr<std::thread> m_acceptThread;

        // 클라이언트 관리
        std::vector<std::shared_ptr<ClientSession>> m_clients;
        mutable std::mutex m_clientsMutex;

        // 프레임 압축 옵션
        bool m_compressFrames;
        int m_jpegQuality;

        // 내부 메서드
        void AcceptThreadFunc();
        void ClientThreadFunc(std::shared_ptr<ClientSession> client);
        bool ProcessCommand(std::shared_ptr<ClientSession> client, const ProtocolHeader& header, const std::vector<uint8_t>& payload);
        bool SendResponse(std::shared_ptr<ClientSession> client, CommandType command, ErrorCode error,
            const void* data, size_t dataSize, uint32_t sequenceId);
        bool SendFrame(std::shared_ptr<ClientSession> client, const uint8_t* frameData, const FrameInfo& info);
        void DisconnectClient(std::shared_ptr<ClientSession> client);

    public:
        NetworkServer();
        ~NetworkServer();

        // 서버 제어
        ErrorCode Start(uint16_t port = 5000);
        ErrorCode Stop();
        bool IsRunning() const { return m_running; }

        // 옵션 설정
        void SetFrameCompression(bool enable, int jpegQuality = 85) {
            m_compressFrames = enable;
            m_jpegQuality = jpegQuality;
        }

        // 클라이언트 정보
        size_t GetClientCount() const;
        std::vector<std::string> GetConnectedClients() const;

        // IFrameCallback 구현
        void OnFrameReady(const uint8_t* frameData, const FrameInfo& info) override;
        void OnError(ErrorCode error, const char* message) override;
        void OnCameraDisconnected() override;
    };

} // namespace XimeaSensor