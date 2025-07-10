#include "pch.h"
#include "NetworkServer.h"
#include "CameraController.h"
#include "Logger.h"
#include <algorithm>
#include <sstream>

namespace XimeaSensor {

    constexpr uint32_t PROTOCOL_MAGIC = 0x58494D45; // 'XIME'
    constexpr uint16_t PROTOCOL_VERSION = 1;
    constexpr size_t MAX_PAYLOAD_SIZE = 10 * 1024 * 1024; // 10MB

    NetworkServer::NetworkServer()
        : m_running(false)
        , m_port(0)
        , m_listenSocket(INVALID_SOCKET)
        , m_compressFrames(false)
        , m_jpegQuality(85) {

        // Winsock 초기화
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            LOG_CRITICAL("WSAStartup failed");
        }
    }

    NetworkServer::~NetworkServer() {
        Stop();
        WSACleanup();
    }

    ErrorCode NetworkServer::Start(uint16_t port) {
        if (m_running) {
            LOG_WARNING("Network server already running");
            return ErrorCode::Success;
        }

        m_port = port;
        LOG_INFO("Starting network server on port " + std::to_string(port));

        // 소켓 생성
        m_listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (m_listenSocket == INVALID_SOCKET) {
            LOG_ERROR("Failed to create socket: " + std::to_string(WSAGetLastError()));
            return ErrorCode::NetworkError;
        }

        // SO_REUSEADDR 설정
        int opt = 1;
        setsockopt(m_listenSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));

        // 바인드
        sockaddr_in serverAddr{};
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = INADDR_ANY;
        serverAddr.sin_port = htons(port);

        if (bind(m_listenSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
            LOG_ERROR("Bind failed: " + std::to_string(WSAGetLastError()));
            closesocket(m_listenSocket);
            m_listenSocket = INVALID_SOCKET;
            return ErrorCode::NetworkError;
        }

        // 리슨
        if (listen(m_listenSocket, SOMAXCONN) == SOCKET_ERROR) {
            LOG_ERROR("Listen failed: " + std::to_string(WSAGetLastError()));
            closesocket(m_listenSocket);
            m_listenSocket = INVALID_SOCKET;
            return ErrorCode::NetworkError;
        }

        m_running = true;

        // Accept 스레드 시작
        m_acceptThread = std::make_unique<std::thread>(&NetworkServer::AcceptThreadFunc, this);

        // 카메라 컨트롤러에 콜백 등록
        CameraController::GetInstance().SetFrameCallback(this);

        LOG_INFO("Network server started successfully");
        return ErrorCode::Success;
    }

    ErrorCode NetworkServer::Stop() {
        if (!m_running) {
            return ErrorCode::Success;
        }

        LOG_INFO("Stopping network server");

        m_running = false;

        // 리슨 소켓 닫기
        if (m_listenSocket != INVALID_SOCKET) {
            closesocket(m_listenSocket);
            m_listenSocket = INVALID_SOCKET;
        }

        // Accept 스레드 종료 대기
        if (m_acceptThread && m_acceptThread->joinable()) {
            m_acceptThread->join();
            m_acceptThread.reset();
        }

        // 모든 클라이언트 연결 종료
        {
            std::lock_guard<std::mutex> lock(m_clientsMutex);
            for (auto& client : m_clients) {
                DisconnectClient(client);
            }
            m_clients.clear();
        }

        // 카메라 콜백 해제
        CameraController::GetInstance().SetFrameCallback(nullptr);

        LOG_INFO("Network server stopped");
        return ErrorCode::Success;
    }

    void NetworkServer::AcceptThreadFunc() {
        LOG_INFO("Accept thread started");

        while (m_running) {
            sockaddr_in clientAddr{};
            int addrLen = sizeof(clientAddr);

            SOCKET clientSocket = accept(m_listenSocket, (sockaddr*)&clientAddr, &addrLen);
            if (clientSocket == INVALID_SOCKET) {
                if (m_running) {
                    LOG_ERROR("Accept failed: " + std::to_string(WSAGetLastError()));
                }
                continue;
            }

            // 클라이언트 정보
            char clientIP[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &clientAddr.sin_addr, clientIP, INET_ADDRSTRLEN);
            uint16_t clientPort = ntohs(clientAddr.sin_port);

            LOG_INFO("Client connected from " + std::string(clientIP) + ":" + std::to_string(clientPort));

            // TCP_NODELAY 설정 (낮은 지연시간)
            int nodelay = 1;
            setsockopt(clientSocket, IPPROTO_TCP, TCP_NODELAY, (char*)&nodelay, sizeof(nodelay));

            // 소켓 버퍼 크기 설정
            int bufSize = 1024 * 1024; // 1MB
            setsockopt(clientSocket, SOL_SOCKET, SO_SNDBUF, (char*)&bufSize, sizeof(bufSize));
            setsockopt(clientSocket, SOL_SOCKET, SO_RCVBUF, (char*)&bufSize, sizeof(bufSize));

            // 클라이언트 세션 생성
            auto client = std::make_shared<ClientSession>();
            client->socket = clientSocket;
            client->address = clientIP;
            client->port = clientPort;
            client->isStreaming = false;
            client->connected = true;
            client->sequenceId = 0;

            // 클라이언트 스레드 시작
            client->thread = std::make_unique<std::thread>(&NetworkServer::ClientThreadFunc, this, client);

            // 클라이언트 목록에 추가
            {
                std::lock_guard<std::mutex> lock(m_clientsMutex);
                m_clients.push_back(client);
            }
        }

        LOG_INFO("Accept thread exiting");
    }

    void NetworkServer::ClientThreadFunc(std::shared_ptr<ClientSession> client) {
        LOG_INFO("Client thread started for " + client->address + ":" + std::to_string(client->port));

        std::vector<uint8_t> recvBuffer(4096);
        std::vector<uint8_t> payloadBuffer;

        while (client->connected && m_running) {
            // 헤더 수신
            ProtocolHeader header;
            int totalReceived = 0;

            while (totalReceived < sizeof(header)) {
                int received = recv(client->socket,
                    reinterpret_cast<char*>(&header) + totalReceived,
                    sizeof(header) - totalReceived, 0);

                if (received <= 0) {
                    if (received < 0) {
                        LOG_ERROR("Recv failed: " + std::to_string(WSAGetLastError()));
                    }
                    client->connected = false;
                    break;
                }

                totalReceived += received;
            }

            if (!client->connected) break;

            // 헤더 검증
            if (header.magic != PROTOCOL_MAGIC) {
                LOG_ERROR("Invalid protocol magic from client");
                client->connected = false;
                break;
            }

            if (header.version != PROTOCOL_VERSION) {
                LOG_ERROR("Unsupported protocol version: " + std::to_string(header.version));
                SendResponse(client, static_cast<CommandType>(header.command),
                    ErrorCode::InvalidParameter, nullptr, 0, header.sequenceId);
                continue;
            }

            // 페이로드 수신
            if (header.payloadSize > 0) {
                if (header.payloadSize > MAX_PAYLOAD_SIZE) {
                    LOG_ERROR("Payload too large: " + std::to_string(header.payloadSize));
                    SendResponse(client, static_cast<CommandType>(header.command),
                        ErrorCode::InvalidParameter, nullptr, 0, header.sequenceId);
                    continue;
                }

                payloadBuffer.resize(header.payloadSize);
                totalReceived = 0;

                while (totalReceived < header.payloadSize) {
                    int toReceive = std::min(static_cast<int>(recvBuffer.size()),
                        static_cast<int>(header.payloadSize - totalReceived));

                    int received = recv(client->socket,
                        reinterpret_cast<char*>(payloadBuffer.data()) + totalReceived,
                        toReceive, 0);

                    if (received <= 0) {
                        client->connected = false;
                        break;
                    }

                    totalReceived += received;
                }

                if (!client->connected) break;
            }

            // 명령 처리
            ProcessCommand(client, header, payloadBuffer);
        }

        // 정리
        DisconnectClient(client);

        // 클라이언트 목록에서 제거
        {
            std::lock_guard<std::mutex> lock(m_clientsMutex);
            m_clients.erase(std::remove(m_clients.begin(), m_clients.end(), client), m_clients.end());
        }

        LOG_INFO("Client thread exiting for " + client->address + ":" + std::to_string(client->port));
    }

    bool NetworkServer::ProcessCommand(std::shared_ptr<ClientSession> client,
        const ProtocolHeader& header,
        const std::vector<uint8_t>& payload) {

        CommandType cmd = static_cast<CommandType>(header.command);
        ErrorCode result = ErrorCode::Success;

        LOG_DEBUG("Processing command: 0x" + std::to_string(header.command));

        switch (cmd) {
        case CommandType::Ping: {
            // Pong 응답
            uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            SendResponse(client, cmd, ErrorCode::Success, &timestamp, sizeof(timestamp), header.sequenceId);
            break;
        }

        case CommandType::OpenCamera: {
            int deviceIndex = 0;
            if (header.payloadSize >= sizeof(int)) {
                deviceIndex = *reinterpret_cast<const int*>(payload.data());
            }
            result = CameraController::GetInstance().OpenCamera(deviceIndex);
            SendResponse(client, cmd, result, nullptr, 0, header.sequenceId);
            break;
        }

        case CommandType::CloseCamera: {
            result = CameraController::GetInstance().CloseCamera();
            SendResponse(client, cmd, result, nullptr, 0, header.sequenceId);
            break;
        }

        case CommandType::StartCapture: {
            result = CameraController::GetInstance().StartCapture();
            SendResponse(client, cmd, result, nullptr, 0, header.sequenceId);
            break;
        }

        case CommandType::StopCapture: {
            result = CameraController::GetInstance().StopCapture();
            SendResponse(client, cmd, result, nullptr, 0, header.sequenceId);
            break;
        }

        case CommandType::SetExposure: {
            if (header.payloadSize >= sizeof(float)) {
                float exposure = *reinterpret_cast<const float*>(payload.data());
                result = CameraController::GetInstance().SetExposureTime(exposure);
            }
            else {
                result = ErrorCode::InvalidParameter;
            }
            SendResponse(client, cmd, result, nullptr, 0, header.sequenceId);
            break;
        }

        case CommandType::SetGain: {
            if (header.payloadSize >= sizeof(float)) {
                float gain = *reinterpret_cast<const float*>(payload.data());
                result = CameraController::GetInstance().SetGain(gain);
            }
            else {
                result = ErrorCode::InvalidParameter;
            }
            SendResponse(client, cmd, result, nullptr, 0, header.sequenceId);
            break;
        }

        case CommandType::SetROI: {
            if (header.payloadSize >= sizeof(ROI)) {
                const ROI* roi = reinterpret_cast<const ROI*>(payload.data());
                result = CameraController::GetInstance().SetROI(*roi);
            }
            else {
                result = ErrorCode::InvalidParameter;
            }
            SendResponse(client, cmd, result, nullptr, 0, header.sequenceId);
            break;
        }

        case CommandType::GetCameraInfo: {
            CameraInfo info;
            result = CameraController::GetInstance().GetCameraInfo(info);
            SendResponse(client, cmd, result, &info, sizeof(info), header.sequenceId);
            break;
        }

        case CommandType::GetSettings: {
            CameraSettings settings;
            result = CameraController::GetInstance().GetCurrentSettings(settings);
            SendResponse(client, cmd, result, &settings, sizeof(settings), header.sequenceId);
            break;
        }

        case CommandType::GetStats: {
            struct StatsData {
                uint64_t totalFrames;
                uint64_t droppedFrames;
                uint64_t errorCount;
                float avgFps;
            } stats;

            CameraController::GetInstance().GetPerformanceStats(
                stats.totalFrames, stats.droppedFrames, stats.errorCount, stats.avgFps);

            SendResponse(client, cmd, ErrorCode::Success, &stats, sizeof(stats), header.sequenceId);
            break;
        }

        case CommandType::StartStreaming: {
            client->isStreaming = true;
            LOG_INFO("Client started streaming");
            SendResponse(client, cmd, ErrorCode::Success, nullptr, 0, header.sequenceId);
            break;
        }

        case CommandType::StopStreaming: {
            client->isStreaming = false;
            LOG_INFO("Client stopped streaming");
            SendResponse(client, cmd, ErrorCode::Success, nullptr, 0, header.sequenceId);
            break;
        }

        case CommandType::Shutdown: {
            LOG_WARNING("Shutdown command received");
            SendResponse(client, cmd, ErrorCode::Success, nullptr, 0, header.sequenceId);
            // TODO: 구현
            break;
        }

        default: {
            LOG_ERROR("Unknown command: 0x" + std::to_string(header.command));
            result = ErrorCode::InvalidParameter;
            SendResponse(client, cmd, result, nullptr, 0, header.sequenceId);
            break;
        }
        }

        return true;
    }

    bool NetworkServer::SendResponse(std::shared_ptr<ClientSession> client, CommandType command,
        ErrorCode error, const void* data, size_t dataSize, uint32_t sequenceId) {
        std::lock_guard<std::mutex> lock(client->sendMutex);

        ResponseHeader header;
        header.magic = PROTOCOL_MAGIC;
        header.version = PROTOCOL_VERSION;
        header.command = static_cast<uint16_t>(command);
        header.errorCode = static_cast<int32_t>(error);
        header.payloadSize = static_cast<uint32_t>(dataSize);
        header.sequenceId = sequenceId;

        // 헤더 전송
        int sent = send(client->socket, reinterpret_cast<const char*>(&header), sizeof(header), 0);
        if (sent != sizeof(header)) {
            LOG_ERROR("Failed to send response header");
            return false;
        }

        // 데이터 전송
        if (data && dataSize > 0) {
            size_t totalSent = 0;
            const char* dataPtr = reinterpret_cast<const char*>(data);

            while (totalSent < dataSize) {
                int toSend = static_cast<int>(std::min(dataSize - totalSent, size_t(65536)));
                sent = send(client->socket, dataPtr + totalSent, toSend, 0);

                if (sent <= 0) {
                    LOG_ERROR("Failed to send response data");
                    return false;
                }

                totalSent += sent;
            }
        }

        return true;
    }

    void NetworkServer::OnFrameReady(const uint8_t* frameData, const FrameInfo& info) {
        std::lock_guard<std::mutex> lock(m_clientsMutex);

        // 스트리밍 중인 클라이언트에게 프레임 전송
        for (auto& client : m_clients) {
            if (client->connected && client->isStreaming) {
                SendFrame(client, frameData, info);
            }
        }
    }

    bool NetworkServer::SendFrame(std::shared_ptr<ClientSession> client,
        const uint8_t* frameData, const FrameInfo& info) {
        // TODO: 프레임 압축 구현 (JPEG)

        // 프레임 헤더
        struct FrameHeader {
            FrameInfo info;
            uint32_t dataSize;
        } frameHeader;

        frameHeader.info = info;
        frameHeader.dataSize = info.width * info.height * info.bytesPerPixel;

        // 프레임 데이터와 함께 전송
        std::vector<uint8_t> sendBuffer(sizeof(ResponseHeader) + sizeof(FrameHeader) + frameHeader.dataSize);

        // 응답 헤더
        ResponseHeader* respHeader = reinterpret_cast<ResponseHeader*>(sendBuffer.data());
        respHeader->magic = PROTOCOL_MAGIC;
        respHeader->version = PROTOCOL_VERSION;
        respHeader->command = 0x8000; // 프레임 데이터 플래그
        respHeader->errorCode = 0;
        respHeader->payloadSize = sizeof(FrameHeader) + frameHeader.dataSize;
        respHeader->sequenceId = client->sequenceId++;

        // 프레임 헤더 복사
        memcpy(sendBuffer.data() + sizeof(ResponseHeader), &frameHeader, sizeof(frameHeader));

        // 프레임 데이터 복사
        memcpy(sendBuffer.data() + sizeof(ResponseHeader) + sizeof(FrameHeader),
            frameData, frameHeader.dataSize);

        // 전송
        std::lock_guard<std::mutex> lock(client->sendMutex);
        int sent = send(client->socket, reinterpret_cast<const char*>(sendBuffer.data()),
            static_cast<int>(sendBuffer.size()), 0);

        if (sent != sendBuffer.size()) {
            LOG_WARNING("Failed to send complete frame to client");
            return false;
        }

        return true;
    }

    void NetworkServer::OnError(ErrorCode error, const char* message) {
        LOG_ERROR("Camera error: " + std::string(message));

        // 모든 클라이언트에게 에러 통지
        std::lock_guard<std::mutex> lock(m_clientsMutex);
        for (auto& client : m_clients) {
            if (client->connected) {
                SendResponse(client, CommandType(0x9000), error, message, strlen(message), 0);
            }
        }
    }

    void NetworkServer::OnCameraDisconnected() {
        LOG_WARNING("Camera disconnected");

        // 모든 클라이언트에게 통지
        std::lock_guard<std::mutex> lock(m_clientsMutex);
        for (auto& client : m_clients) {
            if (client->connected) {
                SendResponse(client, CommandType(0x9001), ErrorCode::CameraNotOpen, nullptr, 0, 0);
            }
        }
    }

    void NetworkServer::DisconnectClient(std::shared_ptr<ClientSession> client) {
        client->connected = false;

        if (client->socket != INVALID_SOCKET) {
            shutdown(client->socket, SD_BOTH);
            closesocket(client->socket);
            client->socket = INVALID_SOCKET;
        }

        if (client->thread && client->thread->joinable()) {
            client->thread->join();
        }
    }

    size_t NetworkServer::GetClientCount() const {
        std::lock_guard<std::mutex> lock(m_clientsMutex);
        return m_clients.size();
    }

    std::vector<std::string> NetworkServer::GetConnectedClients() const {
        std::lock_guard<std::mutex> lock(m_clientsMutex);
        std::vector<std::string> result;

        for (const auto& client : m_clients) {
            if (client->connected) {
                result.push_back(client->address + ":" + std::to_string(client->port));
            }
        }

        return result;
    }

} // namespace XimeaSensor