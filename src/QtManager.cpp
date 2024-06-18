#include "../include/managers/QtManager.hpp"

QtManager::QtManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef) {}

QtManager::~QtManager() {
    close(sockfd);
}

void QtManager::initializeServer() {
    // 소켓 생성
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    // 서버 주소 설정
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(PORT);

    // 소켓에 주소 바인딩
    if (bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    std::cout << "Server listening on port " << PORT << std::endl;
}

bool QtManager::sendMessage(const std::string &message) {
    // 클라이언트에게 데이터 전송
    ssize_t sendBytes = sendto(sockfd, message.c_str(), message.length(), 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
    if (sendBytes == -1) {
        perror("sendto failed");
        return false;
    }
    return true;
}

std::string QtManager::receiveMessage() {
    socklen_t clientLen = sizeof(clientAddr);

    // 클라이언트로부터 데이터 수신
    int recvBytes = recvfrom(sockfd, buffer, BUF_SIZE, 0, (struct sockaddr *)&clientAddr, &clientLen);
    if (recvBytes == -1) {
        perror("recvfrom failed");
        return "";
    }

    buffer[recvBytes] = '\0';
    return std::string(buffer);
}

void QtManager::guiThread() {
    

    while (state.main != Main::Shutdown) {
        switch (state.main.load()) {
            case Main::SystemInit: {
                initializeServer();
                break;
            }
            case Main::Ideal: {
                std::cerr << "Socket is not initialized. Unable to accept connections." << std::endl;
                break;
            }
            case Main::Homing: {
                break;
            }
            case Main::Perform: {
                break;
            }
            case Main::AddStance: {
                break;
            }
            case Main::Check: {
                break;
            }
            case Main::Test: {
                break;
            }
            case Main::Pause: {
                break;
            }
            case Main::Error: {
                break;
            }
            case Main::Shutdown:
                break;
        }
    }
}
