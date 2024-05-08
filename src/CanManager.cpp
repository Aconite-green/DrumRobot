#include "../include/managers/CanManager.hpp"
CanManager::CanManager(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : motors(motorsRef)
{
}

CanManager::~CanManager()
{
    // 모든 소켓 닫기
    for (const auto &socketPair : sockets)
    {
        if (socketPair.second >= 0)
        {
            close(socketPair.second);
        }
    }
    sockets.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////
/*                                Settign Functions [Public]                                 */
//////////////////////////////////////////////////////////////////////////////////////////////

void CanManager::initializeCAN()
{

    list_and_activate_available_can_ports();
    for (const auto &ifname : this->ifnames)
    {
        std::cout << "Processing interface: " << ifname << std::endl;
        int hsocket = createSocket(ifname);
        if (hsocket < 0)
        {
            std::cerr << "Socket creation error for interface: " << ifname << std::endl;
            
        }
        sockets[ifname] = hsocket;
        isConnected[ifname] = true;
        std::cout << "Socket created for " << ifname << ": " << hsocket << std::endl;
    }
}

void CanManager::setSocketsTimeout(int sec, int usec)
{
    for (const auto &socketPair : sockets)
    {
        int socket_fd = socketPair.second;
        if (setSocketTimeout(socket_fd, sec, usec) != 0)
        {
            std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }
}

void CanManager::checkCanPortsStatus()
{

    for (const auto &ifname : this->ifnames)
    {
        isConnected[ifname] = getCanPortStatus(ifname.c_str());

        if (!isConnected[ifname])
        {
            std::cout << "Port " << ifname << " is NOT CONNECTED" << std::endl;
        }
    }

    // 모든 포트가 연결된 경우 1, 아니면 0 반환
}

////////////////////////////////////////////////////////////////////////////////////////////////
/*                                Settign Functions [Private]                                 */
//////////////////////////////////////////////////////////////////////////////////////////////

bool CanManager::getCanPortStatus(const char *port)
{
    char command[50];
    snprintf(command, sizeof(command), "ip link show %s", port);

    FILE *fp = popen(command, "r");
    if (fp == NULL)
    {
        perror("Error opening pipe");
        return false;
    }

    char output[1024];
    while (fgets(output, sizeof(output) - 1, fp) != NULL)
    {
        if (strstr(output, "DOWN") || strstr(output, "does not exist"))
        {
            pclose(fp);
            return false;
        }
        else if (strstr(output, "UP"))
        {
            pclose(fp);
            return true;
        }
    }

    perror("fgets failed");
    printf("Errno: %d\n", errno); // errno 값을 출력
    pclose(fp);
    return false;
}

int CanManager::createSocket(const std::string &ifname)
{
    int result;
    struct sockaddr_can addr;
    struct ifreq ifr;

    int localSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW); // 지역 변수로 소켓 생성
    if (localSocket < 0)
    {
        return ERR_SOCKET_CREATE_FAILURE;
    }

    memset(&ifr, 0, sizeof(struct ifreq));
    memset(&addr, 0, sizeof(struct sockaddr_can));

    strcpy(ifr.ifr_name, ifname.c_str());
    result = ioctl(localSocket, SIOCGIFINDEX, &ifr);
    if (result < 0)
    {
        close(localSocket);
        return ERR_SOCKET_CREATE_FAILURE;
    }

    addr.can_ifindex = ifr.ifr_ifindex;
    addr.can_family = AF_CAN;

    if (bind(localSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        close(localSocket);
        return ERR_SOCKET_CREATE_FAILURE;
    }

    return localSocket; // 생성된 소켓 디스크립터 반환
}

void CanManager::activateCanPort(const char *port)
{
    char command1[100], command2[100];
    snprintf(command1, sizeof(command1), "sudo ip link set %s type can bitrate 1000000 sample-point 0.850", port);
    snprintf(command2, sizeof(command2), "sudo ip link set %s up", port);

    int ret1 = system(command1);
    int ret2 = system(command2);

    if (ret1 != 0 || ret2 != 0)
    {
        fprintf(stderr, "Failed to activate port: %s\n", port);
        
    }
}

void CanManager::list_and_activate_available_can_ports()
{
    int portCount = 0; // CAN 포트 수를 세기 위한 변수

    FILE *fp = popen("ip link show | grep can", "r");
    if (fp == nullptr)
    {
        perror("No available CAN port");
        
    }

    char output[1024];
    while (fgets(output, sizeof(output) - 1, fp) != nullptr)
    {
        std::string line(output);
        std::istringstream iss(line);
        std::string skip, port;
        iss >> skip >> port;

        // 콜론 제거
        if (!port.empty() && port.back() == ':')
        {
            port.pop_back();
        }

        // 포트 이름이 유효한지 확인
        if (!port.empty() && port.find("can") == 0)
        {
            portCount++;
            if (!getCanPortStatus(port.c_str()))
            {
                printf("%s is DOWN, activating...\n", port.c_str());
                activateCanPort(port.c_str());
            }
            else
            {
                printf("%s is already UP\n", port.c_str());
            }

            this->ifnames.push_back(port); // 포트 이름을 ifnames 벡터에 추가
        }
    }

    if (feof(fp) == 0)
    {
        perror("fgets failed");
        printf("Errno: %d\n", errno);
    }

    pclose(fp);

    if (portCount == 0)
    {
        printf("No CAN port found. Exiting...\n");
        
    }
}

void CanManager::clearReadBuffers()
{
    for (const auto &socketPair : sockets)
    {
        int socket_fd = socketPair.second;
        clearCanBuffer(socket_fd);
    }
}

void CanManager::clearCanBuffer(int canSocket)
{
    struct can_frame frame;
    fd_set readSet;
    struct timeval timeout;

    // 수신 대기 시간 설정
    timeout.tv_sec = 0;
    timeout.tv_usec = 0; // 즉시 반환

    while (true)
    {
        FD_ZERO(&readSet);
        FD_SET(canSocket, &readSet);

        // 소켓에서 읽을 데이터가 있는지 확인
        int selectRes = select(canSocket + 1, &readSet, NULL, NULL, &timeout);

        if (selectRes > 0)
        {
            // 수신 버퍼에서 데이터 읽기
            ssize_t nbytes = read(canSocket, &frame, sizeof(struct can_frame));

            if (nbytes <= 0)
            {
                // 읽기 실패하거나 더 이상 읽을 데이터가 없음
                break;
            }
        }
        else
        {
            // 읽을 데이터가 없음
            break;
        }
    }
}

int CanManager::setSocketTimeout(int socket, int sec, int usec)
{
    struct timeval timeout;
    timeout.tv_sec = sec;
    timeout.tv_usec = usec;

    if (setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
    {
        perror("setsockopt failed");
        return -1;
    }

    return 0;
}

void CanManager::setSocketNonBlock()
{
    for (auto &socket : sockets)
    {                                                 // sockets는 std::map<std::string, int> 타입
        int flags = fcntl(socket.second, F_GETFL, 0); // 현재 플래그를 가져옴
        if (flags < 0)
            continue;                                      // 에러 체크
        fcntl(socket.second, F_SETFL, flags | O_NONBLOCK); // 논블록 플래그 추가
    }
}

void CanManager::setSocketBlock()
{
    for (auto &socket : sockets)
    {
        int flags = fcntl(socket.second, F_GETFL, 0);
        if (flags < 0)
            continue;
        fcntl(socket.second, F_SETFL, flags & ~O_NONBLOCK); // 논블록 플래그 제거
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////
/*                                Utility Functions                                          */
//////////////////////////////////////////////////////////////////////////////////////////////

bool CanManager::txFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame)
{
    if (write(motor->socket, &frame, sizeof(frame)) != sizeof(frame))
    {
        perror("CAN write error");
        return false;
    }
    return true;
}

bool CanManager::rxFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame)
{

    if (read(motor->socket, &frame, sizeof(frame)) != sizeof(frame))
    {
        perror("CAN read error");
        return false;
    }
    return true;
}

bool CanManager::sendAndRecv(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame)
{
    if (!txFrame(motor, frame) || !rxFrame(motor, frame))
    {
        perror("Send and receive error");
        return false;
    }
    return true;
}

bool CanManager::sendFromBuff(std::shared_ptr<GenericMotor> &motor)
{
    std::cout << "Erase this function\n";
    return true;
}

bool CanManager::sendMotorFrame(std::shared_ptr<GenericMotor> motor)
{
    struct can_frame frame;
    if (write(motor->socket, &motor->sendFrame, sizeof(frame)) != sizeof(frame))
    {
        perror("CAN write error");
        return false;
    }
    return true;
}

bool CanManager::recvToBuff(std::shared_ptr<GenericMotor> &motor, int readCount)
{
    struct can_frame frame;
    for (int i = 0; i < readCount; i++)
    {
        if (rxFrame(motor, frame))
        {
            motor->recieveBuffer.push(frame);
        }
        else
        {
            return false;
        }
    }
    return true;
}

void CanManager::setMotorsSocket()
{
    struct can_frame frame;
    setSocketsTimeout(0, 10000);
    clearReadBuffers();
    std::map<int, int> localMotorsPerSocket;

    // 모든 소켓에 대해 Maxon 모터에 명령을 보내고 응답을 확인
    for (const auto &socketPair : sockets)
    {
        // 모든 모터에 임의의 소켓값 할당 및 canframe 전송(Maxon 만)
        int socket_fd = socketPair.second;

        for (auto &motor_pair : motors)
        {
            auto &motor = motor_pair.second;

            if (!motor->isConected)
            {

                if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
                {
                    maxoncmd.getCheck(*maxonMotor, &frame);
                }
                motor->socket = socket_fd;

                txFrame(motor, frame);
                usleep(50000);
            }
        }

        // 해당 소켓으로 프레임을 무작정 읽어서 버퍼에 넣음
        int readCount = 0;
        while (readCount < 30)
        {
            ssize_t result = read(socket_fd, &frame, sizeof(frame));

            if (result > 0)
            {
                tempFrames[socket_fd].push_back(frame);
            }
            readCount++;
        }

        // 버퍼에서 하나씩 읽어옴
        for (auto &motor_pair : motors)
        {
            auto &motor = motor_pair.second;
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
            {
                for (auto &frame : tempFrames[motor->socket])
                {
                    if ((frame.can_id & 0xFF) == tMotor->nodeId)
                    {
                        motor->isConected = true;
                        std::tuple<int, float, float, float, int8_t, int8_t> parsedData = tservocmd.motor_receive(&frame);
                        motor->currentPos = std::get<1>(parsedData);
                    }
                }
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
            {
                for (auto &frame : tempFrames[motor->socket])
                {
                    if (frame.can_id == maxonMotor->canReceiveId)
                    {
                        motor->isConected = true;
                        maxonCnt++;
                    }
                }
            }
        }

        tempFrames.clear();
    }

    // 모든 소켓에 대한 검사가 완료된 후, 모터 연결 상태 확인 및 삭제
    for (auto it = motors.begin(); it != motors.end();)
    {
        std::string name = it->first;
        std::shared_ptr<GenericMotor> motor = it->second;
        if (motor->isConected)
        {
            std::cerr << "--------------> Motor [" << name << "] is Connected. " << motor->nodeId << std::endl;
            ++it;
        }
        else
        {
            std::cerr << "Motor [" << name << "] Not Connected. " << motor->nodeId << std::endl;
            it = motors.erase(it);
        }
    }
}

void CanManager::readFramesFromAllSockets()
{
    struct can_frame frame;

    for (const auto &socketPair : sockets)
    {
        int socket_fd = socketPair.second;
        while (read(socket_fd, &frame, sizeof(frame)) == sizeof(frame))
        {
            tempFrames[socket_fd].push_back(frame);
        }
    }
}

bool CanManager::distributeFramesToMotors(bool isHomed)
{
    for (auto &motor_pair : motors)
    {
        auto &motor = motor_pair.second;

        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            // TMotor 처리
            for (auto &frame : tempFrames[motor->socket])
            {
                if ((frame.can_id & 0xFF) == tMotor->nodeId)
                {
                    std::tuple<int, float, float, float, int8_t, int8_t> parsedData = tservocmd.motor_receive(&frame);
                    if (isHomed)
                    {
                        bool isSafe = safetyCheck_T(motor, parsedData);
                        if (!isSafe)
                        {
                            return false;
                        }
                    }
                    tMotor->currentPos = std::get<1>(parsedData);
                    tMotor->currentVel = std::get<2>(parsedData);
                    tMotor->currentTor = std::get<3>(parsedData);
                    tMotor->coordinatePos = (tMotor->currentPos + tMotor->homeOffset) * tMotor->cwDir;
                    tMotor->recieveBuffer.push(frame);
                }
            }
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            // MaxonMotor 처리
            for (auto &frame : tempFrames[motor->socket])
            {
                if (frame.can_id == maxonMotor->rxPdoIds[0])
                {
                    std::tuple<int, float, float, unsigned char> parsedData = maxoncmd.parseRecieveCommand(*maxonMotor, &frame);
                    if (isHomed)
                    {
                        bool isSafe = safetyCheck_M(motor, parsedData);
                        if (!isSafe)
                        {
                            return false;
                        }
                    }
                    maxonMotor->currentPos = std::get<1>(parsedData);
                    maxonMotor->currentTor = std::get<2>(parsedData);
                    maxonMotor->statusBit = std::get<3>(parsedData);
                    maxonMotor->positionValues[maxonMotor->posIndex % 4] = std::get<1>(parsedData);
                    maxonMotor->posIndex++;
                    maxonMotor->recieveBuffer.push(frame);
                    maxonMotor->coordinatePos = maxonMotor->currentPos * maxonMotor->cwDir;
                }
            }
        }
    }
    tempFrames.clear(); // 프레임 분배 후 임시 배열 비우기

    return true;
}

bool CanManager::sendForCheck_Fixed(std::shared_ptr<GenericMotor> motor)
{
    clearReadBuffers();

    if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
    {
        tservocmd.comm_can_set_pos_spd(*tMotor, &tMotor->sendFrame, tMotor->currentPos, tMotor->spd, tMotor->acl);
        // tservocmd.comm_can_set_spd(*tMotor, &tMotor->sendFrame, 1000);
        sendMotorFrame(tMotor);
    }
    else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
    {
        maxoncmd.getSync(&maxonMotor->sendFrame);
        sendMotorFrame(maxonMotor);
    }
    return true;
}

bool CanManager::checkMaxon()
{
    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;

        struct can_frame frame;
        clearReadBuffers();

        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            maxoncmd.getSync(&frame);
            txFrame(motor, frame);
        }
    }
    return true;
}

bool CanManager::checkAllMotors_Fixed()
{
    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;
        if (!motor->isError)
        {
            sendForCheck_Fixed(motor);
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////
/*                                Functions for Thread Case                                      */
//////////////////////////////////////////////////////////////////////////////////////////////

void CanManager::setCANFrame()
{
    vector<float> Pos(9);
    for (auto &motor_pair : motors)
    {
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            MaxonData mData = maxonMotor->commandBuffer.front();
            maxonMotor->commandBuffer.pop();
            Pos[motor_mapping[maxonMotor->myName]] = mData.position;
            maxoncmd.getTargetPosition(*maxonMotor, &maxonMotor->sendFrame, mData.position);
        }
        else if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
        {
            TMotorData tData = tMotor->commandBuffer.front();
            tMotor->commandBuffer.pop();
            Pos[motor_mapping[tMotor->myName]] = tData.position;
            tservocmd.comm_can_set_pos_spd(*tMotor, &tMotor->sendFrame, tData.position, tData.spd, tData.acl);
        }
    }
    Input_pos.push_back(Pos);
}

bool CanManager::safetyCheck_T(std::shared_ptr<GenericMotor> &motor, std::tuple<int, float, float, float, int8_t, int8_t> parsedData)
{
    bool isSafe = true;

    if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
    {
        float coordinationPos = (std::get<1>(parsedData) + tMotor->homeOffset) * tMotor->cwDir;
        if (abs(tMotor->currentPos - std::get<1>(parsedData)) > 0.2 || tMotor->rMin > coordinationPos || tMotor->rMax < coordinationPos)
        {
            if (abs(tMotor->currentPos - std::get<1>(parsedData)) > 0.2)
            {
                std::cout << "Error For " << tMotor->myName << " (Pos Diff)\n";
                cout << "Previous : " << tMotor->currentPos << "\nCrrent : " << std::get<1>(parsedData) << "\n";
                cout << "Diff : " << abs(tMotor->currentPos - std::get<1>(parsedData)) / M_PI * 180 << "deg\n";
            }
            else if (tMotor->rMin > coordinationPos)
            {
                std::cout << "Error For " << tMotor->myName << " (Out of Range : Min)\n";
                cout << "coordinationPos : " << coordinationPos / M_PI * 180 << "deg\n";
            }
            else
            {
                std::cout << "Error For " << tMotor->myName << " (Out of Range : Max)\n";
                cout << "coordinationPos : " << coordinationPos / M_PI * 180 << "deg\n";
            }

            isSafe = false;
            tMotor->isError = true;
            tservocmd.comm_can_set_cb(*tMotor, &tMotor->sendFrame, 0);
            sendMotorFrame(tMotor);
        }
    }

    return isSafe;
}

bool CanManager::safetyCheck_M(std::shared_ptr<GenericMotor> &motor, std::tuple<int, float, float, unsigned char> parsedData)
{
    bool isSafe = true;

    if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
    {
        float coordinationPos = std::get<1>(parsedData) * maxonMotor->cwDir;
        if (abs(maxonMotor->currentPos - std::get<1>(parsedData)) > 0.4 || maxonMotor->rMin > coordinationPos || maxonMotor->rMax < coordinationPos)
        {
            if (abs(maxonMotor->currentPos - std::get<1>(parsedData)) > 0.4)
            {
                std::cout << "Error For " << maxonMotor->myName << " (Pos Diff)\n";
                cout << "Previous : " << maxonMotor->currentPos << "\nCurrent : " << std::get<1>(parsedData) << "\n";
                cout << "Diff : " << abs(maxonMotor->currentPos - std::get<1>(parsedData)) / M_PI * 180 << "deg\n";
            }
            else if (maxonMotor->rMin > coordinationPos)
            {
                std::cout << "Error For " << maxonMotor->myName << " (Out of Range : Min)\n";
                cout << "coordinationPos : " << coordinationPos / M_PI * 180 << "deg\n";
            }
            else
            {
                std::cout << "Error For " << maxonMotor->myName << " (Out of Range : Max)\n";
                cout << "coordinationPos : " << coordinationPos / M_PI * 180 << "deg\n";
            }

            isSafe = false;
            maxonMotor->isError = true;
            maxoncmd.getQuickStop(*maxonMotor, &maxonMotor->sendFrame);
            sendMotorFrame(maxonMotor);
            usleep(5000);
            maxoncmd.getSync(&maxonMotor->sendFrame);
            sendMotorFrame(maxonMotor);
        }
    }

    return isSafe;
}