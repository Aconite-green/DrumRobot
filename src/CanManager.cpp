#include "../include/managers/CanManager.hpp"

// For Qt
// #include "../managers/CanManager.hpp"
CanManager::CanManager(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : motors(motorsRef)
{
    // 파일 저장 시각 계산
    start = std::chrono::high_resolution_clock::now(); 
}

CanManager::~CanManager()
{
    // 모든 소켓 닫기
    for (const auto &socketPair : sockets)
    {
        if (socketPair.second >= 0)
        {
            flushCanBuffer(socketPair.second);
            close(socketPair.second);
        }
    }
    sockets.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////
/*                                Settign Functions [Public]                                 */
//////////////////////////////////////////////////////////////////////////////////////////////
int CanManager::get_com_number_by_hostname() {
    char hostname[1024];
    
    // 호스트 이름을 가져오기
    if (gethostname(hostname, sizeof(hostname)) != 0) {
        perror("gethostname");
        exit(1);
    }
    hostname[1023] = '\0';

    // 호스트 이름에 따라 값을 반환
    if (strcmp(hostname, "shy-desktop") == 0) {
        return 1;
    } else if (strcmp(hostname, "shy-MINIPC-VC66-C2") == 0) {
        return 2;
    } else {
        std::cerr << "Unrecognized hostname: " << hostname << std::endl;
        exit(1);
    }
}

void CanManager::restCanPort(int com_number)
{
    char can1_on[100], can2_on[100], can3_on[100], can1_off[100], can2_off[100], can3_off[100];

    // Reset the commands based on com_number
    if (com_number == 1) {
        //sudo uhubctl 이 명령어 실행하면 포트 검색가능
        //0c72:000c PEAK-System Technik GmbH PCAN-USB (8.6.1)]
        //요런식으로 나오는 애들이 can 통신선임
        //Current status for hub 1-4.1.1 [0bda:5411 Generic USB2.1 Hub, USB 2.10, 5 ports]
        //Port 1: 0503 power highspeed enable connect []
        //Port 2: 0103 power enable connect [0c72:000c PEAK-System Technik GmbH PCAN-USB (8.6.1)]
        //Port 3: 0103 power enable connect [0c72:000c PEAK-System Technik GmbH PCAN-USB (8.6.1)]
        // 예를 들어 포트 3번을 끄려고한다면 다음과 같이 명령어 넣어주면된다.
        //키려면
        //sudo uhubctl -l 1-4.1.1 -p 3 -a off
        //끄려면
        //sudo uhubctl -l 1-4.1.1 -p 3 -a off

        // com_number_1
        snprintf(can1_off, sizeof(can1_off), "sudo uhubctl -l 1-4 -p 1 -a off");
        snprintf(can2_off, sizeof(can2_off), "sudo uhubctl -l 1-4 -p 2 -a off");
        snprintf(can3_off, sizeof(can3_off), "sudo uhubctl -l 1-4 -p 3 -a off");

        snprintf(can1_on, sizeof(can1_on), "sudo uhubctl -l 1-4 -p 1 -a on");
        snprintf(can2_on, sizeof(can2_on), "sudo uhubctl -l 1-4 -p 2 -a on");
        snprintf(can3_on, sizeof(can3_on), "sudo uhubctl -l 1-4 -p 3 -a on");
    } else if (com_number == 2) {
        // com_number_2
        snprintf(can1_off, sizeof(can1_off), "sudo uhubctl -l 1-6.1 -p 1 -a off");
        snprintf(can1_on, sizeof(can1_on), "sudo uhubctl -l 1-6.1 -p 1 -a on");

        // For com_number_2, we only have can1_off and can1_on
        snprintf(can2_off, sizeof(can2_off), " "); // Empty command
        snprintf(can3_off, sizeof(can3_off), " "); // Empty command
        snprintf(can2_on, sizeof(can2_on), " ");  // Empty command
        snprintf(can3_on, sizeof(can3_on), " ");  // Empty command
    } else {
        fprintf(stderr, "Invalid com_number: %d\n", com_number);
        return;
    }
    //만든 명령줄 실행시키기 
    int ret1 = system(can1_off);
    std::cout << std::endl;
    int ret2 = system(can2_off);
    std::cout << std::endl;
    int ret3 = system(can3_off);
    std::cout << std::endl;

    sleep(2);

    int ret4 = system(can1_on);
    std::cout << std::endl;
    int ret5 = system(can2_on);
    std::cout << std::endl;
    int ret6 = system(can3_on);
    std::cout << std::endl;

    
    if (ret1 != 0 || ret2 != 0 || ret3 != 0 || ret4 != 0 || ret5 != 0 || ret6 != 0)
    {
        fprintf(stderr, "Failed to reset port\n");
    }

    sleep(2);
}

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

void CanManager::flushCanBuffer(int socket)
{
    // 버퍼 초기화 (필터 설정으로 모든 패킷 필터링)
    struct can_filter filter = {0};
    setsockopt(socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
}

void CanManager::resetCanFilter(int socket)
{
    // 기본 상태로 CAN 필터 재설정
    setsockopt(socket, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
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
    char command1[100], command2[100], command3[100], command4[100];



    snprintf(command4, sizeof(command4), "sudo ip link set %s down", port);
    snprintf(command1, sizeof(command1), "sudo ip link set %s type can bitrate 1000000 restart-ms 100", port);
    snprintf(command2, sizeof(command2), "sudo ip link set %s up", port);
    snprintf(command3, sizeof(command3), "sudo ifconfig %s txqueuelen 1000", port);

    int ret4 = system(command4);
    int ret1 = system(command1);
    int ret2 = system(command2);    // UP PORT 해줄 때 잔여 명령 실행
    int ret3 = system(command3);

    
    if (ret1 != 0 || ret2 != 0 || ret3 != 0 || ret4 != 0)
    {
        fprintf(stderr, "Failed to activate port: %s\n", port);
    }

    sleep(2);
}

void CanManager::deactivateCanPort(const char *port)
{
    char command[100];
    snprintf(command, sizeof(command), "sudo ip link set %s down", port);

    int ret = system(command);

    if (ret != 0)
    {
        fprintf(stderr, "Failed to deactivate port: %s\n", port);
    }
}

void CanManager::deactivateAllCanPorts()
{
    FILE *fp = popen("ip link show | grep can", "r");
    if (fp == nullptr)
    {
        perror("No available CAN port");
        return;
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
            deactivateCanPort(port.c_str());
        }
    }

    if (feof(fp) == 0)
    {
        perror("fgets failed");
        printf("Errno: %d\n", errno);
    }

    pclose(fp);
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
        // std::cout << "clearcanbuf" << std::endl;
        if (selectRes > 0)
        {
            // 수신 버퍼에서 데이터 읽기
            ssize_t nbytes = read(canSocket, &frame, sizeof(struct can_frame));
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
        std::cout << "CAN read error: " << motor->myName << "\n";
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
    CanManager::appendToCSV_CAN("can_orgin", motor->sendFrame);
    if (write(motor->socket, &motor->sendFrame, sizeof(frame)) != sizeof(frame))
    {
        errorCnt++;
        if (errorCnt > 5)
        {
            deactivateAllCanPorts();
            std::cout << "Go to Error state by CAN write error " << motor->myName << "\n";
            return false;
        }
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

        int socket_fd = socketPair.second;

        for (auto &motor_pair : motors)
        {
            auto &motor = motor_pair.second;

            if (!motor->isConected)
            {
                motor->socket = socket_fd;
                if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
                {
                    maxoncmd.getCheck(*maxonMotor, &frame);
                    txFrame(motor, frame);
                }

                usleep(50000);
            }
        }

        // 해당 소켓으로 프레임을 무작정 읽어서 버퍼에 넣음
        int readCount = 0;
        while (readCount < 10)
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

bool CanManager::distributeFramesToMotors(bool setlimit)
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
                    if (setlimit)
                    {
                        bool isSafe = safetyCheck_T(motor, parsedData);
                        if (!isSafe)
                        {
                            return false;
                        }
                        tMotor->coordinatePos = (tMotor->currentPos + tMotor->homeOffset) * tMotor->cwDir;  // 이거 이 위치가 맞나??
                    }
                    tMotor->currentPos = std::get<1>(parsedData);
                    tMotor->currentVel = std::get<2>(parsedData);
                    tMotor->currentTor = std::get<3>(parsedData);
                    tMotor->recieveBuffer.push(frame);

                    std::string motor_ID = tMotor->myName;
                    std::string file_name = "motor_receive(actual_0)";
                    
                    appendToCSV_CM(motor_ID + file_name, tMotor->currentPos, tMotor->currentTor);
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
                    if (setlimit)
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
        if (motor->isfixed == false)
        {
            motor->fixedPos = tMotor->currentPos;
            motor->isfixed = true;
        }
        std::string motor_ID = tMotor->myName;
        std::string file_name = "Fixed(desired_actial)";
        appendToCSV_CM(motor_ID + file_name, motor->fixedPos, tMotor->currentPos);

        // tservocmd.comm_can_set_pos_spd(*tMotor, &tMotor->sendFrame, motor->fixedPos, 20000, 300000);//tMotor->spd, tMotor->acl);

        // safety check
        float diff_angle = motor->fixedPos - tMotor->currentPos;
        if (abs(diff_angle) > POS_DIFF_LIMIT)
        {
            std::cout << "Go to Error state by safety check (Pos Diff) " << tMotor->myName << "\n";
            return false;
        }
        tservocmd.comm_can_set_pos(*tMotor, &tMotor->sendFrame, motor->fixedPos);

        if (!sendMotorFrame(tMotor))
        {
            return false;
        };
    }
    else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
    {
        if (motor->isfixed == false)
        {
            motor->fixedPos = maxonMotor->currentPos;
            motor->isfixed = true;
        }

        maxoncmd.getTargetPosition(*maxonMotor, &maxonMotor->sendFrame, maxonMotor->fixedPos);
        if (!sendMotorFrame(maxonMotor))
        {
            return false;
        };

        maxoncmd.getSync(&maxonMotor->sendFrame);
        if (!sendMotorFrame(maxonMotor))
        {
            return false;
        };
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
            if (!sendForCheck_Fixed(motor))
            {
                return false;
            }
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////
/*                                Functions for Thread Case                                      */
//////////////////////////////////////////////////////////////////////////////////////////////

bool CanManager::setCANFrame()
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

            if (tMotor_control_mode == POS_LOOP)
            {
                // safety check
                float diff_angle = tData.position - tMotor->currentPos;
                if (abs(diff_angle) > POS_DIFF_LIMIT)
                {
                    std::cout << "Go to Error state by safety check (Pos Diff) " << tMotor->myName << "\n";
                    return false;
                }
                tservocmd.comm_can_set_pos(*tMotor, &tMotor->sendFrame, tData.position);
            }
            else if (tMotor_control_mode == POS_SPD_LOOP)
            {
                tservocmd.comm_can_set_pos_spd(*tMotor, &tMotor->sendFrame, tData.position, tData.spd, tData.acl);
            }
            else if (tMotor_control_mode == SPD_LOOP)
            {
                float diff_angle = tData.position - tMotor->currentPos; // [rad]
                float target_spd = (diff_angle / deltaT) * (60 / 2 / M_PI) * (tMotor->R_Ratio[tMotor->motorType] * tMotor->PolePairs);   // [erpm]
                float P_gain = 1.0;
                tservocmd.comm_can_set_spd(*tMotor, &tMotor->sendFrame, P_gain * target_spd); // [erpm]
                // tservocmd.comm_can_set_spd(*tMotor, &tMotor->sendFrame, tData.spd); // [erpm]
            }
            else
            {
                std::cout << "tMotor control mode ERROR\n";
                return false;
            }
            tMotor->brake_state = tData.isBrake;

            std::string motor_ID = tMotor->myName;
            std::string file_name = "setCANFrame(desired_actial)";
            appendToCSV_CM(motor_ID + file_name, tData.position, tMotor->currentPos);
        }
    }
    Input_pos.push_back(Pos);
    return true;
}

bool CanManager::safetyCheck_T(std::shared_ptr<GenericMotor> &motor, std::tuple<int, float, float, float, int8_t, int8_t> parsedData)
{
    bool isSafe = true;

    if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
    {
        float coordinationPos = (std::get<1>(parsedData) + tMotor->homeOffset) * tMotor->cwDir;
        if (abs(tMotor->currentPos - std::get<1>(parsedData)) > 0.4 || tMotor->rMin > coordinationPos || tMotor->rMax < coordinationPos)
        {
            if (abs(tMotor->currentPos - std::get<1>(parsedData)) > 0.4)
            {
                std::cout << "Error For " << tMotor->myName << " (Pos Diff)\n";
                std::cout << "Previous : " << tMotor->currentPos << "\nCrrent : " << std::get<1>(parsedData) << "\n";
                std::cout << "Diff : " << abs(tMotor->currentPos - std::get<1>(parsedData)) / M_PI * 180 << "deg\n";
            }
            else if (tMotor->rMin > coordinationPos)
            {
                std::cout << "Error For " << tMotor->myName << " (Out of Range : Min)\n";
                std::cout << "coordinationPos : " << coordinationPos / M_PI * 180 << "deg\n";
            }
            else
            {
                std::cout << "Error For " << tMotor->myName << " (Out of Range : Max)\n";
                std::cout << "coordinationPos : " << coordinationPos / M_PI * 180 << "deg\n";
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
        if (/*abs(maxonMotor->currentPos - std::get<1>(parsedData)) > 0.4 || */ maxonMotor->rMin > coordinationPos || maxonMotor->rMax < coordinationPos)
        {
            if (maxonMotor->rMin > coordinationPos)
            {
                std::cout << "Error For " << maxonMotor->myName << " (Out of Range : Min)\n";
                std::cout << "coordinationPos : " << coordinationPos / M_PI * 180 << "deg\n";
            }
            else
            {
                std::cout << "Error For " << maxonMotor->myName << " (Out of Range : Max)\n";
                std::cout << "coordinationPos : " << coordinationPos / M_PI * 180 << "deg\n";
            }

            if (maxonMotor->errorCnt > 10)
            {
                isSafe = false;
                maxonMotor->isError = true;
            }
            else
            {
                maxonMotor->errorCnt++;
            }

            maxoncmd.getQuickStop(*maxonMotor, &maxonMotor->sendFrame);
            sendMotorFrame(maxonMotor);
            // usleep(5000);    // 이거 필요한가???
            usleep(10);
            maxoncmd.getSync(&maxonMotor->sendFrame);
            sendMotorFrame(maxonMotor);
        }
    }

    return isSafe;
}

// 시간를 CSV 파일에 한 줄씩 저장하는 함수
void CanManager::appendToCSV_time(const std::string& filename) {
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> elapsed = now - start;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";  // 기본 경로와 파일 이름을 결합

    // 파일이 이미 존재하는지 확인
    bool fileExists = std::ifstream(fullPath).good();

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);  // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app);  // 이미 파일이 존재하면 append 모드로 열기
    }
    // 파일이 제대로 열렸는지 확인
    if (file.is_open()) {
        // 데이터 추가
        file << elapsed.count() << "\n";
        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}

// 시간과 변수를 CSV 파일에 한 줄씩 저장하는 함수
void CanManager::appendToCSV_CM(const std::string& filename, float fixed_position, float current_position) {
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> elapsed = now - start;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";  // 기본 경로와 파일 이름을 결합

    // 파일이 이미 존재하는지 확인
    bool fileExists = std::ifstream(fullPath).good();

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);  // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app);  // 이미 파일이 존재하면 append 모드로 열기
    }
    // 파일이 제대로 열렸는지 확인

    if (file.is_open()) {
        // 데이터 추가
        file << elapsed.count() << "," << fixed_position << "," << current_position << "\n";  // 시간과 float 변수들을 CSV 형식으로 한 줄에 기록
       
        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}

// 시간과 CAN Feame을 CSV 파일에 한 줄씩 저장하는 함수
void CanManager::appendToCSV_CAN(const std::string& filename, can_frame& c_frame) {
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> elapsed = now - start;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";  // 기본 경로와 파일 이름을 결합

    // 파일이 이미 존재하는지 확인
    bool fileExists = std::ifstream(fullPath).good();

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);  // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app);  // 이미 파일이 존재하면 append 모드로 열기
    }
    // 파일이 제대로 열렸는지 확인

    if (file.is_open()) {
        // 데이터 추가
        file << elapsed.count(); // 시간과 float 변수들을 CSV 형식으로 한 줄에 기록
       
       // can_frame의 data 배열을 CSV 형식으로 저장
        for (int i = 0; i < 8; ++i) {
            file << "," << static_cast<int>(c_frame.data[i]);
        }
        file << "\n";

        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}