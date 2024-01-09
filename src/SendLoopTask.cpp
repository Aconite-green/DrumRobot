#include "../include/SendLoopTask.hpp"

SendLoopTask::SendLoopTask(SystemState &systemStateRef,
                           CanSocketUtils &canUtilsRef,
                           std::map<std::string, std::shared_ptr<TMotor>> &tmotorsRef,
                           std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotorsRef,
                           queue<can_frame> &sendBufferRef)
    : systemState(systemStateRef), canUtils(canUtilsRef), tmotors(tmotorsRef), maxonMotors(maxonMotorsRef), sendBuffer(sendBufferRef), pathManager(sendBufferRef, tmotorsRef,maxonMotorsRef)
{
}

void SendLoopTask::operator()()
{

    while (systemState.main != Main::Shutdown)
    {
        usleep(50000);
        while (systemState.main == Main::Perform)
        {
            usleep(50000);

            if (systemState.runMode == RunMode::Preparing)
            {
                if (CheckAllMotorsCurrentPosition())
                {
                    initializePathManager();
                    pathManager.GetReadyArr();
                    SendReadyLoop();
                    systemState.runMode = RunMode::Ready;
                }
            }
            else if (systemState.runMode == RunMode::Running)
            {
                if (CheckAllMotorsCurrentPosition())
                {
                    SendLoop();
                    systemState.runMode = RunMode::PrePreparation;
                }
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                                  PERFORM                                   */
///////////////////////////////////////////////////////////////////////////////

template <typename MotorMap>
void SendLoopTask::writeToSocket(MotorMap &motorMap, const std::map<std::string, int> &sockets)
{
    struct can_frame frameToProcess;

    for (auto &motor_pair : motorMap)
    {
        auto motor_ptr = motor_pair.second;
        auto interface_name = motor_ptr->interFaceName;

        frameToProcess = sendBuffer.front(); // sendBuffer에서 데이터 꺼내기
        sendBuffer.pop();

        if (sockets.find(interface_name) != sockets.end())
        {
            int socket_descriptor = sockets.at(interface_name);
            ssize_t bytesWritten = write(socket_descriptor, &frameToProcess, sizeof(struct can_frame));

            if (bytesWritten == -1)
            {
                writeFailCount++;
                if (writeFailCount >= 10)
                {
                    systemState.runMode = RunMode::Stop;
                    canUtils.restart_all_can_ports();
                    writeFailCount = 0; // 카운터 리셋
                }
            }
            else
            {
                writeFailCount = 0; // 성공 시 카운터 리셋
            }
        }
        else
        {
            std::cerr << "Socket not found for interface: " << interface_name << std::endl;
        }
    }
}

void SendLoopTask::SendLoop()
{

    struct can_frame frameToProcess;
    chrono::system_clock::time_point external = std::chrono::system_clock::now();

    while (systemState.runMode != RunMode::PrePreparation)
    {

        if (systemState.runMode == RunMode::Pause)
        {
            continue;
        }

        if (sendBuffer.size() <= 10)
        {
            if (pathManager.line < pathManager.total)
            {
                std::cout << "line : " << pathManager.line << ", total : " << pathManager.total << "\n";
                pathManager.PathLoopTask();
                std::cout << sendBuffer.size() << "\n";
                pathManager.line++;
            }
            else if (pathManager.line == pathManager.total)
            {
                std::cout << "Turn Back\n";
                pathManager.GetBackArr();
                pathManager.line++;
            }
            else if (sendBuffer.size() == 0)
            {
                std::cout << "Performance is Over\n";
                systemState.runMode = RunMode::PrePreparation;
                systemState.main = Main::Ideal;
            }
        }

        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);

        if (elapsed_time.count() >= 5000) // 5ms
        {
            external = std::chrono::system_clock::now();

            writeToSocket(tmotors, canUtils.sockets);

            if (!maxonMotors.empty())
            {
                writeToSocket(maxonMotors, canUtils.sockets);

                // sync 신호 전송
                frameToProcess = sendBuffer.front();
                sendBuffer.pop();
                auto it = canUtils.sockets.find(maxonMotors.begin()->second->interFaceName);

                if (it != canUtils.sockets.end())
                {
                    int socket_descriptor_for_sync = it->second;
                    ssize_t bytesWritten = write(socket_descriptor_for_sync, &frameToProcess, sizeof(struct can_frame));

                    handleError(bytesWritten, maxonMotors.begin()->second->interFaceName);
                }
                else
                {
                    std::cerr << "Socket not found for interface: " << maxonMotors.begin()->second->interFaceName << std::endl;
                }
            }
        }
    }

    // CSV 파일명 설정
    std::string csvFileName = "../../READ/DrumData_in.txt";

    // input 파일 저장
    save_to_txt_inputData(csvFileName);
}

void SendLoopTask::save_to_txt_inputData(const string &csv_file_name)
{
    // CSV 파일 열기
    std::ofstream csvFile(csv_file_name);

    if (!csvFile.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    csvFile << "0x007,0x001,0x002,0x003,0x004,0x005,0x006,0x008,0x009\n";

    // 2차원 벡터의 데이터를 CSV 파일로 쓰기
    for (const auto &row : pathManager.p)
    {
        for (const double cell : row)
        {
            csvFile << std::fixed << std::setprecision(5) << cell;
            if (&cell != &row.back())
            {
                csvFile << ","; // 쉼표로 셀 구분
            }
        }
        csvFile << "\n"; // 다음 행으로 이동
    }

    // CSV 파일 닫기
    csvFile.close();

    std::cout << "연주 txt_InData 파일이 생성되었습니다: " << csv_file_name << std::endl;

    std::cout << "SendLoop terminated\n";
}

void SendLoopTask::SendReadyLoop()
{
    cout << "Set Ready.\n";
    struct can_frame frameToProcess;
    chrono::system_clock::time_point external = std::chrono::system_clock::now();

    while (sendBuffer.size() != 0)
    {
        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);

        if (elapsed_time.count() >= 5000) // 5ms
        {
            external = std::chrono::system_clock::now();

            writeToSocket(tmotors, canUtils.sockets);

            if (!maxonMotors.empty())
            {
                writeToSocket(maxonMotors, canUtils.sockets);

                // sync 신호 전송
                frameToProcess = sendBuffer.front();
                sendBuffer.pop();
                auto it = canUtils.sockets.find(maxonMotors.begin()->second->interFaceName);

                if (it != canUtils.sockets.end())
                {
                    int socket_descriptor_for_sync = it->second;
                    ssize_t bytesWritten = write(socket_descriptor_for_sync, &frameToProcess, sizeof(struct can_frame));

                    handleError(bytesWritten, maxonMotors.begin()->second->interFaceName);
                }
                else
                {
                    std::cerr << "Socket not found for interface: " << maxonMotors.begin()->second->interFaceName << std::endl;
                }
            }
        }
    }
    canUtils.clear_all_can_buffers();
}

void SendLoopTask::initializePathManager()
{
    pathManager.motorInitialize(tmotors, maxonMotors);
    pathManager.GetDrumPositoin();
    pathManager.GetMusicSheet();
}

bool SendLoopTask::CheckAllMotorsCurrentPosition()
{
    std::cout << "Checking all positions for motors" << endl;
    bool allMotorsChecked = true;
    for (const auto &motor_pair : tmotors)
    {
        std::shared_ptr<TMotor> motor = motor_pair.second;
        bool motorChecked = CheckTmotorPosition(motor);
        if (!motorChecked)
        {
            cerr << "Failed to check position for motor: " << motor_pair.first << endl;
            allMotorsChecked = false;
        }
    }
    for (const auto &motor_pair : maxonMotors)
    {
        std::shared_ptr<MaxonMotor> motor = motor_pair.second;
        bool motorChecked = CheckMaxonPosition(motor);
        if (!motorChecked)
        {
            cerr << "Failed to check position for motor: " << motor_pair.first << endl;
            allMotorsChecked = false;
        }
    }

    std::cout << "Press Enter to Move On" << endl;
    return allMotorsChecked;
}

bool SendLoopTask::CheckTmotorPosition(std::shared_ptr<TMotor> motor)
{
    struct can_frame frame;
    fillCanFrameFromInfo(&frame, motor->getCanFrameForControlMode());
    canUtils.set_all_sockets_timeout(0, 5000 /*5ms*/);

    canUtils.clear_all_can_buffers();
    auto interface_name = motor->interFaceName;

    // canUtils.restart_all_can_ports();
    if (canUtils.sockets.find(interface_name) != canUtils.sockets.end())
    {
        int socket_descriptor = canUtils.sockets.at(interface_name);
        ssize_t bytesWritten = write(socket_descriptor, &frame, sizeof(can_frame));

        if (bytesWritten == -1)
        {
            cerr << "Failed to write to socket for motor: " << motor->nodeId << " (" << interface_name << ")" << endl;
            cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << endl;
            return false;
        }

        ssize_t bytesRead = read(socket_descriptor, &frame, sizeof(can_frame));
        if (bytesRead == -1)
        {
            cerr << "Failed to read from socket for motor: " << motor->nodeId << " (" << interface_name << ")" << endl;
            cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << endl;
            return false;
        }

        std::tuple<int, float, float, float> parsedData = TParser.parseRecieveCommand(*motor, &frame);
        motor->currentPos = std::get<1>(parsedData);
        std::cout << "Current Position of [" << std::hex << motor->nodeId << std::dec << "] : " << motor->currentPos << endl;
        return true;
    }
    else
    {
        cerr << "Socket not found for interface: " << interface_name << " (" << motor->nodeId << ")" << endl;
        return false;
    }
}

bool SendLoopTask::CheckMaxonPosition(std::shared_ptr<MaxonMotor> motor)
{

    struct can_frame frame;
    fillCanFrameFromInfo(&frame, motor->getCanFrameForSync());
    canUtils.set_all_sockets_timeout(0, 5000 /*5ms*/);

    canUtils.clear_all_can_buffers();
    auto interface_name = motor->interFaceName;

    if (canUtils.sockets.find(interface_name) != canUtils.sockets.end())
    {
        int socket_descriptor = canUtils.sockets.at(interface_name);
        ssize_t bytesWritten = write(socket_descriptor, &frame, sizeof(can_frame));

        if (bytesWritten == -1)
        {
            cerr << "Failed to write to socket for motor: " << motor->nodeId << " (" << interface_name << ")" << endl;
            cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << endl;
            return false;
        }

        ssize_t bytesRead = read(socket_descriptor, &frame, sizeof(can_frame));
        if (bytesRead == -1)
        {
            cerr << "Failed to read from socket for motor: " << motor->nodeId << " (" << interface_name << ")" << endl;
            cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << endl;
            return false;
        }

        std::tuple<int, float, float> parsedData = MParser.parseRecieveCommand(*motor,&frame);
        motor->currentPos = std::get<1>(parsedData);
        std::cout << "Current Position of [" << std::hex << motor->nodeId << std::dec << "] : " << motor->currentPos << endl;
        return true;
    }
    else
    {
        cerr << "Socket not found for interface: " << interface_name << " (" << motor->nodeId << ")" << endl;
        return false;
    }
}
