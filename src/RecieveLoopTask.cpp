#include "../include/RecieveLoopTask.hpp"

RecieveLoopTask::RecieveLoopTask(SystemState &systemStateRef,
                                 CanSocketUtils &canUtilsRef,
                                 std::map<std::string, std::shared_ptr<TMotor>> &tmotorsRef,
                                 std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotorsRef,
                                 queue<can_frame> &recieveBufferRef) : systemState(systemStateRef), canUtils(canUtilsRef), tmotors(tmotorsRef), maxonMotors(maxonMotorsRef), recieveBuffer(recieveBufferRef)
{
}

void RecieveLoopTask::operator()()
{
    auto lastCheckTime = std::chrono::steady_clock::now();

    while (systemState.main != Main::Shutdown)
    {
        auto currentTime = std::chrono::steady_clock::now();
        usleep(50000);
        if (std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastCheckTime).count() >= 3)
        {

            canUtils.checkCanPortsStatus();
            checkMotors();
            lastCheckTime = currentTime; // 마지막 체크 시간 업데이트
        }
        while (systemState.main == Main::Perform)
        {

            usleep(50000); // Perform 상태일 때의 처리

            if (systemState.runMode == RunMode::Running)
            {
                RecieveLoop(recieveBuffer);
            }
        }
    }
}

void RecieveLoopTask::RecieveLoop(queue<can_frame> &recieveBuffer)
{
    chrono::system_clock::time_point external = std::chrono::system_clock::now();

    canUtils.set_all_sockets_timeout(0, 50000);
    canUtils.clear_all_can_buffers();

    sensor.connect();
    if (!sensor.connected)
    {
        cout << "Sensor initialization failed. Skipping sensor related logic." << endl;
    }

    while (systemState.runMode != RunMode::PrePreparation)
    {

        if (systemState.runMode == RunMode::Pause)
        {
            continue;
        }

        /*if (sensor.connected && (sensor.ReadVal() & 1) != 0)
        {
            cout << "Motors at Sensor Location please check!!!\n";
            systemState.runMode = RunMode::Pause;
        }*/

        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::milliseconds elapsed_time = chrono::duration_cast<chrono::milliseconds>(internal - external);
        if (elapsed_time.count() >= TIME_THRESHOLD_MS)
        {
            external = std::chrono::system_clock::now();
            for (const auto &socket_pair : canUtils.sockets)
            {
                int socket_descriptor = socket_pair.second;

                handleSocketRead(socket_descriptor, recieveBuffer);
            }
        }
    }

    parse_and_save_to_csv("../../READ/DrumData_out.txt");
}

void RecieveLoopTask::handleSocketRead(int socket_descriptor, queue<can_frame> &recieveBuffer)
{
    struct can_frame readFrame[NUM_FRAMES];

    ssize_t bytesRead = read(socket_descriptor, &readFrame, sizeof(can_frame) * NUM_FRAMES);
    if (bytesRead == -1)
    {
        std::cerr << "Failed to read from socket." << std::endl;
        return;
    }

    int numFramesRead = bytesRead / sizeof(can_frame); // 중복 연산 최소화
    for (int i = 0; i < numFramesRead; ++i)
    {
        recieveBuffer.push(readFrame[i]);
    }
}

void RecieveLoopTask::parse_and_save_to_csv(const std::string &csv_file_name)
{
    // CSV 파일 열기. 파일이 없으면 새로 생성됩니다.
    std::ofstream ofs(csv_file_name, std::ios::app);
    int id;
    float position, speed, torque;

    if (!ofs.is_open())
    {
        std::cerr << "Failed to open or create the CSV file: " << csv_file_name << std::endl;
        return;
    }

    // 파일이 새로 생성되었으면 CSV 헤더를 추가
    ofs.seekp(0, std::ios::end);
    if (ofs.tellp() == 0)
    {
        ofs << "CAN_ID,p_act,tff_des,tff_act\n";
    };

    while (!recieveBuffer.empty())
    {
        can_frame frame = recieveBuffer.front();
        recieveBuffer.pop();

        for (const auto &pair : tmotors)
        {
            std::shared_ptr<TMotor> motor = pair.second;
            if (motor->nodeId == frame.data[0])
            {
                std::tuple<int, float, float, float> parsedData = TParser.parseRecieveCommand(*motor, &frame);
                id = std::get<0>(parsedData);
                position = std::get<1>(parsedData);
                speed = std::get<2>(parsedData);
                torque = std::get<3>(parsedData);

                ofs << "0x" << std::hex << std::setw(4) << std::setfill('0') << id << ","
                    << std::dec
                    << position << "," << speed << "," << torque << "\n";
            }
        }
        for (auto &entry : maxonMotors)
        {
            std::shared_ptr<MaxonMotor> motor = entry.second;
            if (motor->nodeId == frame.data[0])
            {
                std::tuple<int, float, float> parsedData = MParser.parseRecieveCommand(*motor, &frame);
                id = std::get<0>(parsedData);
                position = std::get<1>(parsedData);
                torque = std::get<2>(parsedData);

                ofs << "0x" << std::hex << std::setw(4) << std::setfill('0') << id << ","
                    << std::dec
                    << position << ","
                    << " "
                    << "," << torque << "\n";
            }
        }
    }

    ofs.close();

    std::cout << "연주 txt_OutData 파일이 생성되었습니다: " << csv_file_name << std::endl;
}

void RecieveLoopTask::checkMotors()
{

    for (const auto &motor_pair : tmotors)
    {
        std::shared_ptr<TMotor> motor = motor_pair.second;
        bool motorChecked = checkTmotors(motor);
        if (!motorChecked)
        {
            cerr << "Failed to check position for motor: " << motor_pair.first << endl;
            motor->isConected = false;
        }
    }
    for (const auto &motor_pair : maxonMotors)
    {
        std::shared_ptr<MaxonMotor> motor = motor_pair.second;
        bool motorChecked = checkMmotors(motor);
        if (!motorChecked)
        {
            cerr << "Failed to check position for motor: " << motor_pair.first << endl;
            motor->isConected = false;
        }
    }
}

bool RecieveLoopTask::checkTmotors(std::shared_ptr<TMotor> motor)
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
        return true;
    }
    else
    {
        cerr << "Socket not found for interface: " << interface_name << " (" << motor->nodeId << ")" << endl;
        return false;
    }
}

bool RecieveLoopTask::checkMmotors(std::shared_ptr<MaxonMotor> motor)
{

    struct can_frame frame;
    fillCanFrameFromInfo(&frame, motor->getCanFrameForCheckMotor());
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

        return true;
    }
    else
    {
        cerr << "Socket not found for interface: " << interface_name << " (" << motor->nodeId << ")" << endl;
        return false;
    }
}