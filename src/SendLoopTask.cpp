#include "../include/SendLoopTask.hpp"

SendLoopTask::SendLoopTask(SystemState &systemStateRef, CanSocketUtils &canUtilsRef)
    : systemState(systemStateRef), canUtils(canUtilsRef)
{
}

// SendLoopTask 클래스의 operator() 함수
void SendLoopTask::operator()()
{
    while (systemState.main != Main::Shutdown)
    {
        switch (systemState.main.load())
        {
        case Main::SystemInit:
            initializeTMotors();
            initializeCanUtils();
            ActivateControlTask();
            systemState.main = Main::Home; // 작업 완료 후 상태 변경
            break;

        case Main::Home:

            break;

        case Main::Tune:
            // Tuning 상태에서의 동작
            break;

        case Main::Perform:
            // Performing 상태에서의 동작
            break;

            // ... 기타 상태에 따른 동작 ...

        default:
            // 기본적인 동작이나 대기 로직
            break;
        }
    }
}

/////////////////////////// [ SYSTEM INITIALIZATION ] ///////////////////////////

void SendLoopTask::initializeTMotors()
{
    tmotors["waist"] = make_shared<TMotor>(0x007, "AK10_9", "can0");

    tmotors["R_arm1"] = make_shared<TMotor>(0x001, "AK70_10", "can0");
    tmotors["L_arm1"] = make_shared<TMotor>(0x002, "AK70_10", "can0");
    tmotors["R_arm2"] = make_shared<TMotor>(0x003, "AK70_10", "can0");

    tmotors["R_arm3"] = make_shared<TMotor>(0x004, "AK70_10", "can0");
    tmotors["L_arm2"] = make_shared<TMotor>(0x005, "AK70_10", "can0");
    tmotors["L_arm3"] = make_shared<TMotor>(0x006, "AK70_10", "can0");

    map<string, shared_ptr<MaxonMotor>> maxonMotors;
    /*maxonMotors["a_maxon"] = make_shared<MaxonMotor>(0x001,
                                                          vector<uint32_t>{0x201, 0x301},
                                                          vector<uint32_t>{0x181},
                                                          "can0");
    maxonMotors["b_maxon"] = make_shared<MaxonMotor>(0x002,
                                                          vector<uint32_t>{0x202, 0x302},
                                                          vector<uint32_t>{0x182},
                                                          "can0");*/
};

void SendLoopTask::initializeCanUtils()
{
    canUtils = CanSocketUtils(extractIfnamesFromMotors(tmotors));
}

vector<string> SendLoopTask::extractIfnamesFromMotors(const map<string, shared_ptr<TMotor>, CustomCompare> &motors)
{
    set<string> interface_names;
    for (const auto &motor_pair : motors)
    {
        interface_names.insert(motor_pair.second->interFaceName);
    }
    return vector<string>(interface_names.begin(), interface_names.end());
}

void SendLoopTask::ActivateControlTask()
{
    struct can_frame frame;

    canUtils.set_all_sockets_timeout(5 /*sec*/, 0);
    canUtils.clear_all_can_buffers();

    if (!tmotors.empty())
    {
        // 첫 번째 for문: 모터 상태 확인 및 제어 모드 설정
        for (auto it = tmotors.begin(); it != tmotors.end();)
        {
            std::string name = it->first;
            std::shared_ptr<TMotor> motor = it->second;

            bool success = true;

            // 상태 확인
            fillCanFrameFromInfo(&frame, motor->getCanFrameForCheckMotor());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [&success](const std::string &motorName, bool result)
                           {
                               if (result)
                               {
                                   std::cout << "Motor [" << motorName << "] status check passed." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Motor [" << motorName << "] status check failed." << std::endl;
                                   success = false;
                               }
                           });

            if (!success)
            {
                // 실패한 경우, 해당 모터를 배열에서 제거하고 다음 모터로 넘어감
                it = tmotors.erase(it);
                continue;
            }

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForControlMode());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "Control mode set for motor [" << motorName << "]." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Failed to set control mode for motor [" << motorName << "]." << std::endl;
                               }
                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForZeroing());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "Zero set for motor [" << motorName << "]." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Failed to set zero for motor [" << motorName << "]." << std::endl;
                               }
                           });

            // 구분자 추가
            std::cout << "=======================================" << std::endl;
            ++it;
        }
    }
    else
    {
        std::cout << "No Tmotors to process." << std::endl;
    }

    // MaxonMotor
    if (!maxonMotors.empty())
    {
        for (const auto &motorPair : maxonMotors)
        {
            std::string name = motorPair.first;
            std::shared_ptr<MaxonMotor> motor = motorPair.second;

            // 상태 확인
            fillCanFrameFromInfo(&frame, motor->getCanFrameForCheckMotor());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "Motor [" << motorName << "] status check passed." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Motor [" << motorName << "] status check failed." << std::endl;
                               }
                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForControlMode());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForPosOffset());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });
            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForTorqueOffset());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForOperational());
            sendNotRead(canUtils.sockets.at(motor->interFaceName), name, frame,
                        [](const std::string &motorName, bool success) {

                        });

            fillCanFrameFromInfo(&frame, motor->getCanFrameForEnable());
            sendNotRead(canUtils.sockets.at(motor->interFaceName), name, frame,
                        [](const std::string &motorName, bool success)
                        {
                            if (success)
                            {
                                std::cout << "Enabled for motor [" << motorName << "]." << std::endl;
                            }
                            else
                            {
                                std::cerr << "Failed to Enable for motor [" << motorName << "]." << std::endl;
                            }
                        });

            fillCanFrameFromInfo(&frame, motor->getCanFrameForSync());
            writeAndReadForSync(canUtils.sockets.at(motor->interFaceName), name, frame, maxonMotors.size(),
                                [](const std::string &motorName, bool success) {

                                });

            // 구분자 추가
            std::cout << "=======================================" << std::endl;
        }
    }
    else
    {
        std::cout << "No Maxon motors to process." << std::endl;
    }
}

/////////////////////////// [ HOME ] ///////////////////////////

void SendLoopTask::CheckCurrentPosition(std::shared_ptr<TMotor> motor)
{
    struct can_frame frame;

    canUtils.set_all_sockets_timeout(0, 5000 /*5ms*/);
    canUtils.clear_all_can_buffers();
    auto interface_name = motor->interFaceName;

    // 상태 확인
    fillCanFrameFromInfo(&frame, motor->getCanFrameForControlMode());
    if (canUtils.sockets.find(interface_name) != canUtils.sockets.end())
    {
        int socket_descriptor = canUtils.sockets.at(interface_name);
        ssize_t bytesWritten = write(socket_descriptor, &frame, sizeof(can_frame));

        if (bytesWritten == -1)
        {
            std::cerr << "Failed to write to socket for interface: " << interface_name << std::endl;
            std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
        }

        usleep(5000);

        ssize_t bytesRead = read(socket_descriptor, &frame, sizeof(can_frame));

        if (bytesRead == -1)
        {
            std::cerr << "Failed to read to socket for interface: " << interface_name << std::endl;
            std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
        }

        std::tuple<int, float, float, float> parsedData = TParser.parseRecieveCommand(*motor, &frame);
        motor->currentPos = std::get<1>(parsedData);

        cout << "Current Position of "
             << "[" << std::hex << motor->nodeId << std::dec << "] : " << motor->currentPos << endl;
    }
    else
    {
        std::cerr << "Socket not found for interface: " << interface_name << std::endl;
    }
}

void SendLoopTask::FixMotorPosition()
{
    struct can_frame frame;
    for (const auto &motorPair : tmotors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<TMotor> motor = motorPair.second;
        CheckCurrentPosition(motor);

        TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, motor->currentPos, 0, 250, 1, 0);
        sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                       [](const std::string &motorName, bool success)
                       {
                           if (success)
                           {
                               std::cout << "Position fixed for motor [" << motorName << "]." << std::endl;
                           }
                           else
                           {
                               std::cerr << "Failed to fix position for motor [" << motorName << "]." << std::endl;
                           }
                       });
    }
}

void SendLoopTask::SendCommandToMotor(std::shared_ptr<TMotor> &motor, struct can_frame &frame, const std::string &motorName)
{
    auto interface_name = motor->interFaceName;
    if (canUtils.sockets.find(interface_name) != canUtils.sockets.end())
    {
        int socket_descriptor = canUtils.sockets.at(interface_name);

        // 명령을 소켓으로 전송합니다.
        ssize_t bytesWritten = write(socket_descriptor, &frame, sizeof(struct can_frame));
        if (bytesWritten == -1)
        {
            std::cerr << "Failed to write to socket for interface: " << interface_name << std::endl;
            std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
            return;
        }

        // 명령에 대한 응답을 기다립니다.
        ssize_t bytesRead = read(socket_descriptor, &frame, sizeof(struct can_frame));
        if (bytesRead == -1)
        {
            std::cerr << "Failed to read from socket for interface: " << interface_name << std::endl;
            std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
        }
    }
    else
    {
        std::cerr << "Socket not found for interface: " << interface_name << std::endl;
    }
}
bool SendLoopTask::PromptUserForHoming(const std::string &motorName)
{
    char userResponse;
    std::cout << "Would you like to start homing mode for motor [" << motorName << "]? (y/n): ";
    std::cin >> userResponse;
    return userResponse == 'y';
}

void SendLoopTask::RotateMotor(std::shared_ptr<TMotor> &motor, const std::string &motorName, double direction)
{
    struct can_frame frameToProcess;
    const double targetRadian = M_PI / 2 * direction;
    int totalSteps = 8000 / 5; // 8초 동안 5ms 간격으로 나누기

    auto startTime = std::chrono::system_clock::now();
    for (int step = 0; step < totalSteps; ++step)
    {
        auto currentTime = std::chrono::system_clock::now();
        while (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count() < 5)
        {
            // 5ms가 되기 전까지 기다림
            currentTime = std::chrono::system_clock::now();
        }

        // 5ms마다 목표 위치 계산 및 프레임 전송
        double targetPosition = targetRadian * (static_cast<double>(step) / totalSteps);

        TParser.parseSendCommand(*motor, &frameToProcess, motor->nodeId, 8, targetPosition, 0, 50, 1, 0);
        SendCommandToMotor(motor, frameToProcess, motorName);

        // 다음 5ms 간격을 위해 시작 시간 업데이트
        startTime = std::chrono::system_clock::now();
    }
}

void SendLoopTask::SetHome()
{
    struct can_frame frameToProcess;
    ActivateSensor();

    // 각 모터의 방향 설정
    std::map<std::string, double> directionSettings = {
        {"R_arm1", 1.0},
        {"L_arm1", 1.0},
        {"R_arm2", 1.0},
        {"R_arm3", 1.0},
        {"L_arm2", -1.0},
        {"L_arm3", -1.0}};

    canUtils.set_all_sockets_timeout(5/*sec*/, 0);

    for (auto &motor_pair : tmotors)
    {
        if (motor_pair.first == "waist")
            continue;
        std::shared_ptr<TMotor> &motor = motor_pair.second;

        if (!PromptUserForHoming(motor_pair.first)) // 사용자에게 홈 설정을 묻는 함수
            continue;

        double initialDirection = 0.2 * directionSettings[motor_pair.first];

        double additionalTorque = (motor_pair.first == "L_arm2" || motor_pair.first == "R_arm2") ? 1 : 0;
        TParser.parseSendCommand(*motor, &frameToProcess, motor->nodeId, 8, 0, initialDirection, 0, 4.5, additionalTorque);
        SendCommandToMotor(motor, frameToProcess, motor_pair.first);

        MoveMotorToSensorLocation(motor, motor_pair.first); // 모터를 센서 위치까지 이동시키는 함수

        cout << "\nPress Enter to move to Home Position\n";
        getchar();

        RotateMotor(motor, motor_pair.first, -directionSettings[motor_pair.first]);

        cout << "----------------------moved 90 degree (Anti clock wise) --------------------------------- \n";
        // 모터를 멈추는 신호를 보냄
        TParser.parseSendCommand(*motor, &frameToProcess, motor->nodeId, 8, 0, 0, 0, 5, 0);
        SendCommandToMotor(motor, frameToProcess, motor_pair.first);

        // 그 상태에서 setzero 명령을 보냄(현재 position을 0으로 인식)
        fillCanFrameFromInfo(&frameToProcess, motor->getCanFrameForZeroing());
        SendCommandToMotor(motor, frameToProcess, motor_pair.first);

        // 상태 확인
        fillCanFrameFromInfo(&frameToProcess, motor->getCanFrameForCheckMotor());
        SendCommandToMotor(motor, frameToProcess, motor_pair.first);

        if (motor_pair.first == "L_arm1" || motor_pair.first == "R_arm1")
        {
            RotateMotor(motor, motor_pair.first, directionSettings[motor_pair.first]);
        }
    }

    cout << "All in Home\n";
    DeactivateSensor();
}
//