#include "../include/SendLoopTask.hpp"

SendLoopTask::SendLoopTask(SystemState &systemStateRef, CanSocketUtils &canUtilsRef)
    : systemState(systemStateRef), canUtils(canUtilsRef), pathManager(sendBuffer, tmotors)
{
}

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
            if (systemState.homeMode == HomeMode::Homing)
            {
                SetHome();
                systemState.homeMode = HomeMode::HomeReady;
            }
            break;

        case Main::Tune:
            TuningLoopTask();
            systemState.main = Main::Home;
            break;

        case Main::Perform:
            SendLoop();
            systemState.main = Main::Home;
            break;

        case Main::Ready:
            if (CheckAllMotorsCurrentPosition())
            {
                pathManager.GetReadyArr();
            };
            SendReadyLoop();
            systemState.homeMode = HomeMode::PosReady;
            systemState.main = Main::Home;
            break;
        case Main::Shutdown:
            std::cout << "======= Shut down system =======\n";
            break;
        }
    }
    DeactivateControlTask();
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 SYSTEM                                     */
///////////////////////////////////////////////////////////////////////////////

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
    canUtils.startCAN(extractIfnamesFromMotors(tmotors));
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

    canUtils.set_all_sockets_timeout(0, 10000);
    if (!tmotors.empty())
    {
        // 첫 번째 단계: 모터 상태 확인 (10ms 타임아웃)

        for (auto it = tmotors.begin(); it != tmotors.end();)
        {
            std::string name = it->first;
            std::shared_ptr<TMotor> motor = it->second;

            bool checkSuccess = true;

            // 상태 확인
            fillCanFrameFromInfo(&frame, motor->getCanFrameForCheckMotor());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [&checkSuccess](const std::string &motorName, bool result)
                           {
                               if (!result)
                               {
                                   std::cerr << "Motor [" << motorName << "] Not Connected." << std::endl;
                                   checkSuccess = false;
                               }
                           });

            if (!checkSuccess)
            {
                // 실패한 경우, 해당 모터를 배열에서 제거
                it = tmotors.erase(it);
                canUtils.clear_all_can_buffers();
                continue;
            }
            else
            {
                ++it;
            }
        }

        // 두 번째 단계: 제어 모드 설정과 제로 설정 (5초 타임아웃)
        for (const auto &socketPair : canUtils.sockets)
        {
            int hsocket = socketPair.second;
            if (set_socket_timeout(hsocket, 5, 0) != 0)
            {
                // 타임아웃 설정 실패 처리
                std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
            }
        }
        for (const auto &motorPair : tmotors)
        {
            std::string name = motorPair.first;
            std::shared_ptr<TMotor> motor = motorPair.second;

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForControlMode());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (!success)
                               {
                                   std::cerr << "Failed to set control mode for motor [" << motorName << "]." << std::endl;
                               }
                           });

            // 제로 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForZeroing());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (!success)
                               {
                                   std::cerr << "Failed to set zero for motor [" << motorName << "]." << std::endl;
                               }
                           });

            // 구분자 추가
            std::cout << "=======================================" << std::endl;
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

    // Sensor 동작 확인
    Sensor sensor;
}

void SendLoopTask::DeactivateControlTask()
{
    struct can_frame frame;

    canUtils.set_all_sockets_timeout(0, 50000);
    // Tmotors
    if (!tmotors.empty())
    {
        for (const auto &motorPair : tmotors)
        {
            std::string name = motorPair.first;
            std::shared_ptr<TMotor> motor = motorPair.second;

            fillCanFrameFromInfo(&frame, motor->getCanFrameForCheckMotor());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });

            fillCanFrameFromInfo(&frame, motor->getCanFrameForExit());

            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "Exiting control mode for motor [" << motorName << "]." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Failed to exit control mode for motor [" << motorName << "]." << std::endl;
                               }
                           });

            // 구분자 추가
            std::cout << "=======================================" << std::endl;
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

            fillCanFrameFromInfo(&frame, motor->getCanFrameForQuickStop());
            sendNotRead(canUtils.sockets.at(motor->interFaceName), name, frame,
                        [](const std::string &motorName, bool success)
                        {
                            if (success)
                            {
                                std::cout << "Exiting for motor [" << motorName << "]." << std::endl;
                            }
                            else
                            {
                                std::cerr << "Failed to exit for motor [" << motorName << "]." << std::endl;
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

/////////////////////////////////////////////////////////////////////////////////
/*                                  HOME                                      */
///////////////////////////////////////////////////////////////////////////////

bool SendLoopTask::CheckCurrentPosition(std::shared_ptr<TMotor> motor)
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

void SendLoopTask::RotateMotor(std::shared_ptr<TMotor> &motor, const std::string &motorName, double direction, double degree, float midpoint)
{
    struct can_frame frameToProcess;
    CheckCurrentPosition(motor);
    // 수정된 부분: 사용자가 입력한 각도를 라디안으로 변환
    const double targetRadian = (degree * M_PI / 180.0) * direction + midpoint; // 사용자가 입력한 각도를 라디안으로 변환 + midpoint
    int totalSteps = 8000 / 5;                                                  // 8초 동안 5ms 간격으로 나누기

    auto startTime = std::chrono::system_clock::now();
    for (int step = 0; step < totalSteps; ++step)
    {
        auto currentTime = std::chrono::system_clock::now();
        while (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count() < 5)
        {
            currentTime = std::chrono::system_clock::now();
        }

        // 5ms마다 목표 위치 계산 및 프레임 전송
        double targetPosition = targetRadian * (static_cast<double>(step) / totalSteps) + motor->currentPos;
        TParser.parseSendCommand(*motor, &frameToProcess, motor->nodeId, 8, targetPosition, 0, 50, 1, 0);
        SendCommandToMotor(motor, frameToProcess, motorName);

        startTime = std::chrono::system_clock::now();
    }
}

struct MotorSettings
{
    double direction;
    int sensorBit;
};

void SendLoopTask::SetHome()
{
    struct can_frame frameToProcess;

    // 각 모터의 방향 및 센서 비트 설정
    std::map<std::string, MotorSettings> motorSettings = {
        {"R_arm1", {1.0, 0}}, // 예시: R_arm1 모터의 방향 1.0, 센서 비트 0
        {"L_arm1", {1.0, 1}},
        {"R_arm2", {1.0, 2}},
        {"R_arm3", {1.0, 3}},
        {"L_arm2", {-1.0, 4}},
        {"L_arm3", {-1.0, 5}},
        // ... 다른 모터 설정 ...
    };

    canUtils.set_all_sockets_timeout(5 /*sec*/, 0);

    for (auto &motor_pair : tmotors)
    {
        // 허리는 home 안잡음
        if (motor_pair.first == "waist")
            continue;

        std::shared_ptr<TMotor> &motor = motor_pair.second;
        MotorSettings &settings = motorSettings[motor_pair.first];
        if (!PromptUserForHoming(motor_pair.first)) // 사용자에게 홈 설정을 묻는 함수
            continue;

        double initialDirection = 0.2 * settings.direction;

        double additionalTorque = (motor_pair.first == "L_arm2" || motor_pair.first == "R_arm2") ? settings.direction * 1 : 0;
        TParser.parseSendCommand(*motor, &frameToProcess, motor->nodeId, 8, 0, initialDirection, 0, 4.5, additionalTorque);
        SendCommandToMotor(motor, frameToProcess, motor_pair.first);

        float midpoint = MoveMotorToSensorLocation(motor, motor_pair.first, settings.sensorBit); // 모터를 센서 위치까지 이동시키는 함수, 센서 비트 전달
                                                                                                 // 모터를 센서 위치까지 이동시키는 함수

        std::cout << "\nPress Enter to move to Home Position\n";
        getchar();

        RotateMotor(motor, motor_pair.first, -settings.direction, 90, midpoint);

        std::cout << "----------------------moved 90 degree (Anti clock wise) --------------------------------- \n";
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
            RotateMotor(motor, motor_pair.first, settings.direction, 90, 0);
        }
    }

    std::cout << "All in Home\n";
}

float SendLoopTask::MoveMotorToSensorLocation(std::shared_ptr<TMotor> &motor, const std::string &motorName, int sensorBit)
{
    float firstPosition = 0.0f, secondPosition = 0.0f;
    bool firstSensorTriggered = false;
    bool secondSensorTriggered = false;
    Sensor sensor;

    std::cout << "Moving " << motorName << " to sensor location.\n";

    while (true)
    {

        bool sensorTriggered = ((sensor.ReadVal() >> sensorBit) & 1) != 0;

        if (!firstSensorTriggered && sensorTriggered)
        {
            // 첫 번째 센서 인식
            firstSensorTriggered = true;
            CheckCurrentPosition(motor);
            firstPosition = motor->currentPos;
            std::cout << motorName << " first sensor position: " << firstPosition << endl;
        }
        else if (firstSensorTriggered && !sensorTriggered)
        {
            // 센서 인식 해제
            secondSensorTriggered = true;
            CheckCurrentPosition(motor);
            secondPosition = motor->currentPos;
            std::cout << motorName << " second sensor position: " << secondPosition << endl;

            break; // while문 탈출
        }

        if (secondSensorTriggered)
            break; // 두 번째 센서 인식 후 반복문 탈출
    }

    // 1번과 2번 위치의 차이의 절반을 저장
    float positionDifference = (secondPosition - firstPosition) / 2.0f;
    std::cout << motorName << " midpoint position: " << positionDifference << endl;

    return positionDifference;
}

/////////////////////////////////////////////////////////////////////////////////
/*                                  TUNE                                      */
///////////////////////////////////////////////////////////////////////////////

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

void SendLoopTask::Tuning(float kp, float kd, float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType)
{
    canUtils.set_all_sockets_timeout(0, 50000);

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2); // 소수점 둘째 자리까지만
    ss << "kp_" << kp << "_kd_" << kd << "_Hz_" << 1 / sine_t;
    std::string parameter = ss.str();

    // CSV 파일명 설정
    std::string FileName1 = "../../READ/" + parameter + "_in.txt";

    // CSV 파일 열기
    std::ofstream csvFileIn(FileName1);

    if (!csvFileIn.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    csvFileIn << "Start file"
              << "\n";

    // CSV 파일명 설정
    std::string FileName2 = "../../READ/" + parameter + "_out.txt";

    // CSV 파일 열기
    std::ofstream csvFileOut(FileName2);

    if (!csvFileOut.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    csvFileOut << "CAN_ID,p_act,tff_des,tff_act\n"; // CSV 헤더

    struct can_frame frame;

    float peakRadian = peakAngle * M_PI / 180.0; // 피크 각도를 라디안으로 변환
    float amplitude = peakRadian;

    float sample_time = 0.005;
    int max_samples = static_cast<int>(sine_t / sample_time);
    float v_des = 0, p_des = 0;
    float tff_des = 0;
    float p_act, v_act, tff_act;
    for (int cycle = 0; cycle < cycles; cycle++)
    {
        for (int i = 0; i < max_samples; i++)
        {
            float time = i * sample_time;

            for (auto &entry : tmotors)
            {
                if (entry.first != selectedMotor)
                    continue;

                std::shared_ptr<TMotor> &motor = entry.second;

                if ((int)motor->nodeId == 7)
                {
                    csvFileIn << std::dec << p_des << "0,0,0,0,0,0";
                }
                else
                {
                    for (int i = 0; i < (int)motor->nodeId; i++)
                    {
                        csvFileIn << "0,";
                    }
                    csvFileIn << std::dec << p_des << ",";
                    for (int i = 0; i < (6 - (int)motor->nodeId); i++)
                    {
                        csvFileIn << "0,";
                    }
                }

                float local_time = std::fmod(time, sine_t);
                if (pathType == 1) // 1-cos 경로
                {
                    p_des = amplitude * (1 - cosf(2 * M_PI * local_time / sine_t)) / 2;
                }
                else if (pathType == 2) // 1-cos 및 -1+cos 결합 경로
                {
                    if (local_time < sine_t / 2)
                        p_des = amplitude * (1 - cosf(4 * M_PI * local_time / sine_t)) / 2;
                    else
                        p_des = amplitude * (-1 + cosf(4 * M_PI * (local_time - sine_t / 2) / sine_t)) / 2;
                }

                TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, v_des, kp, kd, tff_des);
                csvFileOut << "0x" << std::hex << std::setw(4) << std::setfill('0') << motor->nodeId;

                chrono::system_clock::time_point external = std::chrono::system_clock::now();
                while (1)
                {
                    chrono::system_clock::time_point internal = std::chrono::system_clock::now();
                    chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);
                    if (elapsed_time.count() >= 5000)
                    {

                        ssize_t bytesWritten = write(canUtils.sockets.at(motor->interFaceName), &frame, sizeof(struct can_frame));
                        if (bytesWritten == -1)
                        {
                            std::cerr << "Failed to write to socket for interface: " << motor->interFaceName << std::endl;
                            std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
                        }

                        ssize_t bytesRead = read(canUtils.sockets.at(motor->interFaceName), &frame, sizeof(struct can_frame));

                        if (bytesRead == -1)
                        {
                            std::cerr << "Failed to read from socket for interface: " << motor->interFaceName << std::endl;
                            return;
                        }
                        else
                        {
                            std::tuple<int, float, float, float> result = TParser.parseRecieveCommand(*motor, &frame);

                            p_act = std::get<1>(result);
                            v_act = std::get<1>(result);
                            tff_act = std::get<3>(result);
                            tff_des = kp * (p_des - p_act) + kd * (v_des - v_act);
                            csvFileOut << ',' << std::dec << p_act << ',' << tff_des << ',' << tff_act << '\n';
                            break;
                        }
                    }
                }
            }
            csvFileIn << "\n";
        }
    }
    csvFileIn.close();
    csvFileOut.close();
}

void SendLoopTask::TuningLoopTask()
{
    FixMotorPosition();
    std::string userInput, selectedMotor, fileName;
    float kp, kd, peakAngle;
    float sine_t = 4.0;
    int cycles = 2, pathType;

    if (!tmotors.empty())
    {
        selectedMotor = tmotors.begin()->first;
    }

    InitializeTuningParameters(selectedMotor, kp, kd, peakAngle, pathType);
    while (true)
    {
        int result = system("clear");
        if (result != 0)
        {
            std::cerr << "Error during clear screen" << std::endl;
        }

        std::string pathTypeDescription;
        if (pathType == 1)
        {
            pathTypeDescription = "1: 1 - cos (0 -> peak -> 0)";
        }
        else if (pathType == 2)
        {
            pathTypeDescription = "2: 1 - cos & cos - 1 (0 -> peak -> 0 -> -peak -> 0)";
        }

        std::cout << "================ Tuning Menu ================\n";
        std::cout << "Available Motors:\n";
        for (const auto &motor_pair : tmotors)
        {
            std::cout << " - " << motor_pair.first << "\n";
        }
        std::cout << "---------------------------------------------\n";
        std::cout << "Selected Motor: " << selectedMotor << "\n";
        std::cout << "Kp: " << kp << " | Kd: " << kd << "\n";
        std::cout << "Sine Period: " << sine_t << " | Cycles: " << cycles << " | Hz: " << 1 / sine_t << "\n";
        std::cout << "Peak Angle: " << peakAngle << " | Path Type: " << pathTypeDescription << "\n";
        std::cout << "\nCommands:\n";
        std::cout << "[S]elect Motor | [KP] | [KD] | [Peak] | [Type]\n";
        std::cout << "[P]eriod | [C]ycles | [R]un | [A]nalyze | [E]xit\n";
        std::cout << "=============================================\n";
        std::cout << "Enter Command: ";
        std::cin >> userInput;
        std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);

        if (userInput[0] == 'e')
        {
            break;
        }
        else if (userInput[0] == 's')
        {
            while (true)
            {
                std::cout << "Enter the name of the motor to tune: ";
                std::cin >> selectedMotor;
                if (tmotors.find(selectedMotor) != tmotors.end())
                {
                    InitializeTuningParameters(selectedMotor, kp, kd, peakAngle, pathType);
                    break;
                }
                else
                {
                    std::cout << "Invalid motor name. Please enter a valid motor name.\n";
                }
            }
        }
        else if (userInput == "kp")
        {
            std::cout << "Enter Desired Kp: ";
            std::cin >> kp;
        }
        else if (userInput == "kd")
        {
            std::cout << "Enter Desired Kd: ";
            std::cin >> kd;
        }
        else if (userInput == "peak")
        {
            std::cout << "Enter Desired Peak Angle: ";
            std::cin >> peakAngle;
        }
        else if (userInput == "type")
        {
            std::cout << "Select Path Type:\n";
            std::cout << "1: 1 - cos (0 -> peak -> 0)\n";
            std::cout << "2: 1 - cos & cos - 1 (0 -> peak -> 0 -> -peak -> 0)\n";
            std::cout << "Enter Path Type (1 or 2): ";
            std::cin >> pathType;

            if (pathType != 1 && pathType != 2)
            {
                std::cout << "Invalid path type. Please enter 1 or 2.\n";
                pathType = 1; // 기본값으로 재설정
            }
        }
        else if (userInput[0] == 'p')
        {
            std::cout << "Enter Desired Sine Period: ";
            std::cin >> sine_t;
        }
        else if (userInput[0] == 'c')
        {
            std::cout << "Enter Desired Cycles: ";
            std::cin >> cycles;
        }
        else if (userInput[0] == 'r')
        {
            std::cout << "In run mode of Tune\n";
            Tuning(kp, kd, sine_t, selectedMotor, cycles, peakAngle, pathType);
        }
    }
}

void SendLoopTask::InitializeTuningParameters(const std::string selectedMotor, float &kp, float &kd, float &peakAngle, int &pathType)
{
    if (selectedMotor == "waist")
    {
        kp = 200.0;
        kd = 1.0;
        peakAngle = 30;
        pathType = 2;
    }
    else if (selectedMotor == "R_arm1" || selectedMotor == "L_arm1" ||
             selectedMotor == "R_arm2" || selectedMotor == "R_arm3" ||
             selectedMotor == "L_arm2" || selectedMotor == "L_arm3")
    {
        kp = 50.0; // 예시 값, 실제 필요한 값으로 조정
        kd = 1.0;  // 예시 값, 실제 필요한 값으로 조정
        peakAngle = 90;
        pathType = 1;
    }
    // 추가적인 모터 이름과 매개변수를 이곳에 추가할 수 있습니다.
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

    while (systemState.runMode != RunMode::Stop)
    {

        if (systemState.runMode == RunMode::Pause)
        {
            continue;
        }

        if (sendBuffer.size() <= 10)
        {
            if (pathManager.line < pathManager.end)
            {
                std::cout << "line : " << pathManager.line << ", end : " << pathManager.end << "\n";
                pathManager.PathLoopTask(sendBuffer);
                std::cout << sendBuffer.size() << "\n";
                pathManager.line++;
            }
            else if (pathManager.line == pathManager.end)
            {
                std::cout << "Turn Back\n";
                pathManager.GetBackArr();
                pathManager.line++;
            }
            else if (sendBuffer.size() == 0)
            {
                systemState.runMode = RunMode::Stop;
                std::cout << "Performance is Over\n";
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
    std::string csvFileName = "TuningData/DrumData_in.txt";

    // CSV 파일 열기
    std::ofstream csvFile(csvFileName);

    if (!csvFile.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    csvFile << "0x007,0x001,0x002,0x003,0x004,0x005,0x006\n";

    // 2차원 벡터의 데이터를 CSV 파일로 쓰기
    for (const auto &row : pathManager.q)
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

    std::cout << "연주 CSV 파일이 생성되었습니다: " << csvFileName << std::endl;

    std::cout << "SendLoop terminated\n";
}

bool SendLoopTask::CheckAllMotorsCurrentPosition()
{
    std::cout << "Checking all positions for motors" << endl;
    bool allMotorsChecked = true;
    for (const auto &motor_pair : tmotors)
    {
        std::shared_ptr<TMotor> motor = motor_pair.second;
        bool motorChecked = CheckCurrentPosition(motor);
        if (!motorChecked)
        {
            cerr << "Failed to check position for motor: " << motor_pair.first << endl;
            allMotorsChecked = false;
        }
    }
    return allMotorsChecked;
}

void SendLoopTask::SendReadyLoop()
{
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
//