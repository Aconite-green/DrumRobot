#include "../include/StateTask.hpp"

// StateTask 클래스의 생성자
StateTask::StateTask(SystemState &systemStateRef,
                     CanSocketUtils &canUtilsRef,
                     std::map<std::string, std::shared_ptr<TMotor>> &tmotorsRef,
                     std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotorsRef)
    : systemState(systemStateRef), canUtils(canUtilsRef), tmotors(tmotorsRef), maxonMotors(maxonMotorsRef) {}

/////////////////////////////////////////////////////////////////////////////////
/*                               SYSTEM LOOPS                             */
///////////////////////////////////////////////////////////////////////////////

void StateTask::operator()()
{
    while (systemState.main != Main::Shutdown)
    {
        Main currentState = systemState.main.load();
        switch (currentState)
        {
        case Main::SystemInit:
            initializeTMotors();
            initializeCanUtils();
            ActivateControlTask();
            std::cout << "Press Enter to go Home\n";
            getchar();
            systemState.main = Main::Ideal;
            break;
        case Main::Ideal:
            idealStateRoutine();
            break;
        case Main::Homing:
            homeModeLoop();
            break;
        case Main::Perform:
            runModeLoop();
            break;
        case Main::Check:
            CheckAllMotorsCurrentPosition();
            std::cout << "Press Enter to Go home\n";
            getchar();
            systemState.main = Main::Ideal;
            break;
        case Main::Tune:
            TuningLoopTask();
            systemState.main = Main::Ideal;
            break;
        case Main::Shutdown:
            std::cout << "======= Shut down system =======\n";
            break;
        default:
            systemState.main = Main::Ideal;
            break;
        }
        /*if (currentState != systemState.main.load())
        {
            emit stateChanged(systemState.main.load());
        }*/
    }
    DeactivateControlTask();
}

void StateTask::homeModeLoop()
{
    while (systemState.main == Main::Homing)
    {
        displayHomingStatus();

        std::string motorName;
        std::cout << "Enter the name of the motor to home, or 'all' to home all motors: ";
        std::cin >> motorName;

        // L_arm2, L_arm3, R_arm2, R_arm3 입력 시 L_arm1, R_arm1 홈 상태 확인
        if ((motorName == "L_arm2" || motorName == "L_arm3") && !tmotors["L_arm1"]->isHomed)
        {
            std::cout << "Error: L_arm1 must be homed before " << motorName << std::endl;
            continue; // 다음 입력을 위해 반복문의 시작으로 돌아감
        }
        else if ((motorName == "R_arm2" || motorName == "R_arm3") && !tmotors["R_arm1"]->isHomed)
        {
            std::cout << "Error: R_arm1 must be homed before " << motorName << std::endl;
            continue;
        }

        if (motorName == "all")
        {
            // 우선순위가 높은 모터 먼저 홈
            std::vector<std::string> priorityMotors = {"L_arm1", "R_arm1"};
            for (const auto &pmotorName : priorityMotors)
            {
                if (tmotors.find(pmotorName) != tmotors.end() && !tmotors[pmotorName]->isHomed)
                {
                    SetHome(tmotors[pmotorName], pmotorName);
                }
            }

            // 나머지 모터 홈
            for (auto &motor_pair : tmotors)
            {
                if (std::find(priorityMotors.begin(), priorityMotors.end(), motor_pair.first) == priorityMotors.end() && !motor_pair.second->isHomed)
                {
                    SetHome(motor_pair.second, motor_pair.first);
                }
            }
        }
        else if (tmotors.find(motorName) != tmotors.end() && !tmotors[motorName]->isHomed)
        {
            SetHome(tmotors[motorName], motorName);
        }
        else
        {
            std::cout << "Motor not found or already homed: " << motorName << std::endl;
        }

        UpdateHomingStatus();
    }
}

void StateTask::runModeLoop()
{
    while (systemState.main == Main::Perform)
    {
        switch (systemState.runMode.load())
        {
        case RunMode::PrePreparation:
            std::cout << "Press 'r' to set Ready mode: ";
            char input;
            std::cin >> input;
            if (input == 'r')
            {
                systemState.runMode = RunMode::Preparing;
            }
            break;
        case RunMode::Preparing:
            checkUserInput();
            break;
        case RunMode::Ready:
            std::cout << "Press 'p' to start performing: ";
            std::cin >> input;
            if (input == 'p')
            {
                systemState.runMode = RunMode::Running;
            }
            break;
        case RunMode::Running:
            checkUserInput();
            break;
        case RunMode::Pause:
            checkUserInput();
            break;
        case RunMode::Stop:
            systemState.homeMode = HomeMode::HomeDone;
            systemState.main = Main::Ideal;
            break;
        case RunMode::RunError:
            // 오류 처리
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                                STATE UTILITY                               */
///////////////////////////////////////////////////////////////////////////////

void StateTask::displayAvailableCommands() const
{
    std::cout << "Available Commands:\n";

    if (systemState.main == Main::Ideal)
    {
        if (systemState.homeMode == HomeMode::NotHome && systemState.runMode == RunMode::PrePreparation)
        {
            std::cout << "- h : Start Homing Mode\n";
            std::cout << "- x  : Make home state by user\n";
        }
        else if (systemState.homeMode == HomeMode::HomeDone && systemState.runMode == RunMode::PrePreparation)
        {
            std::cout << "- t : Start tuning\n";
            std::cout << "- p : Start Perform Mode\n";
        }
    }
    else if (systemState.main == Main::Homing)
    {
        // 현재 Homing 상태 알 수 있는 로직추가
    }
    else if (systemState.main == Main::Perform)
    {
        // 나중에 필요하면 추가
    }

    std::cout << "- s : Shut down the system\n";
    std::cout << "- c : Check Motors position\n";
}

bool StateTask::processInput(const std::string &input)
{
    if (systemState.main == Main::Ideal)
    {
        if (input == "h" && systemState.homeMode == HomeMode::NotHome)
        {
            systemState.main = Main::Homing;
            return true;
        }
        else if (input == "t" && systemState.homeMode == HomeMode::HomeDone)
        {
            systemState.main = Main::Tune;
            return true;
        }
        else if (input == "p" && systemState.homeMode == HomeMode::HomeDone && systemState.runMode == RunMode::PrePreparation)
        {
            systemState.main = Main::Perform;
            return true;
        }
        else if (input == "x" && systemState.homeMode == HomeMode::NotHome)
        {
            systemState.homeMode = HomeMode::HomeDone;
            return true;
        }
    }
    if (input == "c")
    {
        systemState.main = Main::Check;
        return true;
    }

    if (input == "s")
    {
        systemState.main = Main::Shutdown;
        return true;
    }

    return false;
}

void StateTask::idealStateRoutine()
{
    int ret = system("clear");
    if (ret == -1)
        cout << "system clear error" << endl;

    displayAvailableCommands();

    std::string input;
    std::cout << "Enter command: ";
    std::getline(std::cin, input);

    if (!processInput(input))
    {
        std::cout << "Invalid command or not allowed in current state!\n";
    }

    usleep(2000);
}

void StateTask::checkUserInput()
{
    if (kbhit())
    {
        char input = getchar();
        if (input == 'q')
            systemState.runMode = RunMode::Pause;
        else if (input == 'e')
        {
            systemState.runMode = RunMode::PrePreparation;
            systemState.main = Main::Ideal;
            canUtils.restart_all_can_ports();
        }
        else if (input == 'r')
            systemState.runMode = RunMode::Running;
    }

    usleep(500000);
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 SYSTEM                                     */
///////////////////////////////////////////////////////////////////////////////

void StateTask::initializeTMotors()
{
    tmotors["waist"] = make_shared<TMotor>(0x007, "AK10_9", "can0");

    tmotors["R_arm1"] = make_shared<TMotor>(0x001, "AK70_10", "can0");
    tmotors["L_arm1"] = make_shared<TMotor>(0x002, "AK70_10", "can0");

    tmotors["R_arm2"] = make_shared<TMotor>(0x003, "AK70_10", "can0");
    tmotors["R_arm3"] = make_shared<TMotor>(0x004, "AK70_10", "can0");

    tmotors["L_arm2"] = make_shared<TMotor>(0x005, "AK70_10", "can0");
    tmotors["L_arm3"] = make_shared<TMotor>(0x006, "AK70_10", "can0");

    for (auto &motor_pair : tmotors)
    {
        std::shared_ptr<TMotor> &motor = motor_pair.second;

        // 각 모터 이름에 따른 멤버 변수 설정
        if (motor_pair.first == "waist")
        {
            motor->cwDir = 1.0f;
            motor->rMin = -M_PI * 0.75f; // -120deg
            motor->rMax = M_PI / 2.0f;   // 90deg
            motor->isHomed = true;
        }
        else if (motor_pair.first == "R_arm1")
        {
            motor->cwDir = 1.0f;
            motor->sensorBit = 1;
            motor->rMin = 0.0f; // 0deg
            motor->rMax = M_PI; // 180deg
            motor->isHomed = false;
        }
        else if (motor_pair.first == "L_arm1")
        {
            motor->cwDir = 1.0f;
            motor->sensorBit = 1;
            motor->rMin = 0.0f; // 0deg
            motor->rMax = M_PI; // 180deg
            motor->isHomed = false;
        }
        else if (motor_pair.first == "R_arm2")
        {
            motor->cwDir = 1.0f;
            motor->sensorBit = 1;
            motor->rMin = -M_PI / 4.0f; // -45deg
            motor->rMax = M_PI / 2.0f;  // 90deg
            motor->isHomed = false;
        }
        else if (motor_pair.first == "R_arm3")
        {
            motor->cwDir = 1.0f;
            motor->sensorBit = 1;
            motor->rMin = 0.0f;         // 0deg
            motor->rMax = M_PI * 0.75f; // 135deg
            motor->isHomed = false;
        }
        else if (motor_pair.first == "L_arm2")
        {
            motor->cwDir = -1.0f;
            motor->sensorBit = 0;
            motor->rMin = -M_PI / 2.0f; // -90deg
            motor->rMax = M_PI / 4.0f;  // 45deg
            motor->isHomed = false;
        }
        else if (motor_pair.first == "L_arm3")
        {
            motor->cwDir = -1.0f;
            motor->sensorBit = 2;
            motor->rMin = -M_PI * 0.75f; // -135deg
            motor->rMax = 0.0f;          // 0deg
            motor->isHomed = false;
        }
    }

    map<string, shared_ptr<MaxonMotor>> maxonMotors;
    maxonMotors["L_wrist"] = make_shared<MaxonMotor>(0x001,
                                                     vector<uint32_t>{0x201, 0x301},
                                                     vector<uint32_t>{0x181},
                                                     "can0");
    maxonMotors["R_wrist"] = make_shared<MaxonMotor>(0x002,
                                                     vector<uint32_t>{0x202, 0x302},
                                                     vector<uint32_t>{0x182},
                                                     "can0");
};

void StateTask::initializeCanUtils()
{
    canUtils.initializeCAN(extractIfnamesFromMotors(tmotors));
    canUtils.checkCanPortsStatus();
}

vector<string> StateTask::extractIfnamesFromMotors(const map<string, shared_ptr<TMotor>> &motors)
{
    set<string> interface_names;
    for (const auto &motor_pair : motors)
    {
        interface_names.insert(motor_pair.second->interFaceName);
    }

    return vector<string>(interface_names.begin(), interface_names.end());
}

void StateTask::ActivateControlTask()
{
    struct can_frame frame;

    canUtils.set_all_sockets_timeout(0, 5000);
    if (!tmotors.empty())
    {
        // 첫 번째 단계: 모터 상태 확인 (10ms 타임아웃)

        for (auto it = tmotors.begin(); it != tmotors.end();)
        {
            std::string name = it->first;
            std::shared_ptr<TMotor> motor = it->second;

            bool checkSuccess = true;
            canUtils.clear_all_can_buffers();
            // 상태 확인
            fillCanFrameFromInfo(&frame, motor->getCanFrameForCheckMotor());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [&checkSuccess](const std::string &motorName, bool result) {

                           });

            // 상태 확인
            fillCanFrameFromInfo(&frame, motor->getCanFrameForControlMode());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [&checkSuccess](const std::string &motorName, bool result)
                           {
                               if (!result)
                               {
                                   std::cerr << "Motor [" << motorName << "] Not Connected." << std::endl;
                                   checkSuccess = false;
                               }
                               else
                               {
                                   std::cerr << "--------------> Motor [" << motorName << "] is Connected." << std::endl;
                               }
                           });

            if (!checkSuccess)
            {
                // 실패한 경우, 해당 모터를 배열에서 제거
                it = tmotors.erase(it);

                continue;
            }
            else
            {
                ++it;
            }
        }

        // 구분자 추가
        std::cout << "\n=================== Start Zeroing ====================" << std::endl;

        // 두 번째 단계: 제어 모드 설정과 제로 설정 (5초 타임아웃)
        canUtils.set_all_sockets_timeout(5, 0);
        for (const auto &motorPair : tmotors)
        {
            std::string name = motorPair.first;
            std::shared_ptr<TMotor> motor = motorPair.second;

            // 제로 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForZeroing());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (!success)
                               {
                                   std::cerr << "Failed to set zero for motor [" << motorName << "]." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Motor [" << motorName << "] Now Zero." << std::endl;
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
    canUtils.set_all_sockets_timeout(0, 5000);
    if (!maxonMotors.empty())
    {
        for (auto it = maxonMotors.begin(); it != maxonMotors.end();)
        {
            std::string name = it->first;
            std::shared_ptr<MaxonMotor> motor = it->second;

            bool checkSuccess = true;
            canUtils.clear_all_can_buffers();

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
                               else
                               {
                                   std::cerr << "--------------> Motor [" << motorName << "] is Connected." << std::endl;
                               }
                           });

            if (!checkSuccess)
            {
                // 실패한 경우, 해당 모터를 배열에서 제거
                it = maxonMotors.erase(it);

                continue;
            }
            else
            {
                ++it;
            }
        }
        canUtils.set_all_sockets_timeout(0, 500000);
        for (const auto &motorPair : maxonMotors)
        {
            std::string name = motorPair.first;
            std::shared_ptr<MaxonMotor> motor = motorPair.second;

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
                        [](const std::string &motorName, bool success) {

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

void StateTask::DeactivateControlTask()
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

bool StateTask::CheckAllMotorsCurrentPosition()
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

bool StateTask::CheckTmotorPosition(std::shared_ptr<TMotor> motor)
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

bool StateTask::CheckMaxonPosition(std::shared_ptr<MaxonMotor> motor)
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

        std::tuple<int, float> parsedData = MParser.parseRecieveCommand(&frame);
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

/////////////////////////////////////////////////////////////////////////////////
/*                                  HOME                                      */
///////////////////////////////////////////////////////////////////////////////

void StateTask::SendCommandToMotor(std::shared_ptr<TMotor> &motor, struct can_frame &frame, const std::string &motorName)
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

bool StateTask::PromptUserForHoming(const std::string &motorName)
{
    char userResponse;
    std::cout << "Would you like to start homing mode for motor [" << motorName << "]? (y/n): ";
    std::cin >> userResponse;
    return userResponse == 'y';
}

void StateTask::RotateMotor(std::shared_ptr<TMotor> &motor, const std::string &motorName, double direction, double degree, float midpoint)
{
    struct can_frame frameToProcess;
    chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
    TParser.parseSendCommand(*motor, &frameToProcess, motor->nodeId, 8, motor->currentPos, 0, 150, 1, 0);
    SendCommandToMotor(motor, frameToProcess, motorName);

    // 수정된 부분: 사용자가 입력한 각도를 라디안으로 변환
    const double targetRadian = (degree * M_PI / 180.0 + midpoint) * direction; // 사용자가 입력한 각도를 라디안으로 변환 + midpoint
    int totalSteps = 4000 / 5;                                                  // 4초 동안 5ms 간격으로 나누기

    for (int step = 1; step <= totalSteps; ++step)
    {
        while (1)
        {
            chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
            if (chrono::duration_cast<chrono::microseconds>(currentTime - startTime).count() > 5000)
                break;
        }

        startTime = std::chrono::system_clock::now();

        // 5ms마다 목표 위치 계산 및 프레임 전송
        double targetPosition = targetRadian * (static_cast<double>(step) / totalSteps) + motor->currentPos;
        TParser.parseSendCommand(*motor, &frameToProcess, motor->nodeId, 8, targetPosition, 0, 250, 2.5, 0);
        SendCommandToMotor(motor, frameToProcess, motorName);

        startTime = std::chrono::system_clock::now();
    }
    CheckTmotorPosition(motor);
}

void StateTask::HomeMotor(std::shared_ptr<TMotor> &motor, const std::string &motorName)
{
    struct can_frame frameToProcess;

    // arm2 모터는 -30도, 나머지 모터는 +90도에 센서 위치함.
    double initialDirection = (motorName == "L_arm2" || motorName == "R_arm2") ? (-0.2) * motor->cwDir : 0.2 * motor->cwDir;

    double additionalTorque = 0.0;
    if (motorName == "L_arm2" || motorName == "R_arm2")
    {
        additionalTorque = motor->cwDir * (-2.2);
    }
    else if (motorName == "L_arm3" || motorName == "R_arm3")
    {
        additionalTorque = motor->cwDir * 1.0;
    }

    TParser.parseSendCommand(*motor, &frameToProcess, motor->nodeId, 8, 0, initialDirection, 0, 4.5, additionalTorque);
    SendCommandToMotor(motor, frameToProcess, motorName);

    float midpoint = MoveMotorToSensorLocation(motor, motorName, motor->sensorBit);

    double degree = (motorName == "L_arm2" || motorName == "R_arm2") ? -30.0 : 90.0;
    midpoint = (motorName == "L_arm2" || motorName == "R_arm2") ? -midpoint : midpoint;
    RotateMotor(motor, motorName, -motor->cwDir, degree, midpoint);

    cout << "----------------------moved 90 degree (Anti clock wise) --------------------------------- \n";

    // 모터를 멈추는 신호를 보냄
    TParser.parseSendCommand(*motor, &frameToProcess, motor->nodeId, 8, 0, 0, 0, 5, 0);
    SendCommandToMotor(motor, frameToProcess, motorName);

    // 현재 position을 0으로 인식하는 명령을 보냄
    fillCanFrameFromInfo(&frameToProcess, motor->getCanFrameForZeroing());
    SendCommandToMotor(motor, frameToProcess, motorName);

    // 상태 확인
    fillCanFrameFromInfo(&frameToProcess, motor->getCanFrameForControlMode());
    SendCommandToMotor(motor, frameToProcess, motorName);

    if (motorName == "L_arm1" || motorName == "R_arm1")
    {
        CheckTmotorPosition(motor);
        RotateMotor(motor, motorName, motor->cwDir, 90, 0);
    }
    /*  // homing 잘 됐는지 센서 위치로 다시 돌아가서 확인
    if(motor_pair.first == "L_arm2" || motor_pair.first == "R_arm2")
    {
        CheckTmotorPosition(motor);
        RotateMotor(motor, motor_pair.first, settings.direction, -30, 0);
    }
    if (motor_pair.first == "L_arm3" || motor_pair.first == "R_arm3")
    {
        RotateMotor(motor, motor_pair.first, settings.direction, 90, 0);
    }
    */
}

void StateTask::SetHome(std::shared_ptr<TMotor> &motor, const std::string &motorName)
{
    sensor.OpenDeviceUntilSuccess();
    canUtils.set_all_sockets_timeout(5, 0);

    // 허리는 home 안잡음

    HomeMotor(motor, motorName);
    motor->isHomed = true; // 홈잉 상태 업데이트

    cout << "Homing completed for " << motorName << "\n";
    sensor.closeDevice();
}

float StateTask::MoveMotorToSensorLocation(std::shared_ptr<TMotor> &motor, const std::string &motorName, int sensorBit)
{
    float firstPosition = 0.0f, secondPosition = 0.0f;
    bool firstSensorTriggered = false;
    bool secondSensorTriggered = false;

    std::cout << "Moving " << motorName << " to sensor location.\n";

    while (true)
    {
        bool sensorTriggered = ((sensor.ReadVal() >> sensorBit) & 1) != 0;

        if (!firstSensorTriggered && sensorTriggered)
        {
            // 첫 번째 센서 인식
            firstSensorTriggered = true;
            CheckTmotorPosition(motor);
            firstPosition = motor->currentPos;
            std::cout << motorName << " first sensor position: " << firstPosition << endl;
        }
        else if (firstSensorTriggered && !sensorTriggered)
        {
            // 센서 인식 해제
            secondSensorTriggered = true;
            CheckTmotorPosition(motor);
            secondPosition = motor->currentPos;
            std::cout << motorName << " second sensor position: " << secondPosition << endl;

            break; // while문 탈출
        }

        if (secondSensorTriggered)
            break; // 두 번째 센서 인식 후 반복문 탈출
    }

    // 1번과 2번 위치의 차이의 절반을 저장
    float positionDifference = abs((secondPosition - firstPosition) / 2.0f);
    std::cout << motorName << " midpoint position: " << positionDifference << endl;

    return positionDifference;
}

void StateTask::displayHomingStatus()
{
    std::cout << "Homing Status of Motors:\n";
    for (const auto &motor_pair : tmotors)
    {
        std::cout << motor_pair.first << ": "
                  << (motor_pair.second->isHomed ? "Homed" : "Not Homed") << std::endl;
    }
}

void StateTask::UpdateHomingStatus()
{
    bool allMotorsHomed = true;
    for (const auto &motor_pair : tmotors)
    {
        if (!motor_pair.second->isHomed)
        {
            allMotorsHomed = false;
            break;
        }
    }

    if (allMotorsHomed)
    {
        systemState.homeMode = HomeMode::HomeDone;
        systemState.main = Main::Ideal;
    }
    else
    {
        systemState.homeMode = HomeMode::NotHome;
    }
}
/////////////////////////////////////////////////////////////////////////////////
/*                                  TUNE                                      */
///////////////////////////////////////////////////////////////////////////////

void StateTask::FixMotorPosition()
{
    struct can_frame frame;
    for (const auto &motorPair : tmotors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<TMotor> motor = motorPair.second;
        CheckTmotorPosition(motor);

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
    for (const auto &motorPair : maxonMotors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<MaxonMotor> motor = motorPair.second;
        CheckMaxonPosition(motor);

        MParser.parseSendCommand(*motor, &frame, motor->currentPos);
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

void StateTask::TuningTmotor(float kp, float kd, float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType)
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
    csvFileOut << "CAN_ID,p_des,p_act,tff_des,tff_act\n"; // CSV 헤더

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

void StateTask::TuningMaxon(float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType)
{
    canUtils.set_all_sockets_timeout(0, 50000);

    std::string FileName1 = "../../READ/" + selectedMotor + "_in.txt";

    std::ofstream csvFileIn(FileName1);

    if (!csvFileIn.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    csvFileIn << "Start file"
              << "\n";

    // CSV 파일명 설정
    std::string FileName2 = "../../READ/" + selectedMotor + "_out.txt";

    // CSV 파일 열기
    std::ofstream csvFileOut(FileName2);

    if (!csvFileOut.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }
    csvFileOut << "CAN_ID,p_des,p_act,tff_des,tff_act\n"; // CSV 헤더

    struct can_frame frame;

    float peakRadian = peakAngle * M_PI / 180.0; // 피크 각도를 라디안으로 변환
    float amplitude = peakRadian;

    float sample_time = 0.005;
    int max_samples = static_cast<int>(sine_t / sample_time);
    float p_des = 0;
    float p_act;
    // float tff_des = 0,v_des = 0;
    //float v_act, tff_act;

    for (int cycle = 0; cycle < cycles; cycle++)
    {
        for (int i = 0; i < max_samples; i++)
        {
            float time = i * sample_time;

            for (auto &entry : maxonMotors)
            {
                if (entry.first != selectedMotor)
                    continue;

                std::shared_ptr<MaxonMotor> &motor = entry.second;

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

                MParser.parseSendCommand(*motor, &frame, p_des);
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

                        MParser.makeSync(&frame);
                        bytesWritten = write(canUtils.sockets.at(motor->interFaceName), &frame, sizeof(struct can_frame));
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
                            std::tuple<int, float> result = MParser.parseRecieveCommand(&frame);

                            p_act = std::get<1>(result);
                            // v_act = std::get<1>(result);
                            // tff_act = std::get<3>(result);
                            // tff_des = kp * (p_des - p_act) + kd * (v_des - v_act);
                            csvFileOut << ',' << std::dec << p_act << ',' << '\n';
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

void StateTask::TuningLoopTask()
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
    else
    {
        selectedMotor = maxonMotors.begin()->first;
    }

    InitializeParameters(selectedMotor, kp, kd, peakAngle, pathType);
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
        for (const auto &motor_pair : maxonMotors)
        {
            std::cout << " - " << motor_pair.first << "\n";
        }

        bool isMaxonMotor = maxonMotors.find(selectedMotor) != maxonMotors.end();

        std::cout << "---------------------------------------------\n";
        std::cout << "Selected Motor: " << selectedMotor << "\n";
        if (!isMaxonMotor)
        {
            std::cout << "Kp: " << kp << " | Kd: " << kd << "\n";
        }
        std::cout << "Sine Period: " << sine_t << " | Cycles: " << cycles << " | Hz: " << 1 / sine_t << "\n";
        std::cout << "Peak Angle: " << peakAngle << " | Path Type: " << pathTypeDescription << "\n";
        std::cout << "\nCommands:\n";
        if (!isMaxonMotor)
        {
            std::cout << "[KP] | [KD] |";
        }
        std::cout << "[S]elect Motor | [Peak] | [Type]\n";
        std::cout << "[P]eriod | [C]ycles | [R]un | [E]xit\n";
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
                    InitializeParameters(selectedMotor, kp, kd, peakAngle, pathType);
                    break;
                }
                else
                {
                    std::cout << "Invalid motor name. Please enter a valid motor name.\n";
                }
            }
        }
        else if (userInput == "kp" && !isMaxonMotor)
        {
            std::cout << "Enter Desired Kp: ";
            std::cin >> kp;
        }
        else if (userInput == "kd" && !isMaxonMotor)
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
                pathType = 1;
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
            if (!isMaxonMotor) // Tmotor일 경우
            {
                TuningTmotor(kp, kd, sine_t, selectedMotor, cycles, peakAngle, pathType);
            }
            else // MaxonMotor일 경우
            {
                TuningMaxon(sine_t, selectedMotor, cycles, peakAngle, pathType);
            }
        }
    }
}

void StateTask::InitializeParameters(const std::string selectedMotor, float &kp, float &kd, float &peakAngle, int &pathType)
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
    else if (selectedMotor == "L_wrist" || selectedMotor == "R_wrist")
    {
        peakAngle = 90;
        pathType = 1;
    }
    // 추가적인 모터 이름과 매개변수를 이곳에 추가할 수 있습니다.
}
