#include "../include/tasks/StateTask.hpp"

// StateTask 클래스의 생성자
StateTask::StateTask(SystemState &systemStateRef,
                     CanManager &canManagerRef,
                     std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
                     queue<can_frame> &sendBufferRef,
                     queue<can_frame> &recieveBufferRef)
    : systemState(systemStateRef), canManager(canManagerRef), motors(motorsRef), sendBuffer(sendBufferRef), recieveBuffer(recieveBufferRef), testmanager(canManagerRef, sendBufferRef, recieveBufferRef, motors) {}

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
            initializeMotors();
            initializecanManager();
            maxonSdoSetting();
            MaxonEnable();
            setMaxonMode("CSP");
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
            printCurrentPositions();
            std::cout << "Press Enter to Go home\n";
            getchar();
            systemState.main = Main::Ideal;
            break;
        case Main::Tune:
            MaxonEnable();
            // TuningLoopTask()
            CheckAllMotorsCurrentPosition();
            setMaxonMode("CSP");
            testmanager.run();
            systemState.main = Main::Ideal;
            break;
        case Main::Shutdown:
            std::cout << "======= Shut down system =======\n";
            break;
        case Main::Back:
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

    // MaxonEnable();
    setMaxonMode("HMM");
    while (systemState.main == Main::Homing)
    {
        displayHomingStatus();

        std::string motorName;
        std::cout << "Enter the name of the motor to home, or 'all' to home all motors: ";
        std::cin >> motorName;

        // L_arm2, L_arm3, R_arm2, R_arm3 입력 시 L_arm1, R_arm1 홈 상태 확인
        /*if ((motorName == "L_arm2" || motorName == "L_arm3") && !tmotors["L_arm1"]->isHomed)
        {
            std::cout << "Error: L_arm1 must be homed before " << motorName << std::endl;
            continue; // 다음 입력을 위해 반복문의 시작으로 돌아감
        }
        else if ((motorName == "R_arm2" || motorName == "R_arm3") && !tmotors["R_arm1"]->isHomed)
        {
            std::cout << "Error: R_arm1 must be homed before " << motorName << std::endl;
            continue;
        }
        else if ((motorName == "R_wrist") && !tmotors["R_arm3"]->isHomed)
        {
            std::cout << "Error: R_arm3 must be homed before " << motorName << std::endl;
            continue;
        }
        else if ((motorName == "L_wrist") && !tmotors["L_arm3"]->isHomed)
        {
            std::cout << "Error: L_arm3 must be homed before " << motorName << std::endl;
            continue;
        }*/

        if (motorName == "all")
        {
            // 우선순위가 높은 T모터 먼저 홈
            std::vector<std::string> priorityMotors = {"L_arm1", "R_arm1"};
            for (const auto &pmotorName : priorityMotors)
            {
                if (motors.find(pmotorName) != motors.end() && !motors[pmotorName]->isHomed)
                {
                    SetTmotorHome(motors[pmotorName], pmotorName);
                }
            }
            for (auto &motor_pair : motors)
            {
                auto &motor = motor_pair.second;
                std::string name = motor_pair.first;

                // 타입에 따라 적절한 캐스팅과 초기화 수행
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
                {
                    if (!motor->isHomed)
                    {
                        SetTmotorHome(motors[name], name);
                    }
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
                {
                    if (!motor->isHomed)
                    {
                        SetMaxonHome(motors[name], name);
                    }
                }
            }
        }
        else if (motors.find(motorName) != motors.end() && !motors[motorName]->isHomed)
        {
            // 타입에 따라 적절한 캐스팅과 초기화 수행
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motors[motorName]))
            {
                SetTmotorHome(motors[motorName], motorName);
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors[motorName]))
            {

                SetMaxonHome(motors[motorName], motorName);
            }
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
    MaxonEnable();
    setMaxonMode("CSP");
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
            while (true)
            {
                std::cout << "Enter 't'(test) or 'p'(perform) or 'e'(exit): ";
                std::cin >> input;

                if (input == 'e')
                {
                    systemState.runMode = RunMode::PrePreparation;
                    systemState.main = Main::Ideal;
                }
                else if (input == 't')
                {
                }
                else if (input == 'p')
                {
                    systemState.runMode = RunMode::Running;
                }
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
            std::cout << "- x : Make home state by user\n";
        }
        else if (systemState.homeMode == HomeMode::HomeDone && systemState.runMode == RunMode::PrePreparation)
        {
            std::cout << "- t : Start tuning\n";
            std::cout << "- p : Start Perform Mode\n";
            std::cout << "- b : Back to Zero Postion\n";
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
        else if (input == "c")
        {
            systemState.main = Main::Check;
            return true;
        }
        else if (input == "s")
        {
            systemState.main = Main::Shutdown;
            return true;
        }
        else if (input == "b" && systemState.homeMode == HomeMode::HomeDone)
        {
            systemState.main = Main::Back;
            return true;
        }
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
            canManager.restartCanPorts();
        }
        else if (input == 'r')
            systemState.runMode = RunMode::Running;
    }

    usleep(500000);
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 SYSTEM                                     */
///////////////////////////////////////////////////////////////////////////////

void StateTask::initializeMotors()
{

    motors["waist"] = make_shared<TMotor>(0x007, "AK10_9");
    motors["R_arm1"] = make_shared<TMotor>(0x001, "AK70_10");
    motors["L_arm1"] = make_shared<TMotor>(0x002, "AK70_10");
    motors["R_arm2"] = make_shared<TMotor>(0x003, "AK70_10");
    motors["R_arm3"] = make_shared<TMotor>(0x004, "AK70_10");
    motors["L_arm2"] = make_shared<TMotor>(0x005, "AK70_10");
    motors["L_arm3"] = make_shared<TMotor>(0x006, "AK70_10");
    motors["L_wrist"] = make_shared<MaxonMotor>(0x009);
    motors["R_wrist"] = make_shared<MaxonMotor>(0x008);

    for (auto &motor_pair : motors)
    {
        auto &motor = motor_pair.second;

        // 타입에 따라 적절한 캐스팅과 초기화 수행
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            // 각 모터 이름에 따른 멤버 변수 설정
            if (motor_pair.first == "waist")
            {
                tMotor->cwDir = 1.0f;
                tMotor->rMin = -M_PI / 2.0f; // -90deg
                tMotor->rMax = M_PI / 2.0f;  // 90deg
                tMotor->isHomed = true;
                tMotor->interFaceName = "can0";
            }
            else if (motor_pair.first == "R_arm1")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 3;
                tMotor->rMin = -M_PI; // -180deg
                tMotor->rMax = 0.0f;  // 0deg
                tMotor->isHomed = false;
                tMotor->interFaceName = "can1";
            }
            else if (motor_pair.first == "L_arm1")
            {
                tMotor->cwDir = 1.0f;
                tMotor->sensorBit = 0;
                tMotor->rMin = 0.0f; // 0deg
                tMotor->rMax = M_PI; // 180deg
                tMotor->isHomed = false;
                tMotor->interFaceName = "can0";
            }
            else if (motor_pair.first == "R_arm2")
            {
                tMotor->cwDir = 1.0f;
                tMotor->sensorBit = 4;
                tMotor->rMin = -M_PI / 4.0f; // -45deg
                tMotor->rMax = M_PI / 2.0f;  // 90deg
                tMotor->isHomed = false;
                tMotor->interFaceName = "can1";
            }
            else if (motor_pair.first == "R_arm3")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 5;
                tMotor->rMin = -M_PI * 0.75f; // -135deg
                tMotor->rMax = 0.0f;          // 0deg
                tMotor->isHomed = false;
                tMotor->interFaceName = "can1";
            }
            else if (motor_pair.first == "L_arm2")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 1;
                tMotor->rMin = -M_PI / 2.0f; // -90deg
                tMotor->rMax = M_PI / 4.0f;  // 45deg
                tMotor->isHomed = false;
                tMotor->interFaceName = "can0";
            }
            else if (motor_pair.first == "L_arm3")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 2;
                tMotor->rMin = -M_PI * 0.75f; // -135deg
                tMotor->rMax = 0.0f;          // 0deg
                tMotor->isHomed = false;
                tMotor->interFaceName = "can0";
            }
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            // 각 모터 이름에 따른 멤버 변수 설정
            if (motor_pair.first == "L_wrist")
            {
                maxonMotor->cwDir = -1.0f;
                maxonMotor->rMin = -M_PI * 0.75f; // -120deg
                maxonMotor->rMax = M_PI / 2.0f;   // 90deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x209; // Controlword
                maxonMotor->txPdoIds[1] = 0x309; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x409; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x509; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x189; // Statusword, ActualPosition, ActualTorque
                maxonMotor->interFaceName = "can2";
            }
            else if (motor_pair.first == "R_wrist")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = 0.0f; // 0deg
                maxonMotor->rMax = M_PI; // 180deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x208; // Controlword
                maxonMotor->txPdoIds[1] = 0x308; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x408; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x508; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x188; // Statusword, ActualPosition, ActualTorque
                maxonMotor->interFaceName = "can2";
            }
        }
    }
};

void StateTask::initializecanManager()
{
    canManager.initializeCAN();
    canManager.checkCanPortsStatus();
    canManager.setMotorsSocket();
}

void StateTask::DeactivateControlTask()
{
    struct can_frame frame;

    canManager.setSocketsTimeout(0, 50000);

    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;

        // 타입에 따라 적절한 캐스팅과 초기화 수행
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            tmotorcmd.getCheck(*tMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            tmotorcmd.getExit(*tMotor, &frame);
            if (canManager.sendAndRecv(motor, frame))
            {
                std::cout << "Exiting for motor [" << name << "]" << std::endl;
            }
            else
            {
                std::cerr << "Failed to exit control mode for motor [" << name << "]." << std::endl;
            }
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            maxoncmd.getQuickStop(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);
            if (canManager.recvToBuff(motor, 2))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {
                        std::cout << "Exiting for motor [" << name << "]" << std::endl;
                        break;
                    }
                    motor->recieveBuffer.pop();
                }
            }
            else
            {
                std::cerr << "Failed to exit for motor [" << name << "]." << std::endl;
            }
        }
    }
}

bool StateTask::CheckAllMotorsCurrentPosition()
{

    std::cout << "Checking all positions for motors [rad]\n"
              << endl;
    bool allMotorsChecked = true;
    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;

        if (!checkMotorPosition(motor))
        {
            allMotorsChecked = false;
        }
    }
    return allMotorsChecked;
}

void StateTask::printCurrentPositions()
{
    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;
        std::cout << "[" << std::hex << motor->nodeId << std::dec << "] ";
        std::cout << name << " : " << motor->currentPos << endl;
    }
}

bool StateTask::checkMotorPosition(std::shared_ptr<GenericMotor> motor)
{
    struct can_frame frame;
    canManager.setSocketsTimeout(0, 5000 /*5ms*/);
    canManager.clearReadBuffers();

    if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
    {
        tmotorcmd.getControlMode(*tMotor, &frame);
        if (canManager.sendAndRecv(motor, frame))
        {
            std::tuple<int, float, float, float> parsedData = tmotorcmd.parseRecieveCommand(*tMotor, &frame);
            motor->currentPos = std::get<1>(parsedData);
        }
        else
        {
            return false;
        }
    }
    else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
    {
        maxoncmd.getSync(&frame);
        canManager.txFrame(motor, frame);

        if (canManager.recvToBuff(motor, 2))
        {
            while (!motor->recieveBuffer.empty())
            {
                frame = motor->recieveBuffer.front();
                if (frame.can_id == maxonMotor->rxPdoIds[0])
                {
                    std::tuple<int, float, float> parsedData = maxoncmd.parseRecieveCommand(*maxonMotor, &frame);
                    motor->currentPos = std::get<1>(parsedData);
                }
                motor->recieveBuffer.pop();
            }
        }
        else
        {
            return false;
        }
    }
    return true;
}
/////////////////////////////////////////////////////////////////////////////////
/*                                  HOME                                      */
///////////////////////////////////////////////////////////////////////////////

bool StateTask::PromptUserForHoming(const std::string &motorName)
{
    char userResponse;
    std::cout << "Would you like to start homing mode for motor [" << motorName << "]? (y/n): ";
    std::cin >> userResponse;
    return userResponse == 'y';
}

void StateTask::RotateTMotor(std::shared_ptr<GenericMotor> &motor, const std::string &motorName, double direction, double degree, float midpoint)
{

    struct can_frame frameToProcess;
    std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor);
    chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
    int kp = 250;

    if (motorName == "L_arm1" || motorName == "R_arm1")
        kp = 250;
    else if (motorName == "L_arm2" || motorName == "R_arm2")
        kp = 350;
    else if (motorName == "L_arm3" || motorName == "R_arm3")
        kp = 350;
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
        tmotorcmd.parseSendCommand(*tMotor, &frameToProcess, motor->nodeId, 8, targetPosition, 0, kp, 2.5, 0);
        canManager.sendAndRecv(motor, frameToProcess);

        startTime = std::chrono::system_clock::now();
    }

    totalSteps = 500 / 5;
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
        double targetPosition = targetRadian + motor->currentPos;
        tmotorcmd.parseSendCommand(*tMotor, &frameToProcess, motor->nodeId, 8, targetPosition, 0, kp, 2.5, 0);
        canManager.sendAndRecv(motor, frameToProcess);

        startTime = std::chrono::system_clock::now();
    }

    checkMotorPosition(motor);
}

void StateTask::HomeTMotor(std::shared_ptr<GenericMotor> &motor, const std::string &motorName)
{
    struct can_frame frameToProcess;
    std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor);
    // arm2 모터는 -30도, 나머지 모터는 +90도에 센서 위치함.
    double initialDirection = (motorName == "L_arm2" || motorName == "R_arm2") ? (-0.2) * motor->cwDir : 0.2 * motor->cwDir;

    double additionalTorque = 0.0;
    if (motorName == "L_arm2" || motorName == "R_arm2")
    {
        additionalTorque = motor->cwDir * (-3.0);
    }
    else if (motorName == "L_arm3" || motorName == "R_arm3")
    {
        additionalTorque = motor->cwDir * 1.8;
    }

    tmotorcmd.parseSendCommand(*tMotor, &frameToProcess, motor->nodeId, 8, 0, initialDirection, 0, 4.5, additionalTorque);
    canManager.sendAndRecv(motor, frameToProcess);

    float midpoint = MoveTMotorToSensorLocation(motor, motorName, tMotor->sensorBit);

    double degree = (motorName == "L_arm2" || motorName == "R_arm2") ? -30.0 : 90;
    midpoint = (motorName == "L_arm2" || motorName == "R_arm2") ? -midpoint : midpoint;
    RotateTMotor(motor, motorName, -motor->cwDir, degree, midpoint);

    cout << "----------------------moved 90 degree (Anti clock wise) --------------------------------- \n";

    // 모터를 멈추는 신호를 보냄
    tmotorcmd.parseSendCommand(*tMotor, &frameToProcess, motor->nodeId, 8, 0, 0, 0, 5, 0);
    canManager.sendAndRecv(motor, frameToProcess);

    canManager.setSocketsTimeout(2, 0);
    // 현재 position을 0으로 인식하는 명령을 보냄
    tmotorcmd.getZero(*tMotor, &frameToProcess);
    canManager.sendAndRecv(motor, frameToProcess);

    // 상태 확인
    /*fillCanFrameFromInfo(&frameToProcess, motor->getCanFrameForControlMode());
    SendCommandToTMotor(motor, frameToProcess, motorName);*/

    if (motorName == "L_arm1" || motorName == "R_arm1")
    {
        checkMotorPosition(motor);
        RotateTMotor(motor, motorName, motor->cwDir, 90, 0);
    }
    /*  // homing 잘 됐는지 센서 위치로 다시 돌아가서 확인
    if(motorName == "L_arm2" || motorName == "R_arm2")
    {
        checkMotorPosition(motor);
        RotateTMotor(motor, motorName, motor->cwDir, -30, 0);
    }*/
    if (motorName == "L_arm3" || motorName == "R_arm3")
    {
        checkMotorPosition(motor);
        RotateTMotor(motor, motorName, motor->cwDir, 90, 0);
    }
}

void StateTask::SetTmotorHome(std::shared_ptr<GenericMotor> &motor, const std::string &motorName)
{
    sensor.OpenDeviceUntilSuccess();
    canManager.setSocketsTimeout(5, 0);

    std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor);
    // 허리는 home 안잡음
    cout << "\n<< Homing for " << motorName << " >>\n";

    HomeTMotor(motor, motorName);
    motor->isHomed = true; // 홈잉 상태 업데이트
    sleep(1);
    FixMotorPosition(motor);

    cout << "-- Homing completed for " << motorName << " --\n\n";
    sensor.closeDevice();
}

void StateTask::SetMaxonHome(std::shared_ptr<GenericMotor> &motor, const std::string &motorName)
{
    struct can_frame frame;

    canManager.clearReadBuffers();
    canManager.setSocketsTimeout(2, 0);
    std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor);

    // Start to Move by homing method (일단은 PDO)

    maxoncmd.getStartHoming(*maxonMotor, &frame);
    canManager.txFrame(motor, frame);
    usleep(50000);

    maxoncmd.getSync(&frame);
    canManager.txFrame(motor, frame);
    if (canManager.recvToBuff(motor, 2))
    {
        while (!motor->recieveBuffer.empty())
        {
            frame = motor->recieveBuffer.front();
            if (frame.can_id == maxonMotor->rxPdoIds[0])
            {
                cout << "\n<< Homing for " << motorName << " >>\n";
            }
            motor->recieveBuffer.pop();
        }
    }

    sleep(10);
    // 홈 위치에 도달할 때까지 반복
    while (!motor->isHomed)
    {

        maxoncmd.getSync(&frame);
        canManager.txFrame(motor, frame);
        if (canManager.recvToBuff(motor, 2))
        {
            while (!motor->recieveBuffer.empty())
            {
                frame = motor->recieveBuffer.front();
                if (frame.can_id == maxonMotor->rxPdoIds[0])
                {
                    if (frame.data[1] & 0x80) // 비트 15 확인
                    {
                        motor->isHomed = true; // MaxonMotor 객체의 isHomed 속성을 true로 설정
                                               // 'this'를 사용하여 멤버 함수 호출
                        cout << "-- Homing completed for " << motorName << " --\n\n";
                    }
                }
                motor->recieveBuffer.pop();
            }
        }
        canManager.clearReadBuffers();

        sleep(1); // 100ms 대기
    }
}

float StateTask::MoveTMotorToSensorLocation(std::shared_ptr<GenericMotor> &motor, const std::string &motorName, int sensorBit)
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
            checkMotorPosition(motor);
            firstPosition = motor->currentPos;
            std::cout << motorName << " first sensor position: " << firstPosition << endl;
        }
        else if (firstSensorTriggered && !sensorTriggered)
        {
            // 센서 인식 해제
            secondSensorTriggered = true;
            checkMotorPosition(motor);
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
    for (const auto &motor_pair : motors)
    {
        std::cout << motor_pair.first << ": "
                  << (motor_pair.second->isHomed ? "Homed" : "Not Homed") << std::endl;
    }
}

void StateTask::UpdateHomingStatus()
{
    bool allMotorsHomed = true;
    for (const auto &motor_pair : motors)
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

void StateTask::FixMotorPosition(std::shared_ptr<GenericMotor> &motor)
{
    struct can_frame frame;

    checkMotorPosition(motor);

    if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
    {
        tmotorcmd.parseSendCommand(*tMotor, &frame, motor->nodeId, 8, motor->currentPos, 0, 250, 1, 0);
        if (canManager.sendAndRecv(motor, frame))
        {
            std::cout << "Position fixed for motor [" << motor->nodeId << "]." << std::endl;
        }
        else
        {
            std::cerr << "Failed to fix position for motor [" << motor->nodeId << "]." << std::endl;
        }
    }
    else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
    {
        maxoncmd.getTargetPosition(*maxonMotor, &frame, motor->currentPos);
        if (canManager.sendAndRecv(motor, frame))
        {
            std::cout << "Position fixed for motor [" << motor->nodeId << "]." << std::endl;
        }
        else
        {
            std::cerr << "Failed to fix position for motor [" << motor->nodeId << "]." << std::endl;
        }
    }
}

void StateTask::TuningTmotor(float kp, float kd, float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType)
{
    canManager.setSocketsTimeout(0, 50000);

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
    csvFileIn << "0x007,0x001,0x002,0x003,0x004,0x005,0x006,0x008,0x009\n";

    // CSV 파일명 설정
    std::string FileName2 = "../../READ/" + parameter + "_out.txt";

    // CSV 파일 열기
    std::ofstream csvFileOut(FileName2);

    if (!csvFileOut.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    csvFileOut << "CAN_ID,p_act,v_act,tff_act\n"; // CSV 헤더

    struct can_frame frame;

    float peakRadian = peakAngle * M_PI / 180.0; // 피크 각도를 라디안으로 변환
    float amplitude = peakRadian;

    float sample_time = 0.005;
    int max_samples = static_cast<int>(sine_t / sample_time);
    float v_des = 0, p_des = 0;
    float tff_des = 0;
    float p_act, v_act, tff_act;
    std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motors[selectedMotor]);
    for (int cycle = 0; cycle < cycles; cycle++)
    {
        for (int i = 0; i < max_samples; i++)
        {
            float time = i * sample_time;

            if ((int)tMotor->nodeId == 7)
            {
                csvFileIn << std::dec << p_des << "0,0,0,0,0,0,0,0";
            }
            else
            {
                for (int i = 0; i < (int)tMotor->nodeId; i++)
                {
                    csvFileIn << "0,";
                }
                csvFileIn << std::dec << p_des << ",";
                for (int i = 0; i < (8 - (int)tMotor->nodeId); i++)
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

            tmotorcmd.parseSendCommand(*tMotor, &frame, tMotor->nodeId, 8, p_des, v_des, kp, kd, tff_des);
            csvFileOut << "0x" << std::hex << std::setw(4) << std::setfill('0') << tMotor->nodeId;

            chrono::system_clock::time_point external = std::chrono::system_clock::now();
            while (1)
            {
                chrono::system_clock::time_point internal = std::chrono::system_clock::now();
                chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);
                if (elapsed_time.count() >= 5000)
                {

                    if (canManager.sendAndRecv(motors[selectedMotor], frame))
                    {
                        std::tuple<int, float, float, float> result = tmotorcmd.parseRecieveCommand(*tMotor, &frame);

                        p_act = std::get<1>(result);
                        v_act = std::get<2>(result);
                        tff_act = std::get<3>(result);
                        // tff_des = kp * (p_des - p_act) + kd * (v_des - v_act);
                        csvFileOut << ',' << std::dec << p_act << ',' << v_act << ',' << tff_act << '\n';
                        break;
                    }
                }
            }
            csvFileIn << "\n";
        }
    }
    csvFileIn.close();
    csvFileOut.close();
}

void StateTask::TuningMaxonCSP(float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType)
{

    canManager.setSocketsTimeout(0, 50000);
    std::string FileName1 = "../../READ/" + selectedMotor + "_in.txt";

    std::ofstream csvFileIn(FileName1);

    if (!csvFileIn.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    csvFileIn << "0x007,0x001,0x002,0x003,0x004,0x005,0x006,0x008,0x009\n";

    // CSV 파일명 설정
    std::string FileName2 = "../../READ/" + selectedMotor + "_out.txt";

    // CSV 파일 열기
    std::ofstream csvFileOut(FileName2);

    if (!csvFileOut.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }
    csvFileOut << "CAN_ID,p_act,v_act,tff_act\n"; // CSV 헤더

    struct can_frame frame;

    float peakRadian = peakAngle * M_PI / 180.0; // 피크 각도를 라디안으로 변환
    float amplitude = peakRadian;

    float sample_time = 0.005;
    int max_samples = static_cast<int>(sine_t / sample_time);
    float p_des = 0;
    float p_act;
    // float tff_des = 0,v_des = 0;
    float tff_act;

    std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors[selectedMotor]);
    for (int cycle = 0; cycle < cycles; cycle++)
    {
        for (int i = 0; i < max_samples; i++)
        {
            float time = i * sample_time;

            for (int i = 0; i < (int)maxonMotor->nodeId - 1; i++)
            {
                csvFileIn << "0,";
            }
            csvFileIn << std::dec << p_des << ",";
            for (int i = 0; i < (9 - (int)maxonMotor->nodeId); i++)
            {
                csvFileIn << "0,";
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

            maxoncmd.getTargetPosition(*maxonMotor, &frame, p_des);
            csvFileOut << "0x" << std::hex << std::setw(4) << std::setfill('0') << maxonMotor->nodeId;

            chrono::system_clock::time_point external = std::chrono::system_clock::now();
            while (1)
            {
                chrono::system_clock::time_point internal = std::chrono::system_clock::now();
                chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);
                if (elapsed_time.count() >= 5000)
                {

                    ssize_t bytesWritten = write(canManager.sockets.at(maxonMotor->interFaceName), &frame, sizeof(struct can_frame));
                    if (bytesWritten == -1)
                    {
                        std::cerr << "Failed to write to socket for interface: " << maxonMotor->interFaceName << std::endl;
                        std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
                    }
                    canManager.txFrame(motors[selectedMotor], frame);

                    maxoncmd.getSync(&frame);
                    canManager.txFrame(motors[selectedMotor], frame);

                    if (canManager.recvToBuff(motors[selectedMotor], 2))
                    {
                        while (!motors[selectedMotor]->recieveBuffer.empty())
                        {
                            frame = motors[selectedMotor]->recieveBuffer.front();
                            if (frame.can_id == maxonMotor->rxPdoIds[0])
                            {
                                std::tuple<int, float, float> result = maxoncmd.parseRecieveCommand(*maxonMotor, &frame);

                                p_act = std::get<1>(result);
                                tff_act = std::get<2>(result);
                                // tff_des = kp * (p_des - p_act) + kd * (v_des - v_act);
                                csvFileOut << ',' << std::dec << p_act << ", ," << tff_act << '\n';
                                break;
                            }
                            motors[selectedMotor]->recieveBuffer.pop();
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
    for (auto motor_pair : motors)
    {
        FixMotorPosition(motor_pair.second);
    }

    std::string userInput, selectedMotor, fileName;
    float kp, kd, peakAngle;
    float sine_t = 4.0;
    int cycles = 2, pathType, controlType, des_vel, des_tff, direction;

    selectedMotor = motors.begin()->first;

    InitializeParameters(selectedMotor, kp, kd, peakAngle, pathType, controlType, des_vel, des_tff, direction);
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

        std::string controlTypeDescription;
        if (controlType == 1)
        {
            controlTypeDescription = "CSP";

            setMaxonMode("CSP");
        }
        else if (controlType == 2)
        {
            controlTypeDescription = "CSV";

            setMaxonMode("CSV");
        }
        else if (controlType == 3)
        {
            controlTypeDescription = "CST";

            setMaxonMode("CST");
        }
        else if (controlType == 4)
        {
            controlTypeDescription = "Drum Test";
        }

        std::string directionDescription;
        if (direction == 1)
        {
            directionDescription = "CW";
        }
        else if (direction == 2)
        {
            directionDescription = "CCW";
        }

        std::cout << "================ Tuning Menu ================\n";
        std::cout << "Available Motors:\n";

        for (const auto &motor_pair : motors)
        {
            std::cout << " - " << motor_pair.first << "\n";
        }
        bool isMaxonMotor;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors[selectedMotor]))
        {
            isMaxonMotor = true;
        }
        else
        {
            isMaxonMotor = false;
        }

        std::cout << "---------------------------------------------\n";
        std::cout << "Selected Motor: " << selectedMotor << "\n";
        if (!isMaxonMotor)
        {
            std::cout << "Kp: " << kp << " | Kd: " << kd << "\n";
        }
        else
        {
            std::cout << "Control Type : " << controlTypeDescription;
            std::cout << " | Vel [rpm]: " << des_vel << " | Des Torque: " << des_tff << "\n";
        };
        std::cout << "Sine Period: " << sine_t << " | Cycles: " << cycles << " | Hz: " << 1 / sine_t << "\n";
        std::cout << "Peak Angle: " << peakAngle << " | Path Type [Pos]: " << pathTypeDescription << "\n";
        std::cout << "Direction: " << directionDescription << "\n";
        std::cout << "\nCommands:\n";
        if (!isMaxonMotor)
        {
            std::cout << "[a]: kp | [b]: kd |";
        }
        else
        {
            std::cout << "[a]: des_vel | [b]: des_tff | [c]: Control |\n";
        }
        std::cout << "[d]: Select Motor | [e]: Peak | [f]: Path\n";
        std::cout << "[g]: Period | [h]: Cycles | [i]: Run \n";
        if (isMaxonMotor)
        {
            std::cout << "[k]: Direction | ";
        }
        std::cout << "[j]: Exit\n";
        std::cout << "=============================================\n";
        std::cout << "Enter Command: ";
        std::cin >> userInput;
        std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);

        if (userInput[0] == 'j')
        {
            break;
        }
        else if (userInput[0] == 'd')
        {
            while (true)
            {
                std::cout << "Enter the name of the motor to tune: ";
                std::cin >> selectedMotor;
                if (motors.find(selectedMotor) != motors.end())
                {
                    InitializeParameters(selectedMotor, kp, kd, peakAngle, pathType, controlType, des_vel, des_tff, direction);
                    break;
                }
                else
                {
                    std::cout << "Invalid motor name. Please enter a valid motor name.\n";
                }
            }
        }
        else if (userInput == "k" && isMaxonMotor)
        {
            std::cout << "Enter Desired Direction: ";
            std::cout << "1: Clock Wise\n";
            std::cout << "2: Counter Clock Wise\n";
            std::cout << "Enter Path Type (1 or 2): ";
            std::cin >> direction;

            if (direction != 1 && direction != 2)
            {
                std::cout << "Invalid direction type. Please enter 1 or 2.\n";
                pathType = 1;
            }

            std::cin >> direction;
        }
        else if (userInput == "a" && !isMaxonMotor)
        {
            std::cout << "Enter Desired Kp: ";
            std::cin >> kp;
        }
        else if (userInput == "b" && !isMaxonMotor)
        {
            std::cout << "Enter Desired Kd: ";
            std::cin >> kd;
        }
        else if (userInput == "a" && isMaxonMotor)
        {
            std::cout << "Enter Desired Velocity: ";
            std::cin >> des_vel;
        }
        else if (userInput == "b" && isMaxonMotor)
        {
            std::cout << "Enter Desired Torque Unit: ";
            std::cout << "1 [unit] = ";

            std::cin >> des_tff;
        }
        else if (userInput == "e")
        {
            std::cout << "Enter Desired Peak Angle: ";
            std::cin >> peakAngle;
        }
        else if (userInput == "f")
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
        else if (userInput[0] == 'g')
        {
            std::cout << "Enter Desired Sine Period: ";
            std::cin >> sine_t;
        }
        else if (userInput[0] == 'h')
        {
            std::cout << "Enter Desired Cycles: ";
            std::cin >> cycles;
        }
        else if (userInput == "c" && isMaxonMotor)
        {
            std::cout << "Select Control Type:\n";
            std::cout << "1: Cyclic Synchronous Position Mode (CSP)\n";
            std::cout << "2: Cyclic Synchronous Velocity Mode (CSV)\n";
            std::cout << "3: Cyclic Synchronous Torque Mode (CST)\n";
            std::cout << "4: Maxon Drum Test (CSP)\n";
            std::cout << "Enter Path Type (1 or 2 or 3 or 4): ";
            std::cin >> controlType;

            if (controlType != 1 && controlType != 2 && controlType != 3 && controlType != 4)
            {
                std::cout << "Invalid path type. Please enter 1 or 2 or 3 or 4.\n";
                controlType = 1;
            }
        }
        else if (userInput[0] == 'i')
        {
            if (!isMaxonMotor) // Tmotor일 경우
            {
                TuningTmotor(kp, kd, sine_t, selectedMotor, cycles, peakAngle, pathType);
            }
            else // MaxonMotor일 경우
            {
                if (controlType == 1)
                {

                    TuningMaxonCSP(sine_t, selectedMotor, cycles, peakAngle, pathType);
                }
                else if (controlType == 2)
                {
                    TuningMaxonCSV(selectedMotor, des_vel, direction);
                }
                else if (controlType == 3)
                {

                    TuningMaxonCST(selectedMotor, des_tff, direction);
                }
                else if (controlType == 4)
                {
                    //
                }
                MaxonQuickStopEnable();
            }
        }
    }
}

void StateTask::InitializeParameters(const std::string selectedMotor, float &kp, float &kd, float &peakAngle, int &pathType, int &controlType, int &des_vel, int &des_tff, int &direction)
{
    if (selectedMotor == "waist")
    {
        kp = 200.0;
        kd = 1.0;
        peakAngle = 30;
        pathType = 2;
        des_vel = 0;
        des_tff = 0;
    }
    else if (selectedMotor == "R_arm1" || selectedMotor == "L_arm1" ||
             selectedMotor == "R_arm2" || selectedMotor == "R_arm3" ||
             selectedMotor == "L_arm2" || selectedMotor == "L_arm3")
    {
        kp = 50.0; // 예시 값, 실제 필요한 값으로 조정
        kd = 1.0;  // 예시 값, 실제 필요한 값으로 조정
        peakAngle = 90;
        pathType = 1;
        des_vel = 0;
        des_tff = 0;
    }
    else if (selectedMotor == "L_wrist" || selectedMotor == "R_wrist")
    {
        peakAngle = 90;
        pathType = 1;
        controlType = 1;
        direction = 1;
        des_vel = 0;
        des_tff = 0;
    }
    // 추가적인 모터 이름과 매개변수를 이곳에 추가할 수 있습니다.
}

void StateTask::TuningMaxonCSV(const std::string selectedMotor, int des_vel, int direction)
{
    des_vel = des_vel * 35;

    if (direction == 1)
        des_vel *= 1;
    else
        des_vel *= -1;

    canManager.setSocketsTimeout(0, 50000);
    std::string FileName1 = "../../READ/" + selectedMotor + "_csv_in.txt";

    std::ofstream csvFileIn(FileName1);

    if (!csvFileIn.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    csvFileIn << "0x007,0x001,0x002,0x003,0x004,0x005,0x006,0x008,0x009\n";

    // CSV 파일명 설정
    std::string FileName2 = "../../READ/" + selectedMotor + "_csv_out.txt";

    // CSV 파일 열기
    std::ofstream csvFileOut(FileName2);

    if (!csvFileOut.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }
    csvFileOut << "CAN_ID,pos_act,tff_act\n"; // CSV 헤더

    struct can_frame frame;

    float p_act, tff_act;
    char input = 'a';
    std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors[selectedMotor]);

    for (int i = 0; i < (int)maxonMotor->nodeId - 1; i++)
    {
        csvFileIn << "0,";
    }
    csvFileIn << std::dec << des_vel << ",";
    for (int i = 0; i < (9 - (int)maxonMotor->nodeId); i++)
    {
        csvFileIn << "0,";
    }

    maxoncmd.getTargetVelocity(*maxonMotor, &frame, des_vel);

    chrono::system_clock::time_point external = std::chrono::system_clock::now();
    while (1)
    {

        if (kbhit())
        {
            input = getchar();
        }

        if (input == 'q')
            continue;
        else if (input == 'e')
            break;

        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);
        if (elapsed_time.count() >= 5000)
        {

            ssize_t bytesWritten = write(canManager.sockets.at(maxonMotor->interFaceName), &frame, sizeof(struct can_frame));
            if (bytesWritten == -1)
            {
                std::cerr << "Failed to write to socket for interface: " << maxonMotor->interFaceName << std::endl;
                std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
            }

            maxoncmd.getSync(&frame);
            bytesWritten = write(canManager.sockets.at(maxonMotor->interFaceName), &frame, sizeof(struct can_frame));
            if (bytesWritten == -1)
            {
                std::cerr << "Failed to write to socket for interface: " << maxonMotor->interFaceName << std::endl;
                std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
            }
            ssize_t bytesRead = read(canManager.sockets.at(maxonMotor->interFaceName), &frame, sizeof(struct can_frame));

            if (bytesRead == -1)
            {
                std::cerr << "Failed to read from socket for interface: " << maxonMotor->interFaceName << std::endl;
                return;
            }
            else
            {

                std::tuple<int, float, float> result = maxoncmd.parseRecieveCommand(*maxonMotor, &frame);

                p_act = std::get<1>(result);
                tff_act = std::get<2>(result);
                // tff_des = kp * (p_des - p_act) + kd * (v_des - v_act);
                csvFileOut << "0x" << std::hex << std::setw(4) << std::setfill('0') << maxonMotor->nodeId;
                csvFileOut << ',' << std::dec << p_act << ", ," << tff_act << '\n';
            }
        }
    }

    csvFileIn.close();
    csvFileOut.close();
}

void StateTask::TuningMaxonCST(const std::string selectedMotor, int des_tff, int direction)
{
    if (direction == 1)
        des_tff *= 1;
    else
        des_tff *= -1;

    canManager.setSocketsTimeout(0, 50000);
    std::string FileName1 = "../../READ/" + selectedMotor + "_cst_in.txt";

    std::ofstream csvFileIn(FileName1);

    if (!csvFileIn.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    csvFileIn << "0x007,0x001,0x002,0x003,0x004,0x005,0x006,0x008,0x009\n";

    // CSV 파일명 설정
    std::string FileName2 = "../../READ/" + selectedMotor + "_cst_out.txt";

    // CSV 파일 열기
    std::ofstream csvFileOut(FileName2);

    if (!csvFileOut.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }
    csvFileOut << "CAN_ID,pos_act,tff_act\n"; // CSV 헤더

    struct can_frame frame;

    float p_act, tff_act;
    char input = 'a';
    std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors[selectedMotor]);

    for (int i = 0; i < (int)maxonMotor->nodeId - 1; i++)
    {
        csvFileIn << "0,";
    }
    csvFileIn << std::dec << des_tff << ",";
    for (int i = 0; i < (9 - (int)maxonMotor->nodeId); i++)
    {
        csvFileIn << "0,";
    }

    maxoncmd.getTargetTorque(*maxonMotor, &frame, des_tff);

    chrono::system_clock::time_point external = std::chrono::system_clock::now();
    while (1)
    {

        if (kbhit())
        {
            input = getchar();
        }

        if (input == 'e')
            break;

        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);
        if (elapsed_time.count() >= 5000)
        {

            ssize_t bytesWritten = write(canManager.sockets.at(maxonMotor->interFaceName), &frame, sizeof(struct can_frame));
            if (bytesWritten == -1)
            {
                std::cerr << "Failed to write to socket for interface: " << maxonMotor->interFaceName << std::endl;
                std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
            }

            maxoncmd.getSync(&frame);
            bytesWritten = write(canManager.sockets.at(maxonMotor->interFaceName), &frame, sizeof(struct can_frame));
            if (bytesWritten == -1)
            {
                std::cerr << "Failed to write to socket for interface: " << maxonMotor->interFaceName << std::endl;
                std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
            }
            ssize_t bytesRead = read(canManager.sockets.at(maxonMotor->interFaceName), &frame, sizeof(struct can_frame));

            if (bytesRead == -1)
            {
                std::cerr << "Failed to read from socket for interface: " << maxonMotor->interFaceName << std::endl;
                return;
            }
            else
            {

                std::tuple<int, float, float> result = maxoncmd.parseRecieveCommand(*maxonMotor, &frame);

                p_act = std::get<1>(result);
                tff_act = std::get<2>(result);
                // tff_des = kp * (p_des - p_act) + kd * (v_des - v_act);
                csvFileOut << "0x" << std::hex << std::setw(4) << std::setfill('0') << maxonMotor->nodeId;
                csvFileOut << ',' << std::dec << p_act << ", ," << tff_act << '\n';
            }
        }
    }

    csvFileIn.close();
    csvFileOut.close();
}

void StateTask::MaxonEnable()
{
    struct can_frame frame;
    canManager.setSocketsTimeout(2, 0);

    int maxonMotorCount = 0;
    for (const auto &motor_pair : motors)
    {
        // 각 요소가 MaxonMotor 타입인지 확인
        if (std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            maxonMotorCount++;
        }
    }

    // 제어 모드 설정
    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {

            maxoncmd.getOperational(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            if (canManager.recvToBuff(motor, 2))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {
                        std::cout << "Maxon Enabled \n";
                    }
                    motor->recieveBuffer.pop();
                }
            }

            maxoncmd.getQuickStop(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            if (canManager.recvToBuff(motor, 2))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {
                        std::cout << "Maxon Quick Stopped\n";
                    }
                    motor->recieveBuffer.pop();
                }
            }

            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            if (canManager.recvToBuff(motor, 2))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {
                        std::cout << "Maxon Enabled \n";
                    }
                    motor->recieveBuffer.pop();
                }
            }
        }
    }
};

void StateTask::MaxonQuickStopEnable()
{
    struct can_frame frame;
    canManager.setSocketsTimeout(2, 0);

    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motorPair.second))
        {
            maxoncmd.getQuickStop(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            if (canManager.recvToBuff(motor, 2))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {
                        std::cout << "Maxon Quick Stopped\n";
                    }
                    motor->recieveBuffer.pop();
                }
            }

            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            if (canManager.recvToBuff(motor, 2))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {
                        std::cout << "Maxon Enabled \n";
                    }
                    motor->recieveBuffer.pop();
                }
            }
        }
    }
}

void StateTask::maxonSdoSetting()
{
    struct can_frame frame;
    canManager.setSocketsTimeout(0, 5000);
    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motorPair.second))
        {

            // CSP Settings
            maxoncmd.getCSVMode(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            maxoncmd.getPosOffset(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            maxoncmd.getTorqueOffset(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            // CSV Settings
            maxoncmd.getCSVMode(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            maxoncmd.getVelOffset(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            // CST Settings
            maxoncmd.getCSTMode(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            maxoncmd.getTorqueOffset(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            // HMM Settigns
            maxoncmd.getHomeMode(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            if (name == "L_wrist")
            {

                maxoncmd.getHomingMethodL(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else
            {
                maxoncmd.getHomingMethodR(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }

            maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            maxoncmd.getCurrentThreshold(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            maxoncmd.getHomePosition(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);
        }
    }
}

void StateTask::setMaxonMode(std::string targetMode)
{
    struct can_frame frame;
    canManager.setSocketsTimeout(2, 0);
    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motorPair.second))
        {
            if (targetMode == "CSV")
            {
                maxoncmd.getCSVMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else if (targetMode == "CST")
            {
                maxoncmd.getCSTMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else if (targetMode == "HMM")
            {
                maxoncmd.getHomeMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else if (targetMode == "CSP")
            {
                maxoncmd.getCSPMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
        }
    }
}