#include "../include/tasks/DrumRobot.hpp"

// DrumRobot 클래스의 생성자
DrumRobot::DrumRobot(SystemState &systemStateRef,
                     CanManager &canManagerRef,
                     std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : systemState(systemStateRef), canManager(canManagerRef), motors(motorsRef), testmanager(systemStateRef, canManagerRef, motors) {}

/////////////////////////////////////////////////////////////////////////////////
/*                               SYSTEM LOOPS                             */
///////////////////////////////////////////////////////////////////////////////

void DrumRobot::operator()()
{
    while (systemState.main != Main::Shutdown)
    {
        Main currentState = systemState.main.load();
        switch (currentState)
        {
        case Main::SystemInit:
            initializeMotors();
            initializecanManager();
            motorSettingCmd();
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
            canManager.checkAllMotors();
            printCurrentPositions();
            std::cout << "Press Enter to Go home\n";
            getchar();
            systemState.main = Main::Ideal;
            break;
        case Main::Tune:
            MaxonEnable();
            // TuningLoopTask();
            canManager.checkAllMotors();
            setMaxonMode("CSP");
            systemState.testMode = TestMode::MultiMode;
            testmanager.mainLoop();
            systemState.main = Main::Ideal;
            break;
        case Main::Shutdown:
            std::cout << "======= Shut down system =======\n";
            break;
        case Main::Back:
            break;
        case Main::Ready:
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

void DrumRobot::homeModeLoop()
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

void DrumRobot::runModeLoop()
{
    MaxonEnable();
    setMaxonMode("CSP");
    while (systemState.main == Main::Perform)
    {
        switch (systemState.runMode.load())
        {
        case RunMode::PrePreparation:
            break;
        case RunMode::Ready:
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

void DrumRobot::displayAvailableCommands() const
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

            std::cout << "- r : Move to Ready Position\n";
        }
        else if (systemState.homeMode == HomeMode::HomeDone && systemState.runMode == RunMode::Ready)
        {
            std::cout << "- p : Start Perform\n";
            std::cout << "- t : Start tuning\n";
            std::cout << "- b : Back to Zero Postion\n";
        }
    }
    else if (systemState.main == Main::Perform)
    {
        // 나중에 필요하면 추가
    }

    std::cout << "- s : Shut down the system\n";
    std::cout << "- c : Check Motors position\n";
}

bool DrumRobot::processInput(const std::string &input)
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
        else if (input == "r" && systemState.homeMode == HomeMode::HomeDone && systemState.runMode == RunMode::PrePreparation)
        {
            systemState.main = Main::Ready;
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
        if (input == "p" && systemState.homeMode == HomeMode::HomeDone && systemState.runMode == RunMode::Ready)
        {
            systemState.main = Main::Perform;
            systemState.runMode = RunMode::Running;
            return true;
        }
        else if (input == "b" && systemState.homeMode == HomeMode::HomeDone && systemState.runMode == RunMode::Ready)
        {
            systemState.main = Main::Back;
            return true;
        }
    }

    return false;
}

void DrumRobot::idealStateRoutine()
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

void DrumRobot::checkUserInput()
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

void DrumRobot::initializeMotors()
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
                tMotor->Kp = 350;
                tMotor->isHomed = true;
                tMotor->interFaceName = "can0";
            }
            else if (motor_pair.first == "R_arm1")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 3;
                tMotor->rMin = -M_PI; // -180deg
                tMotor->rMax = 0.0f;  // 0deg
                tMotor->Kp = 200;
                tMotor->isHomed = false;
                tMotor->interFaceName = "can1";
            }
            else if (motor_pair.first == "L_arm1")
            {
                tMotor->cwDir = 1.0f;
                tMotor->sensorBit = 0;
                tMotor->rMin = 0.0f; // 0deg
                tMotor->rMax = M_PI; // 180deg
                tMotor->Kp = 200;
                tMotor->isHomed = false;
                tMotor->interFaceName = "can0";
            }
            else if (motor_pair.first == "R_arm2")
            {
                tMotor->cwDir = 1.0f;
                tMotor->sensorBit = 4;
                tMotor->rMin = -M_PI / 4.0f; // -45deg
                tMotor->rMax = M_PI / 2.0f;  // 90deg
                tMotor->Kp = 250;
                tMotor->isHomed = false;
                tMotor->interFaceName = "can1";
            }
            else if (motor_pair.first == "R_arm3")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 5;
                tMotor->rMin = -M_PI * 0.75f; // -135deg
                tMotor->rMax = 0.0f;          // 0deg
                tMotor->Kp = 200;
                tMotor->isHomed = false;
                tMotor->interFaceName = "can1";
            }
            else if (motor_pair.first == "L_arm2")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 1;
                tMotor->rMin = -M_PI / 2.0f; // -90deg
                tMotor->rMax = M_PI / 4.0f;  // 45deg
                tMotor->Kp = 250;
                tMotor->isHomed = false;
                tMotor->interFaceName = "can0";
            }
            else if (motor_pair.first == "L_arm3")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 2;
                tMotor->rMin = -M_PI * 0.75f; // -135deg
                tMotor->rMax = 0.0f;          // 0deg
                tMotor->Kp = 200;
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

void DrumRobot::initializecanManager()
{
    canManager.initializeCAN();
    canManager.checkCanPortsStatus();
    canManager.setMotorsSocket();
}

void DrumRobot::DeactivateControlTask()
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

void DrumRobot::printCurrentPositions()
{
    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;
        std::cout << "[" << std::hex << motor->nodeId << std::dec << "] ";
        std::cout << name << " : " << motor->currentPos << endl;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                                  HOME                                      */
///////////////////////////////////////////////////////////////////////////////

bool DrumRobot::PromptUserForHoming(const std::string &motorName)
{
    char userResponse;
    std::cout << "Would you like to start homing mode for motor [" << motorName << "]? (y/n): ";
    std::cin >> userResponse;
    return userResponse == 'y';
}

void DrumRobot::RotateTMotor(std::shared_ptr<GenericMotor> &motor, const std::string &motorName, double direction, double degree, float midpoint)
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

    canManager.checkConnection(motor);
}

void DrumRobot::HomeTMotor(std::shared_ptr<GenericMotor> &motor, const std::string &motorName)
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
        additionalTorque = motor->cwDir * 2.0;
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
        canManager.checkConnection(motor);
        RotateTMotor(motor, motorName, motor->cwDir, 90, 0);
    }
    /*  // homing 잘 됐는지 센서 위치로 다시 돌아가서 확인
    if(motorName == "L_arm2" || motorName == "R_arm2")
    {
        canManager.checkConnection(motor);
        RotateTMotor(motor, motorName, motor->cwDir, -30, 0);
    }*/
    if (motorName == "L_arm3" || motorName == "R_arm3")
    {
        canManager.checkConnection(motor);
        RotateTMotor(motor, motorName, motor->cwDir, 90, 0);
    }
}

void DrumRobot::SetTmotorHome(std::shared_ptr<GenericMotor> &motor, const std::string &motorName)
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

void DrumRobot::SetMaxonHome(std::shared_ptr<GenericMotor> &motor, const std::string &motorName)
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

float DrumRobot::MoveTMotorToSensorLocation(std::shared_ptr<GenericMotor> &motor, const std::string &motorName, int sensorBit)
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
            canManager.checkConnection(motor);
            firstPosition = motor->currentPos;
            std::cout << motorName << " first sensor position: " << firstPosition << endl;
        }
        else if (firstSensorTriggered && !sensorTriggered)
        {
            // 센서 인식 해제
            secondSensorTriggered = true;
            canManager.checkConnection(motor);
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

void DrumRobot::displayHomingStatus()
{
    std::cout << "Homing Status of Motors:\n";
    for (const auto &motor_pair : motors)
    {
        std::cout << motor_pair.first << ": "
                  << (motor_pair.second->isHomed ? "Homed" : "Not Homed") << std::endl;
    }
}

void DrumRobot::UpdateHomingStatus()
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

void DrumRobot::MaxonEnable()
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

void DrumRobot::MaxonQuickStopEnable()
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

void DrumRobot::motorSettingCmd()
{
    struct can_frame frame;
    canManager.setSocketsTimeout(2, 0);
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
            else if (name == "R_wrist")
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
        else if (std::shared_ptr<TMotor> tmotor = std::dynamic_pointer_cast<TMotor>(motorPair.second))
        {
            if (name == "waist")
            {
                tmotorcmd.getZero(*tmotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
        }
    }
    std::cout << "Maxon SDO Set\n";
}

void DrumRobot::setMaxonMode(std::string targetMode)
{
    struct can_frame frame;
    canManager.setSocketsTimeout(0, 10000);
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

void DrumRobot::FixMotorPosition(std::shared_ptr<GenericMotor> &motor){
    struct can_frame frame;

    canManager.checkConnection(motor);

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
/////////////////////////////////////////////////////////////////////////////////
/*                                  TUNE                                      */
///////////////////////////////////////////////////////////////////////////////
