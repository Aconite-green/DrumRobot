#include "../include/tasks/DrumRobot.hpp"

// DrumRobot 클래스의 생성자
DrumRobot::DrumRobot(State &stateRef,
                     CanManager &canManagerRef,
                     PathManager &pathManagerRef,
                     HomeManager &homeManagerRef,
                     TestManager &testManagerRef,
                     std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : state(stateRef),
      canManager(canManagerRef),
      pathManager(pathManagerRef),
      homeManager(homeManagerRef),
      testManager(testManagerRef),
      motors(motorsRef)
{
    ReadStandard = chrono::system_clock::now();
    SendStandard = chrono::system_clock::now();
}

/////////////////////////////////////////////////////////////////////////////////
/*                               SYSTEM LOOPS                             */
///////////////////////////////////////////////////////////////////////////////

void DrumRobot::stateMachine()
{

    while (state.main != Main::Shutdown)
    {
        switch (state.main.load())
        {
        case Main::SystemInit:
            initializeMotors();
            initializecanManager();
            motorSettingCmd();
            canManager.setSocketNonBlock();
            std::cout << "System Initialize Complete [ Press Enter ]\n";
            getchar();
            state.main = Main::Ideal;
            break;
        case Main::Ideal:
            idealStateRoutine();
            break;
        case Main::Homing:
            canManager.setSocketBlock();
            homeManager.mainLoop();
            canManager.setSocketNonBlock();
            break;
        case Main::Perform:
            checkUserInput();
            break;
        case Main::Check:
            canManager.setSocketBlock();
            canManager.checkAllMotors();
            printCurrentPositions();
            state.main = Main::Ideal;
            canManager.setSocketNonBlock();
            break;
        case Main::Tune:
            MaxonEnable();
            testManager.mainLoop();
            MaxonDisable();
            break;
        case Main::Shutdown:
            usleep(500000);
            break;
        case Main::Ready:
            usleep(500000);
            break;
        case Main::Back:
            usleep(500000);
            break;
        case Main::Pause:
            checkUserInput();
            break;
        case Main::AddStance:
            checkUserInput();
            break;
        case Main::Error:
            std::cout << "In error state by Postion diff\n";
            sleep(2);
            break;
        }
    }
}

void DrumRobot::sendLoopForThread()
{
    initializePathManager();
    while (state.main != Main::Shutdown)
    {
        switch (state.main.load())
        {
        case Main::SystemInit:
            usleep(500000);
            break;
        case Main::Ideal:
            canManager.checkCanPortsStatus();
            canManager.checkAllMotors();
            usleep(500000);
            break;
        case Main::Homing:
            usleep(500000);
            break;
        case Main::Perform:
            // SendLoop();
            SendPerformProcess(5000);
            break;
        case Main::AddStance:
            usleep(5000);
            // SendAddStanceProcess(5000);
            break;
        case Main::Check:
            usleep(500000);
            break;
        case Main::Tune:
            usleep(500000);
            break;
        case Main::Shutdown:
            save_to_txt_inputData("../../READ/DrumData_in.txt");
            break;
        case Main::Pause:
            usleep(5000);
            break;
        case Main::Ready:
            // 이 State는 삭제예정
            if (!isReady)
            {
                if (canManager.checkAllMotors())
                {
                    MaxonEnable();
                    setMaxonMode("CSP");
                    std::cout << "Get Ready...\n";
                    clearMotorsSendBuffer();
                    pathManager.GetArr(pathManager.standby);
                    SendReadyLoop();
                    isReady = true;
                }
            }
            else
                idealStateRoutine();
            break;
        case Main::Back:
            // 이 State는 삭제예정
            if (!isBack)
            {
                if (canManager.checkAllMotors())
                {
                    std::cout << "Get Back...\n";
                    clearMotorsSendBuffer();
                    pathManager.GetArr(pathManager.backarr);
                    SendReadyLoop();
                    isBack = true;
                }
            }
            else
            {
                state.main = Main::Shutdown;
                DeactivateControlTask();
            }
            break;
        case Main::Error:
            sleep(2);
            break;
        }
    }
}

void DrumRobot::SendPerformProcess(int periodMicroSec)
{
    auto currentTime = chrono::system_clock::now();
    auto elapsed_time = chrono::duration_cast<chrono::microseconds>(currentTime - SendStandard);

    switch (state.perform.load())
    {
    case PerformSub::TimeCheck:
    {
        if (elapsed_time.count() >= periodMicroSec)
        {
            state.perform = PerformSub::CheckBuf; // 주기가 되면 ReadCANFrame 상태로 진입
            SendStandard = currentTime;           // 현재 시간으로 시간 객체 초기화
        }
        break;
    }
    case PerformSub::CheckBuf:
    {
        for (const auto &motor_pair : motors)
        {
            if (motor_pair.second->sendBuffer.size() < 10)
                state.perform = PerformSub::GeneratePath;
            else
                state.perform = PerformSub::SafetyCheck;
            break;
        }
        break;
    }
    case PerformSub::GeneratePath:
    {
        if (pathManager.line < pathManager.total)
        {
            std::cout << "line : " << pathManager.line << ", total : " << pathManager.total << "\n";
            pathManager.PathLoopTask();
            pathManager.line++;
            state.perform = PerformSub::CheckBuf;
        }
        else if (pathManager.line == pathManager.total)
        {
            std::cout << "Perform Done\n";
            state.main = Main::Ready;
            pathManager.line = 0;
        }
        break;
    }
    case PerformSub::SafetyCheck:
        state.perform = PerformSub::SendCANFrame;
        break;
    case PerformSub::BufEmpty:
    {
        bool allBuffersEmpty = true;
        for (const auto &motor_pair : motors)
        {
            if (!motor_pair.second->sendBuffer.empty())
            {
                allBuffersEmpty = false;
                break;
            }
        }
        if (allBuffersEmpty)
        {
            std::cout << "Performance is Over\n";
            state.main = Main::Ideal;
        }
        break;
    }
    case PerformSub::SafetyCheck:
        state.perform = PerformSub::SendCANFrame;
        break;
    case PerformSub::SendCANFrame:
    {
        for (auto &motor_pair : motors)
        {
            shared_ptr<GenericMotor> motor = motor_pair.second;
            canManager.sendMotorFrame(motor);
        }
        if (maxonMotorCount != 0)
        {
            maxoncmd.getSync(&frame);
            canManager.txFrame(virtualMaxonMotor, frame);
        }
        state.perform = PerformSub::TimeCheck;
        break;
    }
    }
}

void DrumRobot::recvLoopForThread()
{

    while (state.main != Main::Shutdown)
    {
        switch (state.main.load())
        {
        case Main::SystemInit:
            usleep(500000);
            break;
        case Main::Ideal:
            usleep(500000);
            // ReadProcess(500000); /*500ms*/
            break;
        case Main::Homing:
            usleep(500000);
            // ReadProcess(5000); /*5ms*/
            break;
        case Main::Perform:
            ReadProcess(5000);
            break;
        case Main::AddStance:
            usleep(5000);
            // ReadProcess(5000);
            break;
        case Main::Check:
            usleep(500000);
            break;
        case Main::Tune:
            usleep(500000);
            break;
        case Main::Shutdown:
            parse_and_save_to_csv("../../READ/DrumData_out.txt");
            break;
        case Main::Pause:
            ReadProcess(5000); /*5ms*/
            break;
        case Main::Ready:
            usleep(500000);
            // 이 State는 삭제예정
            break;
        case Main::Back:
            usleep(500000);
            // 이 State는 삭제예정
            break;
        case Main::Error:
            usleep(500000);
            // 이 State는 삭제예정
            break;
        }
    }
}

void DrumRobot::ReadProcess(int periodMicroSec)
{
    auto currentTime = chrono::system_clock::now();
    auto elapsed_time = chrono::duration_cast<chrono::microseconds>(currentTime - ReadStandard);

    switch (state.read.load())
    {
    case ReadSub::TimeCheck:
        if (elapsed_time.count() >= periodMicroSec)
        {
            state.read = ReadSub::ReadCANFrame; // 주기가 되면 ReadCANFrame 상태로 진입
            ReadStandard = currentTime;         // 현재 시간으로 시간 객체 초기화
        }
        break;
    case ReadSub::ReadCANFrame:
        canManager.readFramesFromAllSockets(); // CAN frame 읽기
        state.read = ReadSub::UpdateMotorInfo; // 다음 상태로 전환
        break;

    case ReadSub::UpdateMotorInfo:
        canManager.distributeFramesToMotors(); // 읽은 데이터를 모터 정보로 업데이트
        state.read = ReadSub::TimeCheck;       // 작업 완료 상태로 전환
        break;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                                STATE UTILITY                               */
///////////////////////////////////////////////////////////////////////////////

void DrumRobot::displayAvailableCommands() const
{
    std::cout << "Available Commands:\n";

    if (state.main == Main::Ideal)
    {
        if (!(state.home == HomeSub::Done))
        {
            std::cout << "- h : Start Homing Mode\n";
            std::cout << "- x : Make home state by user\n";
        }
        else
        {
            std::cout << "- r : Move to Ready Position\n";
            std::cout << "- t : Start tuning\n";
        }
    }
    else if (state.main == Main::Ready)
    {
        std::cout << "- p : Start Perform\n";
        std::cout << "- t : Start tuning\n";
    }
    std::cout << "- s : Shut down the system\n";
    std::cout << "- c : Check Motors position\n";
}

bool DrumRobot::processInput(const std::string &input)
{
    if (state.main == Main::Ideal)
    {
        if (input == "h" && !(state.home == HomeSub::Done))
        {
            state.main = Main::Homing;
            return true;
        }
        else if (input == "t" && state.home == HomeSub::Done)
        {
            state.main = Main::Tune;
            return true;
        }
        else if (input == "r" && state.home == HomeSub::Done)
        {
            state.main = Main::Ready;
            return true;
        }
        else if (input == "x" && !(state.home == HomeSub::Done))
        {
            state.home = HomeSub::Done;
            return true;
        }
        else if (input == "c")
        {
            state.main = Main::Check;
            return true;
        }
        else if (input == "s")
        {
            if (state.home == HomeSub::Done)
            {
                state.main = Main::Back;
            }
            else
            {
                state.main = Main::Shutdown;
            }
            return true;
        }
    }
    else if (state.main == Main::Ready)
    {
        if (input == "p")
        {
            state.main = Main::Perform;
            return true;
        }
        else if (input == "s")
        {
            state.main = Main::Back;
            return true;
        }
        else if (input == "t")
        {
            state.main = Main::Tune;
            return true;
        }
        else if (input == "c")
        {
            state.main = Main::Check;
            return true;
        }
    }

    return false;
}

void DrumRobot::idealStateRoutine()
{
    int ret = system("clear");
    if (ret == -1)
        std::cout << "system clear error" << endl;

    displayAvailableCommands();

    std::string input;
    std::cout << "Enter command: ";
    std::getline(std::cin, input);

    if (!processInput(input))
        std::cout << "Invalid command or not allowed in current state!\n";

    usleep(2000);
}

void DrumRobot::checkUserInput()
{
    if (kbhit())
    {
        char input = getchar();
        if (input == 'q')
            state.main = Main::Pause;
        else if (input == 'e')
        {
            isReady = false;
            state.main = Main::Ready;
            pathManager.line = 0;
        }
        else if (input == 'r')
            state.main = Main::Perform;
    }
    usleep(500000);
}

int DrumRobot::kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
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
    motors["maxonForTest"] = make_shared<MaxonMotor>(0x00A);

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
                tMotor->Kp = 400;
                tMotor->Kd = 3.5;
                tMotor->isHomed = true;
            }
            else if (motor_pair.first == "R_arm1")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 3;
                tMotor->rMin = -M_PI; // -180deg
                tMotor->rMax = 0.0f;  // 0deg
                tMotor->Kp = 200;
                tMotor->Kd = 2.5;
                tMotor->isHomed = false;
            }
            else if (motor_pair.first == "L_arm1")
            {
                tMotor->cwDir = 1.0f;
                tMotor->sensorBit = 0;
                tMotor->rMin = 0.0f; // 0deg
                tMotor->rMax = M_PI; // 180deg
                tMotor->Kp = 200;
                tMotor->Kd = 2.5;
                tMotor->isHomed = false;
            }
            else if (motor_pair.first == "R_arm2")
            {
                tMotor->cwDir = 1.0f;
                tMotor->sensorBit = 4;
                tMotor->rMin = -M_PI / 4.0f; // -45deg
                tMotor->rMax = M_PI / 2.0f;  // 90deg
                tMotor->Kp = 350;
                tMotor->Kd = 3.5;
                tMotor->isHomed = false;
            }
            else if (motor_pair.first == "R_arm3")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 5;
                tMotor->rMin = -M_PI * 0.75f; // -135deg
                tMotor->rMax = 0.0f;          // 0deg
                tMotor->Kp = 250;
                tMotor->Kd = 3.5;
                tMotor->isHomed = false;
            }
            else if (motor_pair.first == "L_arm2")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 1;
                tMotor->rMin = -M_PI / 2.0f; // -90deg
                tMotor->rMax = M_PI / 4.0f;  // 45deg
                tMotor->Kp = 350;
                tMotor->Kd = 3.5;
                tMotor->isHomed = false;
            }
            else if (motor_pair.first == "L_arm3")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 2;
                tMotor->rMin = -M_PI * 0.75f; // -135deg
                tMotor->rMax = 0.0f;          // 0deg
                tMotor->Kp = 250;
                tMotor->Kd = 3.5;
                tMotor->isHomed = false;
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
            }
            else if (motor_pair.first == "maxonForTest")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = 0.0f; // 0deg
                maxonMotor->rMax = M_PI; // 180deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x20A; // Controlword
                maxonMotor->txPdoIds[1] = 0x30A; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x40A; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x50A; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x18A; // Statusword, ActualPosition, ActualTorque
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
                std::cout << "Exiting for motor [" << name << "]" << std::endl;
            else
                std::cerr << "Failed to exit control mode for motor [" << name << "]." << std::endl;
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            maxoncmd.getQuickStop(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);
            if (canManager.recvToBuff(motor, canManager.maxonCnt))
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
                std::cerr << "Failed to exit for motor [" << name << "]." << std::endl;
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

    vector<double> P(6);
    P = pathManager.fkfun();

    std::cout << "Right Hand Position : { " << P[0] << " , " << P[1] << " , " << P[2] << " }\n";
    std::cout << "Left Hand Position : { " << P[3] << " , " << P[4] << " , " << P[5] << " }\n";
    printf("Print Enter to Go Home\n");
    getchar();
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

void DrumRobot::motorSettingCmd()
{

    // Count Maxon Motors
    for (const auto &motor_pair : motors)
    {
        // 각 요소가 MaxonMotor 타입인지 확인
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            maxonMotorCount++;
        }
    }

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

                maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame, 95);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomePosition(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);
            }
            else if (name == "R_wrist")
            {
                maxoncmd.getHomingMethodR(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame, 95);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomePosition(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);
            }
            else if (name == "maxonForTest")
            {
                maxoncmd.getHomingMethodL(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame, 20);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomePosition(*maxonMotor, &frame, 90);
                canManager.sendAndRecv(motor, frame);
            }

            maxoncmd.getCurrentThreshold(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);
        }
        else if (std::shared_ptr<TMotor> tmotor = std::dynamic_pointer_cast<TMotor>(motorPair.second))
        {
            if (name == "waist")
            {
                tmotorcmd.getZero(*tmotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            usleep(5000);
            tmotorcmd.getControlMode(*tmotor, &frame);
            canManager.sendAndRecv(motor, frame);
        }
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
            maxonMotorCount++;
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

            if (canManager.recvToBuff(motor, canManager.maxonCnt))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                        std::cout << "Maxon Enabled \n";
                    motor->recieveBuffer.pop();
                }
            }

            maxoncmd.getQuickStop(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            if (canManager.recvToBuff(motor, canManager.maxonCnt))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                        std::cout << "Maxon Quick Stopped\n";
                    motor->recieveBuffer.pop();
                }
            }

            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            if (canManager.recvToBuff(motor, canManager.maxonCnt))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                        std::cout << "Maxon Enabled \n";
                    motor->recieveBuffer.pop();
                }
            }
        }
    }
};

void DrumRobot::MaxonDisable()
{
    struct can_frame frame;

    canManager.setSocketsTimeout(0, 50000);

    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;

        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            maxoncmd.getQuickStop(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);
            if (canManager.recvToBuff(motor, canManager.maxonCnt))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                        break;
                    motor->recieveBuffer.pop();
                }
            }
            else
                std::cerr << "Failed to exit for motor [" << name << "]." << std::endl;
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////
/*                                 Send Thread Loop                           */
///////////////////////////////////////////////////////////////////////////////

void DrumRobot::SendLoop()
{
    struct can_frame frameToProcess;

    chrono::system_clock::time_point external = std::chrono::system_clock::now();

    while (state.main == Main::Perform || state.main == Main::Pause)
    {

        if (state.main == Main::Pause)
            continue;

        bool isAnyBufferLessThanTen = false;
        for (const auto &motor_pair : motors)
        {
            if (motor_pair.second->sendBuffer.size() < 10)
            {
                isAnyBufferLessThanTen = true;
                break;
            }
        }
        if (isAnyBufferLessThanTen)
        {
            if (pathManager.line < pathManager.total)
            {
                std::cout << "line : " << pathManager.line << ", total : " << pathManager.total << "\n";
                pathManager.PathLoopTask();
                pathManager.line++;
            }
            else if (pathManager.line == pathManager.total)
            {
                std::cout << "Perform Done\n";
                state.main = Main::Ready;
                pathManager.line = 0;
            }
        }

        bool allBuffersEmpty = true;
        for (const auto &motor_pair : motors)
        {
            if (!motor_pair.second->sendBuffer.empty())
            {
                allBuffersEmpty = false;
                break;
            }
        }

        // 모든 모터의 sendBuffer가 비었을 때 성능 종료 로직 실행
        if (allBuffersEmpty)
        {
            std::cout << "Performance is Over\n";
            state.main = Main::Ideal;
        }

        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);

        if (elapsed_time.count() >= 5000) // 5ms
        {
            external = std::chrono::system_clock::now();

            for (auto &motor_pair : motors)
            {
                shared_ptr<GenericMotor> motor = motor_pair.second;
                canManager.sendFromBuff(motor);
            }

            if (maxonMotorCount != 0)
            {
                maxoncmd.getSync(&frameToProcess);
                canManager.txFrame(virtualMaxonMotor, frameToProcess);
            }
        }
    }

    // CSV 파일명 설정
    std::string csvFileName = "../../READ/DrumData_in.txt";

    // input 파일 저장
    save_to_txt_inputData(csvFileName);
}

void DrumRobot::save_to_txt_inputData(const string &csv_file_name)
{
    // CSV 파일 열기
    std::ofstream csvFile(csv_file_name);

    if (!csvFile.is_open())
        std::cerr << "Error opening CSV file." << std::endl;

    // 헤더 추가
    csvFile << "0x007,0x001,0x002,0x003,0x004,0x005,0x006,0x008,0x009\n";

    // 2차원 벡터의 데이터를 CSV 파일로 쓰기
    for (const auto &row : pathManager.p)
    {
        for (const double cell : row)
        {
            csvFile << std::fixed << std::setprecision(5) << cell;
            if (&cell != &row.back())
                csvFile << ","; // 쉼표로 셀 구분
        }
        csvFile << "\n"; // 다음 행으로 이동
    }

    // CSV 파일 닫기
    csvFile.close();

    std::cout << "연주 DrumData_in 파일이 생성되었습니다: " << csv_file_name << std::endl;

    std::cout << "SendLoop terminated\n";
}

void DrumRobot::SendReadyLoop()
{
    std::cout << "Settig...\n";
    struct can_frame frameToProcess;
    std::string maxonCanInterface;
    std::shared_ptr<GenericMotor> virtualMaxonMotor;

    int maxonMotorCount = 0;
    for (const auto &motor_pair : motors)
    {
        // 각 요소가 MaxonMotor 타입인지 확인
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            maxonMotorCount++;
            maxonCanInterface = maxonMotor->interFaceName;
            virtualMaxonMotor = motor_pair.second;
        }
    }
    chrono::system_clock::time_point external = std::chrono::system_clock::now();

    bool allBuffersEmpty;
    do
    {
        allBuffersEmpty = true;
        for (const auto &motor_pair : motors)
        {
            if (!motor_pair.second->sendBuffer.empty())
            {
                allBuffersEmpty = false;
                break;
            }
        }

        if (!allBuffersEmpty)
        {
            chrono::system_clock::time_point internal = std::chrono::system_clock::now();
            chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);

            if (elapsed_time.count() >= 5000) // 5ms
            {
                external = std::chrono::system_clock::now();

                for (auto &motor_pair : motors)
                {
                    shared_ptr<GenericMotor> motor = motor_pair.second;
                    canManager.sendFromBuff(motor);
                }

                if (maxonMotorCount != 0)
                {
                    maxoncmd.getSync(&frameToProcess);
                    canManager.txFrame(virtualMaxonMotor, frameToProcess);
                }
            }
        }
    } while (!allBuffersEmpty);
    canManager.clearReadBuffers();
}

void DrumRobot::initializePathManager()
{
    pathManager.GetDrumPositoin();
    pathManager.GetMusicSheet();
    pathManager.SetReadyAng();
    pathManager.ApplyDir();
}

void DrumRobot::clearMotorsSendBuffer()
{
    for (auto motor_pair : motors)
        motor_pair.second->clearSendBuffer();
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 Recive Thread Loop                         */
///////////////////////////////////////////////////////////////////////////////

void DrumRobot::parse_and_save_to_csv(const std::string &csv_file_name)
{
    // CSV 파일 열기. 파일이 없으면 새로 생성됩니다.
    std::ofstream ofs(csv_file_name, std::ios::app);
    if (!ofs.is_open())
    {
        std::cerr << "Failed to open or create the CSV file: " << csv_file_name << std::endl;
        return;
    }

    // 파일이 새로 생성되었으면 CSV 헤더를 추가
    ofs.seekp(0, std::ios::end);
    if (ofs.tellp() == 0)
        ofs << "CAN_ID,p_act,tff_des,tff_act\n";

    // 각 모터에 대한 처리
    for (const auto &pair : motors)
    {
        auto &motor = pair.second;
        if (!motor->recieveBuffer.empty())
        {
            can_frame frame = motor->recieveBuffer.front();
            motor->recieveBuffer.pop();

            int id = motor->nodeId;
            float position, speed, torque;

            // TMotor 또는 MaxonMotor에 따른 데이터 파싱 및 출력
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
            {
                std::tuple<int, float, float, float> parsedData = tmotorcmd.parseRecieveCommand(*tMotor, &frame);
                position = std::get<1>(parsedData);
                speed = std::get<2>(parsedData);
                torque = std::get<3>(parsedData);
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
            {
                std::tuple<int, float, float> parsedData = maxoncmd.parseRecieveCommand(*maxonMotor, &frame);
                position = std::get<1>(parsedData);
                torque = std::get<2>(parsedData);
                speed = 0.0;
            }

            // 데이터 CSV 파일에 쓰기
            ofs << "0x" << std::hex << std::setw(4) << std::setfill('0') << id << ","
                << std::dec << position << "," << speed << "," << torque << "\n";
        }
    }

    ofs.close();
    std::cout << "연주 txt_OutData 파일이 생성되었습니다: " << csv_file_name << std::endl;
}
