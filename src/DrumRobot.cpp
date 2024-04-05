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
        {
            initializeMotors();
            initializecanManager();
            motorSettingCmd();
            canManager.setSocketNonBlock();
            std::cout << "System Initialize Complete [ Press Enter ]\n";
            getchar();
            state.main = Main::Ideal;
            break;
        }
        case Main::Ideal:
        {
            idealStateRoutine();
            sendCheckFrame = false;
            break;
        }
        case Main::Homing:
        {
            if (state.home == HomeSub::SelectMotorByUser)
            {
                usleep(50000);
            }
            else
            {
                checkUserInput();
            }
            break;
        }
        case Main::Perform:
        {
            checkUserInput();
            break;
        }
        case Main::Check:
        {
            if (!sendCheckFrame)
            {
                canManager.checkAllMotors_test();
                sendCheckFrame = true;
            }

            printCurrentPositions();
            std::cout << "Put any keyboard input\n";
            if (kbhit())
            {
                state.main = Main::Ideal;
            }
            usleep(200000);

            int ret = system("clear");
            if (ret == -1)
                std::cout << "system clear error" << endl;
            break;
        }
        case Main::Test:
        {
            if (state.test == TestSub::SelectParamByUser || state.test == TestSub::SetQValue || state.test == TestSub::SetXYZ || state.test == TestSub::StickTest)
            {
                usleep(50000);
            }
            else
            {
                checkUserInput();
            }
            break;
        }
        case Main::Shutdown:
        {
            break;
        }
        case Main::Pause:
        {
            checkUserInput();
            break;
        }
        case Main::AddStance:
        {
            checkUserInput();
            break;
        }
        case Main::Error:
        {
            checkUserInput();
            break;
        }
        }
    }
    canManager.setSocketBlock();
    DeactivateControlTask();
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
            usleep(200000);
            break;

        case Main::Homing:
            homeManager.SendHomeProcess();
            break;

        case Main::Perform:
            SendPerformProcess(5000);
            break;

        case Main::AddStance:
            SendAddStanceProcess();
            break;

        case Main::Check:
            usleep(500000);
            break;
        case Main::Test:
            testManager.SendTestProcess();
            break;

        case Main::Shutdown:
            save_to_txt_inputData("../../READ/DrumData_in.txt");
            break;

        case Main::Pause:
            usleep(5000);
            break;

        case Main::Error:
            sleep(2);
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
            ReadProcess(200000); /*500ms*/
            break;

        case Main::Homing:
            if (state.home == HomeSub::HomeTmotor || state.home == HomeSub::HomeMaxon || state.home == HomeSub::GetSelectedMotor)
                ReadProcess(5000);
            else
                usleep(5000);
            break;

        case Main::Perform:
            ReadProcess(5000);
            break;

        case Main::AddStance:
            ReadProcess(5000);
            break;

        case Main::Check:
            ReadProcess(200000); // 200ms
            break;
        case Main::Test:
            if (state.test == TestSub::StickTest)
            {
                usleep(500000);
            }
            else
            {
                ReadProcess(5000);
            }
            break;
        case Main::Shutdown:
            parse_and_save_to_csv("../../READ/DrumData_out.txt");
            break;
        case Main::Pause:
            ReadProcess(5000); /*5ms*/
            break;

        case Main::Error:
            sleep(2);
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
        canManager.distributeFramesToMotors();
        for (auto &motor_pair : motors)
        {
            auto &motor = motor_pair.second;
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
            {
                maxonMotor->checked = false;
            }
        }
        state.read = ReadSub::CheckMaxonControl;
        break;
    case ReadSub::CheckMaxonControl:
        for (auto &motor_pair : motors)
        {
            auto &motor = motor_pair.second;
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
            {
                if (maxonMotor->hitting && !maxonMotor->checked)
                {
                    state.read = ReadSub::CheckDrumHit;
                    break;
                }
                else if (maxonMotor->positioning && !maxonMotor->checked)
                {
                    state.read = ReadSub::CheckReachedPosition;
                    break;
                }
                else
                {
                    state.read = ReadSub::TimeCheck;
                }
            }
        }
        break;
    case ReadSub::CheckDrumHit:
        for (auto &motor_pair : motors)
        {
            auto &motor = motor_pair.second;
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
            {
                if (maxonMotor->hitting)
                {
                    if (dct_fun(maxonMotor->positionValues, 0))
                    {
                        cout << "Htting True!!!!!!!!!!!!!!!!!\n";
                        maxonMotor->positioning = true;
                        maxonMotor->hitting = false;
                    }
                    else
                        cout << "Htting..\n";
                    maxonMotor->checked = true;
                }
            }
        }
        state.read = ReadSub::CheckMaxonControl;
        break;
    case ReadSub::CheckReachedPosition:
        for (auto &motor_pair : motors)
        {
            auto &motor = motor_pair.second;
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
            {
                if (maxonMotor->positioning)
                {
                    if (pathManager.wrist_targetPos < maxonMotor->currentPos)
                    {
                        cout << "Positioning True!!!!!!!!!!!!!!!!!!!11\n";
                        maxonMotor->atPosition = true; // 여기서 pathManager 에서 접근
                        maxonMotor->positioning = false;
                    }
                    else
                        cout << "Positioning..\n";
                    maxonMotor->checked = true;
                }
            }
        }
        state.read = ReadSub::CheckMaxonControl;
        break;
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
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
            {
                if (maxonMotor->commandBuffer.size() < 10)
                    state.perform = PerformSub::GeneratePath;
                else
                    state.perform = PerformSub::SafetyCheck;
            }
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
        else
        {
            bool allBuffersEmpty = true;
            for (const auto &motor_pair : motors)
            {
                if (!dynamic_pointer_cast<MaxonMotor>(motor_pair.second)->commandBuffer.empty())
                {
                    allBuffersEmpty = false;
                    break;
                }
            }
            if (allBuffersEmpty)
            {
                std::cout << "Performance is Over\n";
                state.main = Main::AddStance;
                isReady = false;
                getReady = false;
                getBack = true;
                isBack = false;
                pathManager.line = 0;
            }
            else
                state.perform = PerformSub::SafetyCheck;
        }
        break;
    }
    case PerformSub::SafetyCheck:
    {
        bool isSafe = true;
        for (auto &motor_pair : motors)
        {
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
            {
                MaxonData mData = maxonMotor->commandBuffer.front();
                maxonMotor->commandBuffer.pop();
                cout << maxonMotor->myName << "\nPosition : " << mData.position << ",   State : " << mData.WristState << "\n";
                if (mData.WristState == 2)
                { // Stay Before Torque Mode
                    maxonMotor->stay = true;
                    maxonMotor->hitting = false;
                    maxonMotor->positioning = false;
                    maxonMotor->atPosition = false;
                    if (!maxonMotor->isPositionMode)
                    {
                        maxoncmd.getCSPMode(*maxonMotor, &maxonMotor->sendFrame);
                        maxonMotor->isPositionMode = true;
                    }
                }
                else if (mData.WristState == -1) // Get to Torque Mode
                {
                    maxonMotor->stay = false;
                    maxonMotor->hitting = true;
                    maxonMotor->positioning = false;
                    maxonMotor->atPosition = false;
                    if (maxonMotor->isPositionMode)
                    {
                        maxoncmd.getCSTMode(*maxonMotor, &maxonMotor->sendFrame);
                        maxonMotor->isPositionMode = false;
                    }
                }
                else if (mData.WristState == -0.5) // In Position Mode
                {
                    maxonMotor->stay = false;
                    maxonMotor->hitting = false;
                    maxonMotor->positioning = false;
                    maxonMotor->atPosition = false;
                    if (!maxonMotor->isPositionMode)
                    {
                        maxoncmd.getCSPMode(*maxonMotor, &maxonMotor->sendFrame);
                        maxonMotor->isPositionMode = true;
                    }
                }
                else
                {
                    if (maxonMotor->hitting)
                    {
                        maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, 30 * maxonMotor->cwDir * (-1));
                    }
                    else if (maxonMotor->positioning)
                    {
                        maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, 300 * maxonMotor->cwDir);
                    }
                    else if (maxonMotor->atPosition || maxonMotor->stay)
                    {
                        if (!maxonMotor->isPositionMode)
                        {
                            maxoncmd.getCSPMode(*maxonMotor, &maxonMotor->sendFrame);
                            maxonMotor->isPositionMode = true;
                        }
                        else
                        {
                            float coordinationPos = (pathManager.wrist_targetPos) * maxonMotor->cwDir;
                            if (abs(maxonMotor->currentPos - pathManager.wrist_targetPos) > 0.2 || maxonMotor->rMin > coordinationPos || maxonMotor->rMax < coordinationPos)
                            {
                                if (abs(maxonMotor->currentPos - pathManager.wrist_targetPos) > 0.2)
                                {
                                    std::cout << "Error Druing Hybrid Perform For " << maxonMotor->myName << " (Pos Diff)\n";
                                    cout << "Current : " << maxonMotor->currentPos << "\nTarget : " << pathManager.wrist_targetPos << "\n";
                                    cout << "Diff : " << abs(maxonMotor->currentPos - pathManager.wrist_targetPos) / M_PI * 180 << "deg\n";
                                }
                                else if (maxonMotor->rMin > coordinationPos)
                                {
                                    std::cout << "Error Druing Hybrid Perform For " << maxonMotor->myName << " (Out of Range : Min)\n";
                                    cout << "Current : " << maxonMotor->currentPos / M_PI * 180 << "deg\n";
                                }
                                else
                                {
                                    std::cout << "Error Druing Hybrid Perform For " << maxonMotor->myName << " (Out of Range : Max)\n";
                                    cout << "Current : " << maxonMotor->currentPos / M_PI * 180 << "deg\n";
                                }

                                isSafe = false;
                                maxoncmd.getQuickStop(*maxonMotor, &maxonMotor->sendFrame);
                                canManager.sendMotorFrame(maxonMotor);
                                usleep(5000);
                                maxoncmd.getSync(&maxonMotor->sendFrame);
                                canManager.sendMotorFrame(maxonMotor);
                            }
                            else
                            {
                                cout << "Stay Hold!!\n";
                                maxoncmd.getTargetPosition(*maxonMotor, &maxonMotor->sendFrame, pathManager.wrist_targetPos);
                            }
                        }
                    }
                    else // !hitting, !positioning, !atPosition, !stay
                    {
                        float coordinationPos = (mData.position) * maxonMotor->cwDir;
                        if (abs(maxonMotor->currentPos - mData.position) > 0.2 || maxonMotor->rMin > coordinationPos || maxonMotor->rMax < coordinationPos)
                        {
                            if (abs(maxonMotor->currentPos - mData.position) > 0.2)
                            {
                                std::cout << "Error Druing Perform For " << maxonMotor->myName << " (Pos Diff)\n";
                            }
                            else if (maxonMotor->rMin > coordinationPos)
                            {
                                std::cout << "Error Druing Perform For " << maxonMotor->myName << " (Out of Range : Min)\n";
                            }
                            else
                            {
                                std::cout << "Error Druing Perform For " << maxonMotor->myName << " (Out of Range : Max)\n";
                            }
                            isSafe = false;
                            maxoncmd.getQuickStop(*maxonMotor, &maxonMotor->sendFrame);
                            canManager.sendMotorFrame(maxonMotor);
                            usleep(5000);
                            maxoncmd.getSync(&maxonMotor->sendFrame);
                            canManager.sendMotorFrame(maxonMotor);
                        }
                        else
                        {
                            maxoncmd.getTargetPosition(*maxonMotor, &maxonMotor->sendFrame, mData.position);
                        }
                    }
                }
            }
            else if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
            {
                TMotorData tData = tMotor->commandBuffer.front();
                tMotor->commandBuffer.pop();
                float coordinationPos = (tData.position + tMotor->homeOffset) * tMotor->cwDir;
                if (abs(tMotor->currentPos - tData.position) > 0.2 || tMotor->rMin > coordinationPos || tMotor->rMax < coordinationPos)
                {
                    if (abs(tMotor->currentPos - tData.position) > 0.2)
                    {
                        std::cout << "Error Druing Perform For " << tMotor->myName << " (Pos Diff)\n";
                    }
                    else if (tMotor->rMin > coordinationPos)
                    {
                        std::cout << "Error Druing Perform For " << tMotor->myName << " (Out of Range : Min)\n";
                    }
                    else
                    {
                        std::cout << "Error Druing Perform For " << tMotor->myName << " (Out of Range : Max)\n";
                    }

                    isSafe = false;
                    tmotorcmd.getQuickStop(*tMotor, &tMotor->sendFrame);
                    canManager.sendMotorFrame(tMotor);
                    usleep(5000);
                    tmotorcmd.getExit(*tMotor, &tMotor->sendFrame);
                    canManager.sendMotorFrame(tMotor);
                }
                else
                {
                    tmotorcmd.parseSendCommand(*tMotor, &tMotor->sendFrame, tMotor->nodeId, 8, tData.position, tData.velocity, tMotor->Kp, tMotor->Kd, 0.0);
                }
            }
        }

        if (isSafe)
            state.perform = PerformSub::SendCANFrame;
        else
            state.main = Main::Error;
        break;
    }
    case PerformSub::SendCANFrame:
    {
        bool needSync = false;
        for (auto &motor_pair : motors)
        {
            shared_ptr<GenericMotor> motor = motor_pair.second;
            canManager.sendMotorFrame(motor);
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
            {
                virtualMaxonMotor = maxonMotor;
                needSync = true;
            }
        }

        if (needSync)
        {
            maxoncmd.getSync(&virtualMaxonMotor->sendFrame);
            canManager.sendMotorFrame(virtualMaxonMotor);
        }
        state.perform = PerformSub::TimeCheck;
        break;
    }
    }
}

void DrumRobot::SendAddStanceProcess()
{
    switch (state.addstance.load())
    {
    case AddStanceSub::CheckCommand:
    {
        if (getReady)
        {
            if (!testManager.isMaxonEnable)
            {
                homeManager.MaxonEnable();
                homeManager.setMaxonMode("CSP");
                testManager.isMaxonEnable = true;
            }

            std::cout << "Get Ready...\n";
            clearMotorsCommandBuffer();
            state.addstance = AddStanceSub::FillBuf;
        }
        else if (getBack)
        {
            if (!testManager.isMaxonEnable)
            {
                homeManager.MaxonEnable();
                homeManager.setMaxonMode("CSP");
                testManager.isMaxonEnable = true;
            }

            std::cout << "Get Back...\n";
            clearMotorsCommandBuffer();
            state.addstance = AddStanceSub::FillBuf;
        }
        else
        {
            state.main = Main::Ideal;
        }
        break;
    }
    case AddStanceSub::FillBuf:
    {
        if (canManager.checkAllMotors_Fixed())
        {
            usleep(10000);
            if (getReady)
            {
                pathManager.GetArr(pathManager.standby);
            }
            else if (getBack)
            {
                pathManager.GetArr(pathManager.backarr);
            }
        }
        state.addstance = AddStanceSub::TimeCheck;
        break;
    }
    case AddStanceSub::TimeCheck:
    {
        usleep(5000);
        state.addstance = AddStanceSub::CheckBuf;
        break;
    }
    case AddStanceSub::CheckBuf:
    {
        bool allBuffersEmpty = true;

        for (const auto &motor_pair : motors)
        {
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
            {
                if (!maxonMotor->commandBuffer.empty())
                {
                    allBuffersEmpty = false;
                    break;
                }
            }
            else if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
            {
                if (!tMotor->commandBuffer.empty())
                {
                    allBuffersEmpty = false;
                    break;
                }
            }
        }

        if (!allBuffersEmpty)
        {
            state.addstance = AddStanceSub::SafetyCheck;
        }
        else
        {
            state.addstance = AddStanceSub::CheckCommand;
            canManager.clearReadBuffers();
            if (getReady)
            {
                isReady = true;
                getReady = false;
                isBack = false;
                getBack = false;
            }
            else if (getBack)
            {
                isBack = true;
                getBack = false;
                isReady = false;
                getReady = false;
            }
        }
        break;
    }
    case AddStanceSub::SafetyCheck:
    {
        bool isSafe = canManager.safetyCheck("Add Stance");
        if (isSafe)
            state.addstance = AddStanceSub::SendCANFrame;
        else
            state.main = Main::Error;

        break;
    }
    case AddStanceSub::SendCANFrame:
    {

        bool needSync = false;
        for (auto &motor_pair : motors)
        {
            shared_ptr<GenericMotor> motor = motor_pair.second;
            canManager.sendMotorFrame(motor);
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
            {
                virtualMaxonMotor = maxonMotor;
                needSync = true;
            }
        }

        if (needSync)
        {
            maxoncmd.getSync(&virtualMaxonMotor->sendFrame);
            canManager.sendMotorFrame(virtualMaxonMotor);
        }
        state.addstance = AddStanceSub::TimeCheck;
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

    if (state.main == Main::Ideal)
    {
        if (!(state.home == HomeSub::Done))
        {
            std::cout << "- h : Start Homing Mode\n";
            std::cout << "- x : Make home state by user\n";
        }
        else
        {
            if (!isReady)
            {
                std::cout << "- r : Move to Ready Position\n";
                std::cout << "- t : Start tuning\n";
            }
            else
            {
                std::cout << "- p : Start Perform\n";
                std::cout << "- t : Start tuning\n";
            }
        }
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
            state.main = Main::Test;
            return true;
        }
        else if (input == "r" && state.home == HomeSub::Done)
        {
            state.main = Main::AddStance;
            getReady = true;
            isReady = false;
            getBack = false;
            isBack = false;
            return true;
        }
        else if (input == "x" && !(state.home == HomeSub::Done))
        {
            for (auto &entry : motors)
            {
                entry.second->isHomed = true;
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                {
                    tMotor->homeOffset = 0.0;
                    tMotor->coordinatePos = (tMotor->currentPos * tMotor->homeOffset) * tMotor->cwDir;
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
                {
                    maxonMotor->coordinatePos = (maxonMotor->currentPos) * maxonMotor->cwDir;
                }
            }
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
                if (!isBack)
                {
                    state.main = Main::AddStance;
                    getBack = true;
                    isBack = false;
                    isReady = false;
                    getReady = false;
                }
                else if (isBack)
                {
                    state.main = Main::Shutdown;
                }
                else if (!isReady && !isBack)
                {
                    state.main = Main::Shutdown;
                }
            }
            else
            {
                state.main = Main::Shutdown;
            }
            return true;
        }
        else if (input == "p" && isReady)
        {
            state.main = Main::Perform;
            isReady = false;
            return true;
        }
        else if (input == "t")
        {
            state.main = Main::Test;
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
        if (state.main == Main::Perform || state.main == Main::Pause)
        {
            if (input == 'q')
                state.main = Main::Pause;
            else if (input == 'e')
            {
                isReady = false;
                getReady = false;
                getBack = true;
                isBack = false;
                state.main = Main::AddStance;
                pathManager.line = 0;
            }
            else if (input == 'r')
                state.main = Main::Perform;
            else if (input == 's')
                state.main = Main::Shutdown;
        }
        else if (state.main == Main::Error)
        {
            if (input == 's')
                state.main = Main::Shutdown;
        }
        else
        {
            if (input == 'e')
            {
                isReady = false;
                getReady = false;
                getBack = true;
                isBack = false;
                state.main = Main::AddStance;
                pathManager.line = 0;
            }
            else if (input == 's')
                state.main = Main::Shutdown;
        }
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
                tMotor->myName = "waist";
            }
            else if (motor_pair.first == "R_arm1")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 3;
                tMotor->rMin = 0.0f;       // 0deg
                tMotor->rMax = M_PI * 0.8; // 144deg
                tMotor->Kp = 200;
                tMotor->Kd = 2.5;
                tMotor->isHomed = false;
                tMotor->myName = "R_arm1";
            }
            else if (motor_pair.first == "L_arm1")
            {
                tMotor->cwDir = 1.0f;
                tMotor->sensorBit = 0;
                tMotor->rMin = M_PI / 5; // 36deg
                tMotor->rMax = M_PI;     // 180deg
                tMotor->Kp = 200;
                tMotor->Kd = 2.5;
                tMotor->isHomed = false;
                tMotor->myName = "L_arm1";
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
                tMotor->myName = "R_arm2";
            }
            else if (motor_pair.first == "R_arm3")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 5;
                tMotor->rMin = -M_PI / 6.0f; // -30deg
                tMotor->rMax = M_PI * 0.75f; // 135deg
                tMotor->Kp = 250;
                tMotor->Kd = 3.5;
                tMotor->isHomed = false;
                tMotor->myName = "R_arm3";
            }
            else if (motor_pair.first == "L_arm2")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 1;
                tMotor->rMin = -M_PI / 4.0f; // -45deg
                tMotor->rMax = M_PI / 2.0f;  // 90deg
                tMotor->Kp = 350;
                tMotor->Kd = 3.5;
                tMotor->isHomed = false;
                tMotor->myName = "L_arm2";
            }
            else if (motor_pair.first == "L_arm3")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorBit = 2;
                tMotor->rMin = -M_PI / 6.0f; // -30deg
                tMotor->rMax = M_PI * 0.75f; // 135deg
                tMotor->Kp = 250;
                tMotor->Kd = 3.5;
                tMotor->isHomed = false;
                tMotor->myName = "L_arm3";
            }
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            // 각 모터 이름에 따른 멤버 변수 설정
            if (motor_pair.first == "L_wrist")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = -M_PI * 0.5f; // -90deg
                maxonMotor->rMax = M_PI * 0.75f; // 135deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x209; // Controlword
                maxonMotor->txPdoIds[1] = 0x309; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x409; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x509; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x189; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "L_wrist";
            }
            else if (motor_pair.first == "R_wrist")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = -M_PI * 0.5f; // -90deg
                maxonMotor->rMax = M_PI * 0.75f; // 135deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x208; // Controlword
                maxonMotor->txPdoIds[1] = 0x308; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x408; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x508; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x188; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "R_wrist";
            }
            else if (motor_pair.first == "maxonForTest")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = -M_PI * 0.5f; // -90deg
                maxonMotor->rMax = M_PI * 0.75f; // 135deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x20A; // Controlword
                maxonMotor->txPdoIds[1] = 0x30A; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x40A; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x50A; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x18A; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "maxonForTest";
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
            std::cout << "Exiting for motor [" << name << "]" << std::endl;
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
            virtualMaxonMotor = maxonMotor;
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

                maxoncmd.getCurrentThresholdL(*maxonMotor, &frame);
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

                maxoncmd.getCurrentThresholdR(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else if (name == "maxonForTest")
            {
                maxoncmd.getHomingMethodTest(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame, 90);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomePosition(*maxonMotor, &frame, 90);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getCurrentThresholdL(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
        }
        else if (std::shared_ptr<TMotor> tmotor = std::dynamic_pointer_cast<TMotor>(motorPair.second))
        {
            tmotorcmd.getQuickStop(*tmotor, &frame);
            canManager.sendAndRecv(motor, frame);
            usleep(5000);
            tmotorcmd.getControlMode(*tmotor, &frame);
            canManager.sendAndRecv(motor, frame);

            usleep(5000);
            tmotorcmd.getControlMode(*tmotor, &frame);
            canManager.sendAndRecv(motor, frame);

            std::tuple<int, float, float, float> parsedData = tmotorcmd.parseRecieveCommand(*tmotor, &frame);
            float initPosition = abs(std::get<1>(parsedData));

            if (tmotor->myName == "waist" || initPosition > 1.5)
            {
                usleep(5000);
                tmotorcmd.getZero(*tmotor, &frame);
                canManager.sendAndRecv(motor, frame);
                usleep(5000);
                tmotorcmd.getQuickStop(*tmotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 Send Thread Loop                           */
///////////////////////////////////////////////////////////////////////////////

void DrumRobot::save_to_txt_inputData(const string &csv_file_name)
{
    // CSV 파일 열기
    std::ofstream csvFile(csv_file_name);

    if (!csvFile.is_open())
        std::cerr << "Error opening CSV file." << std::endl;

    // 헤더 추가
    csvFile << "0x007,0x001,0x002,0x003,0x004,0x005,0x006,0x008,0x009\n";

    // 2차원 벡터의 데이터를 CSV 파일로 쓰기
    for (const auto &row : pathManager.pos)
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

void DrumRobot::initializePathManager()
{
    pathManager.GetDrumPositoin();
    pathManager.GetMusicSheet();
    pathManager.SetReadyAng();
}

void DrumRobot::clearMotorsSendBuffer()
{
    for (auto motor_pair : motors)
        motor_pair.second->clearSendBuffer();
}

void DrumRobot::clearMotorsCommandBuffer()
{
    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motorPair.second))
        {
            maxonMotor->clearCommandBuffer();
        }
        else if (std::shared_ptr<TMotor> tmotor = std::dynamic_pointer_cast<TMotor>(motorPair.second))
        {
            tmotor->clearCommandBuffer();
        }
    }
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
                std::tuple<int, float, float, int8_t> parsedData = maxoncmd.parseRecieveCommand(*maxonMotor, &frame);
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

bool DrumRobot::dct_fun(float positions[], float vel_th)
{
    // 포지션 배열에서 각각의 값을 추출합니다.
    float the_k = positions[3]; // 가장 최신 값
    float the_k_1 = positions[2];
    float the_k_2 = positions[1];
    float the_k_3 = positions[0]; // 가장 오래된 값

    float ang_k = (the_k + the_k_1) / 2;
    float ang_k_1 = (the_k_1 + the_k_2) / 2;
    float ang_k_2 = (the_k_2 + the_k_3) / 2;
    float vel_k = ang_k - ang_k_1;
    float vel_k_1 = ang_k_1 - ang_k_2;

    if (vel_k > vel_k_1 && vel_k > vel_th && ang_k < 0.05)
        return true;
    else if (ang_k < -0.25)
        return true;
    else
        return false;
}