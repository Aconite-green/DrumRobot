#include "../include/tasks/DrumRobot.hpp"

// For Qt
// #include "../tasks/DrumRobot.hpp"

// DrumRobot 클래스의 생성자
DrumRobot::DrumRobot(State &stateRef,
                     CanManager &canManagerRef,
                     PathManager &pathManagerRef,
                     HomeManager &homeManagerRef,
                     TestManager &testManagerRef,
                     std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
                     Sensor &sensorRef)
    : state(stateRef),
      canManager(canManagerRef),
      pathManager(pathManagerRef),
      homeManager(homeManagerRef),
      testManager(testManagerRef),
      motors(motorsRef),
      sensor(sensorRef)
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
            ClearBufferforRecord();
            idealStateRoutine();
            break;
        }
        case Main::Homing:
        {
            if (state.home == HomeSub::SelectMotorByUser)
            {
                //usleep 50000 -> 5000
                usleep(5000);
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
            bool isWriteError = false;
            if (state.home == HomeSub::Done)
            {
                if (!canManager.checkAllMotors_Fixed())
                {
                    isWriteError = true;
                }
                canManager.checkMaxon();
            }
            printCurrentPositions();
            std::cout << "Put any keyboard input\n";
            if (kbhit())
            {
                state.main = Main::Ideal;
            }

            if (isWriteError)
            {
                state.main = Main::Error;
                break;
            }
            usleep(200000);

            int ret = system("clear");
            if (ret == -1)
                std::cout << "system clear error" << endl;
            break;
        }
        case Main::Test:
        {
            bool isWriteError = false;
            if (state.test == TestSub::SelectParamByUser || state.test == TestSub::SetQValue || state.test == TestSub::SetXYZ || state.test == TestSub::StickTest)
            {
                // canManager.appendToCSV_time("FIXED_POS.txt");
                usleep(5000); // 5ms
                if (!canManager.checkAllMotors_Fixed())
                {
                    isWriteError = true;
                }
            }
            else if (state.test == TestSub::SetSingleTuneParm || state.test == TestSub::SetServoTestParm)
            {
                usleep(500000); // 500ms
            }
            else
            {
                checkUserInput();
            }

            if (isWriteError)
            {
                state.main = Main::Error;
            }
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
            state.main = Main::Shutdown;
            break;
        }
        case Main::Shutdown:
            break;
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
        {
            usleep(800000);
            break;
        }
        case Main::Ideal:
        {
            usleep(500000); // 500ms
            bool isWriteError = false;
            if (state.home == HomeSub::Done)
            {

                if (!canManager.checkAllMotors_Fixed())
                {
                    isWriteError = true;
                }
            }
            else
            {
                // canManager.checkMaxon();
            }

            if (isWriteError)
            {
                state.main = Main::Error;
            }
            break;
        }
        case Main::Homing:
        {
            UnfixedMotor();
            homeManager.SendHomeProcess();
            break;
        }
        case Main::Perform:
        {
            UnfixedMotor();
            SendPerformProcess(5000);
            break;
        }
        case Main::AddStance:
        {
            UnfixedMotor();
            SendAddStanceProcess();
            break;
        }
        case Main::Check:
        {
            usleep(500000);
            break;
        }
        case Main::Test:
        {
            UnfixedMotor();
            testManager.SendTestProcess();
            break;
        }
        case Main::Pause:
        {
            bool isWriteError = false;
            if (!canManager.checkAllMotors_Fixed())
            {
                isWriteError = true;
            }
            if (isWriteError)
            {
                state.main = Main::Error;
            }
            // usleep 50000 -> 500000
            usleep(500000); // 50ms
            break;
        }
        case Main::Error:
        {
            save_to_txt_inputData("../../READ/Error_DrumData_in");
            sleep(2);
            state.main = Main::Shutdown;
            break;
        }
        case Main::Shutdown:
            break;
        }
    }
}

void DrumRobot::recvLoopForThread()
{

    while (state.main != Main::Shutdown)
    {
        // static int n_txt = 0;
        // n_txt++;
        // if (n_txt > 10000)
        // {
        //     n_txt = 0;
        //     canManager.appendToCSV_time("ReadProcess10000_while.txt");
        // }

        switch (state.main.load())
        {
        case Main::SystemInit:
        {
            usleep(500000);
            break;
        }
        case Main::Ideal:
        {
            ReadProcess(5000); /*5ms*/
            break;
        }
        case Main::Homing:
        {
            if (state.home == HomeSub::HomeTmotor || state.home == HomeSub::HomeMaxon || state.home == HomeSub::GetSelectedMotor)
                ReadProcess(5000);
            else
                usleep(5000);
            break;
        }
        case Main::Perform:
        {
            ReadProcess(5000);
            break;
        }
        case Main::AddStance:
        {
            ReadProcess(5000);
            break;
        }
        case Main::Check:
        {
            ReadProcess(5000);
            break;
        }
        case Main::Test:
        {
            if (state.test == TestSub::StickTest)
            {
                usleep(500000);
            }
            else
            {
                ReadProcess(5000);
            }
            break;
        }
        case Main::Pause:
        {
            bool isWriteError = false;
            if (!canManager.checkAllMotors_Fixed())
            {
                isWriteError = true;
            }
            if (isWriteError)
            {
                state.main = Main::Error;
            }
            //usleep 50000 -> 500000
            usleep(500000); // 50ms
            break;
        }
        case Main::Error:
        {
            parse_and_save_to_csv("../../READ/Error_DrumData_out");
            sleep(2);
            state.main = Main::Shutdown;
            break;
        }
        case Main::Shutdown:
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
    {
        if (state.home != HomeSub::Done || state.main == Main::Test)
        {
            canManager.distributeFramesToMotors(false);
        }
        else
        {
            bool isSafe = canManager.distributeFramesToMotors(true);
            if (!isSafe)
            {
                state.main = Main::Error;
            }
        }

        if (maxonMotorCount == 0)
        {
            state.read = ReadSub::TimeCheck;
        }
        else
        {
            for (auto &motor_pair : motors)
            {
                auto &motor = motor_pair.second;
                if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
                {
                    maxonMotor->checked = false;
                }
            }
            state.read = ReadSub::CheckMaxonControl;
        }
        break;
    }
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
                        act = cnt;
                        cout << "Read : Htting True!!!!!!!!!!!!!!!!!\n";
                        maxonMotor->positioning = true;
                        maxonMotor->hitting = false;
                    } /*
                    else
                        cout << "Read : Htting..\n";*/
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
                        cout << "Read : Positioning True!!!!!!!!!!!!!!!!!!!\n";
                        maxonMotor->atPosition = true; // 여기서 pathManager 에서 접근
                        maxonMotor->positioning = false;
                    } /*
                     else
                         cout << "Read : Positioning..\n";*/
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
            cnt++;
            state.perform = PerformSub::CheckBuf; // 주기가 되면 ReadCANFrame 상태로 진입
            SendStandard = currentTime;           // 현재 시간으로 시간 객체 초기화
        }
        break;
    }
    case PerformSub::CheckBuf:
    {
        for (const auto &motor_pair : motors)
        {
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
            {
                if (tMotor->commandBuffer.size() < 10)
                    state.perform = PerformSub::GeneratePath;
                else
                    state.perform = PerformSub::SetCANFrame;
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
            {
                if (maxonMotor->commandBuffer.size() < 10)
                    state.perform = PerformSub::GeneratePath;
                else
                    state.perform = PerformSub::SetCANFrame;
            }
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
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
                {
                    if (!tMotor->commandBuffer.empty())
                    {
                        allBuffersEmpty = false;
                        break;
                    }
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
                {
                    if (!maxonMotor->commandBuffer.empty())
                    {
                        allBuffersEmpty = false;
                        break;
                    }
                }
            }

            if (allBuffersEmpty)
            {
                std::cout << "Performance is Over\n";
                save_to_txt_inputData("../../READ/DrumData_in");
                parse_and_save_to_csv("../../READ/DrumData_out");
                state.main = Main::AddStance;
                isReady = false;
                getReady = false;
                getBack = true;
                isBack = false;
                pathManager.line = 0;
            }
            else
                state.perform = PerformSub::SetCANFrame;
        }
        break;
    }
    case PerformSub::SetCANFrame:
    {
        // WristState가 항상 0으로 저장되면 setCANFrame 함수 사용하는 것과 같음
        canManager.tMotor_control_mode = POS_SPD_LOOP;
        canManager.setCANFrame();

        // vector<float> Pos(9);
        // for (auto &motor_pair : motors)
        // {
        //     if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        //     {
        //         MaxonData mData = maxonMotor->commandBuffer.front();
        //         maxonMotor->commandBuffer.pop();
        //         // cout << "< " << maxonMotor->myName << " >\nPosition : " << mData.position << ",\t\tState : " << mData.WristState << "\n";
        //         if (mData.WristState == 1)
        //         {
        //             des = cnt;
        //         }
        //         if (mData.WristState == 2)
        //         { // Stay Before Torque Mode
        //             maxonMotor->stay = true;
        //             maxonMotor->hitting = false;
        //             maxonMotor->positioning = false;
        //             maxonMotor->atPosition = false;
        //             if (!maxonMotor->isPositionMode)
        //             {
        //                 cout << "Change to Position Mode!!!!!!!!\n";
        //                 maxoncmd.getCSPMode(*maxonMotor, &maxonMotor->sendFrame);
        //                 maxonMotor->isPositionMode = true;
        //             }
        //         }
        //         else if (mData.WristState == -1) // Get to Torque Mode
        //         {
        //             cout << "\n================== Hit Time Diff >> " << (act - des) * 5 << "ms ================\n\n";
        //             maxonMotor->stay = false;
        //             maxonMotor->hitting = true;
        //             maxonMotor->positioning = false;
        //             maxonMotor->atPosition = false;
        //             maxonMotor->clearWrist_BackArr();

        //             if (maxonMotor->isPositionMode)
        //             {
        //                 cout << "Change to Torque Mode!!!!!!!!\n";
        //                 maxoncmd.getCSTMode(*maxonMotor, &maxonMotor->sendFrame);
        //                 maxonMotor->isPositionMode = false;
        //             }
        //         }
        //         else if (mData.WristState == -0.5) // In Position Mode
        //         {
        //             maxonMotor->stay = false;
        //             maxonMotor->hitting = false;
        //             maxonMotor->positioning = false;
        //             maxonMotor->atPosition = false;
        //             if (!maxonMotor->isPositionMode)
        //             {
        //                 cout << "\n================== Hit Time Diff >> " << (act - des) * 5 << "ms ================\n\n";
        //                 cout << "Change to Position Mode!!!!!!!!\n";
        //                 maxoncmd.getCSPMode(*maxonMotor, &maxonMotor->sendFrame);
        //                 maxonMotor->isPositionMode = true;
        //             }
        //         }
        //         else
        //         {
        //             if (maxonMotor->hitting)
        //             {
        //                 maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, 500 * maxonMotor->cwDir * (-1));
        //             }
        //             else if (maxonMotor->positioning)
        //             {
        //                 maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, 10 * maxonMotor->cwDir);
        //             }
        //             else if (maxonMotor->stay)
        //             {
        //                 if (!maxonMotor->isPositionMode)
        //                 {
        //                     cout << "Change to Position Mode!!!!!!!!\n";
        //                     maxoncmd.getCSPMode(*maxonMotor, &maxonMotor->sendFrame);
        //                     maxonMotor->isPositionMode = true;
        //                 }
        //                 else
        //                 {
        //                     mData.position = pathManager.wrist_backPos;
        //                     cout << "Stay Hold!!\n";
        //                     Pos[motor_mapping[maxonMotor->myName]] = pathManager.wrist_backPos;
        //                     maxoncmd.getTargetPosition(*maxonMotor, &maxonMotor->sendFrame, pathManager.wrist_backPos);
        //                 }
        //             }
        //             else if (maxonMotor->atPosition)
        //             {
        //                 if (!maxonMotor->isPositionMode)
        //                 {
        //                     cout << "Change to Position Mode!!!!!!!!\n";
        //                     maxoncmd.getCSPMode(*maxonMotor, &maxonMotor->sendFrame);
        //                     maxonMotor->isPositionMode = true;

        //                     float coordinationPos = (maxonMotor->currentPos + M_PI / 18) * maxonMotor->cwDir;
        //                     pathManager.Get_wrist_BackArr(maxonMotor->myName, coordinationPos, pathManager.wrist_backPos, pathManager.wrist_back_time);
        //                 }
        //                 else
        //                 {
        //                     float data = pathManager.wrist_backPos;
        //                     if (!maxonMotor->wrist_BackArr.empty())
        //                     {
        //                         data = maxonMotor->wrist_BackArr.front();
        //                         maxonMotor->wrist_BackArr.pop();
        //                     }
        //                     cout << "Stay Hold!!\n";
        //                     Pos[motor_mapping[maxonMotor->myName]] = data;
        //                     maxoncmd.getTargetPosition(*maxonMotor, &maxonMotor->sendFrame, data);
        //                 }
        //             }
        //             else // !hitting, !positioning, !atPosition, !stay
        //             {
        //                 Pos[motor_mapping[maxonMotor->myName]] = mData.position;
        //                 maxoncmd.getTargetPosition(*maxonMotor, &maxonMotor->sendFrame, mData.position);
        //             }
        //         }
        //     }
        //     else if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
        //     {
        //         TMotorData tData = tMotor->commandBuffer.front();
        //         tMotor->commandBuffer.pop();
        //         Pos[motor_mapping[tMotor->myName]] = tData.position;

        //         if (canManager.tMotor_control_mode == POS_LOOP)
        //         {
        //             tservocmd.comm_can_set_pos(*tMotor, &tMotor->sendFrame, tData.position);
        //         }
        //         else if (canManager.tMotor_control_mode == POS_SPD_LOOP)
        //         {
        //             tservocmd.comm_can_set_pos_spd(*tMotor, &tMotor->sendFrame, tData.position, tData.spd, tData.acl);
        //         }
        //         else
        //         {
        //             cout << "tMotor control mode ERROR\n";
        //         }
        //         tMotor->break_state = tData.isBreak;
        //     }
        // }
        // Input_pos.push_back(Pos);

        state.perform = PerformSub::SendCANFrame;
        break;
    }
    case PerformSub::SendCANFrame:
    {
        bool isWriteError = false;
        for (auto &motor_pair : motors)
        {
            shared_ptr<GenericMotor> motor = motor_pair.second;
            if (!canManager.sendMotorFrame(motor))
            {
                isWriteError = true;
            }

            // if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
            // {
            //     sensor.writeVal(tMotor, tMotor->break_state);
            // }
        }
        if (maxonMotorCount != 0)
        {
            maxoncmd.getSync(&virtualMaxonMotor->sendFrame);
            if (!canManager.sendMotorFrame(virtualMaxonMotor))
            {
                isWriteError = true;
            }
        }
        if (isWriteError)
        {
            state.main = Main::Error;
        }
        else
        {
            state.perform = PerformSub::TimeCheck;
        }

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
            ClearBufferforRecord();
            std::cout << "Get Ready...\n";
            clearMotorsCommandBuffer();
            state.addstance = AddStanceSub::FillBuf;
        }
        else if (getBack)
        {
            ClearBufferforRecord();
            std::cout << "Get Back...\n";
            clearMotorsCommandBuffer();
            state.addstance = AddStanceSub::FillBuf;
        }
        else
        {
            save_to_txt_inputData("../../READ/AddStance_in");
            parse_and_save_to_csv("../../READ/AddStance_out");
            state.main = Main::Ideal;
        }
        break;
    }
    case AddStanceSub::FillBuf:
    {
        if (getReady || getBack)
        {
            pathManager.GetArr(pathManager.standby);
        }
        else if (isReady)
        {
            pathManager.GetArr(pathManager.readyarr);
        }
        else if (isBack)
        {
            pathManager.GetArr(pathManager.backarr);
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
            state.addstance = AddStanceSub::SetCANFrame;
        }
        else
        {
            if (getReady)
            {
                state.addstance = AddStanceSub::FillBuf;
                isReady = true;
                getReady = false;
                isBack = false;
                getBack = false;
            }
            else if (getBack)
            {
                state.addstance = AddStanceSub::FillBuf;
                isBack = true;
                getBack = false;
                isReady = false;
                getReady = false;
            }
            else if (isReady)
            {
                state.addstance = AddStanceSub::CheckCommand;
                canManager.clearReadBuffers();
            }
            else if (isBack)
            {
                state.addstance = AddStanceSub::CheckCommand;
                canManager.clearReadBuffers();
            }
        }
        break;
    }
    case AddStanceSub::SetCANFrame:
    {
        canManager.setCANFrame();
        state.addstance = AddStanceSub::SendCANFrame;
        break;
    }
    case AddStanceSub::SendCANFrame:
    {
        bool isWriteError = false;
        for (auto &motor_pair : motors)
        {
            shared_ptr<GenericMotor> motor = motor_pair.second;

            if (!canManager.sendMotorFrame(motor))
            {
                isWriteError = true;
            }

            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
            {
                sensor.writeVal(tMotor, tMotor->break_state);
            }
        }

        if (maxonMotorCount != 0)
        {
            maxoncmd.getSync(&virtualMaxonMotor->sendFrame);

            if (!canManager.sendMotorFrame(virtualMaxonMotor))
            {
                isWriteError = true;
            };
        }

        if (isWriteError)
        {
            state.main = Main::Error;
        }
        else
        {
            state.addstance = AddStanceSub::TimeCheck;
        }

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
                }
            }
            homeManager.MaxonEnable();
            homeManager.setMaxonMode("CSP");
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
    bool isWriteError = false;
    if (kbhit())
    {
        char input = getchar();
        if (state.main == Main::Perform || state.main == Main::Pause)
        {
            if (input == 'q')
            {
                state.main = Main::Pause;

                if (!canManager.checkAllMotors_Fixed())
                {
                    isWriteError = true;
                }
            }
            else if (input == 'e')
            {
                std::cout << "Performance is interrupted!\n";
                save_to_txt_inputData("../../READ/interrupted_DrumData_in");
                parse_and_save_to_csv("../../READ/interrupted_DrumData_out");
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
            else if (input == 'b')
            {
                breakOn();
            }
            else if (input == 's')
                state.main = Main::Shutdown;
        }
    }

    if (isWriteError)
    {
        state.main = Main::Error;
    }
    usleep(5000);
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
    motors["waist"] = make_shared<TMotor>(0x00, "AK80_64");
    motors["R_arm1"] = make_shared<TMotor>(0x01, "AK70_10");
    motors["L_arm1"] = make_shared<TMotor>(0x02, "AK70_10");
    motors["R_arm2"] = make_shared<TMotor>(0x03, "AK70_10");
    motors["R_arm3"] = make_shared<TMotor>(0x04, "AK70_10");
    motors["L_arm2"] = make_shared<TMotor>(0x05, "AK70_10");
    motors["L_arm3"] = make_shared<TMotor>(0x06, "AK70_10");
    motors["R_wrist"] = make_shared<MaxonMotor>(0x07);
    motors["L_wrist"] = make_shared<MaxonMotor>(0x08);
    motors["R_foot"] = make_shared<MaxonMotor>(0x09);
    motors["L_foot"] = make_shared<MaxonMotor>(0x0A);
    motors["maxonForTest"] = make_shared<MaxonMotor>(0x0B);

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
                tMotor->rMin = -90.0f * M_PI / 180.0f; // -90deg
                tMotor->rMax = 90.0f * M_PI / 180.0f;  // 90deg
                tMotor->isHomed = true;
                tMotor->myName = "waist";
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                tMotor->sensorWriteBit = 0;
            }
            else if (motor_pair.first == "R_arm1")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorReadBit = 3;
                tMotor->rMin = 0.0f * M_PI / 180.0f;   // 0deg
                tMotor->rMax = 150.0f * M_PI / 180.0f; // 150deg
                tMotor->isHomed = false;
                tMotor->myName = "R_arm1";
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                tMotor->sensorWriteBit = 0;
            }
            else if (motor_pair.first == "L_arm1")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorReadBit = 0;
                tMotor->rMin = 30.0f * M_PI / 180.0f;  // 30deg
                tMotor->rMax = 180.0f * M_PI / 180.0f; // 180deg
                tMotor->isHomed = false;
                tMotor->myName = "L_arm1";
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                tMotor->sensorWriteBit = 0;
            }
            else if (motor_pair.first == "R_arm2")
            {
                tMotor->cwDir = 1.0f;
                tMotor->sensorReadBit = 4;
                tMotor->rMin = -60.0f * M_PI / 180.0f; // -60deg
                tMotor->rMax = 90.0f * M_PI / 180.0f;  // 90deg
                tMotor->isHomed = false;
                tMotor->myName = "R_arm2";
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                tMotor->sensorWriteBit = 0;
            }
            else if (motor_pair.first == "R_arm3")
            {
                tMotor->cwDir = 1.0f;
                tMotor->sensorReadBit = 5;
                tMotor->rMin = -30.0f * M_PI / 180.0f; // -30deg
                tMotor->rMax = 144.0f * M_PI / 180.0f; // 144deg
                tMotor->isHomed = false;
                tMotor->myName = "R_arm3";
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                tMotor->sensorWriteBit = 1;
            }
            else if (motor_pair.first == "L_arm2")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorReadBit = 1;
                tMotor->rMin = -60.0f * M_PI / 180.0f; // -60deg
                tMotor->rMax = 90.0f * M_PI / 180.0f;  // 90deg
                tMotor->isHomed = false;
                tMotor->myName = "L_arm2";
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                tMotor->sensorWriteBit = 0;
            }
            else if (motor_pair.first == "L_arm3")
            {
                tMotor->cwDir = -1.0f;
                tMotor->sensorReadBit = 2;
                tMotor->rMin = -30.0f * M_PI / 180.0f; // -30 deg
                tMotor->rMax = 144.0f * M_PI / 180.0f; // 144 deg
                tMotor->isHomed = false;
                tMotor->myName = "L_arm3";
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                tMotor->sensorWriteBit = 0;
            }
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            // 각 모터 이름에 따른 멤버 변수 설정
            if (motor_pair.first == "R_wrist")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = -108.0f * M_PI / 180.0f; // -108deg
                maxonMotor->rMax = 135.0f * M_PI / 180.0f;  // 135deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x207; // Controlword
                maxonMotor->txPdoIds[1] = 0x307; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x407; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x507; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x187; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "R_wrist";
            }
            else if (motor_pair.first == "L_wrist")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = -108.0f * M_PI / 180.0f; // -108deg
                maxonMotor->rMax = 135.0f * M_PI / 180.0f;  // 135deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x208; // Controlword
                maxonMotor->txPdoIds[1] = 0x308; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x408; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x508; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x188; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "L_wrist";
            }
            else if (motor_pair.first == "R_foot")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = -90.0f * M_PI / 180.0f; // -90deg
                maxonMotor->rMax = 135.0f * M_PI / 180.0f; // 135deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x209; // Controlword
                maxonMotor->txPdoIds[1] = 0x309; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x409; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x509; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x189; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "R_foot";
            }
            else if (motor_pair.first == "L_foot")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = -90.0f * M_PI / 180.0f; // -90deg
                maxonMotor->rMax = 135.0f * M_PI / 180.0f; // 135deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x20A; // Controlword
                maxonMotor->txPdoIds[1] = 0x30A; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x40A; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x50A; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x18A; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "L_foot";
            }
            else if (motor_pair.first == "maxonForTest")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = -90.0f * M_PI / 180.0f; // -90deg
                maxonMotor->rMax = 135.0f * M_PI / 180.0f; // 135deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x20B; // Controlword
                maxonMotor->txPdoIds[1] = 0x30B; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x40B; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x50B; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x18B; // Statusword, ActualPosition, ActualTorque
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

    canManager.serial_fd = canManager.setup_serial_port();
    if (canManager.serial_fd == -1)
    {
        cout << "Serial error";
        return;
    }
}

void DrumRobot::DeactivateControlTask()
{
    struct can_frame frame;

    canManager.setSocketsTimeout(0, 500000);

    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;

        // 타입에 따라 적절한 캐스팅과 초기화 수행
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            tservocmd.comm_can_set_cb(*tMotor, &tMotor->sendFrame, 0);
            canManager.sendMotorFrame(tMotor);
            std::cout << "Exiting for motor [" << name << "]" << std::endl;
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

void DrumRobot::ClearBufferforRecord()
{
    for (auto &motor_pair : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
        {
            tMotor->clearCommandBuffer();
            tMotor->clearReceiveBuffer();
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            maxonMotor->clearCommandBuffer();
            maxonMotor->clearReceiveBuffer();
        }
    }
    canManager.Input_pos.clear();
    Input_pos.clear();
}

void DrumRobot::printCurrentPositions()
{
    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;
        std::cout << "[" << std::hex << motor->nodeId << std::dec << "] ";
        std::cout << name << " Pos: " << motor->currentPos << " Tor: " << motor->currentTor << endl;
    }

    vector<float> P(6);
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

                maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomePosition(*maxonMotor, &frame, 95);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getCurrentThresholdL(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else if (name == "R_wrist")
            {
                maxoncmd.getHomingMethodR(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomePosition(*maxonMotor, &frame, 95);
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
        else if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motorPair.second))
        {
            if (tMotor->myName == "waist")
            {
                tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                canManager.sendMotorFrame(tMotor);
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 Send Thread Loop                           */
///////////////////////////////////////////////////////////////////////////////

void DrumRobot::save_to_txt_inputData(const string &csv_file_name)
{
    // CSV 파일 열기. 파일이 있으면 지우고 새로 생성됩니다.
    std::ofstream ofs_p(csv_file_name + "_pos.txt");

    if (!ofs_p.is_open())
    {
        std::cerr << "Failed to open or create the CSV file: " << csv_file_name << std::endl;
        return;
    }

    // CSV 헤더 추가
    ofs_p << "0x007,0x001,0x002,0x003,0x004,0x005,0x006,0x008,0x009\n";

    for (const auto &row : canManager.Input_pos)
    {
        for (const float cell : row)
        {
            ofs_p << std::fixed << std::setprecision(5) << cell;
            if (&cell != &row.back())
                ofs_p << ","; // 쉼표로 셀 구분
        }
        ofs_p << "\n"; // 다음 행으로 이동
    }

    canManager.Input_pos.clear();
    ofs_p.close();

    std::cout << "DrumData_Input 파일이 생성되었습니다 : " << csv_file_name << std::endl;
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
        else if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motorPair.second))
        {
            tMotor->clearCommandBuffer();
        }
    }
}

void DrumRobot::UnfixedMotor()
{
    for (auto motor_pair : motors)
        motor_pair.second->isfixed = false;
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 Recive Thread Loop                          */
/////////////////////////////////////////////////////////////////////////////////

void DrumRobot::parse_and_save_to_csv(const std::string &csv_file_name)
{
    // CSV 파일 열기. 파일이 있으면 지우고 새로 생성됩니다.
    std::ofstream ofs(csv_file_name + ".txt");
    if (!ofs.is_open())
    {
        std::cerr << "Failed to open or create the CSV file: " << csv_file_name << std::endl;
        return;
    }

    // CSV 헤더 추가
    ofs << "CAN_ID,p_act,v_act,tff_act\n";

    while (true)
    {
        bool allRecvBufferEmpty = true;
        for (const auto &pair : motors)
        {
            auto &motor = pair.second;
            if (!motor->recieveBuffer.empty())
            {
                allRecvBufferEmpty = false;
                can_frame frame = motor->recieveBuffer.front();
                motor->recieveBuffer.pop();

                int id = motor->nodeId;
                float position, speed, torque;

                // TMotor 또는 MaxonMotor에 따른 데이터 파싱 및 출력
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
                {
                    std::tuple<int, float, float, float, int8_t, int8_t> parsedData = tservocmd.motor_receive(&frame);
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

        if (allRecvBufferEmpty)
            break;
    }

    // 각 모터에 대한 처리

    ofs.close();
    std::cout << "DrumData_Output 파일이 생성되었습니다: " << csv_file_name << std::endl;
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

void DrumRobot::breakOn()
{
    static bool isBreak = false;

    if (isBreak)
    {
        char data_to_send = '0'; // 시리얼 포트로 전송할 문자
        canManager.send_char_to_serial(canManager.serial_fd, data_to_send);

        usleep(100000);

        // 데이터 수신
        std::string received_data = canManager.read_char_from_serial(canManager.serial_fd);
        if (!received_data.empty())
        {
            std::cout << "Received data: " << received_data << std::endl;
        }

        isBreak = false;
    }
    else
    {
        char data_to_send = '1'; // 시리얼 포트로 전송할 문자
        canManager.send_char_to_serial(canManager.serial_fd, data_to_send);

        usleep(100000);

        // 데이터 수신
        std::string received_data = canManager.read_char_from_serial(canManager.serial_fd);
        if (!received_data.empty())
        {
            std::cout << "Received data: " << received_data << std::endl;
        }

        isBreak = true;
    }

    return;
}