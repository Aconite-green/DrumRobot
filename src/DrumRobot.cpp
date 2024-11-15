#include "../include/tasks/DrumRobot.hpp"

// For Qt
// #include "../tasks/DrumRobot.hpp"

// DrumRobot 클래스의 생성자
DrumRobot::DrumRobot(State &stateRef,
                     CanManager &canManagerRef,
                     PathManager &pathManagerRef,
                     TestManager &testManagerRef,
                     std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
                     USBIO &usbioRef,
                     Functions &funRef)
    : state(stateRef),
      canManager(canManagerRef),
      pathManager(pathManagerRef),
      testManager(testManagerRef),
      motors(motorsRef),
      usbio(usbioRef),
      fun(funRef)
{
    ReadStandard = chrono::system_clock::now();
    SendStandard = chrono::system_clock::now();
    addStandard = chrono::system_clock::now();

    send_time_point = std::chrono::steady_clock::now();
    recv_time_point = std::chrono::steady_clock::now();
    state_time_point = std::chrono::steady_clock::now();

}

////////////////////////////////////////////////////////////////////////////////
/*                                SYSTEM LOOPS                                */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::stateMachine()
{
    while (state.main != Main::Shutdown)
    {
        state_time_point = std::chrono::steady_clock::now();
        state_time_point += std::chrono::microseconds(5000);    // 주기 : 5ms

        switch (state.main.load())
        {
        case Main::SystemInit:
        {
            initializeMotors();
            initializecanManager();
            motorSettingCmd();
            canManager.setSocketNonBlock();
            usbio.USBIO_4761_init();
            fun.openCSVFile();

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
        case Main::Perform:
        {
            checkUserInput();
            break;
        }
        case Main::Play:
        {
            checkUserInput();
            break;
        }
        case Main::Test:
        {
            bool isWriteError = false;
            if (state.test == TestSub::SelectParamByUser || state.test == TestSub::SetQValue || state.test == TestSub::SetXYZ)
            {
                if (!canManager.checkAllMotors_Fixed()) // stateMachine() 주기가 5ms 라서 delay 필요 없음
                {
                    isWriteError = true;
                }
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
            state.main = Main::Shutdown;
            break;
        }
        case Main::Shutdown:
            break;
        }
        
        std::this_thread::sleep_until(state_time_point);
    }

    if (usbio.useUSBIO)
    {
        usbio.USBIO_4761_exit();
    }
    canManager.setSocketBlock();
    DeactivateControlTask();
}

void DrumRobot::sendLoopForThread()
{
    initializePathManager();
    while (state.main != Main::Shutdown)
    {
        send_time_point = std::chrono::steady_clock::now();
        send_time_point += std::chrono::microseconds(100);  // 주기 : 100us

        switch (state.main.load())
        {
        case Main::SystemInit:
        {
            break;
        }
        case Main::Ideal:
        {
            bool isWriteError = false;
            if (setInitialPosition)
            {
               if (!canManager.checkAllMotors_Fixed())
               {
                   isWriteError = true;
               }
            }
            // else
            // {
            //    canManager.checkMaxon();
            // }
            
            if (isWriteError)
            {
               state.main = Main::Error;
            }
            usleep(5000);   // sendLoopForThread() 주기가 100us 라서 delay 필요
            break;
        }
        case Main::Perform:
        {
            UnfixedMotor();
            SendPerformProcess(5000);
            break;
        }
        case Main::Play:
        {
            UnfixedMotor();
            SendPlayProcess(5000);
            break;
        }
        case Main::AddStance:
        {
            UnfixedMotor();
            SendAddStanceProcess(5000);
            break;
        }
        case Main::Test:
        {
            // UnfixedMotor();
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
            usleep(5000);  // sendLoopForThread() 주기가 100us 라서 delay 필요
            break;
        }
        case Main::Error:
        {
            break;
        }
        case Main::Shutdown:
            break;
        }

        std::this_thread::sleep_until(send_time_point);
    }
}

void DrumRobot::recvLoopForThread()
{
    while (state.main != Main::Shutdown)
    {
        recv_time_point = std::chrono::steady_clock::now();
        recv_time_point += std::chrono::microseconds(100);  // 주기 : 100us

        switch (state.main.load())
        {
        case Main::SystemInit:
        {
            break;
        }
        case Main::Ideal:
        {
            ReadProcess(5000); /*5ms*/
            break;
        }
        case Main::Perform:
        {
            ReadProcess(5000);
            break;
        }
        case Main::Play:
        {
            ReadProcess(5000);
            break;
        }
        case Main::AddStance:
        {
            ReadProcess(5000);
            break;
        }
        case Main::Test:
        {
            ReadProcess(5000);
            break;
        }
        case Main::Pause:
        {
            ReadProcess(5000);
            break;
        }
        case Main::Error:
        {
            break;
        }
        case Main::Shutdown:
        {
            break;
        }
        }

        std::this_thread::sleep_until(recv_time_point);
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
        
        // if ((!setInitialPosition) || state.main == Main::Test) // test 모드에서 에러 검출 안함
        // {
        //     canManager.distributeFramesToMotors(false);
        // }
        if (!setInitialPosition)    // test 모드에서 에러 검출함
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

        // if (maxonMotorCount == 0)
        // {
        //     state.read = ReadSub::TimeCheck;
        // }
        // else
        // {
        //     for (auto &motor_pair : motors)
        //     {
        //         auto &motor = motor_pair.second;
        //         if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        //         {
        //             maxonMotor->checked = false;
        //         }
        //     }
        //     state.read = ReadSub::CheckMaxonControl;
        // }
        state.read = ReadSub::TimeCheck;
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
                    if (pathManager.wrist_targetPos < maxonMotor->motorPosition)
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

void DrumRobot::SendPlayProcess(int periodMicroSec)
{
    auto currentTime = chrono::system_clock::now();
    auto elapsedTime = chrono::duration_cast<chrono::microseconds>(currentTime - SendStandard);

    switch (state.play.load())
    {
    case PlaySub::TimeCheck:
    {
        if (elapsedTime.count() >= periodMicroSec)
        {
            cnt++;
            state.play = PlaySub::GenerateTrajectory;   // 주기가 되면 GenerateTrajectory 상태로 진입
            SendStandard = currentTime;                 // 현재 시간으로 시간 객체 초기화
        }
        break;
    }
    case PlaySub::GenerateTrajectory:
    {
        if (pathManager.line >= pathManager.total)
        {
            std::cout << "Play is Over\n";
            state.main = Main::AddStance;
            state.play = PlaySub::TimeCheck;
            flag_setting("getHome");
            pathManager.line = 0;
        }
        
        if (pathManager.P.empty()) // P가 비어있으면 새로 생성
        {
            std::cout << "\n//////////////////////////////// line : " << pathManager.line << ", total : " << pathManager.total << "\n";
            pathManager.generateTrajectory();
            pathManager.line++;
        }
        
        state.play = PlaySub::SolveIK;

        break;
    }
    case PlaySub::SolveIK:
    {
        PathManager::Pos nextPos; // IK 풀 때 들어갈 다음 xyz
        nextPos = pathManager.P.front(); // P의 맨 앞 값을 다음 목표 위치로
        pathManager.P.pop(); // 앞에꺼 지움

        // VectorXd pR1(3);
        // VectorXd pL1(3);

        // pR1 << nextPos.pR[0], nextPos.pR[1], nextPos.pR[2];
        // pL1 << nextPos.pL[0], nextPos.pL[1], nextPos.pL[2];

        pathManager.solveIK(nextPos.pR, nextPos.pL);

        //IK 하기 전에 다음 위치 목표 x,y,z 값 받아와야댐
        //solveIK 하면 command buffer에  하나 값 넣어줘야댐
        // setCANFrame 함수로 가면 command buffer에 있는 젤 앞에 있는 값 써서 프레임 만들고 send에서 보냄
        //IK 풀어서 setcanFrame에 넘기기
        state.play = PlaySub::SetCANFrame;

        break;
    }
    case PlaySub::SetCANFrame:
    {
        bool isSafe;
        isSafe = canManager.setCANFrame();
        if (!isSafe)
        {
            state.main = Main::Error;
        }

        state.play = PlaySub::SendCANFrame;
        break;
    }
    case PlaySub::SendCANFrame:
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
                usbio.USBIO_4761_set(motor_mapping[motor_pair.first], tMotor->brakeState);
            }
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
            state.play = PlaySub::TimeCheck;
        }

        // brake
        if (usbio.useUSBIO)
        {
            int cnt = 0;
            while(!usbio.USBIO_4761_output())
            {
                cout << "brake Error\n";
                usbio.USBIO_4761_init();
                cnt++;
                if (cnt >= 5) break;
            }
        }

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
            cnt++;
            state.perform = PerformSub::CheckBuf; // 주기가 되면 ReadCANFrame 상태로 진입
            SendStandard = currentTime;           // 현재 시간으로 시간 객체 초기화
        }
        break;
    }
    case PerformSub::CheckBuf:
    {
        bool motor_connected = false;

        for (const auto &motor_pair : motors)
        {
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
            {
                motor_connected = true;
                if (tMotor->commandBuffer.size() < 10)
                    state.perform = PerformSub::GeneratePath;
                else
                    state.perform = PerformSub::SetCANFrame;
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
            {
                motor_connected = true;
                if (maxonMotor->commandBuffer.size() < 10)
                    state.perform = PerformSub::GeneratePath;
                else
                    state.perform = PerformSub::SetCANFrame;
            }
        }

        if(!motor_connected)
        {
            // 모터 연결 안됨 -> 프로그램만 실행
            state.perform = PerformSub::GeneratePath;
        }
        break;
    }
    case PerformSub::GeneratePath:
    {
        if (pathManager.line < pathManager.total)
        {
            std::cout << "\n//////////////////////////////// line : " << pathManager.line << ", total : " << pathManager.total << "\n";
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
                state.main = Main::AddStance;
                state.perform = PerformSub::TimeCheck;
                flag_setting("getHome");
                pathManager.line = 0;
            }
            else
            {
                state.perform = PerformSub::SetCANFrame;
            }
        }
        break;
    }
    case PerformSub::SetCANFrame:
    {
        bool isSafe;
        isSafe = canManager.setCANFrame();
        if (!isSafe)
        {
            state.main = Main::Error;
        }

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

            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
            {
                usbio.USBIO_4761_set(motor_mapping[motor_pair.first], tMotor->brakeState);
            }
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

        // brake
        if (usbio.useUSBIO)
        {
            int cnt = 0;
            while(!usbio.USBIO_4761_output())
            {
                cout << "brake Error\n";
                usbio.USBIO_4761_init();
                cnt++;
                if (cnt >= 5) break;
            }
        }

        break;
    }
    }
}

void DrumRobot::SendAddStanceProcess(int periodMicroSec)
{
    auto currentTime = chrono::system_clock::now();
    auto elapsed_time = chrono::duration_cast<chrono::microseconds>(currentTime - addStandard);

    switch (state.addstance.load())
    {
    case AddStanceSub::CheckCommand:
    {
        if (getHome || getReady || getBackAndShutdown)
        {
            ClearBufferforRecord();
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
        if (getReady)
        {
            std::cout << "Get Ready Pos Array ...\n";
            pathManager.GetArr(pathManager.readyArr);
        }
        else if (getHome)
        {
            std::cout << "Get Home Pos Array ...\n";
            pathManager.GetArr(pathManager.homeArr);
        }
        else if (getBackAndShutdown)
        {
            std::cout << "Get Back Pos Array ...\n";
            pathManager.GetArr(pathManager.backArr);
        }


        state.addstance = AddStanceSub::TimeCheck;
        break;
    }
    case AddStanceSub::TimeCheck:
    {
        if (elapsed_time.count() >= periodMicroSec)
        {
            cnt++;
            state.addstance = AddStanceSub::CheckBuf;
            addStandard = currentTime;           // 현재 시간으로 시간 객체 초기화
        }
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
            if (getBackAndShutdown)
            {
                state.main = Main::Shutdown;
            }
            else if (getHome)
            {
                state.addstance = AddStanceSub::CheckCommand;
                state.main = Main::Ideal;
                canManager.clearReadBuffers();
                flag_setting("isHome");
            }
            else if (getReady)
            {
                state.addstance = AddStanceSub::CheckCommand;
                state.main = Main::Ideal;
                canManager.clearReadBuffers();
                flag_setting("isReady");
            }
        }
        break;
    }
    case AddStanceSub::SetCANFrame:
    {
        bool isSafe;
        isSafe = canManager.setCANFrame();
        if (!isSafe)
        {
            state.main = Main::Error;
        }
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
                usbio.USBIO_4761_set(motor_mapping[motor_pair.first], tMotor->brakeState);
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

        // brake
        if (usbio.useUSBIO)
        {
            if(!usbio.USBIO_4761_output())
            {
                cout << "brake Error\n";
            }
        }

        break;
    }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                                STATE UTILITY                               */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::displayAvailableCommands() const
{
    std::cout << "Available Commands:\n";

    if (state.main == Main::Ideal)
    {
        if (!setInitialPosition)
        {
            std::cout << "- o : Set Zero & Offset setting\n";
            std::cout << "- i : Offset setting\n";
        }
        else
        {
            if (isHome)
            {
                std::cout << "- r : Move to Ready Pos\n";
                std::cout << "- t : Start Test\n";
                std::cout << "- s : Shut down the system\n";
            }
            else if (isReady)
            {
                std::cout << "- f : Start Drumming Old Version\n";
                std::cout << "- p : Play Drum\n";
                std::cout << "- t : Start Test\n";
                std::cout << "- h : Move to Home Pos\n";
                std::cout << "- s : Shut down the system\n";

            }
        }
    }
    else
    {
        std::cout << "- s : Shut down the system\n";
    }
}

bool DrumRobot::processInput(const std::string &input)
{
    if (state.main == Main::Ideal)
    {
        if(!setInitialPosition)
        {
            if((input == "o"))
            {
                // set zero
                for (const auto &motorPair : motors)
                {
                    if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motorPair.second))
                    {
                        if (tMotor->myName == "waist")
                        {
                            tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                            canManager.sendMotorFrame(tMotor);
                        }
                        else if (tMotor->myName == "R_arm1")
                        {
                            tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                            canManager.sendMotorFrame(tMotor);
                        }
                        else if (tMotor->myName == "L_arm1")
                        {
                            tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                            canManager.sendMotorFrame(tMotor);
                        }
                        else if (tMotor->myName == "R_arm2")
                        {
                            tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                            canManager.sendMotorFrame(tMotor);
                        }
                        else if (tMotor->myName == "L_arm2")
                        {
                            tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                            canManager.sendMotorFrame(tMotor);
                        }
                        else if (tMotor->myName == "R_arm3")
                        {
                            tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                            canManager.sendMotorFrame(tMotor);
                        }
                        else if (tMotor->myName == "L_arm3")
                        {
                            tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                            canManager.sendMotorFrame(tMotor);
                        }
                    }   
                }
                std::cout << "set zero and offset setting ~ ~ ~\n";
                sleep(2);   // setZero 명령이 확실히 실행된 후 fixed 함수 실행

                maxonMotorEnable();
                setMaxonMotorMode("CSP");

                setInitialPosition = true;
                flag_setting("isHome");

                return true;
            }
            else if((input == "i") && !(setInitialPosition))
            {
                maxonMotorEnable();
                setMaxonMotorMode("CSP");

                setInitialPosition = true;
                flag_setting("isHome");

                return true;
            }
        }
        else
        {
            if (input == "r" && isHome)
            {
                state.main = Main::AddStance;
                flag_setting("getReady");

                return true;
            }
            else if (input == "f" && isReady)
            {
                std::cout << "\nbpm : " << pathManager.bpm << std::endl;
                state.main = Main::Perform;
                isReady = false;

                return true;
            }
            else if (input == "p" && isReady)
            {
                std::cout << "\nbpm : " << pathManager.bpm << std::endl;
                state.main = Main::Play;
                isReady = false;

                return true;
            }
            else if (input == "h")
            {
                if (isReady)
                {
                    state.main = Main::AddStance;
                    flag_setting("getHome");

                    return true;
                }

            }
            else if (input == "s")
            {
                state.main = Main::AddStance;
                flag_setting("getBackAndShutdown");

                return true;
            }
            else if (input == "t")
            {
                state.main = Main::Test;

                return true;
            }
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

    // usleep(2000);    // sleep_until()
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
                pathManager.line = 0;
                state.main = Main::Shutdown;
            }
            else if (input == 'r')
                state.main = Main::Perform;
            else if (input == 's')
                state.main = Main::Shutdown;
            else if (input == 'h')
            {
                state.main = Main::AddStance;
                flag_setting("getHome");
            }
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
                state.main = Main::Shutdown;
                pathManager.line = 0;
            }
            else if (input == 'h')
            {
                state.main = Main::AddStance;
                flag_setting("getHome");
            }
            else if (input == 's')
                state.main = Main::Shutdown;
        }
    }

    if (isWriteError)
    {
        state.main = Main::Error;
    }
    // usleep(5000);    // sleep_until()
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

////////////////////////////////////////////////////////////////////////////////
/*                                 SYSTEM                                     */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::initializeMotors()
{
    motors["waist"] = make_shared<TMotor>(0x00, "AK10_9");
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
        int can_id = motor_mapping[motor_pair.first];

        // 타입에 따라 적절한 캐스팅과 초기화 수행
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            // 각 모터 이름에 따른 멤버 변수 설정
            if (motor_pair.first == "waist")
            {
                tMotor->cwDir = 1.0f;
                tMotor->timingBeltRatio = 1.0f;
                tMotor->rMin = joint_range_min[can_id] * M_PI / 180.0f; // -90deg
                tMotor->rMax = joint_range_max[can_id] * M_PI / 180.0f;  // 90deg
                tMotor->myName = "waist";
                tMotor->initialJointAngle = initial_joint_angles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 29.8;  // [A]    // ak10-9
            }
            else if (motor_pair.first == "R_arm1")
            {
                tMotor->cwDir = -1.0f;
                tMotor->timingBeltRatio = 1.0f;
                tMotor->rMin = joint_range_min[can_id] * M_PI / 180.0f;   // 0deg
                tMotor->rMax = joint_range_max[can_id] * M_PI / 180.0f; // 150deg
                tMotor->myName = "R_arm1";
                tMotor->initialJointAngle = initial_joint_angles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 23.2;  // [A]    // ak70-10
            }
            else if (motor_pair.first == "L_arm1")
            {
                tMotor->cwDir = -1.0f;
                tMotor->timingBeltRatio = 1.0f;
                tMotor->rMin = joint_range_min[can_id] * M_PI / 180.0f;  // 30deg
                tMotor->rMax = joint_range_max[can_id] * M_PI / 180.0f; // 180deg
                tMotor->myName = "L_arm1";
                tMotor->initialJointAngle = initial_joint_angles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 23.2;  // [A]    // ak70-10
            }
            else if (motor_pair.first == "R_arm2")
            {
                tMotor->cwDir = 1.0f;
                tMotor->timingBeltRatio = 1.0f;
                tMotor->rMin = joint_range_min[can_id] * M_PI / 180.0f; // -60deg
                tMotor->rMax = joint_range_max[can_id] * M_PI / 180.0f;  // 90deg
                tMotor->myName = "R_arm2";
                tMotor->initialJointAngle = initial_joint_angles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 23.2;  // [A]    // ak70-10
            }
            else if (motor_pair.first == "R_arm3")
            {
                tMotor->cwDir = -1.0f;
                tMotor->timingBeltRatio = 3.0f;
                tMotor->rMin = joint_range_min[can_id] * M_PI / 180.0f; // -30deg
                tMotor->rMax = joint_range_max[can_id] * M_PI / 180.0f; // 130deg
                tMotor->myName = "R_arm3";
                tMotor->initialJointAngle = initial_joint_angles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 23.2;  // [A]    // ak70-10
            }
            else if (motor_pair.first == "L_arm2")
            {
                tMotor->cwDir = -1.0f;
                tMotor->timingBeltRatio = 1.0f;
                tMotor->rMin = joint_range_min[can_id] * M_PI / 180.0f; // -60deg
                tMotor->rMax = joint_range_max[can_id] * M_PI / 180.0f;  // 90deg
                tMotor->myName = "L_arm2";
                tMotor->initialJointAngle = initial_joint_angles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 23.2;  // [A]    // ak70-10
            }
            else if (motor_pair.first == "L_arm3")
            {
                tMotor->cwDir = 1.0f;
                tMotor->timingBeltRatio = 3.0f;
                tMotor->rMin = joint_range_min[can_id] * M_PI / 180.0f; // -30 deg
                tMotor->rMax = joint_range_max[can_id] * M_PI / 180.0f; // 130 deg
                tMotor->myName = "L_arm3";
                tMotor->initialJointAngle = initial_joint_angles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 23.2;  // [A]    // ak70-10
            }
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            // 각 모터 이름에 따른 멤버 변수 설정
            if (motor_pair.first == "R_wrist")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = joint_range_min[can_id] * M_PI / 180.0f; // -108deg
                maxonMotor->rMax = joint_range_max[can_id] * M_PI / 180.0f;  // 135deg
                maxonMotor->txPdoIds[0] = 0x207; // Controlword
                maxonMotor->txPdoIds[1] = 0x307; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x407; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x507; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x187; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "R_wrist";
                maxonMotor->initialJointAngle = initial_joint_angles[can_id] * M_PI / 180.0f;
            }
            else if (motor_pair.first == "L_wrist")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = joint_range_min[can_id] * M_PI / 180.0f; // -108deg
                maxonMotor->rMax = joint_range_max[can_id] * M_PI / 180.0f;  // 135deg
                maxonMotor->txPdoIds[0] = 0x208; // Controlword
                maxonMotor->txPdoIds[1] = 0x308; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x408; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x508; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x188; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "L_wrist";
                maxonMotor->initialJointAngle = initial_joint_angles[can_id] * M_PI / 180.0f;
            }
            else if (motor_pair.first == "R_foot")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = joint_range_min[can_id] * M_PI / 180.0f; // -90deg
                maxonMotor->rMax = joint_range_max[can_id] * M_PI / 180.0f; // 135deg
                maxonMotor->txPdoIds[0] = 0x209; // Controlword
                maxonMotor->txPdoIds[1] = 0x309; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x409; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x509; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x189; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "R_foot";
                maxonMotor->initialJointAngle = initial_joint_angles[can_id] * M_PI / 180.0f;
            }
            else if (motor_pair.first == "L_foot")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = joint_range_min[can_id] * M_PI / 180.0f; // -90deg
                maxonMotor->rMax = joint_range_max[can_id] * M_PI / 180.0f; // 135deg
                maxonMotor->txPdoIds[0] = 0x20A; // Controlword
                maxonMotor->txPdoIds[1] = 0x30A; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x40A; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x50A; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x18A; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "L_foot";
                maxonMotor->initialJointAngle = initial_joint_angles[can_id] * M_PI / 180.0f;
            }
            else if (motor_pair.first == "maxonForTest")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = joint_range_min[can_id] * M_PI / 180.0f; // -90deg
                maxonMotor->rMax = joint_range_max[can_id] * M_PI / 180.0f; // 135deg
                maxonMotor->txPdoIds[0] = 0x20B; // Controlword
                maxonMotor->txPdoIds[1] = 0x30B; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x40B; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x50B; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x18B; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "maxonForTest";
                maxonMotor->initialJointAngle = initial_joint_angles[can_id] * M_PI / 180.0f;
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
}

void DrumRobot::printCurrentPositions()
{
    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;
        std::cout << "[" << std::hex << motor->nodeId << std::dec << "] ";
        std::cout << name << " Pos: " << motor->motorPosition << endl;
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
            maxoncmd.getCSPMode(*maxonMotor, &frame);
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

                // maxoncmd.getCurrentThresholdL(*maxonMotor, &frame);
                // canManager.sendAndRecv(motor, frame);
            }
            else if (name == "R_wrist")
            {
                maxoncmd.getHomingMethodR(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomePosition(*maxonMotor, &frame, 95);
                canManager.sendAndRecv(motor, frame);

                // maxoncmd.getCurrentThresholdR(*maxonMotor, &frame);
                // canManager.sendAndRecv(motor, frame);
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
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                                 Send Thread Loop                           */
////////////////////////////////////////////////////////////////////////////////

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

/////////////////////////////////////////////////////////////////////////////////
/*                            flag setting                                     */
/////////////////////////////////////////////////////////////////////////////////

void DrumRobot::flag_setting(string flag)
{
    getReady = false;
    isReady = false;
    getHome = false;
    isHome = false;
    getBackAndShutdown = false;

    // only one flag on
    if (flag == "getReady")
    {
        getReady = true;
    }
    else if (flag == "isReady")
    {
        isReady = true;
    }
    else if (flag == "getHome")
    {
        getHome = true;
    }
    else if (flag == "isHome")
    {
        isHome = true;
    }
    else if (flag == "getBackAndShutdown")
    {
        getBackAndShutdown = true;
    }
    else
    {
        cout << "flag error\n";
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                         Maxon Motor Function                               */
/////////////////////////////////////////////////////////////////////////////////

void DrumRobot::maxonMotorEnable()
{
    struct can_frame frame;
    canManager.setSocketsTimeout(2, 0);

    // 제어 모드 설정
    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            maxoncmd.getHomeMode(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getOperational(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            usleep(100000);

            maxoncmd.getShutdown(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            usleep(100000);
            
            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);
            
            std::cout << "Maxon Enabled(1) \n";

            usleep(100000);

            // maxoncmd.getStartHoming(*maxonMotor, &frame);
            // canManager.txFrame(motor, frame);

            // maxoncmd.getSync(&frame);
            // canManager.txFrame(motor, frame);

            // usleep(100000);
            
            std::cout << "Maxon Enabled(2) \n";
        }
    }
}

void DrumRobot::setMaxonMotorMode(std::string targetMode)
{
    struct can_frame frame;
    canManager.setSocketsTimeout(0, 10000);
    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motorPair.second))
        {
            if (targetMode == "CSV")    // Cyclic Sync Velocity Mode
            {
                maxoncmd.getCSVMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else if (targetMode == "CST")   // Cyclic Sync Torque Mode
            {
                maxoncmd.getCSTMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else if (targetMode == "HMM")   // Homming Mode
            {
                maxoncmd.getHomeMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else if (targetMode == "CSP")   // Cyclic Sync Position Mode
            {
                maxoncmd.getCSPMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
        }
    }
}
