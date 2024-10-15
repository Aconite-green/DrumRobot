#include "../include/tasks/DrumRobot.hpp"

// For Qt
// #include "../tasks/DrumRobot.hpp"

// DrumRobot 클래스의 생성자
DrumRobot::DrumRobot(State &stateRef,
                     CanManager &canManagerRef,
                     PathManager &pathManagerRef,
                     TestManager &testManagerRef,
                     std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
                     USBIO &usbioRef)
    : state(stateRef),
      canManager(canManagerRef),
      pathManager(pathManagerRef),
      testManager(testManagerRef),
      motors(motorsRef),
      usbio(usbioRef)
{
    ReadStandard = chrono::system_clock::now();
    SendStandard = chrono::system_clock::now();
    addStandard = chrono::system_clock::now();

    send_time_point = std::chrono::steady_clock::now();
    recv_time_point = std::chrono::steady_clock::now();
    state_time_point = std::chrono::steady_clock::now();

}

// CanManager - safetyCheck_M, setMotorsSocket 에서 usleep() 함수 존재

/////////////////////////////////////////////////////////////////////////////////
/*                               SYSTEM LOOPS                             */
///////////////////////////////////////////////////////////////////////////////

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
            if (usbio.USBIO_4761_init())
            {
                std::cout << "USBIO-4761 init\n";
            }
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

            int ret = system("clear");
            if (ret == -1)
                std::cout << "system clear error" << endl;
            break;
        }
        case Main::Test:
        {
            bool isWriteError = false;
            if (state.test == TestSub::SelectParamByUser || state.test == TestSub::SetQValue || state.test == TestSub::SetXYZ || state.test == TestSub::StickTest || state.test == TestSub::SetSingleTuneParm)
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
            // checkUserInput();
            // parse_and_save_to_csv("../../READ/Error_DrumData_out");
            // save_to_txt_inputData("../../READ/Error_DrumData_in");
            state.main = Main::Shutdown;
            break;
        }
        case Main::Shutdown:
            break;
        }
        
        // canManager.appendToCSV_time("TIME_stateMachine");
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
            if (state.home == HomeSub::Done)
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
        case Main::AddStance:
        {
            UnfixedMotor();
            SendAddStanceProcess(5000);
            break;
        }
        case Main::Check:
        {
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

        // canManager.appendToCSV_time("TIME_sendLoopForThread");
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

        // canManager.appendToCSV_time("TIME_recvLoopForThread");

        std::string file_name = "fixed";
        float data = 0;

        for (auto &motor : motors)
        {
            if (motor.first == "waist")
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor.second))
                {
                    if (tMotor->isfixed == false)
                    {
                        data = 1.0;
                    }
                    else
                    {
                        data = -1.0;
                    }

                    canManager.appendToCSV_DATA(file_name, data, tMotor->currentPos, 0);
                }
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
        // test 모드에서 에러 검출 안함
        if (state.home != HomeSub::Done || state.main == Main::Test)
        {
            canManager.distributeFramesToMotors(false);
        }
        // test 모드에서 에러 검출함
        // if (state.home != HomeSub::Done)
        // {
        //     canManager.distributeFramesToMotors(false);
        // }
        else
        {
            bool isSafe = canManager.distributeFramesToMotors(true);
            if (!isSafe)
            {
                state.main = Main::Error;
            }
        }

        state.read = ReadSub::TimeCheck;
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
                // save_to_txt_inputData("../../READ/DrumData_in");
                // parse_and_save_to_csv("../../READ/DrumData_out");
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
                usbio.USBIO_4761_set(motor_mapping[motor_pair.first], tMotor->brake_state);
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
        if (getHome)
        {
            ClearBufferforRecord();
            std::cout << "Get Home...\n";
            clearMotorsCommandBuffer();
            state.addstance = AddStanceSub::FillBuf;
        }
        else if (getReady)
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
            // save_to_txt_inputData("../../READ/AddStance_in");
            // parse_and_save_to_csv("../../READ/AddStance_out");
            state.main = Main::Ideal;
        }
        break;
    }
    case AddStanceSub::FillBuf:
    {
        if (getHome)
        {
            pathManager.GetArr(pathManager.standby);
        }
        else if (getReady)
        {
            pathManager.GetArr(pathManager.readyarr);
        }
        else if (getBack)
        {
            pathManager.GetArr(pathManager.backarr);
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
            if (getHome)
            {
                state.addstance = AddStanceSub::CheckCommand;
                canManager.clearReadBuffers();
                flag_setting("isHome");
            }
            else if (getReady)
            {
                state.addstance = AddStanceSub::CheckCommand;
                canManager.clearReadBuffers();
                flag_setting("isReady");
            }
            else if (getBack && isHome)
            {
                state.main = Main::Shutdown;
            }
            else if (getBack)
            {
                state.addstance = AddStanceSub::CheckCommand;
                canManager.clearReadBuffers();
                flag_setting("isBack");
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
                usbio.USBIO_4761_set(motor_mapping[motor_pair.first], tMotor->brake_state);
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
            std::cout << "- o : Offset setting\n";
            std::cout << "- s : Shut down the system\n";
            std::cout << "- c : Check Motors position\n";
        }
        else
        {
            if (isHome)
            {
                std::cout << "- r : Move to Ready Position\n";
                std::cout << "- b : Move to Back Position\n";
                std::cout << "- t : Start Test\n";
                std::cout << "- c : Check Motors position\n";
                std::cout << "- s : Shut down the system\n";
            }
            else if (isReady)
            {
                std::cout << "- p : Start Drumming\n";
                std::cout << "- h : Move to Home Position\n";
                std::cout << "- t : Start Test\n";
                std::cout << "- c : Check Motors position\n";
            }
            else if (isBack)
            {
                std::cout << "- s : Shut down the system\n";
                std::cout << "- h : Move to Home Position\n";
                std::cout << "- t : Start Test\n";
                std::cout << "- c : Check Motors position\n";
            }
        }
    }
    else
    {
        std::cout << "- s : Shut down the system\n";
        std::cout << "- c : Check Motors position\n";
    }
}

bool DrumRobot::processInput(const std::string &input)
{
    if (state.main == Main::Ideal)
    {
        if((input == "o") && !(state.home == HomeSub::Done))
        {
            for (auto &entry : motors)
            {
                entry.second->isHomed = true;
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                {
                    tMotor->homeOffset = tMotor->cwDir * tMotor->initial_position - (tMotor->currentPos);
                }
            }
            homingMaxonEnable();
            homingSetMaxonMode("CSP");

            state.home = HomeSub::Done;
            flag_setting("isBack");

            return true;
        }
        if (input == "h")
        {   
            if (isBack || isReady)
            {
                state.main = Main::AddStance;
                flag_setting("getHome");

                return true;
            }
        }
        else if (input == "r" && isHome)
        {
            state.main = Main::AddStance;
            flag_setting("getReady");

            return true;
        }
        else if (input == "b" && isHome)
        {
            state.main = Main::AddStance;
            flag_setting("getBack");

            return true;
        }
        else if (input == "p" && isReady)
        {
            state.main = Main::Perform;
            isReady = false;

            return true;
        }
        else if (input == "s")
        {
            if (state.home != HomeSub::Done || isBack)
            {
                state.main = Main::Shutdown;

                return true;
            }
            else if (isHome)
            {
                state.main = Main::AddStance;
                getBack = true; // getBack, isHome 을 동시에 켜면 back pos 이동 후 종료

                return true;
            }
        }
        else if (input == "t" && state.home == HomeSub::Done)
        {
            state.main = Main::Test;

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
                // save_to_txt_inputData("../../READ/interrupted_DrumData_in");
                // parse_and_save_to_csv("../../READ/interrupted_DrumData_out");
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

/////////////////////////////////////////////////////////////////////////////////
/*                                 SYSTEM                                     */
///////////////////////////////////////////////////////////////////////////////

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
                tMotor->timingBelt_ratio = 1.0f;
                tMotor->rMin = motorMinArr[can_id] * M_PI / 180.0f; // -90deg
                tMotor->rMax = motorMaxArr[can_id] * M_PI / 180.0f;  // 90deg
                tMotor->isHomed = true;
                tMotor->myName = "waist";
                tMotor->initial_position = initial_positions[can_id];
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                // tMotor->sensorWriteBit = 0;
            }
            else if (motor_pair.first == "R_arm1")
            {
                tMotor->cwDir = -1.0f;
                tMotor->timingBelt_ratio = 1.0f;
                // tMotor->sensorReadBit = 3;
                tMotor->rMin = motorMinArr[can_id] * M_PI / 180.0f;   // 0deg
                tMotor->rMax = motorMaxArr[can_id] * M_PI / 180.0f; // 150deg
                tMotor->isHomed = false;
                tMotor->myName = "R_arm1";
                tMotor->initial_position = initial_positions[can_id];
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                // tMotor->sensorWriteBit = 0;
            }
            else if (motor_pair.first == "L_arm1")
            {
                tMotor->cwDir = -1.0f;
                tMotor->timingBelt_ratio = 1.0f;
                // tMotor->sensorReadBit = 0;
                tMotor->rMin = motorMinArr[can_id] * M_PI / 180.0f;  // 30deg
                tMotor->rMax = motorMaxArr[can_id] * M_PI / 180.0f; // 180deg
                tMotor->isHomed = false;
                tMotor->myName = "L_arm1";
                tMotor->initial_position = initial_positions[can_id];
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                // tMotor->sensorWriteBit = 0;
            }
            else if (motor_pair.first == "R_arm2")
            {
                tMotor->cwDir = 1.0f;
                tMotor->timingBelt_ratio = 1.0f;
                // tMotor->sensorReadBit = 4;
                tMotor->rMin = motorMinArr[can_id] * M_PI / 180.0f; // -60deg
                tMotor->rMax = motorMaxArr[can_id] * M_PI / 180.0f;  // 90deg
                tMotor->isHomed = false;
                tMotor->myName = "R_arm2";
                tMotor->initial_position = initial_positions[can_id];
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                // tMotor->sensorWriteBit = 0;
            }
            else if (motor_pair.first == "R_arm3")
            {
                tMotor->cwDir = -1.0f;
                tMotor->timingBelt_ratio = 3.0f;
                // tMotor->sensorReadBit = 5;
                tMotor->rMin = motorMinArr[can_id] * M_PI / 180.0f; // -30deg
                tMotor->rMax = motorMaxArr[can_id] * M_PI / 180.0f; // 144deg
                tMotor->isHomed = false;
                tMotor->myName = "R_arm3";
                tMotor->initial_position = initial_positions[can_id];
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                // tMotor->sensorWriteBit = 1;
            }
            else if (motor_pair.first == "L_arm2")
            {
                tMotor->cwDir = -1.0f;
                tMotor->timingBelt_ratio = 1.0f;
                // tMotor->sensorReadBit = 1;
                tMotor->rMin = motorMinArr[can_id] * M_PI / 180.0f; // -60deg
                tMotor->rMax = motorMaxArr[can_id] * M_PI / 180.0f;  // 90deg
                tMotor->isHomed = false;
                tMotor->myName = "L_arm2";
                tMotor->initial_position = initial_positions[can_id];
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                // tMotor->sensorWriteBit = 0;
            }
            else if (motor_pair.first == "L_arm3")
            {
                tMotor->cwDir = 1.0f;
                tMotor->timingBelt_ratio = 3.0f;
                // tMotor->sensorReadBit = 2;
                tMotor->rMin = motorMinArr[can_id] * M_PI / 180.0f; // -30 deg
                tMotor->rMax = motorMaxArr[can_id] * M_PI / 180.0f; // 144 deg
                tMotor->isHomed = false;
                tMotor->myName = "L_arm3";
                tMotor->initial_position = initial_positions[can_id];
                tMotor->spd = 1000;
                tMotor->acl = 3000;
                // tMotor->sensorWriteBit = 0;
            }

            // std::cout << can_id << "\t" << tMotor->myName << "\t" << tMotor->rMin << "\t" << tMotor->rMax << endl;
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            // 각 모터 이름에 따른 멤버 변수 설정
            if (motor_pair.first == "R_wrist")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = motorMinArr[can_id] * M_PI / 180.0f; // -108deg
                maxonMotor->rMax = motorMaxArr[can_id] * M_PI / 180.0f;  // 135deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x207; // Controlword
                maxonMotor->txPdoIds[1] = 0x307; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x407; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x507; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x187; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "R_wrist";
                maxonMotor->initial_position = initial_positions[can_id];
            }
            else if (motor_pair.first == "L_wrist")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = motorMinArr[can_id] * M_PI / 180.0f; // -108deg
                maxonMotor->rMax = motorMaxArr[can_id] * M_PI / 180.0f;  // 135deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x208; // Controlword
                maxonMotor->txPdoIds[1] = 0x308; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x408; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x508; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x188; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "L_wrist";
                maxonMotor->initial_position = initial_positions[can_id];
            }
            else if (motor_pair.first == "R_foot")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = motorMinArr[can_id] * M_PI / 180.0f; // -90deg
                maxonMotor->rMax = motorMaxArr[can_id] * M_PI / 180.0f; // 135deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x209; // Controlword
                maxonMotor->txPdoIds[1] = 0x309; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x409; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x509; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x189; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "R_foot";
                maxonMotor->initial_position = initial_positions[can_id];
            }
            else if (motor_pair.first == "L_foot")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = motorMinArr[can_id] * M_PI / 180.0f; // -90deg
                maxonMotor->rMax = motorMaxArr[can_id] * M_PI / 180.0f; // 135deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x20A; // Controlword
                maxonMotor->txPdoIds[1] = 0x30A; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x40A; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x50A; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x18A; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "L_foot";
                maxonMotor->initial_position = initial_positions[can_id];
            }
            else if (motor_pair.first == "maxonForTest")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = motorMinArr[can_id] * M_PI / 180.0f; // -90deg
                maxonMotor->rMax = motorMaxArr[can_id] * M_PI / 180.0f; // 135deg
                maxonMotor->isHomed = false;
                maxonMotor->txPdoIds[0] = 0x20B; // Controlword
                maxonMotor->txPdoIds[1] = 0x30B; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x40B; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x50B; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x18B; // Statusword, ActualPosition, ActualTorque
                maxonMotor->myName = "maxonForTest";
                maxonMotor->initial_position = initial_positions[can_id];
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

/////////////////////////////////////////////////////////////////////////////////
/*                            flag setting                                     */
/////////////////////////////////////////////////////////////////////////////////

void DrumRobot::flag_setting(string flag)
{
    // all flag off
    getHome = false;
    isHome = false;
    getReady = false;
    isReady = false;
    getBack = false;
    isBack = false;

    // only one flag on
    if (flag == "getHome")
    {
        getHome = true;
    }
    else if (flag == "isHome")
    {
        isHome = true;
    }
    else if (flag == "getReady")
    {
        getReady = true;
    }
    else if (flag == "isReady")
    {
        isReady = true;
    }
    else if (flag == "getBack")
    {
        getBack = true;
    }
    else if (flag == "isBack")
    {
        isBack = true;
    }
    else
    {
        cout << "flag error\n";
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                       Homing Process Function                               */
/////////////////////////////////////////////////////////////////////////////////

void DrumRobot::homingMaxonEnable()
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

            maxoncmd.getOperational(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            usleep(100000);
            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);
            std::cout << "Maxon Enabled(1) \n";

            usleep(100000);

            maxoncmd.getQuickStop(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            usleep(100000);
            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            std::cout << "Maxon Enabled(2) \n";
        }
    }
}

void DrumRobot::homingSetMaxonMode(std::string targetMode)
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

/////////////////////////////////////////////////////////////////////////////////
/*                             Log Data                                        */
/////////////////////////////////////////////////////////////////////////////////

void DrumRobot::toCSV_sendData(const string &csv_file_name)
{
    // CSV 파일 열기. 파일이 있으면 지우고 새로 생성됩니다.
    std::ofstream ofs_p(csv_file_name + ".txt");

    if (!ofs_p.is_open())
    {
        std::cerr << "Failed to open or create the CSV file: " << csv_file_name << std::endl;
        return;
    }

    // CSV 헤더 추가
    ofs_p << "0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08\n";

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

    std::cout << "sendData 파일이 생성되었습니다 : " << csv_file_name << std::endl;
}

void DrumRobot::toCSV_recvData(const string &csv_file_name)
{
    // CSV 파일 열기. 파일이 있으면 지우고 새로 생성됩니다.

    std::ofstream ofs(csv_file_name + ".txt");
    if (!ofs.is_open())
    {
        std::cerr << "Failed to open or create the CSV file: " << csv_file_name << std::endl;
        return;
    }

    // CSV 헤더 추가
    ofs << "CAN_ID,position,velocity,current\n";

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
                float position, velocity, current;

                // TMotor 또는 MaxonMotor에 따른 데이터 파싱 및 출력
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
                {
                    std::tuple<int, float, float, float, int8_t, int8_t> parsedData = tservocmd.motor_receive(&frame);
                    position = std::get<1>(parsedData);
                    velocity = std::get<2>(parsedData);
                    current = std::get<3>(parsedData);
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
                {
                    std::tuple<int, float, float, int8_t> parsedData = maxoncmd.parseRecieveCommand(*maxonMotor, &frame);
                    position = std::get<1>(parsedData);
                    current = std::get<2>(parsedData);
                    velocity = 0.0;
                }

                // 데이터 CSV 파일에 쓰기
                ofs << "0x" << std::hex << std::setw(4) << std::setfill('0') << id << ","
                    << std::dec << position << "," << velocity << "," << current << "\n";
            }
        }

        if (allRecvBufferEmpty)
            break;
    }

    

    // 각 모터에 대한 처리

    ofs.close();
    std::cout << "recvData 파일이 생성되었습니다: " << csv_file_name << std::endl;
}