#include "../include/managers/TestManager.hpp" // 적절한 경로로 변경하세요.

// For  Qt
// #include "../managers/TestManager.hpp"
using namespace std;

TestManager::TestManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef, Sensor &sensorRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef), sensor(sensorRef)
{

}

void TestManager::SendTestProcess()
{
    // 선택에 따라 testMode 설정
    switch (state.test.load())
    {
    case TestSub::SelectParamByUser:
    {
        cnt = 0;
        int ret = system("clear");
        if (ret == -1)
            std::cout << "system clear error" << endl;

        float c_MotorAngle[9];
        getMotorPos(c_MotorAngle);

        cout << "[ Current Q Values (Ladian) ]\n";
        for (int i = 0; i < 9; i++)
        {
            q[i] = c_MotorAngle[i];
            cout << "Q[" << i << "] : " << c_MotorAngle[i] << "\n";
        }
        fkfun(c_MotorAngle); // 현재 q값에 대한 fkfun 진행

        cout << "\nSelect Method (1 - 관절각도값 조절, 2 - 좌표값 조절, 3 - 단일 회전, 4 - 멀티 회전, 5 - 스틱 타격, 6 - Command, 7 - 나가기, 8 - Break) : ";
        cin >> method;
        
        if (method == 1)
        {
            state.test = TestSub::SetQValue;
        }
        else if (method == 2)
        {
            state.test = TestSub::SetXYZ;
        }
        else if (method == 3)
        {
            state.test = TestSub::SetSingleTuneParm;
        }
        else if (method == 4)
        {
            multiTestLoop(); // 예전에 사용하던 코드. 서보모드 적용 X
        }
        else if (method == 5)
        {
            state.test = TestSub::StickTest;
        }
        else if (method == 6)
        {
            state.test = TestSub::SetServoTestParm;
        }
        else if (method == 7)
        {
            state.main = Main::Ideal;
        }
        else if (method == 8)
        {
            testBreak();
        }
        else if (method == 9)
        {
            // 이 라인 실행하면 cin 안됨
            state.test = TestSub::SetCommand;
        }

        break;
    }
    case TestSub::SetQValue:
    {
        int userInput = 100;
        int ret = system("clear");

        if (ret == -1)
            std::cout << "system clear error" << endl;
        cout << "[ Current Q Values (Radian) ]\n";
        for (int i = 0; i < 9; i++)
        {
            cout << "q[" << i << "] : " << q[i] << "\n";
        }
        cout << "time : " << t << "s\n";
        cout << "break start time : " << break_start_time << "s\n";
        cout << "break end time : " << break_end_time << "s\n";
        cout << "spd : " << speed_test << "erpm\n";
        cout << "repeat flag to " << repeat_flag << endl;
        cout << "buffer test flag to " << buffer_test_flag << "\n";

        cout << "\nSelect Motor to Change Value (0-8) / Start Test (9) / Time (10) / q 확인 (11) / Speed (12) / BreakTime (13) / Repeat (14) / Save (15) / Buffer (16) / Exit (-1): ";
        cin >> userInput;

        if (userInput == -1)
        {
            state.test = TestSub::SelectParamByUser;
        }
        else if (userInput < 9)
        {
            cout << "Enter q[" << userInput << "] Values (Radian) : ";
            cin >> q[userInput];
        }
        else if (userInput == 9)
        {
            state.test = TestSub::FillBuf;
        }
        else if (userInput == 10)
        {
            cout << "time : ";
            cin >> t;
        }
        else if (userInput == 11)
        {
            float c_MotorAngle[9];
            getMotorPos(c_MotorAngle);

            cout << "[ Current Q Values (Ladian) ]\n";
            for (int i = 0; i < 9; i++)
            {
                
                cout << "Q[" << i << "] : " << q[i] << "\t " << "C_M[" << i << "] : " << c_MotorAngle[i] << "\n";
            }
            cin >> userInput;
        }
        else if (userInput == 12)
        {
            cout << "speed (0~32767) : ";
            cin >> speed_test;
        }
        else if (userInput == 13)
        {
            cout << "break start time (0~" << t << ") : ";
            cin >> break_start_time;

            cout << "break end time (" << t << "~" << 2*t << ") : ";
            cin >> break_end_time;
        }
        else if (userInput == 14)
        {
            if(repeat_flag ==1)
                repeat_flag=0;
            else
                repeat_flag =1;
   
        }
        else if (userInput == 15)
        {
            std::ostringstream fileNameOut;
            fileNameOut << std::fixed << std::setprecision(1); // 소숫점 1자리까지 표시
            fileNameOut << "../../READ/Test_0715";
            std::string fileName = fileNameOut.str();
            parse_and_save_to_csv(fileName);
        }
        else if (userInput == 16)
        {
            if(buffer_test_flag)
            {
                buffer_test_flag = false;
            }
            else
            {
                buffer_test_flag = true;
            }
        }
        
        break;
    }
    case TestSub::SetXYZ:
    {
        int userInput = 100;
        int ret = system("clear");
        if (ret == -1)
            std::cout << "system clear error" << endl;
        cout << "[ Current x, y, z (meter) ]\n";
        cout << "Right : ";
        for (int i = 0; i < 3; i++)
        {
            cout << R_xyz[i] << ", ";
        }
        cout << "\nLeft : ";
        for (int i = 0; i < 3; i++)
        {
            cout << L_xyz[i] << ", ";
        }

        cout << "\nSelect Motor to Change Value (1 - Right, 2 - Left) / Start Test (3) / Exit (-1) : ";
        cin >> userInput;

        if (userInput == -1)
        {
            state.test = TestSub::SelectParamByUser;
        }
        else if (userInput == 1)
        {
            cout << "Enter x, y, z Values (meter) : ";
            cin >> R_xyz[0] >> R_xyz[1] >> R_xyz[2];
        }
        else if (userInput == 2)
        {
            cout << "Enter x, y, z Values (meter) : ";
            cin >> L_xyz[0] >> L_xyz[1] >> L_xyz[2];
        }
        else if (userInput == 3)
        {
            state.test = TestSub::FillBuf;
        }
        break;
    }
    case TestSub::SetSingleTuneParm:
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

        char userInput = '0';
        int ret = system("clear");
        if (ret == -1)
            std::cout << "system clear error" << endl;

        std::cout << "< Current Position >\n";
        for (auto &entry : motors)
        {
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
            {
                entry.second->coordinatePos = (tMotor->currentPos + tMotor->homeOffset) * tMotor->cwDir;
                cout << tMotor->myName << " : " << entry.second->coordinatePos << "\n";
            }
        }
        std::cout << "\n------------------------------------------------------------------------------------------------------------\n";
        std::cout << "Selected Motor : " << selectedMotor << "\n"
                  << "Time : " << t << "[s]\n"
                  << "Cycles : " << cycles << "\n"
                  << "Amplitude : " << amp << "[Radian]\n";
        std::cout << "------------------------------------------------------------------------------------------------------------\n";

        std::cout << "\n[Commands]\n";
        std::cout << "[s] : Select Other Motor\n"
                  << "[t] : Time\t [c] : Cycles\n"
                  << "[a] : Amplitude\n"
                  << "[r] : run\t [e] : Exit\n";
        std::cout << "Enter Command : ";
        std::cin >> userInput;

        if (userInput == 's')
        {
            std::cout << "\nMotor List : \n";
            for (auto &motor_pair : motors)
            {
                std::cout << motor_pair.first << "\n";
            }
            std::cout << "\nEnter Desire Motor : ";
            std::cin >> selectedMotor;
        }
        else if (userInput == 't')
        { // 한 주기 도는데 걸리는 시간
            std::cout << "\nEnter Desire Time [s] : ";
            std::cin >> t;
        }
        else if (userInput == 'c')
        {
            std::cout << "\nEnter Desire Cycles : ";
            std::cin >> cycles;
        }
        else if (userInput == 'a')
        {
            std::cout << "\nEnter Desire Amplitude [Radian] : ";
            std::cin >> amp;
        }
        else if (userInput == 'r')
        {
            state.test = TestSub::FillBuf;
        }
        else if (userInput == 'e')
        {
            state.test = TestSub::SelectParamByUser;
        }
        break;
    }
    case TestSub::StickTest:
    {
        canManager.setSocketBlock();
        TestStickLoop();
        canManager.setSocketNonBlock();
        state.test = TestSub::SelectParamByUser;
        break;
    }
    case TestSub::SetServoTestParm:
    {
        // for (auto &motor_pair : motors)
        // {
        //     if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
        //     {
        //         tMotor->clearCommandBuffer();
        //         tMotor->clearReceiveBuffer();
        //     }
        //     else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        //     {
        //         maxonMotor->clearCommandBuffer();
        //         maxonMotor->clearReceiveBuffer();
        //     }
        //     Input_pos.clear();
        // }

        // char userInput = '0';
        // int ret = system("clear");
        // if (ret == -1)
        //     std::cout << "system clear error" << endl;

        // std::cout << "< Current Position >\n";
        // for (auto &entry : motors)
        // {
        //     if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        //     {
        //         tMotor->coordinatePos = (tMotor->currentPos + tMotor->homeOffset) * tMotor->cwDir;
        //         cout << tMotor->myName << " : " << entry.second->coordinatePos << "\n";
        //         if (tMotor->myName == selectedMotor_servo)
        //         {
        //             targetpos_coo = tMotor->coordinatePos;
        //         }
        //     }
        // }
        // std::cout << "\n------------------------------------------------------------------------------------------------------------\n";
        // std::cout << "Selected Motor : " << selectedMotor_servo << "\n"
        //           << "Time : " << time_servo << "[sec]\n"
        //           << "Target coordinate Pos : " << targetpos_coo << " [Radian]\n"
        //           << "Single Target Pos : " << targetpos_des << " [Radian]\n"
        //           << "Current Break Val : " << current_servo << "[A]\n";
        // std::cout << "------------------------------------------------------------------------------------------------------------\n";

        // std::cout << "\n[Commands]\n";
        // std::cout << "[s] : Select Other Motor\n"
        //           << "[t] : Set Time"
        //           << "[p] : Set Single Target Pos\t [d] : Set Break Current\n"
        //           << "[f] : Set Origin\t [g] : Current Break!!\t [h] : Dont Break\n"
        //           << "[r] : run\t [e] : Exit\n";
        // std::cout << "Enter Command : ";
        // std::cin >> userInput;

        // if (userInput == 's')
        // {
        //     std::cout << "\nMotor List : \n";
        //     for (auto &motor_pair : motors)
        //     {
        //         std::cout << motor_pair.first << "\n";
        //     }
        //     std::cout << "\nEnter Desire Motor : ";
        //     std::cin >> selectedMotor_servo;
        // }
        // else if (userInput == 't')
        // {
        //     std::cout << "\nEnter Desire Time [sec] : ";
        //     std::cin >> time_servo;
        // }
        // else if (userInput == 'p')
        // {
        //     std::cout << "\nEnter Desire Target Position [Degree] : ";
        //     std::cin >> targetpos_des;
        // }
        // else if (userInput == 'd')
        // {
        //     std::cout << "\nEnter Desire Current : ";
        //     std::cin >> current_servo;
        // }
        // else if (userInput == 'f')
        // {
        //     for (auto &entry : motors)
        //     {
        //         if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        //         {
        //             if (tMotor->myName == selectedMotor_servo)
        //             {
        //                 tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
        //                 canManager.sendMotorFrame(tMotor);
        //             }
        //         }
        //     }
        // }
        // else if (userInput == 'g')
        // {
        //     for (auto &entry : motors)
        //     {
        //         if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        //         {
        //             if (tMotor->myName == selectedMotor_servo)
        //             {
        //                 tservocmd.comm_can_set_cb(*tMotor, &tMotor->sendFrame, current_servo);
        //                 canManager.sendMotorFrame(tMotor);
        //             }
        //         }
        //     }
        // }
        // else if (userInput == 'h')
        // {
        //     for (auto &entry : motors)
        //     {
        //         if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        //         {
        //             if (tMotor->myName == selectedMotor_servo)
        //             {
        //                 tservocmd.comm_can_set_cb(*tMotor, &tMotor->sendFrame, 0);
        //                 canManager.sendMotorFrame(tMotor);
        //             }
        //         }
        //     }
        // }
        // else if (userInput == 'r')
        // {
        //     state.test = TestSub::FillBuf;
        // }
        // else if (userInput == 'e')
        // {
        //     state.test = TestSub::SelectParamByUser;
        // }
        // break;

        int userInput = 100;
        bool run_flag = false;

        // mode : 0(spd mode), 1(pos mode), 2(pos spd mode)
        static int mode = 1;
        static float test_spd = 0.0;
        static float test_pos = 0.0;

        float c_MotorAngle[9];
        getMotorPos(c_MotorAngle);

        int ret = system("clear");
        if (ret == -1)
            std::cout << "system clear error" << endl;

        cout << "[ Current Q Values (Radian) ]\n";
        for (int i = 0; i < 9; i++)
        {
            cout << "q[" << i << "] : " << c_MotorAngle[i] << "\n";
            q[i] = c_MotorAngle[i];
        }

        if (mode == 1)
        {
            cout << "speed mode" << "\n";
        }
        else if (mode == 2)
        {
            cout << "position mode" << "\n";
        }
        else if (mode == 3)
        {
            cout << "position speed mode" << "\n";
        }

        cout << "pos : " << test_pos << "rad\n";
        cout << "spd : " << test_spd << "erpm\n";

        cout << "\nSelect Mode (1) / Change Value (2) / Run(0) / Exit (-1): ";

        cin >> userInput;

        // if (!cin)
        // {
        //     cin.clear();     // 상태 플래그를 초기화
        //     cin.ignore(numeric_limits<streamsize>::max(), '\n');    // 버퍼를 비움
        //     cout << "cin error" << endl;
        //     std::cin >> userInput;
        // }

        if (userInput == -1)
        {
            state.test = TestSub::SelectParamByUser;
        }
        else if (userInput == 1)
        {
            cout << "\nspeed mode (1) / position mode (2) / position speed mode (3): ";
            cin >> mode;
        }
        else if (userInput == 2)
        {
            cout << "pos : ";
            cin >> test_pos;
            cout << "spd : ";
            cin >> test_spd;
        }
        else if (userInput == 0)
        {
            run_flag = true;
        }

        if (run_flag)
        {
            for (auto &motor_pair : motors)
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
                {
                    float test_pos2 = (q[motor_mapping[motor_pair.first]] + test_pos) * tMotor->cwDir - tMotor->homeOffset;
                    // test_spd = tMotor->spd;
                    float test_acl = tMotor->acl;
                    

                    if (mode == 1)
                    {
                        tservocmd.comm_can_set_spd(*tMotor, &tMotor->sendFrame, test_spd);
                    }
                    else if (mode == 2)
                    {
                        tservocmd.comm_can_set_pos(*tMotor, &tMotor->sendFrame, test_pos2);
                    }
                    else if (mode == 3)
                    {
                        tservocmd.comm_can_set_pos_spd(*tMotor, &tMotor->sendFrame, test_pos2, test_spd, test_acl);
                    }
                }
            }

            bool needSync = false;
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
                    if (useArduino)
                    {
                        char data_to_send; // 시리얼 포트로 전송할 문자
                        
                        if (tMotor->break_state)
                        {
                            data_to_send = '1';
                        }
                        else
                        {
                            data_to_send = '0';
                        }

                        canManager.send_char_to_serial(canManager.serial_fd, data_to_send);

                        usleep(10);

                        // 데이터 수신
                        std::string received_data = canManager.read_char_from_serial(canManager.serial_fd);
                        if (!received_data.empty()) {
                            std::cout << "Received data: " << received_data << std::endl;
                        }
                    }
                    else
                    {
                        sensor.writeVal(tMotor, tMotor->break_state);
                    }
                }

                if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
                {
                    virtualMaxonMotor = maxonMotor;
                    needSync = true;
                }
            }

            if (needSync)
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
        }
        
        break;

    }
    case TestSub::SetCommand:
    {

        int userInput = 100;
        bool run_flag = false;

        // mode : 0(spd mode), 1(pos mode), 2(pos spd mode)
        static int mode = 1;
        static float test_spd = 0.0;
        static float test_pos = 0.0;

        float c_MotorAngle[9];
        getMotorPos(c_MotorAngle);

        int ret = system("clear");
        if (ret == -1)
            std::cout << "system clear error" << endl;

        cout << "[ Current Q Values (Radian) ]\n";
        for (int i = 0; i < 9; i++)
        {
            cout << "q[" << i << "] : " << c_MotorAngle[i] << "\n";
            q[i] = c_MotorAngle[i];
        }

        if (mode == 1)
        {
            cout << "speed mode" << "\n";
        }
        else if (mode == 2)
        {
            cout << "position mode" << "\n";
        }
        else if (mode == 3)
        {
            cout << "position speed mode" << "\n";
        }

        cout << "pos : " << test_pos << "rad\n";
        cout << "spd : " << test_spd << "erpm\n";

        cout << "\nSelect Mode (1) / Change Value (2) / Run(0) / Exit (-1): ";

        cin >> userInput;

        // if (!cin)
        // {
        //     cin.clear();     // 상태 플래그를 초기화
        //     cin.ignore(numeric_limits<streamsize>::max(), '\n');    // 버퍼를 비움
        //     cout << "cin error" << endl;
        //     std::cin >> userInput;
        // }

        if (userInput == -1)
        {
            state.test = TestSub::SelectParamByUser;
        }
        else if (userInput == 1)
        {
            cout << "\nspeed mode (1) / position mode (2) / position speed mode (3): ";
            cin >> mode;
        }
        else if (userInput == 2)
        {
            cout << "pos : ";
            cin >> test_pos;
            cout << "spd : ";
            cin >> test_spd;
        }
        else if (userInput == 0)
        {
            run_flag = true;
        }

        if (run_flag)
        {
            for (auto &motor_pair : motors)
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
                {
                    float test_pos2 = (q[motor_mapping[motor_pair.first]] + test_pos) * tMotor->cwDir - tMotor->homeOffset;
                    // test_spd = tMotor->spd;
                    float test_acl = tMotor->acl;
                    

                    if (mode == 1)
                    {
                        tservocmd.comm_can_set_spd(*tMotor, &tMotor->sendFrame, test_spd);
                    }
                    else if (mode == 2)
                    {
                        tservocmd.comm_can_set_pos(*tMotor, &tMotor->sendFrame, test_pos2);
                    }
                    else if (mode == 3)
                    {
                        tservocmd.comm_can_set_pos_spd(*tMotor, &tMotor->sendFrame, test_pos2, test_spd, test_acl);
                    }
                }
            }
        }
        
        break;
    }
    case TestSub::FillBuf:
    {
        // Fill motors command Buffer
        if (method == 1)
        {
            GetArr(q);
        }
        else if (method == 2)
        {
            vector<float> Qf(7);
            Qf = ikfun_final(R_xyz, L_xyz, part_length, s, z0); // IK함수는 손목각도가 0일 때를 기준으로 풀림
            Qf.push_back(0.0);                                  // 오른쪽 손목 각도
            Qf.push_back(0.0);                                  // 왼쪽 손목 각도
            for (int i = 0; i < 9; i++)
            {
                q[i] = Qf[i];
                cout << Qf[i] << " ";
            }
            cout << "\n";
            sleep(1);
            GetArr(q);
        }
        else if (method == 3)
        {
            startTest(selectedMotor, t, cycles, amp);
        }
        else if (method == 6)
        {
            std::shared_ptr<TMotor> sMotor = std::dynamic_pointer_cast<TMotor>(motors[selectedMotor_servo]);
            vel = ((abs(targetpos_coo - targetpos_des) / M_PI * 180) / time_servo) * sMotor->R_Ratio[sMotor->motorType] * sMotor->PolePairs * 60 / 360;
            // vel = 327680;
            acl = 327670;
            startTest_servo(selectedMotor_servo, targetpos_des, vel, acl);
        }

        // break를 위한 통신 초기화
        if (useArduino)
        {
            canManager.serial_fd = canManager.setup_serial_port();
            if (canManager.serial_fd == -1) {
                cout << "Serial error";
                return;
            }
        }
        else
        {
            sensor.OpenDeviceUntilSuccess();
        }
        
        state.test = TestSub::CheckBuf;
        break;
    }
    case TestSub::CheckBuf:
    {
        bool allBuffersEmpty = true;
        canManager.appendToCSV_time("CHECKBUF.txt");
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
            state.test = TestSub::TimeCheck;
        else
            state.test = TestSub::Done;
        break;
    }
    case TestSub::TimeCheck:
    {
        usleep(5000);
        state.test = TestSub::SetCANFrame;
        break;
    }
    case TestSub::SetCANFrame:
    {
        canManager.setCANFrame();
        state.test = TestSub::SendCANFrame;
        break;
    }
    case TestSub::SendCANFrame:
    {
        bool needSync = false;
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
                if (useArduino)
                {
                    char data_to_send; // 시리얼 포트로 전송할 문자
                    
                    if (tMotor->break_state)
                    {
                        data_to_send = '1';
                    }
                    else
                    {
                        data_to_send = '0';
                    }

                    canManager.send_char_to_serial(canManager.serial_fd, data_to_send);

                    usleep(10);

                    // 데이터 수신
                    std::string received_data = canManager.read_char_from_serial(canManager.serial_fd);
                    if (!received_data.empty()) {
                        std::cout << "Received data: " << received_data << std::endl;
                    }
                }
                else
                {
                    sensor.writeVal(tMotor, tMotor->break_state);
                }
            }

            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
            {
                virtualMaxonMotor = maxonMotor;
                needSync = true;
            }
        }

        if (needSync)
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
            state.test = TestSub::CheckBuf;
        }

        break;
    }
    case TestSub::Done:
    {
        static int repeat_num = 0;
        usleep(5000);

        allBreakOff();
        
        if (method == 1)
        {
            if (buffer_test_flag && repeat_num == 0)
            {
                repeat_num ++;
                for (int i = 1; i < 9; i++)
                {
                    q[i] += 3.0;
                }
                state.test = TestSub::FillBuf;
            }
            else
            {
                repeat_num = 0;
                state.test = TestSub::SetQValue;
            }
            
            std::ostringstream fileNameOut;
            fileNameOut << std::fixed << std::setprecision(1); // 소숫점 1자리까지 표시
            fileNameOut << "../../READ/Test_0704_P" << q[5]
                        << "_spd" << speed_test
                        << "_BreakTime" << break_start_time;
            std::string fileName = fileNameOut.str();
            //parse_and_save_to_csv(fileName);
        }
        else if (method == 2)
        {
            state.test = TestSub::SetXYZ;
        }
        else if (method == 3)
        {
            std::ostringstream fileNameIn;
            fileNameIn << std::fixed << std::setprecision(1); // 소숫점 1자리까지 표시
            fileNameIn << "../../READ/" << selectedMotor
                       << "_Period" << t
                       << "_Kp" << kp
                       << "_Kd" << kd
                       << "_in";
            std::string fileName = fileNameIn.str();
            save_to_txt_inputData(fileName);

            std::ostringstream fileNameOut;
            fileNameOut << std::fixed << std::setprecision(1); // 소숫점 1자리까지 표시
            fileNameOut << "../../READ/" << selectedMotor
                        << "_Period" << t
                        << "_Kp" << kp
                        << "_Kd" << kd
                        << "_out";
            fileName = fileNameOut.str();
            parse_and_save_to_csv(fileName);

            state.test = TestSub::SetSingleTuneParm;
        }
        else if (method == 4)
        {
            // multiTestLoop(); State로 변경 시 state.test 값 변환으로 변경
        }
        else if (method == 6)
        {
            sleep(1);
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1); // 소숫점 1자리까지 표시
            oss << "../../READ/" << selectedMotor_servo
                << "_T" << time_servo
                << "_P" << abs(targetpos_coo - targetpos_des) / M_PI * 180
                << "_V" << vel
                << "_A" << acl << "_in";
            std::string fileName = oss.str();
            save_to_txt_inputData(fileName);

            oss.str("");

            oss << "../../READ/" << selectedMotor_servo
                << "_T" << time_servo
                << "_P" << abs(targetpos_coo - targetpos_des) / M_PI * 180
                << "_V" << vel
                << "_A" << acl << "_out";
            fileName = oss.str();
            parse_and_save_to_csv(fileName);

            state.test = TestSub::SetServoTestParm;
        }
        else
            state.test = TestSub::SelectParamByUser;

        if (error)
            state.main = Main::Error;
        break;
    }
    }
}

void TestManager::MaxonEnable()
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
};

void TestManager::setMaxonMode(std::string targetMode)
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

void TestManager::fkfun(float theta[])
{
    vector<float> P;

    float r1 = part_length[0], r2 = part_length[1], l1 = part_length[2], l2 = part_length[3], stick = part_length[4];

    P.push_back(0.5 * s * cos(theta[0]) + r1 * sin(theta[3]) * cos(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * cos(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * cos(theta[0] + theta[1]));
    P.push_back(0.5 * s * sin(theta[0]) + r1 * sin(theta[3]) * sin(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * sin(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * sin(theta[0] + theta[1]));
    P.push_back(z0 - r1 * cos(theta[3]) - r2 * cos(theta[3] + theta[4]) - stick * cos(theta[3] + theta[4] + theta[7]));
    P.push_back(-0.5 * s * cos(theta[0]) + l1 * sin(theta[5]) * cos(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * cos(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * cos(theta[0] + theta[2]));
    P.push_back(-0.5 * s * sin(theta[0]) + l1 * sin(theta[5]) * sin(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * sin(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * sin(theta[0] + theta[2]));
    P.push_back(z0 - l1 * cos(theta[5]) - l2 * cos(theta[5] + theta[6]) - stick * cos(theta[5] + theta[6] + theta[8]));

    std::cout << "\nRight Hand Position : { " << P[0] << " , " << P[1] << " , " << P[2] << " }\n";
    std::cout << "Left Hand Position : { " << P[3] << " , " << P[4] << " , " << P[5] << " }\n";
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 Values Test Mode                           */
///////////////////////////////////////////////////////////////////////////////

// 여기1
void TestManager::getMotorPos(float c_MotorAngle[])
{
    // 각 모터의 현재위치 값 불러오기 ** CheckMotorPosition 이후에 해야함(변수값을 불러오기만 해서 갱신 필요)
    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            c_MotorAngle[motor_mapping[entry.first]] = (tMotor->currentPos + tMotor->homeOffset) * tMotor->cwDir;
        }
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            c_MotorAngle[motor_mapping[entry.first]] = maxonMotor->currentPos * maxonMotor->cwDir;
        }
    }
}

vector<float> TestManager::connect(float Q1[], float Q2[], int k, int n)
{
    vector<float> Qi;
    std::vector<float> A, B;

    // Compute A and Bk
    for (long unsigned int i = 0; i < 9; ++i)
    {
        A.push_back(0.5 * (Q1[i] - Q2[i]));
        B.push_back(0.5 * (Q1[i] + Q2[i]));
    }

    // Compute Qi using the provided formula
    for (long unsigned int i = 0; i < 9; ++i)
    {
        float val = A[i] * cos(M_PI * k / n) + B[i];
        Qi.push_back(val);
    }

    return Qi;
}

void TestManager::GetArr(float arr[])
{
    cout << "Get Array...\n";

    vector<float> Qi;
    vector<vector<float>> q_setting;
    float c_MotorAngle[9];

    getMotorPos(c_MotorAngle);

    //int n = 800; // 4초동안 실행
    int n = (int)(1000*t/5);    // t초동안 실행
    int n_break = (int)(1000*break_start_time/5);
    int n_break_end = (int)(1000*break_end_time/5) - n;
    int nn = 1;
    
    for(int i = 0; i < nn; i++)
    {
        for (int k = 0; k < n; ++k)
        {
            // Make GetBack Array
            Qi = connect(c_MotorAngle, arr, k, n);
            q_setting.push_back(Qi);

            // Send to Buffer
            for (auto &entry : motors)
            {
                if (std::shared_ptr<TMotor> tmotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                {
                    TMotorData newData;
                    newData.position = arr[motor_mapping[entry.first]] * tmotor->cwDir - tmotor->homeOffset;
                    //newData.spd = tmotor->spd;
                    //newData.acl = tmotor->acl;
                    newData.spd = speed_test;
                    newData.acl = 32767;
                    if (k < n_break)
                    {
                        newData.isBreak = false;
                    }
                    else
                    {
                        newData.isBreak = true;
                    }
                    tmotor->commandBuffer.push(newData);
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
                {
                    MaxonData newData;
                    newData.position = Qi[motor_mapping[entry.first]] * maxonMotor->cwDir;
                    newData.WristState = 0.5;
                    maxonMotor->commandBuffer.push(newData);
                }
            }
        }
        if(repeat_flag ==1)
        {
            for (int k = 0; k < n; ++k)
            {
                // Make GetBack Array
                Qi = connect(arr, c_MotorAngle, k, n);
                q_setting.push_back(Qi);

                // Send to Buffer
                for (auto &entry : motors)
                {
                    if (std::shared_ptr<TMotor> tmotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                    {
                        TMotorData newData;
                        newData.position = c_MotorAngle[motor_mapping[entry.first]] * tmotor->cwDir - tmotor->homeOffset;
                        //newData.spd = tmotor->spd;
                        //newData.acl = tmotor->acl;
                        newData.spd = speed_test;
                        newData.acl = 32767;
                        if (k < n_break_end)
                        {
                            newData.isBreak = true;
                        }
                        else
                        {
                            newData.isBreak = false;
                        }
                        tmotor->commandBuffer.push(newData);
                    }
                    else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
                    {
                        MaxonData newData;
                        newData.position = Qi[motor_mapping[entry.first]] * maxonMotor->cwDir;
                        newData.WristState = 0.5;
                        maxonMotor->commandBuffer.push(newData);
                    }
                }
            }
        }
    }
}

vector<float> TestManager::ikfun_final(float pR[], float pL[], float part_length[], float s, float z0)
{
    float direction = 0.0 * M_PI;

    float X1 = pR[0], Y1 = pR[1], z1 = pR[2];
    float X2 = pL[0], Y2 = pL[1], z2 = pL[2];
    float r1 = part_length[0];
    float r2 = part_length[1] + part_length[4];
    float L1 = part_length[2];
    float L2 = part_length[3] + part_length[5];

    int j = 0;
    float the3[1351];
    float zeta = z0 - z2;
    vector<float> Qf(7);
    float the0_f = 0;

    // the3 배열 초기화
    for (int i = 0; i < 1351; ++i)
        the3[i] = (-M_PI * 0.25) + (i / 1350.0 * M_PI * 0.75); // the3 범위 : -45deg ~ 90deg

    for (int i = 0; i < 1351; ++i)
    {
        float det_the4 = (z0 - z1 - r1 * cos(the3[i])) / r2;

        if (det_the4 < 1 && det_the4 > -1)
        {
            float the34 = acos((z0 - z1 - r1 * cos(the3[i])) / r2);
            float the4 = the34 - the3[i];

            if (the4 >= 0 && the4 < (M_PI * 0.75)) // the4 범위 : 0deg ~ 135deg
            {
                float r = r1 * sin(the3[i]) + r2 * sin(the34);
                float det_the1 = (X1 * X1 + Y1 * Y1 - r * r - s * s / 4.0) / (s * r);

                if (det_the1 < 1 && det_the1 > -1)
                {
                    float the1 = acos(det_the1);
                    if (the1 > 0 && the1 < (M_PI * 0.8)) // the1 범위 : 0deg ~ 144deg
                    {
                        float alpha = asin(X1 / sqrt(X1 * X1 + Y1 * Y1));
                        float det_the0 = (s / 4.0 + (X1 * X1 + Y1 * Y1 - r * r) / s) / sqrt(X1 * X1 + Y1 * Y1);

                        if (det_the0 < 1 && det_the0 > -1)
                        {
                            float the0 = asin(det_the0) - alpha;
                            if (the0 > (-M_PI / 2) && the0 < (M_PI / 2)) // the0 범위 : -90deg ~ 90deg
                            {
                                float L = sqrt((X2 - 0.5 * s * cos(the0 + M_PI)) * (X2 - 0.5 * s * cos(the0 + M_PI)) + (Y2 - 0.5 * s * sin(the0 + M_PI)) * (Y2 - 0.5 * s * sin(the0 + M_PI)));
                                float det_the2 = (X2 - 0.5 * s * cos(the0 + M_PI)) / L;

                                if (det_the2 < 1 && det_the2 > -1)
                                {
                                    float the2 = acos(det_the2) - the0;
                                    if (the2 > (M_PI * 0.2) && the2 < M_PI) // the2 범위 : 36deg ~ 180deg
                                    {
                                        float Lp = sqrt(L * L + zeta * zeta);
                                        float det_the6 = (Lp * Lp - L1 * L1 - L2 * L2) / (2 * L1 * L2);

                                        if (det_the6 < 1 && det_the6 > -1)
                                        {
                                            float the6 = acos(det_the6);
                                            if (the6 >= 0 && the6 < (M_PI * 0.75)) // the6 범위 : 0deg ~ 135deg
                                            {
                                                float T = (zeta * zeta + L * L + L1 * L1 - L2 * L2) / (L1 * 2);
                                                float det_the5 = L * L + zeta * zeta - T * T;

                                                if (det_the5 > 0)
                                                {
                                                    float sol = T * L - zeta * sqrt(L * L + zeta * zeta - T * T);
                                                    sol /= (L * L + zeta * zeta);
                                                    float the5 = asin(sol);
                                                    if (the5 > (-M_PI * 0.25) && the5 < (M_PI / 2)) // the5 범위 : -45deg ~ 90deg
                                                    {
                                                        if (j == 0 || fabs(the0 - direction) < fabs(the0_f - direction))
                                                        {
                                                            Qf[0] = the0;
                                                            Qf[1] = the1;
                                                            Qf[2] = the2;
                                                            Qf[3] = the3[i];
                                                            Qf[4] = the4;
                                                            Qf[5] = the5;
                                                            Qf[6] = the6;

                                                            the0_f = the0;
                                                            j = 1;
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (j == 0)
    {
        cout << "IKFUN is not solved!!\n";
        state.main = Main::Error;
    }

    return Qf;
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 Single Test Mode                           */
///////////////////////////////////////////////////////////////////////////////

void TestManager::singleTestLoop()
{
    char userInput = '0';
    int result = system("clear");
    if (result != 0)
    {
        cerr << "Error during clear screen" << std::endl;
    }

    std::cout << "< Current Position >\n";
    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            entry.second->coordinatePos = (tMotor->currentPos + tMotor->homeOffset) * tMotor->cwDir;
            cout << tMotor->myName << " : " << entry.second->coordinatePos << "\n";
        }
    }
    std::cout << "\n------------------------------------------------------------------------------------------------------------\n";
    std::cout << "Selected Motor : " << selectedMotor << "\n"
              << "Time : " << t << "\n"
              << "Cycles : " << cycles << "\n"
              << "Amplitude : " << amp << "[radian]\n"
              << "Kp : " << kp << ",\tKd : " << kd << "\n"
              << "Kp for Fixed : " << Kp_for_Fixed << ",\tKd for Fixed : " << Kd_for_Fixed << "\n";
    std::cout << "------------------------------------------------------------------------------------------------------------\n";

    std::cout << "\n[Commands]\n";
    std::cout << "[s] : Select Other Motor\n"
              << "[t] : Time\t [c] : Cycles\n"
              << "[a] : Amplitude\t [p] : Kp\t [d] : Kd\n"
              << "[f] : Change Fixed Parameter\n"
              << "[r] : run\t [e] : Exit\n";
    std::cout << "Enter Command : ";
    std::cin >> userInput;

    if (userInput == 's')
    {
        std::cout << "\nMotor List : \n";
        for (auto &motor_pair : motors)
        {
            std::cout << motor_pair.first << "\n";
        }
        std::cout << "\nEnter Desire Motor : ";
        std::cin >> selectedMotor;
    }
    else if (userInput == 't')
    {
        std::cout << "\nEnter Desire Time : ";
        std::cin >> t;
    }
    else if (userInput == 'c')
    {
        std::cout << "\nEnter Desire Cycles : ";
        std::cin >> cycles;
    }
    else if (userInput == 'a')
    {
        std::cout << "\nEnter Desire Amplitude : ";
        std::cin >> amp;
    }
    else if (userInput == 'p')
    {
        std::cout << "\nEnter Desire Kp : ";
        std::cin >> kp;
    }
    else if (userInput == 'd')
    {
        std::cout << "\nEnter Desire Kd : ";
        std::cin >> kd;
    }
    else if (userInput == 'f')
    {
        std::cout << "\nEnter Desire Kp for Fixed : ";
        std::cin >> Kp_for_Fixed;
        std::cout << "\nEnter Desire Kd for Fixed : ";
        std::cin >> Kd_for_Fixed;
    }
    else if (userInput == 'r')
    {
        state.test = TestSub::FillBuf;
    }
    else if (userInput == 'e')
    {
        state.test = TestSub::SelectParamByUser;
    }
}

void TestManager::startTest(string selectedMotor, float t, int cycles, float amp)
{
    std::cout << "Test Start!!\n";
    vector<float> Pos(9);

    float dt = 0.005;
    int time = (t / 2.0) / dt;
    for (auto &motor_pair : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
        {
            if (tMotor->myName == selectedMotor)
            {
                tMotor->isfixed = false;
                vel = ((amp / M_PI * 180) / (t / 2)) * tMotor->R_Ratio[tMotor->motorType] * tMotor->PolePairs * 60 / 360;
            }
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            if (maxonMotor->myName == selectedMotor)
            {
                maxonMotor->isfixed = false;
            }
        }
    }

    for (int c = 0; c < cycles; c++)
    {
        for (int i = 0; i < time; i++)
        {
            for (const auto &motor_pair : motors)
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
                {
                    TMotorData newData;
                    if (tMotor->myName == selectedMotor)
                    {
                        newData.position = tMotor->currentPos + amp;
                        newData.spd = vel * 1.5;
                        newData.acl = tMotor->acl;
                        newData.isBreak = false;
                        tMotor->commandBuffer.push(newData);
                    }
                    else
                    {
                        if (tMotor->isfixed == false)
                        {
                            tMotor->fixedPos = tMotor->currentPos;
                            tMotor->isfixed = true;
                        }
                        newData.position = tMotor->fixedPos;
                        newData.spd = tMotor->spd;
                        newData.acl = tMotor->acl;
                        newData.isBreak = false;
                        tMotor->commandBuffer.push(newData);
                    }
                }
                if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
                {
                    MaxonData newData;
                    newData.position = maxonMotor->currentPos;
                    newData.WristState = 0.0;
                    maxonMotor->commandBuffer.push(newData);
                }
            }
            Input_pos.push_back(Pos);
        }
        for (int i = 0; i < time; i++)
        {
            for (const auto &motor_pair : motors)
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
                {
                    TMotorData newData;
                    if (tMotor->myName == selectedMotor)
                    {
                        newData.position = tMotor->currentPos;
                        newData.spd = vel * 1.5;
                        newData.acl = tMotor->acl;
                        newData.isBreak = false;
                        tMotor->commandBuffer.push(newData);
                    }
                    else
                    {
                        if (tMotor->isfixed == false)
                        {
                            tMotor->fixedPos = tMotor->currentPos;
                            tMotor->isfixed = true;
                        }
                        newData.position = tMotor->fixedPos;
                        newData.spd = tMotor->spd;
                        newData.acl = tMotor->acl;
                        newData.isBreak = false;
                        tMotor->commandBuffer.push(newData);
                    }
                }
                if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
                {
                    MaxonData newData;
                    newData.position = maxonMotor->currentPos;
                    newData.WristState = 0.0;
                    maxonMotor->commandBuffer.push(newData);
                }
            }
            Input_pos.push_back(Pos);
        }
    }
}

void TestManager::save_to_txt_inputData(const string &csv_file_name)
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

    std::cout << "Tunning Input Data (pos / vel) 파일이 생성되었습니다 : " << csv_file_name << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 Multi Test Mode                           */
///////////////////////////////////////////////////////////////////////////////

void TestManager::mkArr(vector<string> &motorName, int time, int cycles, int LnR, float amp)
{
    struct can_frame frame;
    int Kp_fixed = 450;
    float Kd_fixed = 4.5;
    map<string, bool> TestMotor;
    if (LnR == 0) // 양쪽 다 고정
    {
        for (auto &motorname : motorName)
        {
            TestMotor[motorname] = false;
        }
    }
    else if (LnR == 1) // 오른쪽만 Test
    {
        for (auto &motorname : motorName)
        {
            if (motorname[0] == 'L')
                TestMotor[motorname] = false;
            else
                TestMotor[motorname] = true;
        }
    }
    else if (LnR == 2) // 왼쪽만 Test
    {
        for (auto &motorname : motorName)
        {
            if (motorname[0] == 'R')
                TestMotor[motorname] = false;
            else
                TestMotor[motorname] = true;
        }
    }
    else if (LnR == 3) // 양쪽 다 Test
    {
        for (auto &motorname : motorName)
        {
            TestMotor[motorname] = true;
        }
    }

    amp = amp / 180.0 * M_PI; // Degree -> Radian 변경
    for (const auto &motorname : motorName)
    {
        if (motors.find(motorname) != motors.end())
        {
            std::cout << motorname << " ";
            if (TestMotor[motorname])
            { // Test 하는 모터
                std::cout << "Move\n";
                InputData[0] += motorname + ",";
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motors[motorname]))
                {
                    int kp = tMotor->Kp;
                    float kd = tMotor->Kd;

                    for (int c = 0; c < cycles; c++)
                    {
                        for (int i = 0; i < time; i++)
                        {
                            float val = tMotor->currentPos + (1.0 - cos(2.0 * M_PI * i / time)) / 2 * amp * tMotor->cwDir;
                            tmotorcmd.parseSendCommand(*tMotor, &frame, tMotor->nodeId, 8, val, 0, kp, kd, 0.0);
                            tMotor->sendBuffer.push(frame);
                            InputData[time * c + i + 1] += to_string(val) + ",";
                        }
                    }
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors[motorname]))
                {
                    for (int c = 0; c < cycles; c++)
                    {
                        for (int i = 0; i < time; i++)
                        {
                            float val = maxonMotor->currentPos + (1.0 - cos(2.0 * M_PI * i / time)) / 2 * amp * maxonMotor->cwDir;
                            maxoncmd.getTargetPosition(*maxonMotor, &frame, val);
                            maxonMotor->sendBuffer.push(frame);
                            InputData[time * c + i + 1] += to_string(val) + ",";
                        }
                    }
                }
            }
            else
            { // Fixed 하는 모터
                std::cout << "Fixed\n";
                InputData[0] += motorname + ",";
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motors[motorname]))
                {
                    for (int c = 0; c < cycles; c++)
                    {
                        for (int i = 0; i < time; i++)
                        {
                            float val = tMotor->currentPos;
                            tmotorcmd.parseSendCommand(*tMotor, &frame, tMotor->nodeId, 8, val, 0, Kp_fixed, Kd_fixed, 0.0);
                            tMotor->sendBuffer.push(frame);
                            InputData[time * c + i + 1] += to_string(val) + ",";
                        }
                    }
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors[motorname]))
                {
                    for (int c = 0; c < cycles; c++)
                    {
                        for (int i = 0; i < time; i++)
                        {
                            float val = maxonMotor->currentPos;
                            maxoncmd.getTargetPosition(*maxonMotor, &frame, val);
                            maxonMotor->sendBuffer.push(frame);
                            InputData[time * c + i + 1] += to_string(val) + ",";
                        }
                    }
                }
            }
        }
    }
}

void TestManager::SendLoop()
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

                // canManager.readFramesFromAllSockets();
                // canManager.distributeFramesToMotors();
            }
        }
    } while (!allBuffersEmpty);
    canManager.clearReadBuffers();
}

void TestManager::multiTestLoop()
{
    string userInput;
    vector<float> c_deg;
    float t = 4.0;
    int cycles = 1;
    int type = 0b00001;
    int LnR = 1;
    float amplitude[5] = {30.0, 30.0, 30.0, 30.0, 30.0};

    while (state.main == Main::Test)
    {
        int result = system("clear");
        if (result != 0)
        {
            cerr << "Error during clear screen" << std::endl;
        }

        string typeDescription;
        if ((type | 0b11110) == 0b11111)
        {
            typeDescription += "Arm3 Turn, ";
        }
        if ((type | 0b11101) == 0b11111)
        {
            typeDescription += "Arm2 Turn, ";
        }
        if ((type | 0b11011) == 0b11111)
        {
            typeDescription += "Arm1 Turn, ";
        }
        if ((type | 0b10111) == 0b11111)
        {
            typeDescription += "Waist Turn, ";
        }
        if ((type | 0b01111) == 0b11111)
        {
            typeDescription += "Wrist Turn, ";
        }

        std::string LeftAndRight;
        if (LnR == 1)
        {
            LeftAndRight = "<< Right move >>\n";
        }
        else if (LnR == 2)
        {
            LeftAndRight = "<< Left move >>\n";
        }
        else if (LnR == 3)
        {
            LeftAndRight = "<< Left and Right move >>\n";
        }

        std::cout << "------------------------------------------------------------------------------------------------------------\n";
        std::cout << "< Current Position >\n";
        for (auto &motor : motors)
        {
            c_deg.push_back(motor.second->currentPos * motor.second->cwDir / M_PI * 180);
            std::cout << motor.first << " : " << motor.second->currentPos * motor.second->cwDir / M_PI * 180 << "deg\n";
        }
        std::cout << "\n"
                  << LeftAndRight;
        std::cout << "Type : " << typeDescription << "\n";
        std::cout << "Period : " << t << "\n";
        std::cout << "Cycles : " << cycles << "\n";
        std::cout << "\nAmplitude :\n";
        std::cout << "1) Arm3 = " << amplitude[0] << "\n";
        std::cout << "2) Arm2 = " << amplitude[1] << "\n";
        std::cout << "3) Arm1 = " << amplitude[2] << "\n";
        std::cout << "4) Waist = " << amplitude[3] << "\n";
        std::cout << "5) Wrist = " << amplitude[4] << "\n";
        std::cout << "------------------------------------------------------------------------------------------------------------\n";

        std::cout << "[Commands]\n";
        std::cout << "[d] : Left and Right | [t] : Type | [p] : Period | [c] : Cycles\n"
                  << "[a] : Amplitude | [kp] : Kp | [kd] : Kd | [m] : move | [r] : run | [e] : Exit\n";
        std::cout << "Enter Command: ";
        std::cin >> userInput;

        if (userInput[0] == 'e')
        {
            state.main = Main::Ideal;
            break;
        }
        else if (userInput[0] == 'd')
        {
            std::cout << "\n[Enter Desired Direction]\n";
            std::cout << "1: Right Move\n";
            std::cout << "2: Left Move\n";
            std::cout << "3: Left and Right Move\n";
            std::cout << "\nEnter Path Type (1 or 2 or 3): ";
            std::cin >> LnR;
        }
        else if (userInput[0] == 't')
        {
            int num;
            std::cout << "\n[Enter Desired Type]\n";
            std::cout << "1: Arm3 Turn ON/OFF\n";
            std::cout << "2: Arm2 Turn ON/OFF\n";
            std::cout << "3: Arm1 Turn ON/OFF\n";
            std::cout << "4: Waist Turn ON/OFF\n";
            std::cout << "5: Wrist Turn ON/OFF\n";
            std::cout << "\nEnter Path Type (1 or 2 or 3 or 4 or 5): ";
            std::cin >> num;

            if (num == 1)
            {
                type = type ^ 0b00001;
            }
            else if (num == 2)
            {
                type = type ^ 0b00010;
            }
            else if (num == 3)
            {
                type = type ^ 0b00100;
            }
            else if (num == 4)
            {
                type = type ^ 0b01000;
            }
            else if (num == 5)
            {
                type = type ^ 0b10000;
            }
        }
        else if (userInput[0] == 'p')
        {
            std::cout << "\nEnter Desired Period : ";
            std::cin >> t;
        }
        else if (userInput[0] == 'c')
        {
            std::cout << "\nEnter Desired Cycles : ";
            std::cin >> cycles;
        }
        else if (userInput[0] == 'a')
        {
            int input;
            std::cout << "\n[Select Motor]\n";
            std::cout << "1: Arm3\n";
            std::cout << "2: Arm2\n";
            std::cout << "3: Arm1\n";
            std::cout << "4: Waist\n";
            std::cout << "5: Wrist\n";
            std::cout << "\nEnter Desired Motor : ";
            std::cin >> input;

            std::cout << "\nEnter Desired Amplitude(degree) : ";
            std::cin >> amplitude[input - 1];
        }
        else if (userInput == "kp")
        {
            char input;
            int kp;
            std::cout << "\n[Select Motor]\n";
            std::cout << "1: Arm3\n";
            std::cout << "2: Arm2\n";
            std::cout << "3: Arm1\n";
            std::cout << "4: Waist\n";
            std::cout << "\nEnter Desired Motor : ";
            std::cin >> input;

            if (input == '1')
            {
                std::cout << "Arm3's Kp : " << motors["R_arm3"]->Kp << "\n";
                std::cout << "Enter Arm3's Desired Kp : ";
                std::cin >> kp;
                motors["R_arm3"]->Kp = kp;
                // motors["L_arm3"]->Kp = kp;
            }
            else if (input == '2')
            {
                std::cout << "Arm2's Kp : " << motors["R_arm2"]->Kp << "\n";
                std::cout << "Enter Arm2's Desired Kp : ";
                std::cin >> kp;
                motors["R_arm2"]->Kp = kp;
                motors["L_arm2"]->Kp = kp;
            }
            else if (input == '3')
            {
                std::cout << "Arm1's Kp : " << motors["R_arm1"]->Kp << "\n";
                std::cout << "Enter Arm1's Desired Kp : ";
                std::cin >> kp;
                motors["R_arm1"]->Kp = kp;
                motors["L_arm1"]->Kp = kp;
            }
            else if (input == '4')
            {
                std::cout << "Waist's Kp : " << motors["waist"]->Kp << "\n";
                std::cout << "Enter Waist's Desired Kp : ";
                std::cin >> kp;
                motors["waist"]->Kp = kp;
            }
        }
        else if (userInput == "kd")
        {
            char input;
            int kd;
            std::cout << "\n[Select Motor]\n";
            std::cout << "1: Arm3\n";
            std::cout << "2: Arm2\n";
            std::cout << "3: Arm1\n";
            std::cout << "4: Waist\n";
            std::cout << "\nEnter Desired Motor : ";
            std::cin >> input;

            if (input == '1')
            {
                std::cout << "Arm3's Kd : " << motors["R_arm3"]->Kd << "\n";
                std::cout << "Enter Arm3's Desired Kd : ";
                std::cin >> kd;
                motors["R_arm3"]->Kd = kd;
                // motors["L_arm3"]->Kd = kd;
            }
            else if (input == '2')
            {
                std::cout << "Arm2's Kd : " << motors["R_arm2"]->Kd << "\n";
                std::cout << "Enter Arm2's Desired Kd : ";
                std::cin >> kd;
                motors["R_arm2"]->Kd = kd;
                motors["L_arm2"]->Kd = kd;
            }
            else if (input == '3')
            {
                std::cout << "Arm1's Kd : " << motors["R_arm1"]->Kd << "\n";
                std::cout << "Enter Arm1's Desired Kd : ";
                std::cin >> kd;
                motors["R_arm1"]->Kd = kd;
                motors["L_arm1"]->Kd = kd;
            }
            else if (input == '4')
            {
                std::cout << "Waist's Kd : " << motors["waist"]->Kd << "\n";
                std::cout << "Enter Waist's Desired Kd : ";
                std::cin >> kd;
                motors["waist"]->Kd = kd;
            }
        } /*
         else if (userInput[0] == 'm')
         {
             while (true)
             {
                 string input;
                 float deg;
                 cout << "\n[Move to]\n";
                 int i = 0;
                 for (auto &motor : motors)
                 {
                     cout << i + 1 << " - " << motor.first << " : " << c_deg[i] << "deg\n";
                     i++;
                 }
                 cout << "m - Move\n";
                 cout << "e - Exit\n";
                 cout << "\nEnter Desired Option : ";
                 cin >> input;

                 if (input[0] == 'e')
                 {
                     break;
                 }
                 else if (input[0] == 'm')
                 {
                     // 움직이는 함수 작성
                     // 나중에 이동할 위치 값 : c_deg * motor.second->cwDir / 180 * M_PI
                     break;
                 }
                 else
                 {
                     cout << "\nEnter Desired Degree : ";
                     cin >> deg;
                     c_deg[stoi(input) - 1] = deg;
                 }
             }
         }*/
        else if (userInput[0] == 'r')
        {
            TestArr(t, cycles, type, LnR, amplitude);
        }
    }
}

void TestManager::TestArr(float t, int cycles, int type, int LnR, float amp[])
{
    std::cout << "Test Start!!\n";

    int time = t / 0.005;
    std::vector<std::string> SmotorName;
    InputData.clear();
    InputData.resize(time * cycles + 1);

    SmotorName = {"waist"};
    if ((type | 0b10111) == 0b11111) // Turn Waist
        mkArr(SmotorName, time, cycles, LnR, amp[3]);
    else
        mkArr(SmotorName, time, cycles, 0, 0);

    SmotorName = {"R_arm1", "L_arm1"};
    if ((type | 0b11011) == 0b11111) // Turn Arm1
        mkArr(SmotorName, time, cycles, LnR, amp[2]);
    else
        mkArr(SmotorName, time, cycles, 0, 0);

    SmotorName = {"R_arm2", "L_arm2"};
    if ((type | 0b11101) == 0b11111) // Turn Arm2
        mkArr(SmotorName, time, cycles, LnR, amp[1]);
    else
        mkArr(SmotorName, time, cycles, 0, 0);

    SmotorName = {"R_arm3", "L_arm3"};
    if ((type | 0b11110) == 0b11111) // Turn Arm3
        mkArr(SmotorName, time, cycles, LnR, amp[0]);
    else
        mkArr(SmotorName, time, cycles, 0, 0);

    SmotorName = {"R_wrist", "L_wrist", "maxonForTest"};
    if ((type | 0b01111) == 0b11111) // Turn Wrist
        mkArr(SmotorName, time, cycles, LnR, amp[4]);
    else
        mkArr(SmotorName, time, cycles, 0, 0);

    // TXT 파일 열기
    string FileNamein = "../../READ/test_in.txt";
    ofstream csvFileIn(FileNamein);
    if (!csvFileIn.is_open())
    {
        std::cerr << "Error opening TXT file." << std::endl;
    }

    // TXT 파일 입력
    for (auto &data : InputData)
    {
        csvFileIn << data << "\n";
    }

    // CSV 파일 닫기
    csvFileIn.close();
    std::cout << "연주 txt_InData 파일이 생성되었습니다: " << FileNamein << std::endl;

    SendLoop();

    parse_and_save_to_csv("../../READ/test_out");
}

void TestManager::parse_and_save_to_csv(const std::string &csv_file_name)
{
    // CSV 파일 열기. 파일이 있으면 지우고 새로 생성됩니다.
    std::ofstream ofs(csv_file_name + ".txt");
    if (!ofs.is_open())
    {
        std::cerr << "Failed to open or create the CSV file: " << csv_file_name << std::endl;
        return;
    }

    // CSV 헤더 추가
    ofs << "CAN_ID,p_act,v_act,c_act,a_act\n";

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
                float position, speed, torque, acceleration;

                // TMotor 또는 MaxonMotor에 따른 데이터 파싱 및 출력
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
                {
                    std::tuple<int, float, float, float, int8_t, int8_t> parsedData = tservocmd.motor_receive(&frame);
                    position = std::get<1>(parsedData);
                    speed = std::get<2>(parsedData);
                    torque = std::get<3>(parsedData);
                    acceleration = (speed - motor->pre_spd) / 0.002;
                    motor->pre_spd = speed;
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
                {
                    std::tuple<int, float, float, int8_t> parsedData = maxoncmd.parseRecieveCommand(*maxonMotor, &frame);
                    position = std::get<1>(parsedData);
                    torque = std::get<2>(parsedData);
                    speed = 0.0;
                }

                // 데이터 CSV 파일에 쓰기
                ofs << "0x" << std::hex << std::setw(2) << std::setfill('0') << id << ","
                    << std::dec << position << "," << speed << "," << torque << "," << acceleration << "\n";
            }
        }

        if (allRecvBufferEmpty)
            break;
    }

    // 각 모터에 대한 처리

    ofs.close();
    std::cout << "Tunning Output Data 파일이 생성되었습니다 : " << csv_file_name << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 Stick Test Mode                           */
///////////////////////////////////////////////////////////////////////////////

void TestManager::TestStickLoop()
{

    std::string userInput;
    std::string selectedMotor = "maxonForTest";
    float des_tff = 0;
    float posThreshold = 1.57; // 위치 임계값 초기화
    float tffThreshold = 18;   // 토크 임계값 초기화
    int backTorqueUnit = 150;

    while (true)
    {

        int result = system("clear");
        if (result != 0)
        {
            std::cerr << "Error during clear screen" << std::endl;
        }

        std::cout << "================ Tuning Menu ================\n";
        std::cout << "Available Motors for Stick Mode:\n";
        for (const auto &motor_pair : motors)
        {
            if (motor_pair.first == "maxonForTest")
                std::cout << " - " << motor_pair.first << "\n";
        }

        bool isMaxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors[selectedMotor]) != nullptr;
        if (!isMaxonMotor)
            break;

        std::cout << "---------------------------------------------\n";
        std::cout << "Selected Motor: " << selectedMotor << "\n";

        std::cout << "Des Torque: " << des_tff * 31.052 / 1000 << "[mNm]\n";
        std::cout << "Torque Threshold: " << tffThreshold << " [mNm]\n"; // 현재 토크 임계값 출력
        std::cout << "Position Threshold: " << posThreshold << " [rad]\n";
        std::cout << "Back Torque: " << backTorqueUnit * 31.052 / 1000 << " [mNm]\n";
        std::cout << "drumHitDuration: " << drumHitDuration << " [ms]\n";
        std::cout << "drumReachedDuration: " << drumReachedDuration << " [ms]\n";
        std::cout << "\nCommands:\n";
        std::cout << "[a]: des_tff | [b]: Direction | [c]: Back Torque\n";
        std::cout << "[f]: Run | [g]: Exit\n";
        std::cout << "=============================================\n";
        std::cout << "Enter Command: ";
        std::cin >> userInput;
        std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);

        if (userInput[0] == 'g')
        {
            break;
        }

        else if (userInput == "c")
        {
            std::cout << "Enter Desired [Back] Torque In Unit: ";
            std::cout << "100 [unit] = 3.1052 [mNm]\n";
            std::cin >> backTorqueUnit;
        }
        else if (userInput == "a" && isMaxonMotor)
        {
            std::cout << "Enter Desired Torque In Unit: ";
            std::cout << "-100 [unit] = -3.1052 [mNm]\n";
            std::cin >> des_tff;
        }
        else if (userInput == "d" && isMaxonMotor)
        {
            std::cout << "Enter Desired Torque Threshold: ";
            std::cout << "-100 [unit] = -3.1052 [mNm]\n";
            std::cin >> tffThreshold;
        }
        else if (userInput == "e" && isMaxonMotor)
        {
            std::cout << "Enter Desired Position Threshold: ";
            std::cin >> posThreshold;
        }
        else if (userInput[0] == 'f' && isMaxonMotor)
        {
            TestStick(selectedMotor, des_tff, tffThreshold, posThreshold, backTorqueUnit);
        }
    }
}

void TestManager::TestStick(const std::string selectedMotor, int des_tff, float tffThreshold, float posThreshold, int backTorqueUnit)
{

    canManager.setSocketsTimeout(0, 50000);
    std::string FileName1 = "../../READ/" + selectedMotor + "_cst_in.txt";

    std::ofstream csvFileIn(FileName1);

    if (!csvFileIn.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // Input File
    csvFileIn << "0x007,0x001,0x002,0x003,0x004,0x005,0x006,0x008,0x009\n";
    std::string FileName2 = "../../READ/" + selectedMotor + "_cst_out.txt";
    std::ofstream csvFileOut(FileName2);

    if (!csvFileOut.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }
    csvFileOut << "CAN_ID,pos_act,tff_act\n"; // CSV 헤더

    struct can_frame frame;

    float p_act, tff_act;

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
    bool reachedDrum = false;
    bool motorFixed = false;

    chrono::system_clock::time_point external = std::chrono::system_clock::now();
    bool motorModeSet = false;

    float positionValues[4] = {0}; // 포지션 값 저장을 위한 정적 배열
    int posIndex = 0;              // 현재 포지션 값 인덱스

    chrono::system_clock::time_point start, drumHitTime, drumReachedTime;
    bool drumHit = false;     // 드럼을 친 시점을 기록하기 위한 변수
    bool drumReached = false; // 기준 위치에 도달한 시점을 기록하기 위한 변수

    start = std::chrono::system_clock::now();

    while (1)
    {

        if (!motorModeSet)
        {
            maxoncmd.getCSTMode(*maxonMotor, &frame);
            canManager.sendAndRecv(motors[selectedMotor], frame);
            motorModeSet = true; // 모터 모드 설정 완료
        }
        if (motorFixed)
        {
            break;
        }

        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);
        if (elapsed_time.count() >= 5000)
        {
            maxoncmd.getTargetTorque(*maxonMotor, &frame, des_tff);
            canManager.txFrame(motors[selectedMotor], frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motors[selectedMotor], frame);

            if (canManager.recvToBuff(motors[selectedMotor], canManager.maxonCnt))
            {
                while (!motors[selectedMotor]->recieveBuffer.empty())
                {
                    frame = motors[selectedMotor]->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {
                        std::tuple<int, float, float, int8_t> result = maxoncmd.parseRecieveCommand(*maxonMotor, &frame);

                        p_act = std::get<1>(result);
                        tff_act = std::get<2>(result);
                        csvFileOut << "0x" << std::hex << std::setw(4) << std::setfill('0') << maxonMotor->nodeId;
                        csvFileOut << ',' << std::dec << p_act << "," << tff_act << '\n';

                        positionValues[posIndex % 4] = p_act;
                        posIndex++;

                        if (!reachedDrum && dct_fun(positionValues, 0))
                        {
                            des_tff = backTorqueUnit;
                            reachedDrum = true;
                            drumHitTime = std::chrono::system_clock::now(); // 드럼을 친 시점의 시간 기록
                            drumHit = true;
                        }

                        // 특정 각도에 도달했는지 확인하는 조건
                        if (p_act > posThreshold && reachedDrum)
                        {
                            maxoncmd.getCSPMode(*maxonMotor, &frame);
                            canManager.sendAndRecv(motors[selectedMotor], frame);

                            maxoncmd.getTargetPosition(*maxonMotor, &frame, p_act);
                            canManager.txFrame(motors[selectedMotor], frame);
                            maxoncmd.getSync(&frame);
                            canManager.txFrame(motors[selectedMotor], frame);
                            if (canManager.recvToBuff(motors[selectedMotor], canManager.maxonCnt))
                            {
                                while (!motors[selectedMotor]->recieveBuffer.empty())
                                {
                                    frame = motors[selectedMotor]->recieveBuffer.front();
                                    if (frame.can_id == maxonMotor->rxPdoIds[0] && !motorFixed)
                                    {
                                        motorFixed = true;
                                        if (!drumReached)
                                        {
                                            drumReachedTime = std::chrono::system_clock::now(); // 기준 위치에 도달한 시점의 시간 기록
                                            drumReached = true;
                                        }
                                    }
                                    motors[selectedMotor]->recieveBuffer.pop();
                                }
                            }
                        }
                    }
                    if (!motors[selectedMotor]->recieveBuffer.empty())
                    {
                        motors[selectedMotor]->recieveBuffer.pop();
                    }
                }
            }
        }
    }

    if (drumHit)
    {
        drumHitDuration = chrono::duration_cast<chrono::milliseconds>(drumHitTime - start).count();
    }

    // 드럼을 친 시점부터 기준 위치까지 올라온 시간을 계산 및 출력
    if (drumReached)
    {
        drumReachedDuration = chrono::duration_cast<chrono::milliseconds>(drumReachedTime - drumHitTime).count();
    }

    csvFileIn.close();
    csvFileOut.close();

    getchar();
}

bool TestManager::dct_fun(float positions[], float vel_th)
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
/*                                 Servo Test Mode                           */
///////////////////////////////////////////////////////////////////////////////

void TestManager::startTest_servo(const string selectedMotor_servo, float pos, float vel, float acl)
{
    std::cout << "Test Start For " << selectedMotor_servo << "\n";

    float dt = 0.005;
    std::shared_ptr<TMotor> sMotor = std::dynamic_pointer_cast<TMotor>(motors[selectedMotor_servo]);
    float time = time_servo + 1;
    int n = time / dt;

    for (int i = 0; i < n; i++)
    {
        for (const auto &motor_pair : motors)
        {
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
            {
                TMotorData newData;
                if (tMotor->myName == selectedMotor_servo)
                {
                    tMotor->isfixed = false;
                    newData.position = pos * tMotor->cwDir - tMotor->homeOffset;
                    newData.spd = vel;
                    newData.acl = acl;
                    newData.isBreak = false;
                    tMotor->commandBuffer.push(newData);
                }
                else
                {
                    if (tMotor->isfixed == false)
                    {
                        tMotor->fixedPos = tMotor->currentPos;
                        tMotor->isfixed = true;
                    }
                    newData.position = tMotor->fixedPos;
                    newData.spd = tMotor->spd;
                    newData.acl = tMotor->acl;
                    newData.isBreak = false;
                    tMotor->commandBuffer.push(newData);
                }
            }
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
            {
                if (maxonMotor->isfixed == false)
                {
                    maxonMotor->fixedPos = maxonMotor->currentPos;
                    maxonMotor->isfixed = true;
                }
                MaxonData newData;
                newData.position = maxonMotor->fixedPos;
                newData.WristState = 0.0;
                maxonMotor->commandBuffer.push(newData);
            }
        }
    }
}

void TestManager::testBreak()
{
    int num, val;
    char data_to_send; // 시리얼 포트로 전송할 문자

    if(useArduino)
    {
        canManager.serial_fd = canManager.setup_serial_port();
        if (canManager.serial_fd == -1) {
            cout << "Serial error";
            return;
        }

        while(true)
        {
            cout << "\n나가기 : -1";
            cout << "\nSelect val : ";
            cin >> val;

            if (val == 0)
            {
                data_to_send = '0';
            }
            else if (val == -1)
            {
                break;
            }
            else
            {
                data_to_send = '1';
            }

            canManager.send_char_to_serial(canManager.serial_fd, data_to_send);

            usleep(100000);

            // 데이터 수신
            std::string received_data = canManager.read_char_from_serial(canManager.serial_fd);
            if (!received_data.empty()) {
                std::cout << "Received data: " << received_data << std::endl;
            }
        }

        //close(canManager.serial_fd);
    }
    else
    {
        if(sensor.OpenDeviceUntilSuccess())
        {
            while(true)
            {
                cout << "\n나가기 : -1";
                cout << "\nSelect num : ";
                cin >> num;
                if (num == -1) break;
                cout << "\nSelect val : ";
                cin >> val;

                sensor.writeValTest(num, val);
            }
        }

        sensor.closeDevice();
    }
}

void TestManager::allBreakOff()
{
    if (useArduino)
    {
        char data_to_send = '0'; // 시리얼 포트로 전송할 문자

        canManager.send_char_to_serial(canManager.serial_fd, data_to_send);

        usleep(100000);

        // 데이터 수신
        std::string received_data = canManager.read_char_from_serial(canManager.serial_fd);
        if (!received_data.empty()) {
            std::cout << "Received data: " << received_data << std::endl;
        }

        //close(canManager.serial_fd);
    }
    else
    {
        for (auto &motor_pair : motors)
        {
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
            {
                sensor.writeVal(tMotor, false);
            }
        }

        sensor.closeDevice();
    }
}