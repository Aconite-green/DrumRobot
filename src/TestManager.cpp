#include "../include/managers/TestManager.hpp" // 적절한 경로로 변경하세요.

// For  Qt
// #include "../managers/TestManager.hpp"
using namespace std;

TestManager::TestManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef, USBIO &usbioRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef), usbio(usbioRef)
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

        std::cout << "[ Current Q Values (Ladian) ]\n";
        for (int i = 0; i < 9; i++)
        {
            q[i] = c_MotorAngle[i];
            std::cout << "Q[" << i << "] : " << c_MotorAngle[i] << "\n";
        }
        fkfun(c_MotorAngle); // 현재 q값에 대한 fkfun 진행

        std::cout << "\nSelect Method (1 - 관절각도값 조절, 2 - 좌표값 조절, 3 - 단일 회전, 4 - Command, 5 - 서보모드, 6 - USBIO-4761, -1 - 나가기) : ";
        std::cin >> method;
        
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
            state.test = TestSub::StickTest;    // command test
        }
        else if (method == 5)
        {
            state.test = TestSub::SetServoTestParm;
        }
        else if (method == 6)
        {
            if(usbio.useUSBIO)
            {
                testUSBIO_4761();
            }
            else
            {
                std::cout << "\n USBIO not connected\n\n";
                usleep(1000000);
            }
        }
        else if (method == 7)
        {
            if (usbio.USBIO_4761_init())
            {
                std::cout << "USBIO-4761 init\n";
                usleep(1000000);
            }
        }
        else if (method == -1)
        {
            state.main = Main::Ideal;
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
        cout << "delta t : " << canManager.deltaT << "s\n";
        cout << "break start time : " << brake_start_time << "s\n";
        cout << "break end time : " << brake_end_time << "s\n";
        cout << "spd : " << speed_test << "erpm\n";
        if (profile_flag)
        {
            cout << "make profile\n";
        }
        else
        {
            cout << "sin profile\n";
        }
        if (canManager.tMotor_control_mode == POS_LOOP)
        {
            cout << "position loop mode\n";
        }
        else if (canManager.tMotor_control_mode == POS_SPD_LOOP)
        {
            cout << "position speed loop mode\n";
        }
        else if (canManager.tMotor_control_mode == SPD_LOOP)
        {
            cout << "speed loop mode\n";
        }

        cout << "\nSelect Motor to Change Value (0-8) / Start Test (9) / Time (10) / q 확인 (11) / Speed (12) / BreakTime (13) / Delata t (14) / Save (15) / Profile (17) / Loop mode (18) / Exit (-1): ";
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
            UnfixedMotor();      
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
            cin >> brake_start_time;

            cout << "break end time (" << brake_start_time << "~" << t << ") : ";
            cin >> brake_end_time;
        }
        else if (userInput == 14)
        {
            cout << "delta t : ";
            cin >> canManager.deltaT;
        }
        else if (userInput == 15)
        {
            std::ostringstream fileNameOut;
            fileNameOut << std::fixed << std::setprecision(1); // 소숫점 1자리까지 표시
            fileNameOut << "../../READ/Test_Qvalue";
            std::string fileName = fileNameOut.str();
            parse_and_save_to_csv(fileName);
        }
        else if (userInput == 17)
        {
            if (profile_flag)
            {
                profile_flag = false;
            }
            else
            {
                profile_flag = true;
            }
        }
        else if (userInput == 18)
        {
            if (canManager.tMotor_control_mode == POS_LOOP)
            {
                canManager.tMotor_control_mode = POS_SPD_LOOP;
            }
            else if (canManager.tMotor_control_mode == POS_SPD_LOOP)
            {
                canManager.tMotor_control_mode = SPD_LOOP;
            }
            else if (canManager.tMotor_control_mode == SPD_LOOP)
            {
                canManager.tMotor_control_mode = POS_LOOP;
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
    case TestSub::StickTest:    // command test
    {
        int userInput = 100;
        bool run_flag = false;
        static int nn = 1;
        static int delay = 5;

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
        cout << "n : " << nn << "\n";
        cout << "delay : " << delay << "ms\n";

        cout << "\nSelect Mode (1) / Change Value (2) / Run(0) / Exit (-1): ";

        cin >> userInput;

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
            cout << "n : ";
            cin >> nn;
            cout << "delay : ";
            cin >> delay;
        }
        else if (userInput == 0)
        {
            run_flag = true;
        }

        if (run_flag)
        {
            for (int i = 0; i < nn ; i++)
            {
                // float c_MotorAngle[9];
                // getMotorPos(c_MotorAngle);
                // for (int i = 0; i < 9; i++)
                // {
                //     q[i] = c_MotorAngle[i];
                // }


                for (auto &motor_pair : motors)
                {
                    if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
                    {
                        float test_pos2 = (q[motor_mapping[motor_pair.first]] + i * test_pos) * tMotor->cwDir - tMotor->homeOffset;
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

                usleep(delay*1000);
            }
        }
        
        break;
    }
    case TestSub::SetServoTestParm:
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
            Input_pos.clear();
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
                tMotor->coordinatePos = (tMotor->currentPos + tMotor->homeOffset) * tMotor->cwDir;
                cout << tMotor->myName << " : " << entry.second->coordinatePos << "\n";
                if (tMotor->myName == selectedMotor_servo)
                {
                    targetpos_coo = tMotor->coordinatePos;
                }
            }
        }
        std::cout << "\n------------------------------------------------------------------------------------------------------------\n";
        std::cout << "Selected Motor : " << selectedMotor_servo << "\n"
                  << "Time : " << time_servo << "[sec]\n"
                  << "Target coordinate Pos : " << targetpos_coo << " [Radian]\n"
                  << "Single Target Pos : " << targetpos_des << " [Radian]\n"
                  << "Current Break Val : " << current_servo << "[A]\n";
        std::cout << "------------------------------------------------------------------------------------------------------------\n";

        std::cout << "\n[Commands]\n";
        std::cout << "[s] : Select Other Motor\n"
                  << "[t] : Set Time"
                  << "[p] : Set Single Target Pos\t [d] : Set Break Current\n"
                  << "[f] : Set Origin\t [g] : Current Break!!\t [h] : Dont Break\n"
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
            std::cin >> selectedMotor_servo;
        }
        else if (userInput == 't')
        {
            std::cout << "\nEnter Desire Time [sec] : ";
            std::cin >> time_servo;
        }
        else if (userInput == 'p')
        {
            std::cout << "\nEnter Desire Target Position [Degree] : ";
            std::cin >> targetpos_des;
        }
        else if (userInput == 'd')
        {
            std::cout << "\nEnter Desire Current : ";
            std::cin >> current_servo;
        }
        else if (userInput == 'f')
        {
            for (auto &entry : motors)
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                {
                    if (tMotor->myName == selectedMotor_servo)
                    {
                        tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                        canManager.sendMotorFrame(tMotor);
                    }
                }
            }
        }
        else if (userInput == 'g')
        {
            for (auto &entry : motors)
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                {
                    if (tMotor->myName == selectedMotor_servo)
                    {
                        tservocmd.comm_can_set_cb(*tMotor, &tMotor->sendFrame, current_servo);
                        canManager.sendMotorFrame(tMotor);
                    }
                }
            }
        }
        else if (userInput == 'h')
        {
            for (auto &entry : motors)
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                {
                    if (tMotor->myName == selectedMotor_servo)
                    {
                        tservocmd.comm_can_set_cb(*tMotor, &tMotor->sendFrame, 0);
                        canManager.sendMotorFrame(tMotor);
                    }
                }
            }
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
        
        state.test = TestSub::CheckBuf;
        break;
    }
    case TestSub::CheckBuf:
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
            state.test = TestSub::TimeCheck;
        else
            state.test = TestSub::Done;
        break;
    }
    case TestSub::TimeCheck:
    {
        usleep(1000000*canManager.deltaT);
        state.test = TestSub::SetCANFrame;
        break;
    }
    case TestSub::SetCANFrame:
    {
        bool isSafe;
        isSafe = canManager.setCANFrame();
        if (!isSafe)
        {
            state.main = Main::Error;
        }
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
                usbio.USBIO_4761_set(motor_mapping[motor_pair.first], tMotor->brake_state);
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
    case TestSub::Done:
    {
        usleep(5000);
        
        if (method == 1)
        {
            state.test = TestSub::SetQValue;
            
            // std::ostringstream fileNameOut;
            // fileNameOut << std::fixed << std::setprecision(1); // 소숫점 1자리까지 표시
            // fileNameOut << "../../READ/Test_0704_P" << q[5]
            //             << "_spd" << speed_test
            //             << "_BrakeTime" << brake_start_time;
            // std::string fileName = fileNameOut.str();
            // parse_and_save_to_csv(fileName);
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

vector<float> TestManager::cal_Vmax(float q0[], float q1[], float t1)
{
    vector<float> Vmax;
    const float acc = 100.0;    // rad/s^2

    for (int i = 0; i < 9; i++)
    {
        float val;
        float S = q1[i] - q0[i];


        cout << i << "번쨰 q0 : " << q0[i] << "rad, q1 : " << q1[i] << "rad\n";
        cout << "S : " << S << "rad, t1 : " << t1 << "s\n";

        // 부호 확인
        if (S < 0)
        {
            S = -1 * S;
        }

        if (S > t1*t1*acc/4)
        {
            // 최대가속도로 도달 불가능
            // -1 반환
            val = -1;
        }
        else
        {
            float A = 1/acc;
            float B = -1*t1;
            float C = S;

            float sol1 = (-B+sqrt(B*B-4*A*C))/2/A;
            float sol2 = (-B-sqrt(B*B-4*A*C))/2/A;

            if (sol1 >= 0 && sol1 <= acc*t1/2)
            {
                val = sol1;
            }
            else if (sol2 >= 0 && sol2 <= acc*t1/2)
            {
                val = sol2;
            }
            else
            {
                val = -2;
            }
        }

        Vmax.push_back(val);

        cout << "Vmax_" << i << " : " << val << "rad/s\n";
    }

    return Vmax;
}

vector<float> TestManager::makeProfile(float Q1[], float Q2[], float k, float n)
{
    vector<float> Qi;
    float acceleration = 100; //320000 / 21 / 10 * 2 * M_PI / 60;  // rad/s^2 
    int sign;
    float Vmax = 0;
    float S;
    static int loop_count =0;

    for (int i = 0; i < 9; i++)
    {
        float val;

        S = Q2[i] - Q1[i];

        if (S < 0)
        {
            S = -1 * S;
            sign = -1;
        }
        else
        {
            sign = 1;
        }
        // 2차 방정식의 계수들
        float a = 1.0 / acceleration;
        float b = -n;
        float c = S;
        float discriminant = (b * b) - (4 * a * c);

        if (discriminant < 0)
        {
            // if(i ==4)
            // {
            // std::cout << "No real solution for Vmax." << std::endl;
            // sleep(1);
            // }
            val = -1;   //Qi.push_back(-1); // 실수 해가 없을 경우 -1 추가
            Qi.push_back(val);
            continue;
        }
        else
        {
            // 2차 방정식의 해 구하기
            float Vmax1 = (-b + std::sqrt(discriminant)) / (2 * a);
            float Vmax2 = (-b - std::sqrt(discriminant)) / (2 * a);
            
            // 두 해 중 양수인 해 선택
            if (Vmax1 > 0 && Vmax1 < 0.5*n*acceleration)
            {
                Vmax = Vmax1;

            }
            else if (Vmax2 > 0 && Vmax2 < 0.5*n*acceleration)
            {
                Vmax = Vmax2;

            }
            else
            {
                //std::cout << "No real solution for Vmax." << std::endl;
                Qi.push_back(Q1[i]); // 실수 해가 없을 경우
                continue;
            }
            //std::cout << "Calculated Vmax: " << Vmax << std::endl;
        }

        if (S == 0)
        {
            // 정지
            val = Q1[i];
        }
        else// if ((Vmax * Vmax / acceleration) < S)
        {
            // 가속
            if (k < Vmax / acceleration)
            {
                val = Q1[i] + sign * 0.5 * acceleration * k * k;
                // if(i==4)
                // {
                // std::cout <<"가속 : " <<val<< std::endl;
                // }
            }
            // 등속
            else if (k < S / Vmax)
            {
                val = Q1[i] + (sign * 0.5 * Vmax * Vmax / acceleration) + (sign * Vmax * (k - Vmax / acceleration)); 
            //    if(i==4)
            //     {
            //     std::cout <<"등속 : " <<val<< std::endl;
            //     }            
            }
            // 감속
            else if (k < Vmax / acceleration + S / Vmax)
            {
                val = Q2[i] - sign * 0.5 * acceleration * (S / Vmax + Vmax / acceleration - k) * (S / Vmax + Vmax / acceleration - k);
                // if(i ==4)
                // {
                // std::cout <<"감속 : " <<val<< std::endl;
                // }               
            }           
            else 
            {
                val = Q2[i];
                // if(i ==4)                
                // {
                // std::cout <<"else : " <<val<< std::endl;
                // }                   
            }
        }

        Qi.push_back(val);

    }
    loop_count ++;
    // cout << " Qi[3] : "<< Qi[3] << " Qi[4] : "<< Qi[4] <<endl;
    return Qi;
}

vector<float> TestManager::sinProfile(float q1[], float q2[], float t, float t2)
{
    vector<float> Qi;

    for (int i = 0; i < 9; i++)
    {
        float val;
        
        float A = q2[i] - q1[i];
        float w = 2.0*M_PI/t2;
        
        val = A * sin(w*t) + q1[i];

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
    vector<float> Q_control_mode_test;

    getMotorPos(c_MotorAngle);

    int n = (int)(t/canManager.deltaT);    // t초동안 실행
    int n_brake = (int)(brake_start_time/canManager.deltaT);
    int n_brake_end = (int)(brake_end_time/canManager.deltaT);

    // cal_Vmax(c_MotorAngle, arr, t);
    
    for (int k = 1; k <= n; ++k)
    {
        // Make GetBack Array
        Qi = connect(c_MotorAngle, arr, k, n);
        q_setting.push_back(Qi);

        if (profile_flag)
        {
            Q_control_mode_test = makeProfile(c_MotorAngle, arr, t*k/n, t);
        }
        else
        {
            Q_control_mode_test = sinProfile(c_MotorAngle, arr, t*k/n, t);
        }

        // Send to Buffer
        for (auto &entry : motors)
        {
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
            {
                TMotorData newData;

                if (canManager.tMotor_control_mode == POS_SPD_LOOP)
                {
                    newData.position = arr[motor_mapping[entry.first]] * tMotor->cwDir - tMotor->homeOffset;
                    newData.spd = speed_test;
                    newData.acl = 32767;
                }
                else if (canManager.tMotor_control_mode == POS_LOOP)
                {
                    newData.position = Q_control_mode_test[motor_mapping[entry.first]] * tMotor->cwDir - tMotor->homeOffset;
                    newData.spd = tMotor->spd;
                    newData.acl = tMotor->acl;
                }
                else if (canManager.tMotor_control_mode == SPD_LOOP)
                {
                    newData.position = Q_control_mode_test[motor_mapping[entry.first]] * tMotor->cwDir - tMotor->homeOffset;
                    newData.spd = speed_test;
                    newData.acl = tMotor->acl;
                }

                
                if (k < n_brake)
                {
                    newData.isBrake = false;
                }
                else if (k < n_brake_end)
                {
                    newData.isBrake = true;
                }
                else
                {
                    newData.isBrake = false;
                }
                tMotor->commandBuffer.push(newData);
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
                        newData.isBrake = false;
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
                        newData.isBrake = false;
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
                        newData.isBrake = false;
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
                        newData.isBrake = false;
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
                    newData.isBrake = false;
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
                    newData.isBrake = false;
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

void TestManager::UnfixedMotor()
{
    for (auto motor_pair : motors)
        motor_pair.second->isfixed = false;
}

void TestManager::testUSBIO_4761()
{
    // int inputVal = 0;
    
    // do
    // {
    //     std::cout << "Input a value for DO port (-1 : Exit) : ";
    //     std::cin >> inputVal;

    //     if (inputVal == -1)
    //     {
    //         break;
    //     }

    //     usbio.USBIO_4761_testset(inputVal);
    //     bool usbio_output = usbio.USBIO_4761_output();
        
    //     usleep(100000);

    //     if (usbio_output)
    //     {
    //         std::cout << "\n DO output completed !\n\n";
    //     }
    //     else
    //     {
    //         break;
    //     }
    // } while (true);

    int i;
    for (i = 0; i < 200; i++)
    {
        usbio.USBIO_4761_testset(i%8);
        bool usbio_output = usbio.USBIO_4761_output();
        
        usleep(100000);

        if (usbio_output)
        {
            if(i == 199)
            {
                std::cout << "\n DO output completed !\n\n";
                usleep(1000000);
            }
        }
        else
        {
            int cnt = 0;
            while(!usbio.USBIO_4761_output())
            {
                cout << "brake Error\n";
                usbio.USBIO_4761_init();
                cnt++;
                if (cnt >= 5) break;
            }
            //cout << "brake Error\n";
            //usbio.USBIO_4761_init();
            // std::cout << "\n Error " << i << "\n\n";
            // usleep(1000000);
            // break;
        }
    }
    
}