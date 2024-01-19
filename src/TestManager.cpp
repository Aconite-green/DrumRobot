#include "../include/managers/TestManager.hpp" // 적절한 경로로 변경하세요.

TestManager::TestManager(SystemState &systemStateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : systemState(systemStateRef), canManager(canManagerRef), motors(motorsRef)
{
}

void TestManager::mainLoop()
{
    int choice;
    canManager.checkAllMotors();
    setMaxonMode("CSP");
    while (systemState.testMode != TestMode::Exit)
    {
        // 사용자에게 선택지 제공
        std::cout << "1: MultiMode\n2: SingleMode\n3: StickMode\n4: Exit\n";
        std::cout << "Select mode (1-4): ";
        std::cin >> choice;

        // 선택에 따라 testMode 설정
        switch (choice)
        {
        case 1:
            systemState.testMode.store(TestMode::MultiMode);
            break;
        case 2:
            systemState.testMode.store(TestMode::SingleMode);
            break;
        case 3:
            systemState.testMode.store(TestMode::StickMode);
            break;
        case 4:
            systemState.testMode.store(TestMode::Exit);
            break;
        default:
            std::cout << "Invalid choice. Please try again.\n";
            continue;
        }

        // testMode에 따라 적절한 함수 호출
        switch (systemState.testMode.load())
        {
        case TestMode::MultiMode:
            multiTestLoop();
            break;
        case TestMode::SingleMode:
            TuningLoopTask();
            break;
        case TestMode::StickMode:
            // StickMode 로직
            break;
        case TestMode::Ideal:
            break;
        case TestMode::Exit:
            systemState.main = Main::Ideal;
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 Multi Test Mode                           */
///////////////////////////////////////////////////////////////////////////////

// 각 관절에 해당하는 열
map<string, int> motor_mapping = {
    {"waist", 0}, {"R_arm1", 1}, {"L_arm1", 2}, {"R_arm2", 3}, {"R_arm3", 4}, {"L_arm2", 5}, {"L_arm3", 6}, {"R_wrist", 7}, {"L_wrist", 8}};
// 각 열에 해당하는 관절방향
map<int, int> motor_dir = { // 1 : CW , -1 : CCW
    {0, 1},
    {1, 1},
    {2, 1},
    {3, 1},
    {4, 1},
    {5, 1},
    {6, 1},
    {7, 1},
    {8, 1}};

map<int, int> motor_Kp = { // Desired Kp Gain
    {0, 200},
    {1, 200},
    {2, 200},
    {3, 200},
    {4, 200},
    {5, 200},
    {6, 200},
    {7, 200},
    {8, 200}};

vector<double> c_MotorAngle = {0, -M_PI / 2, M_PI / 2, M_PI / 4, -M_PI / 2.4, -M_PI / 4, -M_PI / 2.4, 0, 0};

void TestManager::motorInitialize(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
{
    this->motors = motorsRef;

    // 모터 방향에 따른 +/- 값 적용
    ApplyDir();
}

void TestManager::ApplyDir()
{ // CW / CCW에 따른 방향 및 Kp값 적용
    for (auto &entry : motors)
    {
        shared_ptr<GenericMotor> motor = entry.second;
        motor_dir[motor_mapping[entry.first]] = motor->cwDir;
        motor_Kp[motor_mapping[entry.first]] = motor->Kp;
    }
}

void TestManager::getMotorPos()
{
    // 각 모터의 현재위치 값 불러오기 ** CheckMotorPosition 이후에 해야함(변수값을 불러오기만 해서 갱신 필요)
    for (auto &entry : motors)
    {
        shared_ptr<GenericMotor> motor = entry.second;
        c_MotorAngle[motor_mapping[entry.first]] = motor->currentPos;
        // 각 모터의 현재 위치 출력
        cout << "Motor " << entry.first << " current position: " << entry.second->currentPos << "\n";
    }
}

void TestManager::waistarr(vector<vector<double>> &T, int time, double amp, int kp[])
{
    amp = amp / 180.0 * M_PI; // Degree -> Radian 변경
    kp[0] = motor_Kp[0];

    for (int i = 0; i < time; i++)
    {
        double val = c_MotorAngle[0] + sin(2.0 * M_PI * i / time) / 2 * amp * motor_dir[0];
        T[i][0] = val;
    }
}

void TestManager::arm1arr(vector<vector<double>> &T, int time, int LnR, double amp, int kp[])
{
    vector<int> lnr;
    if (LnR == 1)
    {
        lnr.push_back(1);
        kp[1] = motor_Kp[1];
    }
    else if (LnR == 2)
    {
        lnr.push_back(2);
        kp[2] = motor_Kp[2];
    }
    else if (LnR == 3)
    {
        lnr.push_back(1);
        lnr.push_back(2);
        kp[1] = motor_Kp[1];
        kp[2] = motor_Kp[2];
    }

    amp = amp / 180.0 * M_PI; // Degree -> Radian 변경

    for (int i = 0; i < time; i++)
    {
        for (long unsigned int j = 0; j < lnr.size(); j++)
        {
            double val = c_MotorAngle[lnr[j]] + sin(2.0 * M_PI * i / time) / 2 * amp * motor_dir[lnr[j]];
            T[i][lnr[j]] = val;
        }
    }
}

void TestManager::arm2arr(vector<vector<double>> &T, int time, int LnR, double amp, int kp[])
{
    vector<int> lnr;
    if (LnR == 1)
    {
        lnr.push_back(3);
        kp[3] = motor_Kp[3];
    }
    else if (LnR == 2)
    {
        lnr.push_back(5);
        kp[5] = motor_Kp[5];
    }
    else if (LnR == 3)
    {
        lnr.push_back(3);
        lnr.push_back(5);
        kp[3] = motor_Kp[3];
        kp[5] = motor_Kp[5];
    }

    amp = amp / 180.0 * M_PI; // Degree -> Radian 변경

    for (int i = 0; i < time; i++)
    {
        for (long unsigned int j = 0; j < lnr.size(); j++)
        {
            double val = c_MotorAngle[lnr[j]] + (1.0 - cos(2.0 * M_PI * i / time)) / 2 * amp * motor_dir[lnr[j]];
            T[i][lnr[j]] = val;
        }
    }
}

void TestManager::arm3arr(vector<vector<double>> &T, int time, int LnR, double amp, int kp[])
{
    vector<int> lnr;
    if (LnR == 1)
    {
        lnr.push_back(4);
        kp[4] = motor_Kp[4];
    }
    else if (LnR == 2)
    {
        lnr.push_back(6);
        kp[6] = motor_Kp[6];
    }
    else if (LnR == 3)
    {
        lnr.push_back(4);
        lnr.push_back(6);
        kp[4] = motor_Kp[4];
        kp[6] = motor_Kp[6];
    }

    amp = amp / 180.0 * M_PI; // Degree -> Radian 변경

    for (int i = 0; i < time; i++)
    {
        for (long unsigned int j = 0; j < lnr.size(); j++)
        {
            double val = c_MotorAngle[lnr[j]] + (1.0 - cos(2.0 * M_PI * i / time)) / 2 * amp * motor_dir[lnr[j]];
            T[i][lnr[j]] = val;
        }
    }
}

void TestManager::wristarr(vector<vector<double>> &T, int time, int LnR, double amp, int kp[])
{
    vector<int> lnr;
    if (LnR == 1)
        lnr.push_back(7);
    else if (LnR == 2)
        lnr.push_back(8);
    else if (LnR == 3)
    {
        lnr.push_back(7);
        lnr.push_back(8);
    }

    amp = amp / 180.0 * M_PI; // Degree -> Radian 변경

    for (int i = 0; i < time; i++)
    {
        for (long unsigned int j = 0; j < lnr.size(); j++)
        {
            double val = c_MotorAngle[lnr[j]] + (1.0 - cos(2.0 * M_PI * i / time)) / 2 * amp * motor_dir[lnr[j]];
            T[i][lnr[j]] = val;
        }
    }
}

void TestManager::SendLoop()
{
    cout << "Settig...\n";
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

void TestManager::multiTestLoop()
{
    ApplyDir();
    getMotorPos();

    char userInput;
    double t = 4.0;
    int cycles = 1;
    int type = 0b00001;
    int LnR = 1;
    double amplitude[5] = {30.0, 30.0, 30.0, 30.0, 30.0};

    while (true)
    {
        int result = system("clear");
        if (result != 0)
        {
            cerr << "Error during clear screen" << std::endl;
        }

        string typeDescription;
        if ((type | 0b01111) == 0b11111)
        {
            typeDescription += "Wrist Turn, ";
        }
        if ((type | 0b10111) == 0b11111)
        {
            typeDescription += "Waist Turn, ";
        }
        if ((type | 0b11011) == 0b11111)
        {
            typeDescription += "Arm1 Turn, ";
        }
        if ((type | 0b11101) == 0b11111)
        {
            typeDescription += "Arm2 Turn, ";
        }
        if ((type | 0b11110) == 0b11111)
        {
            typeDescription += "Arm3 Turn, ";
        }

        std::string LeftAndRight;
        if (LnR == 1)
        {
            LeftAndRight = "[ Right move ]\n";
        }
        else if (LnR == 2)
        {
            LeftAndRight = "[ Left move ]\n";
        }
        else if (LnR == 3)
        {
            LeftAndRight = "[ Left and Right move ]\n";
        }

        cout << "------------------------------------------------------------------------------------------------------------\n";
        cout << LeftAndRight;
        cout << "Type : " << typeDescription << "\n";
        cout << "Period : " << t << "\n";
        cout << "Cycles : " << cycles << "\n";
        cout << "Amplitude :\n";
        cout << "1) Arm3 = " << amplitude[0] << "\n";
        cout << "2) Arm2 = " << amplitude[1] << "\n";
        cout << "3) Arm1 = " << amplitude[2] << "\n";
        cout << "4) Waist = " << amplitude[3] << "\n";
        cout << "5) Wrist = " << amplitude[4] << "\n";
        cout << "Motor_Kp :\n";
        cout << "1) Arm3 = " << motor_Kp[4] << "\n";
        cout << "2) Arm2 = " << motor_Kp[2] << "\n";
        cout << "3) Arm1 = " << motor_Kp[1] << "\n";
        cout << "4) Waist = " << motor_Kp[0] << "\n";
        cout << "------------------------------------------------------------------------------------------------------------\n";

        cout << "Commands:\n";
        cout << "[d] : Left and Right | [t] : Type | [p] : Period | [c] : Cycles | [a] : Amplitude | [p] : Kp | [r] : run | [e] : Exit\n";
        cout << "Enter Command: ";
        cin >> userInput;

        if (userInput == 'e')
        {
            systemState.testMode = TestMode::Exit;
            break;
        }
        else if (userInput == 'd')
        {
            cout << "Enter Desired Direction:\n";
            cout << "1: Right Move\n";
            cout << "2: Left Move\n";
            cout << "3: Left and Right Move\n";
            cout << "Enter Path Type (1 or 2 or 3): ";
            cin >> LnR;
        }
        else if (userInput == 't')
        {
            int num;
            cout << "Enter Desired Type:\n";
            cout << "1: Arm3 Turn ON/OFF\n";
            cout << "2: Arm2 Turn ON/OFF\n";
            cout << "3: Arm1 Turn ON/OFF\n";
            cout << "4: Waist Turn ON/OFF\n";
            cout << "5: Wrist Turn ON/OFF\n";
            cout << "Enter Path Type (1 or 2 or 3 or 4 or 5): ";
            cin >> num;

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
        else if (userInput == 'p')
        {
            cout << "Enter Desired Period : ";
            cin >> t;
        }
        else if (userInput == 'c')
        {
            cout << "Enter Desired Cycles : ";
            cin >> cycles;
        }
        else if (userInput == 'a')
        {
            int input;
            cout << "Choose Motor\n";
            cout << "1: Arm3\n";
            cout << "2: Arm2\n";
            cout << "3: Arm1\n";
            cout << "4: Waist\n";
            cout << "5: Wrist\n";
            cout << "Enter Desired Motor : ";
            cin >> input;

            cout << "Enter Desired Amplitude(degree) : ";
            cin >> amplitude[input - 1];
        }
        else if (userInput == 'p')
        {
            char input;
            int kp;
            cout << "Choose Motor\n";
            cout << "1: Arm3\n";
            cout << "2: Arm2\n";
            cout << "3: Arm1\n";
            cout << "4: Waist\n";
            cout << "Enter Desired Motor : ";
            cin >> input;

            if (input == '1')
            {
                cout << "Enter Arm3's Desired Kp : ";
                cin >> kp;
                motor_Kp[4] = kp;
                motor_Kp[6] = kp;
            }
            else if (input == '2')
            {
                cout << "Enter Arm2's Desired Kp : ";
                cin >> kp;
                motor_Kp[3] = kp;
                motor_Kp[5] = kp;
            }
            else if (input == '3')
            {
                cout << "Enter Arm1's Desired Kp : ";
                cin >> kp;
                motor_Kp[1] = kp;
                motor_Kp[2] = kp;
            }
            else if (input == '4')
            {
                cout << "Enter Waist's Desired Kp : ";
                cin >> kp;
                motor_Kp[0] = kp;
            }
        }
        else if (userInput == 'r')
        {
            TestArr(t, cycles, type, LnR, amplitude);
        }
    }
}

void TestManager::TestArr(double t, int cycles, int type, int LnR, double amp[])
{
    cout << "Test Start!!\n";

    int time = t / 0.005;
    vector<vector<double>> T;
    int Kp_fixed = 450;
    int kp[7] = {Kp_fixed, Kp_fixed, Kp_fixed, Kp_fixed, Kp_fixed, Kp_fixed, Kp_fixed};

    for (int i = 0; i < time; i++)
    {
        T.push_back(c_MotorAngle);
    }

    if ((type | 0b01111) == 0b11111) // Turn Wrist
    {
        wristarr(T, time, LnR, amp[4], kp);
    }
    if ((type | 0b10111) == 0b11111) // Turn Waist
    {
        waistarr(T, time, amp[3], kp);
    }
    if ((type | 0b11011) == 0b11111) // Turn Arm1
    {
        arm1arr(T, time, LnR, amp[2], kp);
    }
    if ((type | 0b11101) == 0b11111) // Turn Arm2
    {
        arm2arr(T, time, LnR, amp[1], kp);
    }
    if ((type | 0b11110) == 0b11111) // Turn Arm3
    {
        arm3arr(T, time, LnR, amp[0], kp);
    }

    for (int i = 1; i < cycles; i++)
    {
        for (int j = 0; j < time; j++)
        {
            T.push_back(T[j]);
        }
    }

    for (long unsigned int i = 0; i < T.size(); i++)
    {
        struct can_frame frame;

        vector<double> Ti;
        Ti = T[i];

        for (auto &entry : motors)
        {
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
            {
                float p_des = Ti[motor_mapping[entry.first]];

                tmotorcmd.parseSendCommand(*tMotor, &frame, tMotor->nodeId, 8, p_des, 0, kp[motor_mapping[entry.first]], 3.0, 0.0);
                entry.second->sendBuffer.push(frame);
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
            {
                float p_des = Ti[motor_mapping[entry.first]];
                maxoncmd.getTargetPosition(*maxonMotor, &frame, p_des);
                entry.second->sendBuffer.push(frame);
            }
        }
    }

    SendLoop();

    // CSV 입력 파일 열기
    string FileNamein = "../../READ/test_in.txt";
    ofstream csvFileIn(FileNamein);
    if (!csvFileIn.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }
    csvFileIn << "0x007,0x001,0x002,0x003,0x004,0x005,0x006,0x008,0x009\n"; // header

    // 2차원 벡터의 데이터를 CSV 파일로 쓰기
    for (const auto &row : T)
    {
        for (const double cell : row)
        {
            csvFileIn << std::fixed << std::setprecision(5) << cell;
            if (&cell != &row.back())
            {
                csvFileIn << ","; // 쉼표로 셀 구분
            }
        }
        csvFileIn << "\n"; // 다음 행으로 이동
    }

    // CSV 파일 닫기
    csvFileIn.close();

    std::cout << "연주 txt_InData 파일이 생성되었습니다: " << FileNamein << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 Single Test Mode                           */
///////////////////////////////////////////////////////////////////////////////

void TestManager::FixMotorPosition(std::shared_ptr<GenericMotor> &motor)
{
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

void TestManager::TuningTmotor(float kp, float kd, float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType)
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

void TestManager::TuningMaxonCSP(float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType)
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

void TestManager::TuningLoopTask()
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
            }
        }
    }
}

void TestManager::InitializeParameters(const std::string selectedMotor, float &kp, float &kd, float &peakAngle, int &pathType, int &controlType, int &des_vel, int &des_tff, int &direction)
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

void TestManager::TuningMaxonCSV(const std::string selectedMotor, int des_vel, int direction)
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

void TestManager::TuningMaxonCST(const std::string selectedMotor, int des_tff, int direction)
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
