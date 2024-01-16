#include "../include/managers/TestManager.hpp" // 적절한 경로로 변경하세요.

TestManager::TestManager(CanManager &canManagerRef, queue<can_frame> &sendBufferRef, queue<can_frame> &recieveBufferRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : canManager(canManagerRef), sendBuffer(sendBufferRef), recieveBuffer(recieveBufferRef), motors(motorsRef)
{
}

/////////////////////////////////////////////////////////////////////////////////
/*                            SEND BUFFER TO MOTOR                            */
///////////////////////////////////////////////////////////////////////////////

void TestManager::motorInitialize(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
{
    this->motors = motorsRef;

    // 모터 방향에 따른 +/- 값 적용
    ApplyDir();
}

/////////////////////////////////////////////////////////////////////////////////
/*                               SYSTEM FUNCTION                              */
///////////////////////////////////////////////////////////////////////////////

void TestManager::ApplyDir()
{ // CW / CCW에 따른 방향 적용
    for (auto &entry : motors)
    {
        motor_dir[motor_mapping[entry.first]] = entry.second->cwDir;
    }
}

void TestManager::getMotorPos()
{
    // 각 모터의 현재위치 값 불러오기 ** CheckMotorPosition 이후에 해야함(변수값을 불러오기만 해서 갱신 필요)
    for (auto &entry : motors)
    {
        c_MotorAngle[motor_mapping[entry.first]] = entry.second->currentPos;
        // 각 모터의 현재 위치 출력
        cout << "Motor " << entry.first << " current position: " << entry.second->currentPos << "\n";
    }
}

void TestManager::waistarr(vector<vector<double>> &T, int time, double amp)
{
    time = time / 2;
    for (int i = 0; i < time; i++)
    {
        double val = c_MotorAngle[0] - (1.0 - cos(2.0 * M_PI * i / time)) / 2 * amp * motor_dir[0];
        T[i][0] = val;
    }
    for (int i = 0; i < time; i++)
    {
        double val = c_MotorAngle[0] + (1.0 - cos(2.0 * M_PI * i / time)) / 2 * amp * motor_dir[0];
        T[i + time][0] = val;
    }
}

void TestManager::arm1arr(vector<vector<double>> &T, int time, int LnR, double amp)
{
    vector<int> lnr;
    if (LnR == 1)
        lnr.push_back(1);
    else if (LnR == 2)
        lnr.push_back(2);
    else if (LnR == 3)
    {
        lnr.push_back(1);
        lnr.push_back(2);
    }

    time = time / 2;
    for (int i = 0; i < time; i++)
    {
        for (long unsigned int j = 0; j < lnr.size(); j++)
        {
            double val = c_MotorAngle[lnr[j]] - (1.0 - cos(2.0 * M_PI * i / time)) / 2 * amp * motor_dir[lnr[j]];
            T[i][lnr[j]] = val;
        }
    }
    for (int i = 0; i < time; i++)
    {
        for (long unsigned int j = 0; j < lnr.size(); j++)
        {
            double val = c_MotorAngle[lnr[j]] + (1.0 - cos(2.0 * M_PI * i / time)) / 2 * amp * motor_dir[lnr[j]];
            T[i + time][lnr[j]] = val;
        }
    }
}

void TestManager::arm2arr(vector<vector<double>> &T, int time, int LnR, double amp)
{
    vector<int> lnr;
    if (LnR == 1)
        lnr.push_back(3);
    else if (LnR == 2)
        lnr.push_back(5);
    else if (LnR == 3)
    {
        lnr.push_back(3);
        lnr.push_back(5);
    }

    for (int i = 0; i < time; i++)
    {
        for (long unsigned int j = 0; j < lnr.size(); j++)
        {
            double val = c_MotorAngle[lnr[j]] + (1.0 - cos(2.0 * M_PI * i / time)) / 2 * amp * motor_dir[lnr[j]];
            T[i][lnr[j]] = val;
        }
    }
}

void TestManager::arm3arr(vector<vector<double>> &T, int time, int LnR, double amp)
{
    vector<int> lnr;
    if (LnR == 1)
        lnr.push_back(4);
    else if (LnR == 2)
        lnr.push_back(6);
    else if (LnR == 3)
    {
        lnr.push_back(4);
        lnr.push_back(6);
    }

    for (int i = 0; i < time; i++)
    {
        for (long unsigned int j = 0; j < lnr.size(); j++)
        {
            double val = c_MotorAngle[lnr[j]] + (1.0 - cos(2.0 * M_PI * i / time)) / 2 * amp * motor_dir[lnr[j]];
            T[i][lnr[j]] = val;
        }
    }
}

void TestManager::wristarr(vector<vector<double>> &T, int time, int LnR, double amp)
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

    for (int i = 0; i < time; i++)
    {
        for (long unsigned int j = 0; j < lnr.size(); j++)
        {
            double val = c_MotorAngle[lnr[j]] + (1.0 - cos(2.0 * M_PI * i / time)) / 2 * amp * motor_dir[lnr[j]];
            T[i][lnr[j]] = val;
        }
    }
}

void TestManager::writeToSocket(const std::map<std::string, int> &sockets)
{
    struct can_frame frameToProcess;

    for (auto &motor_pair : motors)
    {
        auto motor_ptr = motor_pair.second;
        auto interface_name = motor_ptr->interFaceName;

        frameToProcess = sendBuffer.front(); // sendBuffer에서 데이터 꺼내기
        sendBuffer.pop();

        if (sockets.find(interface_name) != sockets.end())
        {
            int socket_descriptor = sockets.at(interface_name);
            ssize_t bytesWritten = write(socket_descriptor, &frameToProcess, sizeof(struct can_frame));
        }
        else
        {
            std::cerr << "Socket not found for interface: " << interface_name << std::endl;
        }
    }
}

void TestManager::SendLoop()
{
    cout << "Settig...\n";
    struct can_frame frameToProcess;
    std::string maxonCanInterface;

    int maxonMotorCount = 0;
    for (const auto &motor_pair : motors)
    {
        // 각 요소가 MaxonMotor 타입인지 확인
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            maxonMotorCount++;
            maxonCanInterface = maxonMotor->interFaceName;
        }
    }
    chrono::system_clock::time_point external = std::chrono::system_clock::now();
    std::cout << "SendBuffer size" << sendBuffer.size() << "\n";
    while (sendBuffer.size() != 0)
    {
        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);

        if (elapsed_time.count() >= 5000) // 5ms
        {
            external = std::chrono::system_clock::now();

            writeToSocket(canManager.sockets);

            if (maxonMotorCount != 0)
            {

                // sync 신호 전송
                frameToProcess = sendBuffer.front();
                sendBuffer.pop();
                auto it = canManager.sockets.find(maxonCanInterface);

                if (it != canManager.sockets.end())
                {
                    int socket_descriptor_for_sync = it->second;
                    ssize_t bytesWritten = write(socket_descriptor_for_sync, &frameToProcess, sizeof(struct can_frame));

                    handleError(bytesWritten, maxonCanInterface);
                }
                else
                {
                    std::cerr << "Socket not found for interface: " << maxonCanInterface << std::endl;
                }
            }
        }
    }
    canManager.clearReadBuffers();
}

/////////////////////////////////////////////////////////////////////////////////
/*                                  MAKE PATH                                 */
///////////////////////////////////////////////////////////////////////////////

void TestManager::run()
{
    //getMotorPos();
    motorInitialize(motors);
    
    char userInput;
    double t = 4.0;
    int cycles = 1;
    int type = 0b00001;
    int LnR = 1;
    double amplitude[5] = {M_PI / 2, M_PI / 4, M_PI / 6, M_PI / 6, M_PI / 4};

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
        cout << "------------------------------------------------------------------------------------------------------------\n";

        cout << "Commands:\n";
        cout << "[d] : Left and Right | [t] : Type | [p] : Period | [c] : Cycles | [a] : Amplitude | [r] : run | [e] : Exit\n";
        cout << "Enter Command: ";
        cin >> userInput;

        if (userInput == 'e')
        {
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
            while (true)
            {
                char input;
                cout << "Choose Motor\n";
                cout << "1: Arm3\n";
                cout << "2: Arm2\n";
                cout << "3: Arm1\n";
                cout << "4: Waist\n";
                cout << "5: Wrist\n";
                cout << "d: Done\n";
                cout << "Enter Desired Num : ";
                cin >> input;

                if (input == '1')
                {
                    cout << "Enter Arm3's Desired Amplitude : ";
                    cin >> amplitude[0];
                }
                else if (input == '2')
                {
                    cout << "Enter Arm2's Desired Amplitude : ";
                    cin >> amplitude[1];
                }
                else if (input == '3')
                {
                    cout << "Enter Arm1's Desired Amplitude : ";
                    cin >> amplitude[2];
                }
                else if (input == '4')
                {
                    cout << "Enter Waist's Desired Amplitude : ";
                    cin >> amplitude[3];
                }
                else if (input == '5')
                {
                    cout << "Enter Wrist's Desired Amplitude : ";
                    cin >> amplitude[4];
                }
                else if (input == 'd')
                {
                    break;
                }
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

    for (int i = 0; i < time; i++)
    {
        T.push_back(c_MotorAngle);
    }

    if ((type | 0b01111) == 0b11111) // Turn Wrist
    {
        wristarr(T, time, LnR, amp[4]);
    }
    if ((type | 0b10111) == 0b11111) // Turn Waist
    {
        waistarr(T, time, amp[3]);
    }
    if ((type | 0b11011) == 0b11111) // Turn Arm1
    {
        arm1arr(T, time, LnR, amp[2]);
    }
    if ((type | 0b11101) == 0b11111) // Turn Arm2
    {
        arm2arr(T, time, LnR, amp[1]);
    }
    if ((type | 0b11110) == 0b11111) // Turn Arm3
    {
        arm3arr(T, time, LnR, amp[0]);
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

                TParser.parseSendCommand(*tMotor, &frame, tMotor->nodeId, 8, p_des, 0, 200.0, 3.0, 0.0);
                sendBuffer.push(frame);
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
            {
                float p_des = Ti[motor_mapping[entry.first]];
                MParser.getTargetPosition(*maxonMotor, &frame, p_des);
                sendBuffer.push(frame);
            }
        }
        MParser.getSync(&frame);
        sendBuffer.push(frame);
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