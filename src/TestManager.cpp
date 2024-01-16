#include "../include/managers/TestManager.hpp" // 적절한 경로로 변경하세요.

TestManager::TestManager(queue<can_frame> &sendBufferRef, queue<can_frame> &recieveBufferRef, map<string, shared_ptr<TMotor>> &tmotorsRef, std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotorsRef)
    : sendBuffer(sendBufferRef), recieveBuffer(recieveBufferRef), tmotors(tmotorsRef), maxonMotors(maxonMotorsRef)
{
}

/////////////////////////////////////////////////////////////////////////////////
/*                            SEND BUFFER TO MOTOR                            */
///////////////////////////////////////////////////////////////////////////////

void TestManager::motorInitialize(map<string, shared_ptr<TMotor>> &tmotorsRef, std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotorsRef)
{
    this->tmotors = tmotorsRef;
    this->maxonMotors = maxonMotorsRef;
    // 참조 확인
    cout << "tmotors size in PathManager constructor: " << tmotors.size() << endl;

    // 모터 방향에 따른 +/- 값 적용
    ApplyDir();
}

/////////////////////////////////////////////////////////////////////////////////
/*                               SYSTEM FUNCTION                              */
///////////////////////////////////////////////////////////////////////////////

void TestManager::ApplyDir()
{ // CW / CCW에 따른 방향 적용
    for (auto &entry : tmotors)
    {
        motor_dir[motor_mapping[entry.first]] = entry.second->cwDir;
    }

    for (auto &entry : maxonMotors)
    {
        motor_dir[motor_mapping[entry.first]] = entry.second->cwDir;
    }
}

void TestManager::waistarr(vector<vector<double>> &T, int time)
{
    double WaistMove = M_PI / 4.0;

    time = time / 2;
    for (int i = 0; i < time; i++)
    {
        double val = M_PI / 2 - (1.0 - cos(2.0 * M_PI * i / time)) / 2 * WaistMove;
        T[i][0] = val;
    }
    for (int i = 0; i < time; i++)
    {
        double val = M_PI / 2 + (1.0 - cos(2.0 * M_PI * i / time)) / 2 * WaistMove;
        T[i + time][0] = val;
    }
}

void TestManager::arm1arr(vector<vector<double>> &T, int time, int LnR)
{
    double Arm1Move = M_PI / 4.0;

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
        double val = M_PI / 2 - (1.0 - cos(2.0 * M_PI * i / time)) / 2 * Arm1Move;
        for (int j = 0; j < lnr.size(); j++)
        {
            T[i][lnr[j]] = val;
        }
    }
    for (int i = 0; i < time; i++)
    {
        double val = M_PI / 2 + (1.0 - cos(2.0 * M_PI * i / time)) / 2 * Arm1Move;
        for (int j = 0; j < lnr.size(); j++)
        {
            T[i + time][lnr[j]] = val;
        }
    }
}

void TestManager::arm2arr(vector<vector<double>> &T, int time, int LnR)
{
    double Arm2Move = M_PI / 4.0;

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
        double val = (1.0 - cos(2.0 * M_PI * i / time)) / 2 * Arm2Move;
        for (int j = 0; j < lnr.size(); j++)
        {
            T[i][lnr[j]] = val;
        }
    }
}

void TestManager::arm3arr(vector<vector<double>> &T, int time, int LnR)
{
    double Arm3Move = M_PI / 2.0;

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
        double val = (1.0 - cos(2.0 * M_PI * i / time)) / 2 * Arm3Move;
        for (int j = 0; j < lnr.size(); j++)
        {
            T[i][lnr[j]] = val;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                                  MAKE PATH                                 */
///////////////////////////////////////////////////////////////////////////////

void TestManager::run()
{
    double t = 0.0;
    int cycles = 0;
    int type = 0b0000;
    int LnR = 0;
    
    while(true)
    {
        int result = system("clear");
        if (result != 0)
        {
            cerr << "Error during clear screen" << std::endl;
        }

        string typeDescription;
        if (type || 0b0111)
        {
            typeDescription += "Waist Turn, ";
        }
        if (type || 0b1011)
        {
            typeDescription += "Arm1 Turn, ";
        }
        if (type || 0b1101)
        {
            typeDescription += "Arm2 Turn, ";
        }
        if (type || 0b1110)
        {
            typeDescription += "Arm3 Turn, ";
        }

        std::string LeftAndRight;
        if(LnR == 1){
            LeftAndRight = "Left move"
        }
        else if (LnR == 2){

        }
        else if (LnR == 3){
            
        }
    }
}
void TestManager::TestArr(double t, int cycles, int type, int LnR)
{
    cout << "Test Start!!\n";

    // CSV 입력 파일 열기
    string FileNamein = "../../READ/test_in.txt";
    ofstream csvFileIn(FileNamein);
    if (!csvFileIn.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }
    csvFileIn << "0x007,0x001,0x002,0x003,0x004,0x005,0x006,0x008,0x009\n"; // header

    int time = t / 0.005;
    vector<vector<double>> T;
    vector<double> A = {0.0, M_PI / 2, M_PI / 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < time; i++)
    {
        T.push_back(A);
    }

    if (type || 0b0111 == 0b1111) // Turn Waist
    {
        waistarr(T, time);
    }
    else if (type || 0b1011 == 0b1111) // Turn Arm1
    {
        arm1arr(T, time, LnR);
    }
    else if (type || 0b1101 == 0b1111) // Turn Arm2
    {
        arm2arr(T, time, LnR);
    }
    else if (type || 0b1110 == 0b1111) // Turn Arm3
    {
        arm3arr(T, time, LnR);
    }

    for (int i = 1; i < cycles; i++)
    {
        for (int j = 0; j < time; j++)
        {
            T.push_back(T[j]);
        }
    }

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