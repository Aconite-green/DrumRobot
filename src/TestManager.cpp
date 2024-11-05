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
            int ret = system("clear");
            if (ret == -1)
                std::cout << "system clear error" << endl;

            float c_MotorAngle[9];
            getMotorPos(c_MotorAngle);

            std::cout << "[ Current Q Values (Radian) ]\n";
            for (int i = 0; i < 9; i++)
            {
                q[i] = c_MotorAngle[i];
                std::cout << "Q[" << i << "] : " << c_MotorAngle[i] << "\n";
            }
            fkfun(c_MotorAngle); // 현재 q값에 대한 fkfun 진행

            std::cout << "\nSelect Method (1 - 관절각도값 조절, 2 - 좌표값 조절, -1 - 나가기) : ";
            std::cin >> method;
            
            if (method == 1)
            {
                state.test = TestSub::SetQValue;
            }
            else if (method == 2)
            {
                state.test = TestSub::SetXYZ;
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

            float c_MotorAngle[9] = {0};
            getMotorPos(c_MotorAngle);


            if(sin_flag)
            {
                std::cout << "Mode : Sin Wave\n";
            }
            else
            {
                std::cout << "Mode : Go to target point\n";
            }
            std::cout << "\n[Current Q Values] [Target Q Values] (Radian)\n";
            for (int i = 0; i < 9; i++)
            {
                std::cout << "Q[" << i << "] : " << c_MotorAngle[i] << "\t<->\t" << q[i] << std::endl;
            }

            std::cout << "\ntime : " << t << "s";
            if(!sin_flag) std::cout << " + " << extra_time << "s";
            std::cout << "\nnumber of repeat : " << n_repeat << std::endl << std::endl;

            for (int i = 0; i < 7; i++)
            {
                if (brake_flag[i])
                {
                    std::cout << "Joint " << i << " brake on : " << brake_start_time[i] << "s ~ " << brake_end_time[i] << "s\n";
                }
                else
                {
                    std::cout << "Joint " << i << " brake off\n";
                }
            }
            
            std::cout << "\nSelect Motor to Change Value (0-8) / Run (9) / Time (10) / Extra Time (11) / Repeat(12) / Brake (13) / initialize test (14) / Sin Profile (15) / break on off (16) / Exit (-1): ";
            std::cin >> userInput;

            if (userInput == -1)
            {
                state.test = TestSub::SelectParamByUser;
            }
            else if (userInput < 9)
            {
                float degree_angle;
                
                std::cout << "\nRange : " << motorMinArr[userInput] << "~" << motorMaxArr[userInput] << "(Degree)\n";
                std::cout << "Enter q[" << userInput << "] Values (Degree) : ";
                std::cin >> degree_angle;
                q[userInput] = degree_angle * M_PI / 180.0;
            }
            else if (userInput == 9)
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
                state.test = TestSub::FillBuf;
                usleep(5000);
                UnfixedMotor();
            }
            else if (userInput == 10)
            {
                std::cout << "time : ";
                std::cin >> t;
            }
            else if (userInput == 11)
            {
                std::cout << "extra time : ";
                std::cin >> extra_time;
            }
            else if (userInput == 12)
            {
                std::cout << "number of repeat : ";
                std::cin >> n_repeat;
            }
            else if (userInput == 13)
            {
                int input_brake;
                std::cout << "Select joint : ";
                std::cin >> input_brake;

                if(input_brake < 7)
                {
                    if(brake_flag[input_brake])
                    {
                        brake_flag[input_brake] = false;
                    }
                    else
                    {
                        brake_flag[input_brake] = true;

                        std::cout << "brake start time (0~" << t+extra_time << ") : ";
                        std::cin >> brake_start_time[input_brake];

                        std::cout << "brake end time (" << brake_start_time[input_brake] << "~" << t+extra_time << ") : ";
                        std::cin >> brake_end_time[input_brake];
                    }
                }
            }
            else if (userInput == 14)
            {
                for (int i = 0; i <= 6; ++i)
                {
                    brake_flag[i] = false;
                    float degree_angle;
                    
                    // 1과 2는 90도, 나머지는 0도로 설정
                    if (i == 1 || i == 2) {
                        degree_angle = 90.0;
                    } else {
                        degree_angle = 0.0;
                    }
                    
                    std::cout << "\nRange : " << motorMinArr[i] << "~" << motorMaxArr[i] << "(Degree)\n";
                    std::cout << "Enter q[" << i << "] Values (Degree) : " << degree_angle << "\n";
                    
                    // degree 값을 radian으로 변환하여 q 배열에 저장
                    q[i] = degree_angle * M_PI / 180.0;
                }
                t = 4.0;
                extra_time = 1.0;
                n_repeat = 1;
                sin_flag = false;

                state.test = TestSub::FillBuf;
                usleep(5000);
                UnfixedMotor();

            }

            else if (userInput == 15)
            {
                if(sin_flag)
                {
                    sin_flag = false;
                    extra_time = 1.0;
                }
                else
                {
                    sin_flag = true;
                    extra_time = 0.0;
                }
            }

            else if (userInput == 16)
            {
                bool usbio_output = false;

                std::cout << "Current brake states:" << std::endl;
                for (int i = 0; i < 7; i++)
                {
                    std::cout << "Brake " << i << ": " << (single_brake_flag[i] ? "Active" : "Inactive") << std::endl;
                }

                int break_num;
                cin >> break_num;

                if(!single_brake_flag[break_num])
                {
                    usbio.USBIO_4761_set(break_num % 7, true);
                    usbio_output = usbio.USBIO_4761_output();
                    single_brake_flag[break_num] = true;
                }
                else
                {
                    usbio.USBIO_4761_set(break_num % 7, false);
                    usbio_output = usbio.USBIO_4761_output();
                    single_brake_flag[break_num] = false;
                }

                if(!usbio_output)
                {
                    std::cout << "OUTPUT Error" << endl;
                    usleep(5000000);
                    break;
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
            c_MotorAngle[motor_mapping[entry.first]] = (maxonMotor->currentPos + maxonMotor->homeOffset) * maxonMotor->cwDir;
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

vector<float> TestManager::cal_Vmax(float q1[], float q2[],  float acc, float t2)
{
    vector<float> Vmax;

    for (long unsigned int i = 0; i < 9; i++)
    {
        float val;
        float S = q2[i] - q1[i];

        // 이동거리 양수로 변경
        if (S < 0)
        {
            S = -1 * S;
        }

        if (S > t2*t2*acc/4)
        {
            // 가속도로 도달 불가능
            // -1 반환
            val = -1;
        }
        else
        {
            // 2차 방정식 계수
            float A = 1/acc;
            float B = -1*t2;
            float C = S;
            float sol1 = (-B+sqrt(B*B-4*A*C))/2/A;
            float sol2 = (-B-sqrt(B*B-4*A*C))/2/A;
            if (sol1 >= 0 && sol1 <= acc*t2/2)
            {
                val = sol1;
            }
            else if (sol2 >= 0 && sol2 <= acc*t2/2)
            {
                val = sol2;
            }
            else
            {
                // 해가 범위 안에 없음
                // -2 반환
                val = -2;
            }
        }
        Vmax.push_back(val);
        cout << "Vmax_" << i << " : " << val << "rad/s\n";
    }

    return Vmax;
}

vector<float> TestManager::makeProfile(float q1[], float q2[], vector<float> &Vmax, float acc, float t, float t2)
{
    vector<float> Qi;
    for(long unsigned int i = 0; i < 9; i++)
    {
        float val, S;
        int sign;
        S = q2[i] - q1[i];   
        // 부호 확인
        if (S < 0)
        {
            S = -1 * S;
            sign = -1;
        }
        else
        {
            sign = 1;
        }
        // 궤적 생성
        if (S == 0)
        {
            // 정지
            val = q1[i];
        }
        else if (Vmax[i] < 0)
        {
            // Vmax 값을 구하지 못했을 때 삼각형 프로파일 생성
            float acc_tri = 4 * S / t2 / t2;
            if (t < t2/2)
            {
                val = q1[i] + sign * 0.5 * acc_tri * t * t;
            }
            else if (t < t2)
            {
                val = q2[i] - sign * 0.5 * acc_tri * (t2 - t) * (t2 - t);
            }
            else
            {
                val = q2[i];
            }
        }
        else
        {
            // 사다리꼴 프로파일
            if (t < Vmax[i] / acc)
            {
                // 가속
                val = q1[i] + sign * 0.5 * acc * t * t;
            }
            else if (t < S / Vmax[i])
            {
                // 등속
                val = q1[i] + (sign * 0.5 * Vmax[i] * Vmax[i] / acc) + (sign * Vmax[i] * (t - Vmax[i] / acc));          
            }
            else if (t < Vmax[i] / acc + S / Vmax[i])
            {
                // 감속
                val = q2[i] - sign * 0.5 * acc * (S / Vmax[i] + Vmax[i] / acc - t) * (S / Vmax[i] + Vmax[i] / acc - t);          
            }
            else 
            {
                val = q2[i];
            }
        }
        Qi.push_back(val);
    }
    return Qi;

    // vector<float> Qi;
    // float acceleration = 100; //320000 / 21 / 10 * 2 * M_PI / 60;  // rad/s^2 
    // int sign;
    // float Vmax = 0;
    // float S;
    // static int loop_count =0;

    // for (int i = 0; i < 9; i++)
    // {
    //     float val;

    //     S = Q2[i] - Q1[i];

    //     if (S < 0)
    //     {
    //         S = -1 * S;
    //         sign = -1;
    //     }
    //     else
    //     {
    //         sign = 1;
    //     }
    //     // 2차 방정식의 계수들
    //     float a = 1.0 / acceleration;
    //     float b = -n;
    //     float c = S;
    //     float discriminant = (b * b) - (4 * a * c);

    //     if (discriminant < 0)
    //     {
    //         // if(i ==4)
    //         // {
    //         // std::cout << "No real solution for Vmax." << std::endl;
    //         // sleep(1);
    //         // }
    //         val = -1;   //Qi.push_back(-1); // 실수 해가 없을 경우 -1 추가
    //         Qi.push_back(val);
    //         continue;
    //     }
    //     else
    //     {
    //         // 2차 방정식의 해 구하기
    //         float Vmax1 = (-b + std::sqrt(discriminant)) / (2 * a);
    //         float Vmax2 = (-b - std::sqrt(discriminant)) / (2 * a);
            
    //         // 두 해 중 양수인 해 선택
    //         if (Vmax1 > 0 && Vmax1 < 0.5*n*acceleration)
    //         {
    //             Vmax = Vmax1;

    //         }
    //         else if (Vmax2 > 0 && Vmax2 < 0.5*n*acceleration)
    //         {
    //             Vmax = Vmax2;

    //         }
    //         else
    //         {
    //             //std::cout << "No real solution for Vmax." << std::endl;
    //             Qi.push_back(Q1[i]); // 실수 해가 없을 경우
    //             continue;
    //         }
    //         //std::cout << "Calculated Vmax: " << Vmax << std::endl;
    //     }

    //     if (S == 0)
    //     {
    //         // 정지
    //         val = Q1[i];
    //     }
    //     else// if ((Vmax * Vmax / acceleration) < S)
    //     {
    //         // 가속
    //         if (k < Vmax / acceleration)
    //         {
    //             val = Q1[i] + sign * 0.5 * acceleration * k * k;
    //             // if(i==4)
    //             // {
    //             // std::cout <<"가속 : " <<val<< std::endl;
    //             // }
    //         }
    //         // 등속
    //         else if (k < S / Vmax)
    //         {
    //             val = Q1[i] + (sign * 0.5 * Vmax * Vmax / acceleration) + (sign * Vmax * (k - Vmax / acceleration)); 
    //         //    if(i==4)
    //         //     {
    //         //     std::cout <<"등속 : " <<val<< std::endl;
    //         //     }            
    //         }
    //         // 감속
    //         else if (k < Vmax / acceleration + S / Vmax)
    //         {
    //             val = Q2[i] - sign * 0.5 * acceleration * (S / Vmax + Vmax / acceleration - k) * (S / Vmax + Vmax / acceleration - k);
    //             // if(i ==4)
    //             // {
    //             // std::cout <<"감속 : " <<val<< std::endl;
    //             // }               
    //         }           
    //         else 
    //         {
    //             val = Q2[i];
    //             // if(i ==4)                
    //             // {
    //             // std::cout <<"else : " <<val<< std::endl;
    //             // }                   
    //         }
    //     }

    //     Qi.push_back(val);

    // }
    // loop_count ++;
    // // cout << " Qi[3] : "<< Qi[3] << " Qi[4] : "<< Qi[4] <<endl;
    // return Qi;
}

vector<float> TestManager::sinProfile(float q1[], float q2[], float t, float t2)
{
    vector<float> Qi;

    for (int i = 0; i < 9; i++)
    {
        float val;
        
        float A = q2[i] - q1[i];
        float w = 2.0 * M_PI/t2;
        
        val = -0.5*((A * cos(w*t)) - A)+ q1[i];

        Qi.push_back(val);
    }

    return Qi;
}

void TestManager::GetArr(float arr[])
{
    const float acc_max = 100.0;    // rad/s^2
    vector<float> Qi;
    vector<float> Vmax;
    float Q2[9] = {0.0};
    
    float c_MotorAngle[9] = {0.0};
    int n;
    int n_p;    // 목표위치까지 가기 위한 추가 시간
    int n_brake_start[7] = {0};
    int n_brake_end[7] = {0};

    std::cout << "Get Array...\n";

    getMotorPos(c_MotorAngle);

    // Timing Belt
    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            Q2[motor_mapping[entry.first]] = arr[motor_mapping[entry.first]] / tMotor->timingBelt_ratio;
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            Q2[motor_mapping[entry.first]] = arr[motor_mapping[entry.first]];
        }
    }

    n = (int)(t/canManager.deltaT);    // t초동안 이동
    Vmax = cal_Vmax(c_MotorAngle, Q2, acc_max, t);
    n_p = (int)(extra_time/canManager.deltaT);  // 추가 시간

    for (int i = 0; i < 7; i++)
    {
        if (brake_flag[i])
        {
            n_brake_start[i] = (int)(brake_start_time[i]/canManager.deltaT);
            n_brake_end[i] = (int)(brake_end_time[i]/canManager.deltaT);
        }
    }
    
    for (int i = 0; i < n_repeat; i++)
    {
        for (int k = 1; k <= n + n_p; ++k)
        {
            if(!sin_flag)
            {
                // Make Vector
                if ((i%2) == 0)
                {
                    Qi = makeProfile(c_MotorAngle, Q2, Vmax, acc_max, t*k/n, t);
                }
                else
                {
                    Qi = makeProfile(Q2, c_MotorAngle, Vmax, acc_max, t*k/n, t);
                }
            }
            else
            {
                Qi = sinProfile(c_MotorAngle, Q2, t*k/n, t);
            }

            // Send to Buffer
            for (auto &entry : motors)
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                {
                    TMotorData newData;

                    if (canManager.tMotor_control_mode == POS_SPD_LOOP)
                    {
                        newData.position = Q2[motor_mapping[entry.first]] * tMotor->cwDir - tMotor->homeOffset;
                        newData.spd = tMotor->spd;
                        newData.acl = tMotor->acl;
                    }
                    else if (canManager.tMotor_control_mode == POS_LOOP)
                    {
                        newData.position = Qi[motor_mapping[entry.first]] * tMotor->cwDir - tMotor->homeOffset;
                        newData.spd = tMotor->spd;
                        newData.acl = tMotor->acl;
                    }
                    else if (canManager.tMotor_control_mode == SPD_LOOP)
                    {
                        newData.position = Qi[motor_mapping[entry.first]] * tMotor->cwDir - tMotor->homeOffset;
                        newData.spd = tMotor->spd;
                        newData.acl = tMotor->acl;
                    }

                    
                    if (k < n_brake_start[motor_mapping[entry.first]])
                    {
                        newData.isBrake = false;
                    }
                    else if (k < n_brake_end[motor_mapping[entry.first]])
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
                    newData.position = Qi[motor_mapping[entry.first]] * maxonMotor->cwDir - maxonMotor->homeOffset;
                    newData.WristState = 0.5;
                    maxonMotor->commandBuffer.push(newData);
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

    if (j == 0)struct TMotorData
{
    float position;
    int32_t spd;
    int32_t acl;
    bool isBrake;
};
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