#include "../include/managers/HomeManager.hpp"

HomeManager::HomeManager(SystemState &systemStateRef,
                         CanManager &canManagerRef,
                         std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : systemState(systemStateRef), canManager(canManagerRef), motors(motorsRef)
{
}

void HomeManager::mainLoop()
{
    // MaxonEnable();
    setMaxonMode("HMM");
    while (systemState.main == Main::Homing)
    {
        displayHomingStatus();

        std::string motorName;
        std::cout << "Enter the name of the motor to home, or 'all' to home all motors: ";
        std::cin >> motorName;

        // L_arm2, L_arm3, R_arm2, R_arm3 입력 시 L_arm1, R_arm1 홈 상태 확인
        /*if ((motorName == "L_arm2" || motorName == "L_arm3") && !tmotors["L_arm1"]->isHomed)
        {
            std::cout << "Error: L_arm1 must be homed before " << motorName << std::endl;
            continue; // 다음 입력을 위해 반복문의 시작으로 돌아감
        }
        else if ((motorName == "R_arm2" || motorName == "R_arm3") && !tmotors["R_arm1"]->isHomed)
        {
            std::cout << "Error: R_arm1 must be homed before " << motorName << std::endl;
            continue;
        }
        else if ((motorName == "R_wrist") && !tmotors["R_arm3"]->isHomed)
        {
            std::cout << "Error: R_arm3 must be homed before " << motorName << std::endl;
            continue;
        }
        else if ((motorName == "L_wrist") && !tmotors["L_arm3"]->isHomed)
        {
            std::cout << "Error: L_arm3 must be homed before " << motorName << std::endl;
            continue;
        }*/

        if (motorName == "all")
        {
            /* 모터 한개씩 진행
            // 우선순위가 높은 T모터 먼저 홈
            std::vector<std::string> priorityMotors = {"L_arm1", "R_arm1"};
            for (const auto &pmotorName : priorityMotors)
            {
                if (motors.find(pmotorName) != motors.end() && !motors[pmotorName]->isHomed)
                {
                    SetTmotorHome(motors[pmotorName], pmotorName);
                }
            }
            for (auto &motor_pair : motors)
            {
                auto &motor = motor_pair.second;
                std::string name = motor_pair.first;

                // 타입에 따라 적절한 캐스팅과 초기화 수행
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
                {
                    if (!motor->isHomed)
                    {
                        SetTmotorHome(motors[name], name);
                    }
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
                {
                    if (!motor->isHomed)
                    {
                        SetMaxonHome(motors[name], name);
                    }
                }
            }*/

            // 우선순위가 높은 Arm1모터 먼저 홈
            std::vector<std::string> PmotorNames = {"L_arm1", "R_arm1"};
            std::vector<std::shared_ptr<GenericMotor>> Pmotors;
            for (const auto &pmotorName : PmotorNames)
            {
                if (motors.find(pmotorName) != motors.end() && !motors[pmotorName]->isHomed)
                {
                    Pmotors.push_back(motors[pmotorName]);
                }
            }
            SetTmotorHome(Pmotors, PmotorNames);
            Pmotors.clear();

            // 다음 우선순위 : Arm2
            PmotorNames = {"L_arm2", "R_arm2"};
            for (const auto &pmotorName : PmotorNames)
            {
                if (motors.find(pmotorName) != motors.end() && !motors[pmotorName]->isHomed)
                {
                    Pmotors.push_back(motors[pmotorName]);
                }
            }
            SetTmotorHome(Pmotors, PmotorNames);
            Pmotors.clear();

            // 다음 우선순위 : Arm3
            PmotorNames = {"L_arm3", "R_arm3"};
            for (const auto &pmotorName : PmotorNames)
            {
                if (motors.find(pmotorName) != motors.end() && !motors[pmotorName]->isHomed)
                {
                    Pmotors.push_back(motors[pmotorName]);
                }
            }
            SetTmotorHome(Pmotors, PmotorNames);
            Pmotors.clear();

            // 다음 우선순위 : Wrist
            PmotorNames = {"L_wrist", "R_wrist"};
            for (const auto &pmotorName : PmotorNames)
            {
                if (motors.find(pmotorName) != motors.end() && !motors[pmotorName]->isHomed)
                {
                    Pmotors.push_back(motors[pmotorName]);
                }
            }
            SetMaxonHome(Pmotors);
            Pmotors.clear();
        }
        else if (motors.find(motorName) != motors.end() && !motors[motorName]->isHomed)
        {
            // 타입에 따라 적절한 캐스팅과 초기화 수행
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motors[motorName]))
            {
                SetTmotorHome(motors[motorName], motorName);
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors[motorName]))
            {

                SetMaxonHome(motors[motorName], motorName);
            }
        }
        else
        {
            std::cout << "Motor not found or already homed: " << motorName << std::endl;
        }

        UpdateHomingStatus();
    }
}

bool HomeManager::PromptUserForHoming(const std::string &motorName)
{
    char userResponse;
    std::cout << "Would you like to start homing mode for motor [" << motorName << "]? (y/n): ";
    std::cin >> userResponse;
    return userResponse == 'y';
}

void HomeManager::SetTmotorHome(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames)
{
    sensor.OpenDeviceUntilSuccess();
    canManager.setSocketsTimeout(5, 0);

    for (auto &motorname : motorNames)
    {
        cout << "\n<< Homing for " << motorname << " >>\n";
    }

    HomeTMotor(motors, motorNames);
    for (auto &motor : motors)
    {
        motor->isHomed = true; // 홈잉 상태 업데이트
        sleep(1);
        FixMotorPosition(motor);
    }

    for (auto &motorname : motorNames)
    {
        cout << "-- Homing completed for " << motorname << " --\n\n";
    }

    sensor.closeDevice();
}

void HomeManager::HomeTMotor(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames)
{ // arm2 모터는 -30도, 나머지 모터는 +90도에 센서 위치함.
    struct can_frame frameToProcess;
    vector<shared_ptr<TMotor>> tMotors;
    vector<int> sensorsBit;

    // 속도 제어 - 센서 방향으로 이동
    for (long unsigned int i = 0; i < motorNames.size(); i++)
    {
        tMotors.push_back(dynamic_pointer_cast<TMotor>(motors[i]));

        double initialDirection;
        if (motorNames[i] == "L_arm2" || motorNames[i] == "R_arm2")
            initialDirection = (-0.2) * motors[i]->cwDir;
        else
            initialDirection = 0.2 * motors[i]->cwDir;

        double additionalTorque = 0.0;
        if (motorNames[i] == "L_arm2" || motorNames[i] == "R_arm2")
            additionalTorque = motors[i]->cwDir * (-3.0);
        else if (motorNames[i] == "L_arm3" || motorNames[i] == "R_arm3")
            additionalTorque = motors[i]->cwDir * (2.0);

        sensorsBit.push_back(tMotors[i]->sensorBit);

        tmotorcmd.parseSendCommand(*tMotors[i], &frameToProcess, motors[i]->nodeId, 8, 0, initialDirection, 0, 4.5, additionalTorque);
        canManager.sendAndRecv(motors[i], frameToProcess);
    }

    vector<float> midpoints = MoveTMotorToSensorLocation(motors, motorNames, sensorsBit);

    vector<double> directions, degrees;
    for (long unsigned int i = 0; i < motorNames.size(); i++)
    {
        if (motorNames[i] == "L_arm2" || motorNames[i] == "R_arm2")
        {
            degrees.push_back(-30.0);
            midpoints[i] *= (-1);
        }
        else
        {
            degrees.push_back(90.0);
        }
        directions.push_back(-motors[i]->cwDir);
    }

    RotateTMotor(motors, motorNames, directions, degrees, midpoints);

    cout << "----------------------moved 90 degree (Anti clock wise) --------------------------------- \n";

    for (long unsigned int i = 0; i < motors.size(); i++)
    {
        // 모터를 멈추는 신호를 보냄
        tmotorcmd.parseSendCommand(*tMotors[i], &frameToProcess, motors[i]->nodeId, 8, 0, 0, 0, 5, 0);
        canManager.sendAndRecv(motors[i], frameToProcess);

        canManager.setSocketsTimeout(2, 0);
        // 현재 position을 0으로 인식하는 명령을 보냄
        tmotorcmd.getZero(*tMotors[i], &frameToProcess);
        canManager.sendAndRecv(motors[i], frameToProcess);

        if (motorNames[i] == "L_arm1" || motorNames[i] == "R_arm1")
        {
            degrees[i] = 90.0;
        }
        /*if (motorNames[i] == "L_arm2" || motorNames[i] == "R_arm2"){
            degrees[i] = -30.0;
        }*/
        if (motorNames[i] == "L_arm3" || motorNames[i] == "R_arm3")
        {
            degrees[i] = 90.0;
        }

        midpoints[i] = 0.0;
    }

    RotateTMotor(motors, motorNames, directions, degrees, midpoints);
}

vector<float> HomeManager::MoveTMotorToSensorLocation(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames, vector<int> &sensorsBit)
{
    struct can_frame frameToProcess;
    vector<shared_ptr<TMotor>> tMotors;
    vector<float> firstPosition, secondPosition, positionDifference;
    vector<bool> firstSensorTriggered, TriggeredDone;

    for (long unsigned int i = 0; i < sensorsBit.size(); i++)
    {
        tMotors.push_back(dynamic_pointer_cast<TMotor>(motors[i]));
        firstPosition.push_back(0.0f);
        secondPosition.push_back(0.0f);
        firstSensorTriggered.push_back(false);
        TriggeredDone.push_back(false);

        cout << "Moving " << motorNames[i] << " to sensor location.\n";
    }

    while (true)
    {
        // 모든 모터 센싱 완료 시 break
        bool done = true;
        for (long unsigned int i = 0; i < sensorsBit.size(); i++)
        {
            if (!TriggeredDone[i])
                done = false;
        }
        if (done)
            break;

        for (long unsigned int i = 0; i < sensorsBit.size(); i++)
        {
            if (!TriggeredDone[i])
            {
                bool sensorTriggered = ((sensor.ReadVal() >> sensorsBit[i]) & 1) != 0;

                if (!firstSensorTriggered[i] && sensorTriggered)
                {
                    // 첫 번째 센서 인식
                    firstSensorTriggered[i] = true;
                    canManager.checkConnection(motors[i]);
                    firstPosition[i] = motors[i]->currentPos;
                    std::cout << motorNames[i] << " first sensor position: " << firstPosition[i] << endl;
                }
                else if (firstSensorTriggered[i] && !sensorTriggered)
                {
                    // 센서 인식 해제
                    canManager.checkConnection(motors[i]);
                    secondPosition[i] = motors[i]->currentPos;
                    std::cout << motorNames[i] << " second sensor position: " << secondPosition[i] << endl;

                    TriggeredDone[i] = true;
                }
            }
            else
            {
                tmotorcmd.parseSendCommand(*tMotors[i], &frameToProcess, motors[i]->nodeId, 8, secondPosition[i], 0, motors[i]->Kp, 2.5, 0);
                canManager.sendAndRecv(motors[i], frameToProcess);
            }
        }
    }

    for (long unsigned int i = 0; i < sensorsBit.size(); i++)
    {
        positionDifference.push_back(abs((secondPosition[i] - firstPosition[i]) / 2.0f));
        cout << motorNames[i] << " midpoint position: " << positionDifference[i] << endl;
    }

    return positionDifference;
}

void HomeManager::RotateTMotor(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames, vector<double> &directions, vector<double> &degrees, vector<float> &midpoints)
{

    struct can_frame frameToProcess;
    vector<shared_ptr<TMotor>> tMotors;
    vector<double> targetRadians;
    for (long unsigned int i = 0; i < motorNames.size(); i++)
    {
        tMotors.push_back(dynamic_pointer_cast<TMotor>(motors[i]));
        targetRadians.push_back((degrees[i] * M_PI / 180.0 + midpoints[i]) * directions[i]);
    }

    chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
    int totalSteps = 4000 / 5; // 4초 동안 이동 - 5ms 간격으로 나누기
    for (int step = 1; step <= totalSteps; ++step)
    {
        while (1)
        {
            chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
            if (chrono::duration_cast<chrono::microseconds>(currentTime - startTime).count() > 5000)
                break;
        }

        startTime = std::chrono::system_clock::now();

        // 5ms마다 목표 위치 계산 및 프레임 전송
        for (long unsigned int i = 0; i < motorNames.size(); i++)
        {
            double targetPosition = targetRadians[i] * (static_cast<double>(step) / totalSteps) + motors[i]->currentPos;
            tmotorcmd.parseSendCommand(*tMotors[i], &frameToProcess, motors[i]->nodeId, 8, targetPosition, 0, motors[i]->Kp, 2.5, 0);
            canManager.sendAndRecv(motors[i], frameToProcess);
        }
    }

    totalSteps = 500 / 5;
    for (int step = 1; step <= totalSteps; ++step)
    {
        while (1)
        {
            chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
            if (chrono::duration_cast<chrono::microseconds>(currentTime - startTime).count() > 5000)
                break;
        }

        startTime = std::chrono::system_clock::now();

        // 5ms마다 목표 위치 계산 및 프레임 전송
        for (long unsigned int i = 0; i < motorNames.size(); i++)
        {
            double targetPosition = targetRadians[i] + motors[i]->currentPos;
            tmotorcmd.parseSendCommand(*tMotors[i], &frameToProcess, motors[i]->nodeId, 8, targetPosition, 0, motors[i]->Kp, 2.5, 0);
            canManager.sendAndRecv(motors[i], frameToProcess);
        }
    }

    for (auto &motor : motors)
    {
        canManager.checkConnection(motor);
    }
}

void HomeManager::SetMaxonHome(vector<std::shared_ptr<GenericMotor>> &motors)
{
    struct can_frame frame;

    canManager.clearReadBuffers();
    canManager.setSocketsTimeout(2, 0);
    vector<shared_ptr<MaxonMotor>> maxonMotors;
    for (long unsigned int i = 0; i < motors.size(); i++)
    {
        maxonMotors.push_back(dynamic_pointer_cast<MaxonMotor>(motors[i]));

        // Start to Move by homing method (일단은 PDO)

        maxoncmd.getStartHoming(*maxonMotors[i], &frame);
        canManager.txFrame(motors[i], frame);
        usleep(50000);
    }

    maxoncmd.getSync(&frame);
    canManager.txFrame(motors[0], frame);
    if (canManager.recvToBuff(motors[0], canManager.maxonCnt))
    {
        while (!motors[0]->recieveBuffer.empty())
        {
            frame = motors[0]->recieveBuffer.front();
            for (long unsigned int i = 0; i < motors.size(); i++)
            {
                if (frame.can_id == maxonMotors[i]->rxPdoIds[0])
                {
                    cout << "\nHoming Start!!\n";
                }
            }
            motors[0]->recieveBuffer.pop();
        }
    }

    sleep(10);
    // 홈 위치에 도달할 때까지 반복
    bool done = false;
    while (!done)
    {
        //! motor->isHomed
        done = true;
        for (auto &motor : motors)
        {
            if(!motor->isHomed)
                done = false;
        }

        maxoncmd.getSync(&frame);
        canManager.txFrame(motors[0], frame);
        if (canManager.recvToBuff(motors[0], 2))
        {
            while (!motors[0]->recieveBuffer.empty())
            {
                frame = motors[0]->recieveBuffer.front();
                for (long unsigned int i = 0; i < motors.size(); i++)
                {
                    if (frame.can_id == maxonMotors[i]->rxPdoIds[0])
                    {
                        if (frame.data[1] & 0x80) // 비트 15 확인
                        {
                            motors[i]->isHomed = true; // MaxonMotor 객체의 isHomed 속성을 true로 설정
                                                       // 'this'를 사용하여 멤버 함수 호출
                        }
                    }
                }
                motors[0]->recieveBuffer.pop();
            }
        }
        canManager.clearReadBuffers();

        sleep(1); // 100ms 대기
    }
}


void HomeManager::RotateTMotor(std::shared_ptr<GenericMotor> &motor, const std::string &motorName, double direction, double degree, float midpoint)
{

    struct can_frame frameToProcess;
    std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor);
    chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
    int kp = 250;

    if (motorName == "L_arm1" || motorName == "R_arm1")
        kp = 250;
    else if (motorName == "L_arm2" || motorName == "R_arm2")
        kp = 350;
    else if (motorName == "L_arm3" || motorName == "R_arm3")
        kp = 350;
    // 수정된 부분: 사용자가 입력한 각도를 라디안으로 변환
    const double targetRadian = (degree * M_PI / 180.0 + midpoint) * direction; // 사용자가 입력한 각도를 라디안으로 변환 + midpoint
    int totalSteps = 4000 / 5;                                                  // 4초 동안 5ms 간격으로 나누기

    for (int step = 1; step <= totalSteps; ++step)
    {
        while (1)
        {
            chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
            if (chrono::duration_cast<chrono::microseconds>(currentTime - startTime).count() > 5000)
                break;
        }

        startTime = std::chrono::system_clock::now();

        // 5ms마다 목표 위치 계산 및 프레임 전송
        double targetPosition = targetRadian * (static_cast<double>(step) / totalSteps) + motor->currentPos;
        tmotorcmd.parseSendCommand(*tMotor, &frameToProcess, motor->nodeId, 8, targetPosition, 0, kp, 2.5, 0);
        canManager.sendAndRecv(motor, frameToProcess);

        startTime = std::chrono::system_clock::now();
    }

    totalSteps = 500 / 5;
    for (int step = 1; step <= totalSteps; ++step)
    {
        while (1)
        {
            chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
            if (chrono::duration_cast<chrono::microseconds>(currentTime - startTime).count() > 5000)
                break;
        }

        startTime = std::chrono::system_clock::now();

        // 5ms마다 목표 위치 계산 및 프레임 전송
        double targetPosition = targetRadian + motor->currentPos;
        tmotorcmd.parseSendCommand(*tMotor, &frameToProcess, motor->nodeId, 8, targetPosition, 0, kp, 2.5, 0);
        canManager.sendAndRecv(motor, frameToProcess);

        startTime = std::chrono::system_clock::now();
    }

    canManager.checkConnection(motor);
}

void HomeManager::HomeTMotor(std::shared_ptr<GenericMotor> &motor, const std::string &motorName)
{
    struct can_frame frameToProcess;
    std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor);
    // arm2 모터는 -30도, 나머지 모터는 +90도에 센서 위치함.
    double initialDirection = (motorName == "L_arm2" || motorName == "R_arm2") ? (-0.2) * motor->cwDir : 0.2 * motor->cwDir;

    double additionalTorque = 0.0;
    if (motorName == "L_arm2" || motorName == "R_arm2")
    {
        additionalTorque = motor->cwDir * (-3.0);
    }
    else if (motorName == "L_arm3" || motorName == "R_arm3")
    {
        additionalTorque = motor->cwDir * 2.0;
    }

    tmotorcmd.parseSendCommand(*tMotor, &frameToProcess, motor->nodeId, 8, 0, initialDirection, 0, 4.5, additionalTorque);
    canManager.sendAndRecv(motor, frameToProcess);

    float midpoint = MoveTMotorToSensorLocation(motor, motorName, tMotor->sensorBit);

    double degree = (motorName == "L_arm2" || motorName == "R_arm2") ? -30.0 : 90;
    midpoint = (motorName == "L_arm2" || motorName == "R_arm2") ? -midpoint : midpoint;
    RotateTMotor(motor, motorName, -motor->cwDir, degree, midpoint);

    cout << "----------------------moved 90 degree (Anti clock wise) --------------------------------- \n";

    // 모터를 멈추는 신호를 보냄
    tmotorcmd.parseSendCommand(*tMotor, &frameToProcess, motor->nodeId, 8, 0, 0, 0, 5, 0);
    canManager.sendAndRecv(motor, frameToProcess);

    canManager.setSocketsTimeout(2, 0);
    // 현재 position을 0으로 인식하는 명령을 보냄
    tmotorcmd.getZero(*tMotor, &frameToProcess);
    canManager.sendAndRecv(motor, frameToProcess);

    // 상태 확인
    /*fillCanFrameFromInfo(&frameToProcess, motor->getCanFrameForControlMode());
    SendCommandToTMotor(motor, frameToProcess, motorName);*/

    if (motorName == "L_arm1" || motorName == "R_arm1")
    {
        canManager.checkConnection(motor);
        RotateTMotor(motor, motorName, motor->cwDir, 90, 0);
    }
    /*  // homing 잘 됐는지 센서 위치로 다시 돌아가서 확인
    if(motorName == "L_arm2" || motorName == "R_arm2")
    {
        canManager.checkConnection(motor);
        RotateTMotor(motor, motorName, motor->cwDir, -30, 0);
    }*/
    if (motorName == "L_arm3" || motorName == "R_arm3")
    {
        canManager.checkConnection(motor);
        RotateTMotor(motor, motorName, motor->cwDir, 90, 0);
    }
}

void HomeManager::SetTmotorHome(std::shared_ptr<GenericMotor> &motor, const std::string &motorName)
{
    sensor.OpenDeviceUntilSuccess();
    canManager.setSocketsTimeout(5, 0);

    std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor);
    // 허리는 home 안잡음
    cout << "\n<< Homing for " << motorName << " >>\n";

    HomeTMotor(motor, motorName);
    motor->isHomed = true; // 홈잉 상태 업데이트
    sleep(1);
    FixMotorPosition(motor);

    cout << "-- Homing completed for " << motorName << " --\n\n";
    sensor.closeDevice();
}

void HomeManager::SetMaxonHome(std::shared_ptr<GenericMotor> &motor, const std::string &motorName)
{
    struct can_frame frame;

    canManager.clearReadBuffers();
    canManager.setSocketsTimeout(2, 0);
    std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor);

    // Start to Move by homing method (일단은 PDO)

    maxoncmd.getStartHoming(*maxonMotor, &frame);
    canManager.txFrame(motor, frame);
    usleep(50000);

    maxoncmd.getSync(&frame);
    canManager.txFrame(motor, frame);
    if (canManager.recvToBuff(motor, canManager.maxonCnt))
    {
        while (!motor->recieveBuffer.empty())
        {
            frame = motor->recieveBuffer.front();
            if (frame.can_id == maxonMotor->rxPdoIds[0])
            {
                cout << "\n<< Homing for " << motorName << " >>\n";
            }
            motor->recieveBuffer.pop();
        }
    }

    sleep(10);
    // 홈 위치에 도달할 때까지 반복
    while (!motor->isHomed)
    {

        maxoncmd.getSync(&frame);
        canManager.txFrame(motor, frame);
        if (canManager.recvToBuff(motor, 2))
        {
            while (!motor->recieveBuffer.empty())
            {
                frame = motor->recieveBuffer.front();
                if (frame.can_id == maxonMotor->rxPdoIds[0])
                {
                    if (frame.data[1] & 0x80) // 비트 15 확인
                    {
                        motor->isHomed = true; // MaxonMotor 객체의 isHomed 속성을 true로 설정
                                               // 'this'를 사용하여 멤버 함수 호출
                        cout << "-- Homing completed for " << motorName << " --\n\n";
                    }
                }
                motor->recieveBuffer.pop();
            }
        }
        canManager.clearReadBuffers();

        sleep(1); // 100ms 대기
    }
}

float HomeManager::MoveTMotorToSensorLocation(std::shared_ptr<GenericMotor> &motor, const std::string &motorName, int sensorBit)
{
    float firstPosition = 0.0f, secondPosition = 0.0f;
    bool firstSensorTriggered = false;
    bool secondSensorTriggered = false;

    std::cout << "Moving " << motorName << " to sensor location.\n";

    while (true)
    {
        bool sensorTriggered = ((sensor.ReadVal() >> sensorBit) & 1) != 0;

        if (!firstSensorTriggered && sensorTriggered)
        {
            // 첫 번째 센서 인식
            firstSensorTriggered = true;
            canManager.checkConnection(motor);
            firstPosition = motor->currentPos;
            std::cout << motorName << " first sensor position: " << firstPosition << endl;
        }
        else if (firstSensorTriggered && !sensorTriggered)
        {
            // 센서 인식 해제
            secondSensorTriggered = true;
            canManager.checkConnection(motor);
            secondPosition = motor->currentPos;
            std::cout << motorName << " second sensor position: " << secondPosition << endl;

            break; // while문 탈출
        }

        if (secondSensorTriggered)
            break; // 두 번째 센서 인식 후 반복문 탈출
    }

    // 1번과 2번 위치의 차이의 절반을 저장
    float positionDifference = abs((secondPosition - firstPosition) / 2.0f);
    std::cout << motorName << " midpoint position: " << positionDifference << endl;

    return positionDifference;
}

void HomeManager::displayHomingStatus()
{
    std::cout << "Homing Status of Motors:\n";
    for (const auto &motor_pair : motors)
    {
        std::cout << motor_pair.first << ": "
                  << (motor_pair.second->isHomed ? "Homed" : "Not Homed") << std::endl;
    }
}

void HomeManager::UpdateHomingStatus()
{
    bool allMotorsHomed = true;
    for (const auto &motor_pair : motors)
    {
        if (!motor_pair.second->isHomed)
        {
            allMotorsHomed = false;
            break;
        }
    }

    if (allMotorsHomed)
    {
        systemState.homeMode = HomeMode::HomeDone;
        systemState.main = Main::Ideal;
    }
    else
    {
        systemState.homeMode = HomeMode::NotHome;
    }
}

void HomeManager::MaxonEnable()
{
    struct can_frame frame;
    canManager.setSocketsTimeout(2, 0);

    int maxonMotorCount = 0;
    for (const auto &motor_pair : motors)
    {
        // 각 요소가 MaxonMotor 타입인지 확인
        if (std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            maxonMotorCount++;
        }
    }

    // 제어 모드 설정
    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {

            maxoncmd.getOperational(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            if (canManager.recvToBuff(motor, 2))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {
                        std::cout << "Maxon Enabled \n";
                    }
                    motor->recieveBuffer.pop();
                }
            }

            maxoncmd.getQuickStop(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            if (canManager.recvToBuff(motor, 2))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {
                        std::cout << "Maxon Quick Stopped\n";
                    }
                    motor->recieveBuffer.pop();
                }
            }

            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            if (canManager.recvToBuff(motor, 2))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {
                        std::cout << "Maxon Enabled \n";
                    }
                    motor->recieveBuffer.pop();
                }
            }
        }
    }
};

void HomeManager::MaxonQuickStopEnable()
{
    struct can_frame frame;
    canManager.setSocketsTimeout(2, 0);

    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motorPair.second))
        {
            maxoncmd.getQuickStop(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            if (canManager.recvToBuff(motor, 2))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {
                        std::cout << "Maxon Quick Stopped\n";
                    }
                    motor->recieveBuffer.pop();
                }
            }

            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            if (canManager.recvToBuff(motor, 2))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {
                        std::cout << "Maxon Enabled \n";
                    }
                    motor->recieveBuffer.pop();
                }
            }
        }
    }
}

void HomeManager::motorSettingCmd()
{
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
            }
            else if (name == "R_wrist")
            {
                maxoncmd.getHomingMethodR(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }

            maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            maxoncmd.getCurrentThreshold(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            maxoncmd.getHomePosition(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);
        }
        else if (std::shared_ptr<TMotor> tmotor = std::dynamic_pointer_cast<TMotor>(motorPair.second))
        {
            if (name == "waist")
            {
                tmotorcmd.getZero(*tmotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
        }
    }
    std::cout << "Maxon SDO Set\n";
}

void HomeManager::setMaxonMode(std::string targetMode)
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

void HomeManager::FixMotorPosition(std::shared_ptr<GenericMotor> &motor)
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

