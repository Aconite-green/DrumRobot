#include "../include/managers/HomeManager.hpp"
// For Qt
// #include "../managers/HomeManager.hpp"
HomeManager::HomeManager(State &stateRef,
                         CanManager &canManagerRef,
                         std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef)
{
}

// Utility Function
void HomeManager::displayHomingStatus()
{
    std::cout << "Homing Status of Motors:\n";
    for (const auto &motor_pair : motors)
    {
        std::cout << motor_pair.first << ": "
                  << (motor_pair.second->isHomed ? "Homed" : "Not Homed") << std::endl;
    }
}

void HomeManager::saveHomingInfoToFile()
{
    // 실행 파일 위치인 bin 디렉토리로부터 상대 경로 설정
    const std::string directory = "../include/managers/"; // bin의 상위 디렉토리에서 include/managers 접근
    // 디렉토리 생성 시도 (이미 존재하면 무시)
    mkdir(directory.c_str(), 0777); // 리눅스/유닉스 전용, 권한 설정 주의

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream fileName;
    fileName << directory << "Homing_" << std::put_time(&tm, "%Y-%m-%d") << ".csv";

    // 기존 파일이 존재하는지 확인 후 삭제
    std::string filePath = fileName.str();
    std::ifstream ifile(filePath);
    if (ifile)
    {
        std::remove(filePath.c_str());
    }

    std::ofstream file(filePath);
    if (!file.is_open())
    {
        std::cout << "Failed to open file for writing: " << filePath << std::endl;
        return;
    }

    for (const auto &motor : motors)
    {

        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor.second))
        {
            if (motor.first == "R_arm1" || motor.first == "L_arm1" || motor.first == "R_arm3" || motor.first == "L_arm3")
            {
                tMotor->homeOffset = M_PI / 2 * tMotor->cwDir - tMotor->sensorLocation;
            }
            else if (motor.first == "R_arm2" || motor.first == "L_arm2")
            {
                tMotor->homeOffset = -M_PI / 6 * tMotor->cwDir - tMotor->sensorLocation;
            }
            else if (motor.first == "waist")
            {
                tMotor->homeOffset = -tMotor->currentPos;
            }

            file << motor.first << ", " << tMotor->homeOffset << std::endl;
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor.second))
        {
            if (motor.first == "R_wrist" || motor.first == "L_wrist")
            {
                maxonMotor->homeOffset = M_PI * (0.528) /*95 degree*/ * maxonMotor->cwDir - maxonMotor->bumperLocation;
            }

            file << motor.first << ", " << maxonMotor->homeOffset << std::endl;
        }
    }
    file.close();
}

void HomeManager::loadHomingInfoFromFile()
{
    const std::string directory = "../include/managers/";
    std::string command = "find " + directory + " -name 'Homing_*.csv' | sort -r | head -n 1";

    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());

    std::ifstream file(result); // 개행 문자가 포함될 수 있으니 제거 후 파일 열기
    if (!file.is_open())
    {
        std::cout << "Failed to open the latest homing info file: " + result << std::endl;
        return;
    }

    std::string line, motorName;
    double Offset;
    while (std::getline(file, line))
    {
        std::cout << "Reading line: " << line << std::endl;
        std::istringstream iss(line);
        std::getline(iss, motorName, ','); // 쉼표까지 읽고 멈춤
        iss >> std::ws;                    // 다음 입력에서 공백을 무시

        if (iss >> Offset)
        {
            std::cout << "Parsed: " << motorName << " - " << Offset << std::endl; // 파싱된 데이터 확인

            if (motors.find(motorName) != motors.end())
            {

                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motors[motorName]))
                {
                    tMotor->homeOffset = Offset;
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors[motorName]))
                {
                    maxonMotor->homeOffset = Offset;
                }
            }
        }
    }

    for (auto &motor_pair : motors)
    {
        auto &motor = motor_pair.second;

        // 타입에 따라 적절한 캐스팅과 초기화 수행
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            tMotor->isHomed = true;
            std::cout << tMotor->myName << " : " << tMotor->homeOffset << "\n";
        }
    }

    file.close();
}

// Homing Process Function
void HomeManager::MaxonEnable()
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

void HomeManager::MaxonDisable()
{
    struct can_frame frame;

    canManager.setSocketsTimeout(0, 50000);

    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;

        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            maxoncmd.getQuickStop(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);
            if (canManager.recvToBuff(motor, canManager.maxonCnt))
            {
                while (!motor->recieveBuffer.empty())
                {
                    frame = motor->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {

                        break;
                    }
                    motor->recieveBuffer.pop();
                }
            }
            else
            {
                std::cerr << "Failed to exit for motor [" << name << "]." << std::endl;
            }
        }
    }
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

void HomeManager::SendHomeProcess()
{
    switch (state.home.load())
    {
    case HomeSub::SelectMotorByUser:
    {
        displayHomingStatus();

        std::cout << "Enter a 'motor name', 'all', 'load' for loading home file : ";
        std::cin >> motorName;
        state.home = HomeSub::MakeHomingOrderBuf;
        break;
    }
    case HomeSub::MakeHomingOrderBuf:
    {
        if (motorName == "all")
        {
            for (const auto &motor_pair : motors)
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
                {
                    tmotorServocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                    canManager.sendMotorFrame(tMotor);
                }
            }
            canManager.checkMaxon(); // Waist 모터 포지션 업데이트

            for (auto &PmotorNames : Priority)
            {
                vector<shared_ptr<GenericMotor>> temp;
                for (const auto &pmotorName : PmotorNames)
                {
                    if (motors.find(pmotorName) != motors.end() && !motors[pmotorName]->isHomed)
                    {
                        temp.push_back(motors[pmotorName]);
                    }
                }
                if (!temp.empty())
                {
                    HomingMotorsArr.push_back(temp);
                }
            }
            state.home = HomeSub::GetSelectedMotor;
        }
        else if (motors.find(motorName) != motors.end() && !motors[motorName]->isHomed)
        {
            for (const auto &motor_pair : motors)
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
                {
                    tmotorServocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                    canManager.sendMotorFrame(tMotor);
                }
            }

            vector<shared_ptr<GenericMotor>> temp;
            temp.push_back(motors[motorName]);
            HomingMotorsArr.push_back(temp);
            state.home = HomeSub::GetSelectedMotor;
        }
        else if (motorName == "load")
        {
            loadHomingInfoFromFile();
            setMaxonMode("CSP");
            MaxonEnable();
            canManager.checkMaxon();
            state.home = HomeSub::Done;
            state.main = Main::Ideal;
        }
        else
        {
            std::cout << "Invalid Home command: " << motorName << std::endl;
            state.home = HomeSub::SelectMotorByUser;
        }
        break;
    }
    case HomeSub::GetSelectedMotor:
    {
        cout << "Now Im in GetSelectedMotor state\n";
        if (!HomingMotorsArr.empty())
        {
            vector<shared_ptr<GenericMotor>> currentMotors = HomingMotorsArr.front();
            // 첫 번째 요소를 currentMotors에 할당
            for (const auto &motor : currentMotors)
            {
                if (motor->myName == "L_arm1" || motor->myName == "R_arm1" || motor->myName == "L_arm2" || motor->myName == "R_arm2" || motor->myName == "L_arm3" || motor->myName == "R_arm3")
                {
                    state.home = HomeSub::HomeTmotor;
                    state.homeTmotor = HomeTmotor::SelectHMotor;
                    break; // 내부 for 루프 탈출
                }
                else if (motor->myName == "L_wrist" || motor->myName == "R_wrist" || motor->myName == "maxonForTest")
                {
                    setMaxonMode("HMM");
                    usleep(50000);
                    MaxonEnable();
                    state.home = HomeSub::HomeMaxon;
                    state.homeMaxon = HomeMaxon::StartHoming;
                    break;
                }
            }
        }
        else
        {
            cout << "Homing Motor arr is Empty\n";
            bool AllInHome = true;
            for (const auto &motor : motors)
            {
                if (!motor.second->isHomed)
                {
                    AllInHome = false;
                }
            }
            if (!AllInHome)
            {
                state.home = HomeSub::SelectMotorByUser;
            }
            else
            {
                state.home = HomeSub::Done;
            }
        }
        // 이 부분에 호밍 작업이 모드 끝났을경우에 대한 처리를 할 것
        break;
    }
    case HomeSub::HomeTmotor:
    {
        HomeTmotor();
        break;
    }
    case HomeSub::HomeMaxon:
    {
        HomeMaxon();
        break;
    }
    case HomeSub::Done:
    {
        state.main = Main::Ideal;
        saveHomingInfoToFile();
        break;
    }
    }
}

void HomeManager::HomeTmotor()
{
    switch (state.homeTmotor.load())
    {
    case HomeTmotor::SelectHMotor:
    {
        sensorsBit.clear();
        firstPosition.clear();
        secondPosition.clear();
        positionDifference.clear();
        firstSensorTriggered.clear();
        TriggeredDone.clear();
        targetRadians.clear();
        midpoints.clear();
        directions.clear();
        tMotors.clear(); // 동시 homing 하는 모터배열 저장

        if (sensor.OpenDeviceUntilSuccess())
        {
            vector<shared_ptr<GenericMotor>> currentMotors = HomingMotorsArr.front();
            // 속도 제어 - 센서 방향으로 이동
            for (long unsigned int i = 0; i < currentMotors.size(); i++)
            {
                tMotors.push_back(dynamic_pointer_cast<TMotor>(currentMotors[i]));
                std::cout << "<< Homing for " << tMotors[i]->myName << " >>\n";

                sensorsBit.push_back(tMotors[i]->sensorBit);
                firstPosition.push_back(0.0f);
                secondPosition.push_back(0.0f);
                firstSensorTriggered.push_back(false);
                TriggeredDone.push_back(false);

                std::cout << "Moving " << tMotors[i]->myName << " to sensor location.\n";
            }
        }
        state.homeTmotor = HomeTmotor::MoveToSensor;
        break;
    }
    case HomeTmotor::MoveToSensor:
    {
        for (auto &motor_pair : motors)
        {
            if (shared_ptr<TMotor> motor = dynamic_pointer_cast<TMotor>(motor_pair.second))
            {
                int hMotoridx = -1;
                for (long unsigned int i = 0; i < tMotors.size(); i++)
                {
                    if (tMotors[i]->myName == motor->myName)
                    {
                        hMotoridx = i;
                    }
                }

                if (hMotoridx >= 0) // homing 하는 모터
                {
                    if (TriggeredDone[hMotoridx]) // 센서 위치에 도달했으면 대기
                    {
                        if (motor->isfixed == false)
                        {
                            motor->fixedPos = motor->currentPos;
                            motor->isfixed = true;
                        }
                        tmotorServocmd.comm_can_set_pos_spd(*motor, &motor->sendFrame, motor->fixedPos, motor->spd, motor->acl);
                    }
                    else // 센서 위치로 속도제어로 이동
                    {
                        motor->isfixed = false;
                        float speed = 500;
                        float add_speed = 300;

                        if (motor->myName == "L_arm2" || motor->myName == "R_arm2" || motor->myName == "L_arm3" || motor->myName == "R_arm3")
                        {
                            speed += add_speed;
                        }

                        if (motor->myName == "L_arm2" || motor->myName == "R_arm2" || motor->myName == "R_arm1")
                            speed *= (-1);

                        tmotorServocmd.comm_can_set_spd(*motor, &motor->sendFrame, speed * motor->cwDir);
                    }
                }
                else // homing 하지 않는 모터
                {
                    if (motor->isfixed == false)
                    {
                        motor->fixedPos = motor->currentPos;
                        motor->isfixed = true;
                    }
                    tmotorServocmd.comm_can_set_pos_spd(*motor, &motor->sendFrame, motor->fixedPos, motor->spd, motor->acl);
                }
                canManager.sendMotorFrame(motor);
            }
        }

        state.homeTmotor = HomeTmotor::SensorCheck;
        break;
    }
    case HomeTmotor::SensorCheck:
    {
        bool doneSensing = true;
        for (long unsigned int i = 0; i < tMotors.size(); i++)
        {
            if (!TriggeredDone[i])
                doneSensing = false;
        }

        if (doneSensing)
        {
            state.homeTmotor = HomeTmotor::FillBuf;
            sensor.closeDevice();
            break;
        }
        else
        {
            state.homeTmotor = HomeTmotor::MoveToSensor;
            for (long unsigned int i = 0; i < tMotors.size(); i++)
            {
                if (!TriggeredDone[i])
                {
                    bool sensorTriggered = ((sensor.ReadVal() >> sensorsBit[i]) & 1) != 0;

                    if (!firstSensorTriggered[i] && sensorTriggered)
                    {
                        // 첫 번째 센서 인식
                        firstSensorTriggered[i] = true;
                        firstPosition[i] = tMotors[i]->currentPos;
                        std::cout << tMotors[i]->myName << " first sensor position: " << firstPosition[i] << endl;
                    }
                    else if (firstSensorTriggered[i] && !sensorTriggered)
                    {
                        for (auto &motor_pair : motors)
                        {
                            if (shared_ptr<TMotor> motor = dynamic_pointer_cast<TMotor>(motor_pair.second))
                            {
                                tmotorServocmd.comm_can_set_pos_spd(*motor, &motor->sendFrame, motor->currentPos, motor->spd, motor->acl);
                                canManager.sendMotorFrame(motor);
                            }
                        }
                        // 센서 인식 해제
                        secondPosition[i] = tMotors[i]->currentPos;
                        std::cout << tMotors[i]->myName << " second sensor position: " << secondPosition[i] << endl;
                        TriggeredDone[i] = true;
                    }
                    else
                    {
                        usleep(5000); // 50ms
                    }
                }
            }
        }
        break;
    }
    case HomeTmotor::FillBuf:
    {
        for (long unsigned int i = 0; i < tMotors.size(); i++)
        {
            midpoints.push_back((secondPosition[i] - firstPosition[i]) / 2.0f);
            tMotors[i]->sensorLocation = ((secondPosition[i] + firstPosition[i]) / 2.0f);
            cout << tMotors[i]->myName << " midpoint position: " << midpoints[i] << endl;
            cout << tMotors[i]->myName << " Sensor location: " << tMotors[i]->sensorLocation << endl;

            float addPos = 0.0;
            if (tMotors[i]->myName == "R_arm1")
            {
                addPos = (-1) * (M_PI * 0.25) * tMotors[i]->cwDir;
            }
            else if (tMotors[i]->myName == "L_arm1")
            {
                addPos = (M_PI * 0.25) * tMotors[i]->cwDir;
            }

            targetRadians.push_back(addPos - midpoints[i]);
        }

        for (auto &motor_pair : motors)
        {
            if (shared_ptr<TMotor> motor = dynamic_pointer_cast<TMotor>(motor_pair.second))
                motor->clearCommandBuffer();
        }

        int totalSteps = 2000 / 5; // 2초
        for (int step = 1; step <= totalSteps; ++step)
        {
            for (auto &motor_pair : motors)
            {
                if (shared_ptr<TMotor> motor = dynamic_pointer_cast<TMotor>(motor_pair.second))
                {
                    int hMotoridx = -1;
                    for (long unsigned int i = 0; i < tMotors.size(); i++)
                    {
                        if (tMotors[i]->myName == motor->myName)
                        {
                            hMotoridx = i;
                        }
                    }

                    TMotorData newData;
                    if (hMotoridx >= 0) // homing 진행중인 모터
                    {
                        motor->isfixed = false;
                        newData.position = motor->currentPos + targetRadians[hMotoridx];
                    }
                    else // homing 진행중이지 않은 모터
                    {
                        if (motor->isfixed == false)
                        {
                            motor->fixedPos = motor->currentPos;
                            motor->isfixed = true;
                        }
                        newData.position = motor->fixedPos;
                    }
                    newData.spd = motor->spd;
                    newData.acl = motor->acl;
                    motor->commandBuffer.push(newData);
                }
            }
        }

        state.homeTmotor = HomeTmotor::CheckBuf;
        break;
    }
    case HomeTmotor::CheckBuf:
    {
        bool isEmpty = true;
        for (auto &motor_pair : motors)
        {
            if (shared_ptr<TMotor> motor = dynamic_pointer_cast<TMotor>(motor_pair.second))
            {
                if (!motor->commandBuffer.empty())
                {
                    isEmpty = false;
                    break;
                }
            }
        }

        if (isEmpty)
        {
            for (long unsigned int i = 0; i < tMotors.size(); i++)
            {
                tMotors[i]->isHomed = true;
            }

            state.homeTmotor = HomeTmotor::Done;
        }
        else
        {
            state.homeTmotor = HomeTmotor::SetCANFrame;
        }
        break;
    }
    case HomeTmotor::SetCANFrame:
    {
        for (auto &motor_pair : motors)
        {
            if (shared_ptr<TMotor> motor = dynamic_pointer_cast<TMotor>(motor_pair.second))
            {
                TMotorData tData = motor->commandBuffer.front();
                motor->commandBuffer.pop();
                tmotorServocmd.comm_can_set_pos_spd(*motor, &motor->sendFrame, tData.position, tData.spd, tData.acl);
            }
        }
        state.homeTmotor = HomeTmotor::SendCANFrameForZeroPos;
        break;
    }
    case HomeTmotor::SendCANFrameForZeroPos:
    {
        usleep(5000);
        for (auto &motor_pair : motors)
        {
            if (shared_ptr<TMotor> motor = dynamic_pointer_cast<TMotor>(motor_pair.second))
                canManager.sendMotorFrame(motor);
        }
        state.homeTmotor = HomeTmotor::CheckBuf;
        break;
    }
    case HomeTmotor::Done:
    {
        cout << "Now Im in Done state\n";
        HomingMotorsArr.erase(HomingMotorsArr.begin());
        state.home = HomeSub::GetSelectedMotor;
        break;
    }
    }
}

void HomeManager::HomeMaxon()
{
    switch (state.homeMaxon.load())
    {
    case HomeMaxon::StartHoming:
    {
        maxonMotors.clear();
        vector<shared_ptr<GenericMotor>> currentMotors = HomingMotorsArr.front();

        for (long unsigned int i = 0; i < currentMotors.size(); i++)
        {
            cout << "<< Homing for " << currentMotors[i]->myName << " >>\n";
            maxonMotors.push_back(dynamic_pointer_cast<MaxonMotor>(currentMotors[i]));

            maxoncmd.getStartHoming(*maxonMotors[i], &maxonMotors[i]->sendFrame);
            canManager.sendMotorFrame(maxonMotors[i]);
            usleep(50000);
        }

        maxoncmd.getSync(&maxonMotors[0]->sendFrame);
        canManager.sendMotorFrame(maxonMotors[0]);
        cout << "\nMaxon Homing Start!!\n";
        state.homeMaxon = HomeMaxon::CheckHomeStatus;
        break;
    }
    case HomeMaxon::CheckHomeStatus:
    {
        bool done = true;
        for (long unsigned int i = 0; i < maxonMotors.size(); i++)
        {
            if (!maxonMotors[i]->isHomed)
            {
                done = false;
                break;
            }
        }

        if (!done)
        {
            usleep(50000); // 50ms
            maxoncmd.getSync(&maxonMotors[0]->sendFrame);
            canManager.sendMotorFrame(maxonMotors[0]);
            for (long unsigned int i = 0; i < maxonMotors.size(); i++)
            {
                if (maxonMotors[i]->statusBit & 0x80)
                {
                    maxonMotors[i]->isHomed = true;
                }
            }

            for (auto &motor_pair : motors)
            {
                if (shared_ptr<TMotor> motor = dynamic_pointer_cast<TMotor>(motor_pair.second))
                {
                    TMotorData newData;
                    if (motor->isfixed == false)
                    {
                        motor->fixedPos = motor->currentPos;
                        motor->isfixed = true;
                    }
                    newData.position = motor->fixedPos;
                    newData.spd = motor->spd;
                    newData.acl = motor->acl;
                    // 위치속도제어 모드로 변경
                    tmotorServocmd.comm_can_set_pos_spd(*motor, &motor->sendFrame, newData.position, newData.spd, newData.acl);
                    canManager.sendMotorFrame(motor);
                }
            }
        }
        else
        {
            state.homeMaxon = HomeMaxon::Done;
        }
        break;
    }
    case HomeMaxon::Done:
    {
        state.home = HomeSub::GetSelectedMotor;
        canManager.setSocketBlock();
        setMaxonMode("CSP");
        canManager.setSocketNonBlock();
        HomingMotorsArr.erase(HomingMotorsArr.begin());
        break;
    }
    }
}