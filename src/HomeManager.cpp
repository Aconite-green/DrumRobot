#include "../include/managers/HomeManager.hpp"

HomeManager::HomeManager(State &stateRef,
                         CanManager &canManagerRef,
                         std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef)
{
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
        state.home = HomeSub::Done;
        state.main = Main::Ideal;
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

            if (canManager.recvToBuff(motor, canManager.maxonCnt))
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

            if (canManager.recvToBuff(motor, canManager.maxonCnt))
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

            if (canManager.recvToBuff(motor, canManager.maxonCnt))
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

void HomeManager::FixMotorPosition(std::shared_ptr<GenericMotor> &motor)
{
    struct can_frame frame;

    canManager.checkConnection(motor);

    if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
    {
        tmotorcmd.parseSendCommand(*tMotor, &frame, motor->nodeId, 8, motor->currentPos, 0, 450, 1, 0);
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

void HomeManager::SendHomeProcess()
{
    switch (state.home.load())
    {
    case HomeSub::SelectMotorByUser:
    {
        displayHomingStatus();

        std::cout << "Enter the name of the motor to home, or 'all' to home all motors: ";
        std::cin >> motorName;
        state.home = HomeSub::MakeHomingOrderBuf;
        break;
    }
    case HomeSub::MakeHomingOrderBuf:
    {
        if (motorName == "all")
        {
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

                HomingMotorsArr.push_back(temp);
            }
        }
        else if (motors.find(motorName) != motors.end() && !motors[motorName]->isHomed)
        {
            vector<shared_ptr<GenericMotor>> temp;
            temp.push_back(motors[motorName]);
            HomingMotorsArr.push_back(temp);
        }
        else
        {
            std::cout << "Motor not found or already homed: " << motorName << std::endl;
        }

        state.home = HomeSub::GetSelectedMotor;
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
                    state.homeTmotor = HomeTmotor::MoveToSensor;
                    break; // 내부 for 루프 탈출
                }
                else if (motor->myName == "L_wrist" || motor->myName == "R_wrist" || motor->myName == "maxonForTest")
                {
                    canManager.setSocketBlock();
                    setMaxonMode("HMM");
                    MaxonEnable();
                    canManager.setSocketNonBlock();
                    state.homeMaxon = HomeMaxon::StartHoming;
                    state.home = HomeSub::HomeMaxon;
                    break; // 내부 for 루프 탈출
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

        HomeTmotor_test();
        break;
    case HomeSub::HomeMaxon:

        HomeMaxon_test();
        break;
    case HomeSub::Done:
        state.main = Main::Ideal;
        cout << "Press Enter\n";
        getchar();
        break;
    }
}

void HomeManager::HomeTmotor_test()
{
    switch (state.homeTmotor.load())
    {
    case HomeTmotor::MoveToSensor:
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
        degrees.clear();
        tMotors.clear();

        if (sensor.OpenDeviceUntilSuccess())
        {

            vector<shared_ptr<GenericMotor>> currentMotors = HomingMotorsArr.front();

            // 속도 제어 - 센서 방향으로 이동
            for (long unsigned int i = 0; i < currentMotors.size(); i++)
            {
                std::cout << "<< Homing for " << currentMotors[i]->myName << " >>\n";
                tMotors.push_back(dynamic_pointer_cast<TMotor>(currentMotors[i]));

                double initialDirection;
                if (tMotors[i]->myName == "L_arm2" || tMotors[i]->myName == "R_arm2")
                    initialDirection = (-0.2) * tMotors[i]->cwDir;
                else
                    initialDirection = 0.2 * tMotors[i]->cwDir;

                double additionalTorque = 0.0;
                if (tMotors[i]->myName == "L_arm2" || tMotors[i]->myName == "R_arm2")
                    additionalTorque = currentMotors[i]->cwDir * (-3.0);
                else if (tMotors[i]->myName == "L_arm3" || tMotors[i]->myName == "R_arm3")
                    additionalTorque = tMotors[i]->cwDir * (2.1);

                sensorsBit.push_back(tMotors[i]->sensorBit);
                firstPosition.push_back(0.0f);
                secondPosition.push_back(0.0f);
                firstSensorTriggered.push_back(false);
                TriggeredDone.push_back(false);

                std::cout << "Moving " << tMotors[i]->myName << " to sensor location.\n";

                tmotorcmd.parseSendCommand(*tMotors[i], &tMotors[i]->sendFrame, tMotors[i]->nodeId, 8, 0, initialDirection, 0, 4.5, additionalTorque);
                canManager.sendMotorFrame(tMotors[i]);
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
                        // 센서 인식 해제
                        secondPosition[i] = tMotors[i]->currentPos;
                        std::cout << tMotors[i]->myName << " second sensor position: " << secondPosition[i] << endl;
                        TriggeredDone[i] = true;
                    }
                    else
                    {
                        usleep(5000);
                        canManager.sendMotorFrame(tMotors[i]);
                    }
                }
                else
                {
                    tmotorcmd.parseSendCommand(*tMotors[i], &tMotors[i]->sendFrame, tMotors[i]->nodeId, 8, secondPosition[i], 0, tMotors[i]->Kp, 2.5, 0);
                    canManager.sendMotorFrame(tMotors[i]);
                }
            }
        }
        break;
    }
    case HomeTmotor::FillBuf:
    {

        bool isForZero = true;
        for (long unsigned int i = 0; i < tMotors.size(); i++)
        {
            if ((tMotors[i]->giveOffset == true && tMotors[i]->isHomed))
            {
                isForZero = false;
                break;
            }
        }

        if (isForZero)
        {
            for (long unsigned int i = 0; i < tMotors.size(); i++)
            {
                midpoints.push_back(abs((secondPosition[i] - firstPosition[i]) / 2.0f));
                tMotors[i]->sensorLocation = ((secondPosition[i] + firstPosition[i]) / 2.0f);
                cout << tMotors[i]->myName << " midpoint position: " << midpoints[i] << endl;
                cout << tMotors[i]->myName << " Sensor location: " << tMotors[i]->sensorLocation << endl;
            }
            for (long unsigned int i = 0; i < tMotors.size(); i++)
            {
                if (tMotors[i]->myName == "L_arm2" || tMotors[i]->myName == "R_arm2")
                {
                    degrees.push_back(-30.0);
                    midpoints[i] = midpoints[i] * (-1);
                }
                else
                {
                    degrees.push_back(10.0);
                }
                directions.push_back(-tMotors[i]->cwDir);
            }

            for (long unsigned int i = 0; i < tMotors.size(); i++)
            {
                targetRadians.push_back((degrees[i] * M_PI / 180.0 + midpoints[i]) * directions[i]);
                tMotors[i]->clearCommandBuffer();
            }
        }
        else
        {
            cout << "Turning For offset\n";
            degrees.clear();
            midpoints.clear();
            directions.clear();
            targetRadians.clear();

            for (long unsigned int i = 0; i < tMotors.size(); i++)
            {
                degrees[i] = 0.0;
                directions[i] = tMotors[i]->cwDir;
                midpoints[i] = 0.0;
            }

            for (long unsigned int i = 0; i < tMotors.size(); i++)
            {
                degrees[i] = tMotors[i]->homeOffset;
            }

            for (long unsigned int i = 0; i < tMotors.size(); i++)
            {
                targetRadians.push_back((degrees[i] * M_PI / 180.0 + midpoints[i]) * directions[i]);
                tMotors[i]->clearCommandBuffer();
                tMotors[i]->giveOffset = false;
            }
        }

        int totalSteps = 4000 / 5;
        for (int step = 1; step <= totalSteps; ++step)
        {
            for (long unsigned int i = 0; i < tMotors.size(); i++)
            {
                double targetPosition = targetRadians[i] * (static_cast<double>(step) / totalSteps) + tMotors[i]->currentPos;
                TMotorData newData;
                newData.position = targetPosition;
                newData.velocity = 0;

                tMotors[i]->commandBuffer.push(newData);
            }
        }

        totalSteps = 500 / 5;
        for (int step = 1; step <= totalSteps; ++step)
        {
            for (long unsigned int i = 0; i < tMotors.size(); i++)
            {
                double targetPosition = targetRadians[i] + tMotors[i]->currentPos;
                TMotorData newData;
                newData.position = targetPosition;
                newData.velocity = 0;

                tMotors[i]->commandBuffer.push(newData);
            }
        }
        state.homeTmotor = HomeTmotor::CheckBuf;
        break;
    }

    case HomeTmotor::CheckBuf:
    {
        bool isEmpty = true;
        for (long unsigned int i = 0; i < tMotors.size(); i++)
        {
            if (!tMotors[i]->commandBuffer.empty())
            {
                isEmpty = false;
                break;
            }
        }

        if (isEmpty)
        {
            for (long unsigned int i = 0; i < tMotors.size(); i++)
            {
                if (tMotors[i]->isHomed)
                {
                    cout << "Buf for offset is Empty and now im going to done state\n";
                    state.homeTmotor = HomeTmotor::Done;
                }
                else
                {
                    state.homeTmotor = HomeTmotor::SetZero;
                }
            }
        }
        else
        {
            state.homeTmotor = HomeTmotor::SafetyCheck;
        }
        break;
    }
    case HomeTmotor::SafetyCheck:
    {
        bool isSafe = true;
        for (long unsigned int i = 0; i < tMotors.size(); i++)
        {
            TMotorData tData = tMotors[i]->commandBuffer.front();
            tMotors[i]->commandBuffer.pop();
            if (abs(tMotors[i]->currentPos - tData.position) > 0.2)
            {
                std::cout << "Error Druing Homing For" << tMotors[i]->myName << " (Pos Diff)\n";
                isSafe = false;
                tmotorcmd.getQuickStop(*tMotors[i], &tMotors[i]->sendFrame);
                canManager.sendMotorFrame(tMotors[i]);
                usleep(5000);
                tmotorcmd.getExit(*tMotors[i], &tMotors[i]->sendFrame);
                canManager.sendMotorFrame(tMotors[i]);
                break;
            }
            else
            {
                tmotorcmd.parseSendCommand(*tMotors[i], &tMotors[i]->sendFrame, tMotors[i]->nodeId, 8, tData.position, tData.velocity, tMotors[i]->Kp, tMotors[i]->Kd, 0.0);
            }
        }

        if (isSafe)
        {
            state.homeTmotor = HomeTmotor::SendCANFrameForZeroPos;
        }
        else
        {
            std::cout << "Go to Error State\n";
            state.main = Main::Error;
        }
        break;
    }
    case HomeTmotor::SendCANFrameForZeroPos:
    {
        usleep(5000);
        for (long unsigned int i = 0; i < tMotors.size(); i++)
        {
            canManager.sendMotorFrame(tMotors[i]);
        }

        state.homeTmotor = HomeTmotor::CheckBuf;
        break;
    }
    case HomeTmotor::SetZero:
    {
        // for (long unsigned int i = 0; i < tMotors.size(); i++)
        //{
        //     tmotorcmd.parseSendCommand(*tMotors[i], &tMotors[i]->sendFrame, tMotors[i]->nodeId, 8, 0, 0, 0, 5, 0);
        //     if (canManager.sendMotorFrame(tMotors[i]))
        //     {
        //         cout << "Set " << tMotors[i]->myName << " speed Zero.\n";
        //     }
        //     usleep(50000);
        //     tmotorcmd.getZero(*tMotors[i], &tMotors[i]->sendFrame);
        //     if (canManager.sendMotorFrame(tMotors[i]))
        //     {
        //         cout << "Set Zero.\n";
        //     }
        // }
        // cout << "Sleeping For 5 sec\n";
        // sleep(5);
        for (long unsigned int i = 0; i < tMotors.size(); i++)
        {
            cout << tMotors[i]->myName << " Position : " << tMotors[i]->currentPos << "\n";
            tMotors[i]->isHomed = true;
        }

        bool noHomeOffset = true;
        for (long unsigned int i = 0; i < tMotors.size(); i++)
        {
            if (tMotors[i]->giveOffset == true)
            {
                noHomeOffset = false;
                break;
            }
        }

        if (noHomeOffset)
        {
            state.homeTmotor = HomeTmotor::Done;
        }
        else
        {
            state.homeTmotor = HomeTmotor::FillBuf;
        }
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

void HomeManager::HomeMaxon_test()
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
        MaxonDisable();
        canManager.setSocketNonBlock();
        HomingMotorsArr.erase(HomingMotorsArr.begin());
        break;
    }
    }
}