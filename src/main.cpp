#include <thread>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <memory>
#include <map>

#include "../include/Motor.hpp"
#include "../include/MotorPathTask.hpp"
#include "../include/MotorSignalSendTask.hpp"
#include "../include/SharedBuffer.hpp"
#include "../include/MotorResponseReadTask.hpp"
#include "../include/SensorSignalReadTask.hpp"
#include "../include/InitializeTask.hpp"

int main()
{

    // Buffer
    SharedBuffer<can_frame> sendBuffer;
    SharedBuffer<can_frame> receiveBuffer;
    SharedBuffer<int> sensorBuffer;

    // Motor Declariration
    std::map<std::string, std::shared_ptr<TMotor>> tmotors;
    tmotors["waist"] = std::make_shared<TMotor>(1, "AK10_9");

    // Canport Initialization
    std::vector<std::string> ifnames = {"can0", "can1"};
    CanSocketUtils canUtils(ifnames);

    // Tasks For Threads
    InitializeTask initialTask(tmotors, canUtils.getSockets());
    MotorPathTask pathTask(tmotors);
    MotorSignalSendTask sendTask(tmotors, canUtils.getSockets());
    MotorResponseReadTask readTask(tmotors, canUtils.getSockets());
    //SensorSignalReadTask sensorTask;


    // Begain Operation
    initialTask();
    pathTask(sendBuffer);

    std::string userInput;
    while (true)
    {
        std::cout << "Enter 'run' to continue or 'exit' to quit: ";
        std::cin >> userInput;
        std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);

        if (userInput == "exit")
        {
            break;
        }
        else if (userInput == "run")
        {
            // Task 실행을 위한 스레드 생성 및 실행
            std::thread sendThread(sendTask, std::ref(sendBuffer));
            std::thread readThread(readTask, std::ref(receiveBuffer));
            // std::thread sensorThread(sensorTask, std::ref(sensorBuffer), std::ref(tmotors));

            // 모든 스레드가 종료될 때까지 대기
            sendThread.join();
            readThread.join();
            // sensorThread.join();
        }
    }

    return 0;
}
