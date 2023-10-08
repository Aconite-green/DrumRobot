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
#include "../include/ActivateControlTask.hpp"
#include "../include/DeactivateControlTask.hpp"
#include <atomic>

int main()
{

    // Buffer
    SharedBuffer<can_frame> sendBuffer;
    SharedBuffer<can_frame> receiveBuffer;
    SharedBuffer<int> sensorBuffer;
    std::atomic<bool> paused(false);
    std::atomic<bool> stop(false);

    // Canport Initialization
    std::vector<std::string> ifnames = {"can0"};
    //CanSocketUtils canUtils(ifnames);

    // Motor Declariration
    std::map<std::string, std::shared_ptr<TMotor>> tmotors;
    tmotors["arm1"] = std::make_shared<TMotor>(0x02, "AK70_10", "can0");

    std::map<std::string, std::shared_ptr<MaxonMotor>> maxonMotors;
    maxonMotors["a"] = std::make_shared<MaxonMotor>(0x01, std::vector<uint32_t>{0x201, 0x301}, "can0");



    // Tasks For Threads
    //ActivateControlTask activateTask(tmotors, maxonMotors, canUtils.getSockets());
    MotorPathTask pathTask(tmotors);
    //MotorSignalSendTask sendTask(tmotors, canUtils.getSockets(), paused, stop);
    //MotorResponseReadTask readTask(tmotors, canUtils.getSockets(), paused, stop);
    //SensorSignalReadTask sensorTask(tmotors, paused, stop);
    //DeactivateControlTask deactivateTask(tmotors, maxonMotors, canUtils.getSockets());

    // Begain Operation
    //activateTask();
    pathTask(sendBuffer);
    sendBuffer.print_buffer();
/*
    
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
                //std::thread readThread(readTask, std::ref(receiveBuffer));
                //std::thread sensorThread(sensorTask, std::ref(sensorBuffer));

                // 모든 스레드가 종료될 때까지 대기
                sendThread.join();
                //readThread.join();
                //sensorThread.join();
            }
        }
    

    deactivateTask();
    receiveBuffer.print_buffer();
    
    */
   return 0;
}
