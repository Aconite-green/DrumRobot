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
#include "../include/ThreadLoopTask.hpp"
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
    CanSocketUtils canUtils(ifnames);

    // Motor Declariration
    std::map<std::string, std::shared_ptr<TMotor>> tmotors;
    tmotors["arm1"] = std::make_shared<TMotor>(0x02, "AK70_10", "can0");
    tmotors["arm2"] = std::make_shared<TMotor>(0x01, "AK70_10", "can0");
    tmotors["waist"] = std::make_shared<TMotor>(0x03, "AK10_9", "can0");

    std::map<std::string, std::shared_ptr<MaxonMotor>> maxonMotors;
    // maxonMotors["a"] = std::make_shared<MaxonMotor>(0x01, std::vector<uint32_t>{0x201, 0x301}, "can0");

    // Tasks For Threads
    ActivateControlTask activateTask(tmotors, maxonMotors, canUtils.getSockets());
    MotorPathTask pathTask(tmotors);
    MotorSignalSendTask sendTask(tmotors, maxonMotors, canUtils.getSockets(), paused, stop);
    MotorResponseReadTask readTask(tmotors, maxonMotors, canUtils.getSockets(), paused, stop);
    // SensorSignalReadTask sensorTask(tmotors, paused, stop);
    DeactivateControlTask deactivateTask(tmotors, maxonMotors, canUtils.getSockets());

    ThreadLoopTask threadLoopTask(activateTask, deactivateTask, pathTask, sendTask, readTask, sendBuffer, receiveBuffer, stop);
    std::thread threadLoop(threadLoopTask);
    threadLoop.join();

    return 0;
}
