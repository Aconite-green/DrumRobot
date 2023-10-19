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
#include "../include/PathManager.hpp"
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
    std::vector<std::string> ifnames = {"can0", "can1"};
    CanSocketUtils canUtils(ifnames);
    // Motor Declariration
    std::map<std::string, std::shared_ptr<TMotor>> tmotors;
    tmotors["1_waist"] = std::make_shared<TMotor>(0x007, "AK10_9", "can1");

    tmotors["2_R_arm1"] = std::make_shared<TMotor>(0x001, "AK70_10", "can1");
    tmotors["3_L_arm1"] = std::make_shared<TMotor>(0x002, "AK70_10", "can1");
    tmotors["4_R_arm2"] = std::make_shared<TMotor>(0x003, "AK70_10", "can1");

    // tmotors["5_R_arm3"] = std::make_shared<TMotor>(0x004, "AK70_10", "can0");
    // tmotors["6_L_arm2"] = std::make_shared<TMotor>(0x005, "AK70_10", "can0");
    // tmotors["7_L_arm3"] = std::make_shared<TMotor>(0x006, "AK70_10", "can0");

    std::map<std::string, std::shared_ptr<MaxonMotor>> maxonMotors;
    maxonMotors["a_maxon"] = std::make_shared<MaxonMotor>(0x001,
                                                          std::vector<uint32_t>{0x201, 0x301},
                                                          std::vector<uint32_t>{0x181},
                                                          "can0");
    maxonMotors["b_maxon"] = std::make_shared<MaxonMotor>(0x002,
                                                          std::vector<uint32_t>{0x202, 0x302},
                                                          std::vector<uint32_t>{0x182},
                                                          "can0");

    // Tasks For Threads
    ActivateControlTask activateTask(tmotors, maxonMotors, canUtils.getSockets());
    MotorPathTask pathTask(tmotors, maxonMotors);
    MotorSignalSendTask sendTask(tmotors, maxonMotors, canUtils.getSockets(), paused, stop);
    MotorResponseReadTask readTask(tmotors, maxonMotors, canUtils.getSockets(), paused, stop);
    // SensorSignalReadTask sensorTask(tmotors, paused, stop);
    DeactivateControlTask deactivateTask(tmotors, maxonMotors, canUtils.getSockets());

    PathManager PathManager(tmotors);

    ThreadLoopTask threadLoopTask(activateTask, deactivateTask, pathTask, PathManager, sendTask, readTask, sendBuffer, receiveBuffer, stop);
    std::thread threadLoop(threadLoopTask);
    threadLoop.join();

    return 0;
}
