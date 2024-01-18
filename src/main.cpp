#include <thread>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <memory>
#include <map>

#include "../include/motors/Motor.hpp"
#include "../include/managers/PathManager.hpp"
#include "../include/managers/CanManager.hpp"
#include "../include/managers/TestManager.hpp"
#include "../include/managers/HomeManager.hpp"

#include "../include/tasks/DrumRobot.hpp"
#include "../include/tasks/SystemState.hpp"
#include "../include/tasks/SendLoopTask.hpp"
#include "../include/tasks/RecieveLoopTask.hpp"
#include <atomic>

using namespace std;

int main(int argc, char *argv[])
{

    // Create Share Resource
    SystemState systemState;
    std::map<std::string, std::shared_ptr<GenericMotor>> motors;

    CanManager canManager(motors);
    PathManager pathManager(systemState, canManager, motors);
    TestManager testManager(systemState, canManager, motors);
    HomeManager homeManager(systemState, canManager, motors);

    DrumRobot drumRobot(systemState, canManager, pathManager, homeManager, testManager, motors);

    // Create Threads
    std::thread stateThread(&DrumRobot::stateMachine, &drumRobot);
    std::thread sendThread(&DrumRobot::sendLoopForThread, &drumRobot);
    std::thread receiveThread(&DrumRobot::recvLoopForThread, &drumRobot);

    // Wait Threads
    stateThread.join();
    sendThread.join();
    receiveThread.join();
}
