#include <thread>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <memory>
#include <map>
#include <atomic>

#include "../include/motors/Motor.hpp"
#include "../include/managers/PathManager.hpp"
#include "../include/managers/CanManager.hpp"
#include "../include/managers/TestManager.hpp"
#include "../include/managers/HomeManager.hpp"
#include "../include/tasks/DrumRobot.hpp"
#include "../include/tasks/SystemState.hpp"


using namespace std;

// 스레드 우선순위 설정 함수
bool setThreadPriority(std::thread &th, int priority, int policy = SCHED_FIFO)
{
    sched_param sch_params;
    sch_params.sched_priority = priority;
    if (pthread_setschedparam(th.native_handle(), policy, &sch_params))
    {
        std::cerr << "Failed to set Thread scheduling : " << std::strerror(errno) << std::endl;
        return false;
    }
    return true;
}


int main(int argc, char *argv[])
{

    // Create Share Resource
    State state;
    std::map<std::string, std::shared_ptr<GenericMotor>> motors;

    CanManager canManager(motors);
    PathManager pathManager(state, canManager, motors);
    TestManager testManager(state, canManager, motors);
    HomeManager homeManager(state, canManager, motors);

    DrumRobot drumRobot(state, canManager, pathManager, homeManager, testManager, motors);

    // Create Threads
    std::thread stateThread(&DrumRobot::stateMachine, &drumRobot);
    std::thread sendThread(&DrumRobot::sendLoopForThread, &drumRobot);
    std::thread receiveThread(&DrumRobot::recvLoopForThread, &drumRobot);

    // Threads Priority Settings
    if (!setThreadPriority(sendThread, 3))
    {
        std::cerr << "Error setting priority for sendCanFrame" << std::endl;
        return -1;
    }
    if (!setThreadPriority(receiveThread, 2))
    {
        std::cerr << "Error setting priority for receiveCanFrame" << std::endl;
        return -1;
    }
    if (!setThreadPriority(stateThread, 1))
    {
        std::cerr << "Error setting priority for stateMachine" << std::endl;
        return -1;
    }

    // Wait Threads
    stateThread.join();
    sendThread.join();
    receiveThread.join();
}
