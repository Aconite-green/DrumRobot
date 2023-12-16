#include <thread>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <memory>
#include <map>

#include "../include/Motor.hpp"

#include "../include/StateTask.hpp"
#include "../include/SystemState.hpp"
#include "../include/SendLoopTask.hpp"
#include "../include/RecieveLoopTask.hpp"
#include <atomic>

using namespace std;

int main(int argc, char *argv[])
{

    // Create Share Resource

    SystemState systemState;
    CanSocketUtils canUtils;
    map<string, shared_ptr<TMotor>> tmotors;
    map<std::string, std::shared_ptr<MaxonMotor>> maxonMotors;
    queue<can_frame> sendBuffer;
    queue<can_frame> recieveBuffer;

    // Create Tasks for Threads
    StateTask stateTask(systemState, canUtils, tmotors, maxonMotors);
    SendLoopTask sendLoopTask(systemState, canUtils, tmotors, maxonMotors, sendBuffer);
    RecieveLoopTask recieveLoopTask(systemState, canUtils, tmotors, maxonMotors, recieveBuffer);

    // Create Threads
    std::thread state_thread(stateTask);
    std::thread send_thread(sendLoopTask);
    std::thread receive_thread(recieveLoopTask);

    // Wait Threads
    state_thread.join();
    send_thread.join();
    receive_thread.join();
}
