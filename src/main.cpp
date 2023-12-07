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
    queue<can_frame> sendBuffer;
    map<string, shared_ptr<TMotor>> tmotors;

    // Create Tasks for Threads
    StateTask stateTask(systemState);
    SendLoopTask sendLoopTask(systemState, canUtils, tmotors, sendBuffer);
    RecieveLoopTask recieveLoopTask(systemState, canUtils);

    // Create Threads
    std::thread state_thread(stateTask);
    std::thread send_thread(sendLoopTask);
    std::thread receive_thread(recieveLoopTask);

    // Wait Threads
    state_thread.join();
    send_thread.join();
    receive_thread.join();
}
