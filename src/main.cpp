#include <thread>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <memory>
#include <map>

#include "../include/motors/Motor.hpp"

#include "../include/tasks/StateTask.hpp"
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

    queue<can_frame> sendBuffer;
    queue<can_frame> recieveBuffer;
    CanManager canManager(sendBuffer, recieveBuffer,motors);
    
    // Create Tasks for Threads
    StateTask stateTask(systemState, canManager, motors, sendBuffer, recieveBuffer);
    SendLoopTask sendLoopTask(systemState, canManager, motors, sendBuffer, recieveBuffer);
    RecieveLoopTask recieveLoopTask(systemState, canManager, motors, recieveBuffer);

    // Create Threads
    std::thread state_thread(stateTask);
    std::thread send_thread(sendLoopTask);
    std::thread receive_thread(recieveLoopTask);

    // Wait Threads
    state_thread.join();
    send_thread.join();
    receive_thread.join();
}
