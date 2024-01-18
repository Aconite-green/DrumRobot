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

    CanManager canManager(motors);
    
    // Create Tasks for Threads
    StateTask stateTask(systemState, canManager, motors);
    SendLoopTask sendLoopTask(systemState, canManager, motors);
    RecieveLoopTask recieveLoopTask(systemState, canManager, motors);

    // Create Threads
    std::thread state_thread(stateTask);
    std::thread send_thread(sendLoopTask);
    std::thread receive_thread(recieveLoopTask);

    // Wait Threads
    state_thread.join();
    send_thread.join();
    receive_thread.join();
}
