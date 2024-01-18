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

    SendLoopTask sendLoopTask(systemState, canManager, motors);
    RecieveLoopTask recieveLoopTask(systemState, canManager, motors);

    // Create Threads
    std::thread state_thread(drumRobot);
    std::thread send_thread(sendLoopTask);
    std::thread receive_thread(recieveLoopTask);

    /*
    SystemState systemState;
    StateMachine stateMachine(systemState, );

    // 스레드 생성 및 관리
    //std::thread task1(&StateMachine::someOperation, &stateMachine);

*/
    // Wait Threads
    state_thread.join();
    send_thread.join();
    receive_thread.join();
}
