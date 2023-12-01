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

    SystemState systemState;
    CanSocketUtils canUtils;
    
    StateTask stateTask(systemState);
    SendLoopTask sendLoopTask(systemState, canUtils);
    RecieveLoopTask recieveLoopTask(systemState, canUtils);

    // 스레드 생성
    std::thread state_thread(stateTask);
    std::thread send_thread(sendLoopTask);
    std::thread receive_thread(recieveLoopTask);


    // 스레드 종료 대기
    state_thread.join();
    send_thread.join();
    receive_thread.join();

}
