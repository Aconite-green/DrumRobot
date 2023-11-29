#include <thread>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <memory>
#include <map>

#include "../include/Motor.hpp"
#include "../include/Task.hpp"
#include "../include/TaskUtility.hpp"
#include "../include/ChartHandler.hpp"
#include "../include/StateTask.hpp"
#include "../include/State.hpp"
#include "../include/SendLoopTask.hpp"
#include <atomic>
#include <QApplication>
#include <QtCharts/QLineSeries>

using namespace std;



int main(int argc, char *argv[])
{

    

    SystemState systemState;
    CanSocketUtils canUtils;
    
    StateTask stateTask(systemState);
    SendLoopTask sendLoopTask(systemState, canUtils);


    // 스레드 생성
    std::thread state_thread(stateTask);
    std::thread send_thread(sendLoopTask);
    std::thread receive_thread(receiveThread, std::ref(state));


    // 스레드 종료 대기
    state_thread.join();
    send_thread.join();
    receive_thread.join();

}
