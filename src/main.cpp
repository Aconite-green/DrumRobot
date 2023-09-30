#include <thread>
#include <vector>
#include "../include/MotorPathTask.hpp"
#include "../include/MotorSignalSendTask.hpp"
#include "../include/SharedBuffer.hpp"
#include "../include/MotorResponseReadTask.hpp"
#include "../include/SensorSignalReadTask.hpp"
#include "../include/InitializeTask.hpp"

const int THREAD_NUM = 4;

int main() {
    SharedBuffer sendBuffer, recieveBuffer, sensorBuffer;

    std::vector<std::thread> threads;

    // Create Thread for InitializeTask
    InitializeTask initailTask;
    threads.push_back(std::thread(initailTask, std::ref(sendBuffer/*여기에 버퍼가 담길 필요는 없을 것 같음*/)));

    // MotorPathTask용 스레드 생성
    MotorPathTask pathTask;
    threads.push_back(std::thread(pathTask, std::ref(sendBuffer)));

    // MotorSignalSendTask용 스레드 생성음
    MotorSignalSendTask sendTask;
    threads.push_back(std::thread(sendTask, std::ref(sendBuffer)));

    // Create Thread for MotorResponseReadTask 
    MotorResponseReadTask readTask;
    threads.push_back(std::thread(readTask, std::ref(recieveBuffer)));

    // Create Thread for MotorResponseReadTask 
    SensorSignalReadTask sensorTask;
    threads.push_back(std::thread(sensorTask, std::ref(sensorBuffer)));

   


    // 모든 스레드가 완료될 때까지 기다림
    for (auto& th : threads) {
        th.join();
    }

    return 0;
}
