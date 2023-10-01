#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include "../include/MotorPathTask.hpp"
#include "../include/MotorSignalSendTask.hpp"
#include "../include/SharedBuffer.hpp"
#include "../include/MotorResponseReadTask.hpp"
#include "../include/SensorSignalReadTask.hpp"
#include "../include/InitializeTask.hpp"

std::mutex mtx;
std::condition_variable cv;
bool initialAndPathDone = false;
int numMotor = 0;

void executeTasks(SharedBuffer<can_frame>& sendBuffer, SharedBuffer<can_frame>& receiveBuffer, SharedBuffer<int>& sensorBuffer) {
    std::vector<std::thread> threads;
    threads.emplace_back(MotorSignalSendTask(), std::ref(sendBuffer));
    threads.emplace_back(MotorResponseReadTask(), std::ref(receiveBuffer));
    threads.emplace_back(SensorSignalReadTask(), std::ref(sensorBuffer));
    for (auto& th : threads) {
        th.join();
    }
}

int main() {
    SharedBuffer<can_frame> sendBuffer;
    SharedBuffer<can_frame> receiveBuffer;
    SharedBuffer<int> sensorBuffer;

    InitializeTask initialTask;
    MotorPathTask pathTask;
    initialTask();
    pathTask(sendBuffer);
{
    std::unique_lock<std::mutex> lock(mtx);
    initialAndPathDone = true;
    cv.notify_all();
}
    std::string userInput;
    while (true) {
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [] { return initialAndPathDone; });
        }

        std::cout << "Enter 'run' to continue or 'exit' to quit: ";
        std::cin >> userInput;
        std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);

        if (userInput == "exit") {
            break;
        } else if (userInput == "run") {
            executeTasks(sendBuffer, receiveBuffer, sensorBuffer);
        }
    }
    return 0;
}
