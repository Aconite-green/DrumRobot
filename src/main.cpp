#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <algorithm> // for std::transform
#include <cctype>    // for std::tolower

// 사용자가 정의한 헤더파일들
#include "../include/MotorPathTask.hpp"
#include "../include/MotorSignalSendTask.hpp"
#include "../include/SharedBuffer.hpp"
#include "../include/MotorResponseReadTask.hpp"
#include "../include/SensorSignalReadTask.hpp"
#include "../include/InitializeTask.hpp"

// 초기화 및 경로 작업이 끝났는지 판단하기 위한 전역 변수와 동기화 객체
std::mutex mtx;                  // 뮤텍스 (상호 배제를 위한)
std::condition_variable cv;      // 조건 변수
bool initialAndPathDone = false; // 초기화와 경로 작업이 끝났는지 여부

// initialTask와 pathTask를 실행하고, 끝났음을 알리는 함수
void runInitialAndPath(InitializeTask &initialTask, MotorPathTask &pathTask, SharedBuffer &sendBuffer)
{
    initialTask();        // 모터 초기화 작업
    pathTask(sendBuffer); // 경로 생성 작업

    // 끝났음을 알리기 위해 뮤텍스를 잠그고, 조건 변수를 통해 알림
    std::unique_lock<std::mutex> lock(mtx);
    initialAndPathDone = true;
    cv.notify_all();
}

int main()
{
    SharedBuffer sendBuffer, receiveBuffer, sensorBuffer; // 공유 버퍼 선언

    // initialTask와 pathTask 객체 생성
    InitializeTask initialTask;
    MotorPathTask pathTask;

    // initialTask와 pathTask를 실행할 스레드 생성
    std::thread initAndPathThread(runInitialAndPath, std::ref(initialTask), std::ref(pathTask), std::ref(sendBuffer));
    initAndPathThread.join(); // 스레드가 끝날 때까지 기다림

    // initialTask와 pathTask가 끝났는지 확인
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, []
                { return initialAndPathDone; });
    }

    std::string userInput;
    while (true)
    {
        std::vector<std::thread> threads; // 나머지 작업을 위한 스레드들을 저장할 벡터

        // MotorSignalSendTask용 스레드 생성
        MotorSignalSendTask sendTask;
        threads.push_back(std::thread(sendTask, std::ref(sendBuffer)));

        // MotorResponseReadTask용 스레드 생성
        MotorResponseReadTask readTask;
        threads.push_back(std::thread(readTask, std::ref(receiveBuffer)));

        // SensorSignalReadTask용 스레드 생성
        SensorSignalReadTask sensorTask;
        threads.push_back(std::thread(sensorTask, std::ref(sensorBuffer)));

        // 모든 스레드가 완료될 때까지 기다림
        for (auto &th : threads)
        {
            th.join();
        }

        std::cout << "Enter 'run' to continue or 'exit' to quit: ";
        std::cin >> userInput;

        std::transform(userInput.begin(), userInput.end(), userInput.begin(), [](unsigned char c)
                       { return std::tolower(c); });
        if (userInput == "exit")
        {
            break;
        }
        else if (userInput == "run")
        {
            continue;
        }
    }

    return 0; // 프로그램 종료
}
