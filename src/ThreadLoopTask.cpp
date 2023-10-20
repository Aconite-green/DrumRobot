#include "../include/ThreadLoopTask.hpp"

ThreadLoopTask::ThreadLoopTask(ActivateControlTask &activateTask_,
                               DeactivateControlTask &deactivateTask_,
                               MotorPathTask &pathTask_,
                               PathManager &pathManagerTask,
                               SineSignalSendTask &tuning,
                               MotorSignalSendTask &sendTask_,
                               MotorResponseReadTask &readTask_,
                               SharedBuffer<can_frame> &sendBuffer_,
                               SharedBuffer<can_frame> &receiveBuffer_,
                               std::atomic<bool> &stop)
    : activateTask(activateTask_),
      deactivateTask(deactivateTask_),
      pathTask(pathTask_),
      pathManagerTask(pathManagerTask),
      tuning(tuning),
      sendTask(sendTask_),
      readTask(readTask_),
      sendBuffer(sendBuffer_),
      receiveBuffer(receiveBuffer_),
      stop(stop)
{
}

void ThreadLoopTask::operator()()
{
    // Begin Operation
    activateTask();

    std::string userInput;
    while (true)
    {
        std::cout << "Enter 'run','exit','sinewave','tuning' : ";
        std::cin >> userInput;
        std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);

        if (userInput == "exit")
        {
            break;
        }
        else if (userInput == "sine")
        {

            std::thread pathThread(pathTask, std::ref(sendBuffer));
            std::thread sendThread(sendTask, std::ref(sendBuffer));
            std::thread readThread(readTask, std::ref(receiveBuffer));

            pathThread.join();
            sendThread.join();
            readThread.join();
        }
        else if (userInput == "run")
        {

            std::thread pathThread(pathManagerTask, std::ref(sendBuffer));
            std::thread sendThread(sendTask, std::ref(sendBuffer));
            std::thread readThread(readTask, std::ref(receiveBuffer));

            pathThread.join();
            sendThread.join();
            readThread.join();
        }
        else if (userInput == "tuning"){
            tuning();
        }
    }

    deactivateTask();
}
