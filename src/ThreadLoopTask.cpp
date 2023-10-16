#include "../include/ThreadLoopTask.hpp"

ThreadLoopTask::ThreadLoopTask(ActivateControlTask &activateTask_,
                               DeactivateControlTask &deactivateTask_,
                               MotorPathTask &pathTask_,
                               PathManager &pathManagerTask,
                               MotorSignalSendTask &sendTask_,
                               MotorResponseReadTask &readTask_,
                               SharedBuffer<can_frame> &sendBuffer_,
                               SharedBuffer<can_frame> &receiveBuffer_,
                               std::atomic<bool> &stop)
    : activateTask(activateTask_),
      deactivateTask(deactivateTask_),
      pathTask(pathTask_),
      pathManagerTask(pathManagerTask),
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
        std::cout << "Enter 'run' to continue or 'exit' to quit or 'sine to test: ";
        std::cin >> userInput;
        std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);

        if (userInput == "exit")
        {
            break;
        }
        else if (userInput == "sine")
        {
            
            pathTask(sendBuffer);
            std::thread sendThread(sendTask, std::ref(sendBuffer));
            std::thread readThread(readTask, std::ref(receiveBuffer));

            sendThread.join();
            readThread.join();
        }
        else if (userInput == "run")
        {
            
            pathManagerTask(sendBuffer);
            std::thread sendThread(sendTask, std::ref(sendBuffer));
            std::thread readThread(readTask, std::ref(receiveBuffer));

            sendThread.join();
            readThread.join();
        }
    }

    deactivateTask();
}
