#include "../include/ThreadLoopTask.hpp"


ThreadLoopTask::ThreadLoopTask(ActivateControlTask &activateTask_,
                               DeactivateControlTask &deactivateTask_,
                               MotorPathTask &pathTask_,
                               PathManager &pathManagerTask,
                               TuningTask &tuningTask,
                               MotorSignalSendTask &sendTask_,
                               MotorResponseReadTask &readTask_,
                               SharedBuffer<can_frame> &sendBuffer_,
                               SharedBuffer<can_frame> &receiveBuffer_,
                               std::atomic<bool> &stop)
    : activateTask(activateTask_),
      deactivateTask(deactivateTask_),
      pathTask(pathTask_),
      pathManagerTask(pathManagerTask),
      tuningTask(tuningTask),
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
    pathManagerTask.ready(sendBuffer);

    sendBuffer.print_buffer_size();

    std::string userInput;
    while (true)
    {
        int result = system("clear");
        if (result != 0)
        {
            std::cout << "error during sys function";
        }

        std::cout << "Enter 'run','exit','test': ";
        std::cin >> userInput;
        std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);

        if (userInput == "exit")
        {
            break;
        }
        else if (userInput == "run")
        {
            int result = system("clear");
            if (result != 0)
            {
                std::cout << "error during sys function";
            }

            std::cout << "Performing ...... \n";

            std::thread pathThread(pathManagerTask, std::ref(sendBuffer));
            std::thread sendThread(sendTask, std::ref(sendBuffer));
            std::thread readThread(readTask, std::ref(receiveBuffer));

            pathThread.join();
            sendThread.join();
            readThread.join();

            std::cout << "........End performance \n";
            pathManagerTask.ready(sendBuffer);
        }
        else if (userInput == "test")
        {
            while (true)
            {

                std::cout << "Enter 'tuning', 'waves', 'exit' :";
                std::cin >> userInput;
                std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);
                if (userInput == "tuning")
                {
                    while (true)
                    {
                        int result = system("clear");
                        if (result != 0)
                        {
                            printf("error using sys function\n");
                        }
                        std::cout << "Current Kp : " << tuningTask.kp << "\n";
                        std::cout << "Current Kd : " << tuningTask.kd << "\n";
                        std::cout << "Time for Sine period : " << tuningTask.sine_t << "\n";

                        // 파일 존재 여부를 확인하고 사용자에게 알림
                        if (std::filesystem::exists(tuningTask.fileName))
                        {
                            std::cout << "Current File (" << tuningTask.fileName << ") Already Exists\n";
                        }
                        else
                        {
                            std::cout << "Current File (" << tuningTask.fileName << ") Does Not Exist\n";
                        }
                        std::cout << "\n\n";
                        std::cout << "Enter run, kp, kd, period, exit : \n";
                        std::cin >> userInput;
                        std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);
                        if (userInput == "run")
                        {
                            tuningTask();
                        }
                        else if (userInput == "kp")
                        {
                            std::cout << "Current Kp : " << tuningTask.kp << "\n";
                            std::cout << "Enter Desired Kp : "
                                      << "\n";
                            std::cin >> tuningTask.kp;
                        }
                        else if (userInput == "kd")
                        {
                            std::cout << "Current Kd : " << tuningTask.kd << "\n";
                            std::cout << "Enter Desired Kd : "
                                      << "\n";
                            std::cin >> tuningTask.kd;
                        }
                        else if (userInput == "period")
                        {
                            std::cout << "Current Time for Sine period : " << tuningTask.sine_t << "\n";
                            std::cout << "Enter Desired Sine period : "
                                      << "\n";
                            std::cin >> tuningTask.sine_t;
                        }
                        else if (userInput == "file")
                        {
                            std::cout << "Enter Desired File Name: ";
                            std::cin >> tuningTask.fileName;
                        }
                        else if (userInput == "exit")
                        {
                            break;
                        }
                    }
                }
                else if (userInput == "waves")
                {
                    while (true)
                    {
                        std::thread pathThread(pathTask, std::ref(sendBuffer));
                        std::thread sendThread(sendTask, std::ref(sendBuffer));
                        std::thread readThread(readTask, std::ref(receiveBuffer));

                        pathThread.join();
                        sendThread.join();
                        readThread.join();
                    }
                }
                else if (userInput == "exit")
                {
                    break;
                }
            }
        }
    }

    deactivateTask();
}
