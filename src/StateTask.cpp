#include "../include/StateTask.hpp"

// StateTask 클래스의 생성자
StateTask::StateTask(SystemState &systemStateRef) : systemState(systemStateRef) {}

// StateTask 클래스의 operator() 함수
void StateTask::operator()()
{
    while (systemState.main != Main::Shutdown)
    {
        if (systemState.main == Main::Ideal)
        {
            int ret = system("clear");
            if (ret == -1)
                cout << "system clear error" << endl;

            std::cout << "\nCurrent State: " << getStateName() << "\n";
            displayAvailableCommands();

            std::string input;
            std::cout << "Enter command: ";
            std::getline(std::cin, input);

            if (!processInput(input))
            {
                std::cout << "Invalid command or not allowed in current state!\n";
            }
        }
        usleep(2000);
    }
    std::cout<<"Out of StateTask Loop\n";
}

std::string StateTask::getStateName() const
{
    switch (systemState.main.load())
    {
    case Main::SystemInit:
        return "System Initialization";
    case Main::Ideal:
        return "Ideal";
    case Main::Tune:
        return "Tune";
    case Main::Perform:
        return "Performing";
    case Main::Ready:
        return "Ready";
    // 다른 상태들에 대한 이름 추가...
    default:
        return "Unknown";
    }
}

void StateTask::displayAvailableCommands() const
{
    std::cout << "Available Commands:\n";
    if (systemState.main == Main::Ideal 
        && systemState.homeMode == HomeMode::NotHome
        && systemState.runMode == RunMode::Stop)
    {
        std::cout << "- h : Start homing\n";
        std::cout << "- x  : Make home state by user\n";
    }
    else if (systemState.main == Main::Ideal 
            && systemState.homeMode == HomeMode::HomeReady 
            && systemState.runMode == RunMode::Stop)
    {
        std::cout << "- t : Start tuning\n";
        std::cout << "- r : Go to ready position\n";
    }
    else if (systemState.main == Main::Ideal
            && systemState.homeMode == HomeMode::HomeReady 
            && systemState.runMode == RunMode::Ready)
    {
        std::cout << "- p : Start performing\n";
    }
    std::cout << "- s : Shut down the system\n";
     std::cout << "- c : Check Motors position\n";
}

bool StateTask::processInput(const std::string &input)
{
    if (input == "h" && systemState.main == Main::Ideal && systemState.homeMode == HomeMode::NotHome)
    {
        systemState.main = Main::Homing; // 상태 변경 예시
        std::cout << "Now In homing\n";
        return true;
    }
    else if (input == "t" && systemState.main == Main::Ideal && systemState.homeMode == HomeMode::HomeReady)
    {
        systemState.main = Main::Tune; // 상태 변경 예시
        return true;
    }
    else if (input == "p" && systemState.main == Main::Ideal && systemState.homeMode == HomeMode::HomeReady && systemState.runMode == RunMode::Ready)
    {
        systemState.main = Main::Perform; // 상태 변경 예시
        return true;
    }
    else if (input == "r" && systemState.main == Main::Ideal && systemState.homeMode == HomeMode::HomeReady && systemState.runMode == RunMode::Stop)
    {
        systemState.main = Main::Ready;
        return true;
    }
    else if (input == "x" && systemState.main == Main::Ideal && systemState.homeMode == HomeMode::NotHome)
    {
        systemState.homeMode = HomeMode::HomeReady; // 상태 변경 예시
        return true;
    }
    else if (input == "s")
    {
        systemState.main = Main::Shutdown;
        return true;
    }
    else if (input == "c")
    {
        systemState.main = Main::Check;
        return true;
    }
    return false;
}
