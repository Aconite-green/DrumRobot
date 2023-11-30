#include "../include/StateTask.hpp"


// StateTask 클래스의 생성자
StateTask::StateTask(SystemState& systemStateRef) : systemState(systemStateRef) {}

// StateTask 클래스의 operator() 함수
void StateTask::operator()() {
    while (systemState.main != Main::Shutdown) {
        std::cout << "\nCurrent State: " << getStateName() << "\n";
        displayAvailableCommands();

        std::string input;
        std::cout << "Enter command: ";
        std::getline(std::cin, input);

        if (!processInput(input)) {
            std::cout << "Invalid command or not allowed in current state!\n";
        }
    }
}

std::string StateTask::getStateName() const {
    switch (systemState.main.load()) {
        case Main::SystemInit: return "System Initialization";
        case Main::Home: return "Home";
        case Main::Tune: return "Tune";
        case Main::Perform: return "Performing";
        // 다른 상태들에 대한 이름 추가...
        default: return "Unknown";
    }
}

void StateTask::displayAvailableCommands() const {
    std::cout << "Available Commands:\n";
    if (systemState.main == Main::SystemInit) {
        std::cout << "- home: Set home position\n";
    } else if (systemState.homeMode == HomeMode::HomeReady) {
        std::cout << "- tune: Start tuning\n";
        std::cout << "- perform: Start performing\n";
    }
    std::cout << "- shutdown: Shut down the system\n";
}

bool StateTask::processInput(const std::string& input) {
    if (input == "home" && systemState.main == Main::SystemInit) {
        systemState.main = Main::Home;  
        return true;
    } else if (input == "tune" && systemState.main == Main::Home && systemState.homeMode == HomeMode::HomeReady) {
        systemState.main = Main::Tune;  // 상태 변경 예시
        return true;
    } else if (input == "perform" && systemState.main == Main::Home && systemState.homeMode == HomeMode::HomeReady) {
        systemState.main = Main::Perform;  // 상태 변경 예시
        return true;
    } 
    else if (input == "shutdown") {
        systemState.main = Main::Shutdown;
        return true;
    }
    return false; 
}
