#include "../include/StateTask.hpp"

// StateTask 클래스의 생성자
StateTask::StateTask(std::atomic<State>& stateRef) : state(stateRef) {}

// StateTask 클래스의 operator() 함수
void StateTask::operator()() {
    while (state != State::Shutdown) {
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
    switch (state.load()) {
        case State::Connected: return "Connected";
        case State::Homing: return "Homing";
        case State::HomeReady: return "Home Ready";
        case State::Tuning: return "Tuning";
        case State::Performing: return "Performing";
        // 다른 상태들에 대한 이름 추가...
        default: return "Unknown";
    }
}

void StateTask::displayAvailableCommands() const {
    std::cout << "Available Commands:\n";
    if (state == State::Connected) {
        std::cout << "- home: Set home position\n";
    } else if (state == State::HomeReady) {
        std::cout << "- tune: Start tuning\n";
        std::cout << "- perform: Start performing\n";
    }
    std::cout << "- shutdown: Shut down the system\n";
}

bool StateTask::processInput(const std::string& input) {
    if (input == "connect" && state == State::SystemInit) {
        state = State::Connected;
        return true;
    } else if (input == "home" && state == State::Connected) {
        state = State::Homing;
        return true;
    } else if (input == "tune" && state == State::HomeReady) {
        state = State::Tuning;
        return true;
    } else if (input == "perform" && state == State::HomeReady) {
        state = State::Performing;
        return true;
    } else if (input == "shutdown") {
        state = State::Shutdown;
        return true;
    }
    return false; // 명령어가 유효하지 않거나 현재 상태에서 허용되지 않음
}
