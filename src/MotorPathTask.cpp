#include "../include/MotorPathTask.hpp"
#include "../include/SharedBuffer.hpp"
#include <string>

void MotorPathTask::operator()(SharedBuffer& buffer) {
    std::string command = "PATH_COMMAND"; // 예시
    buffer.push(command);
}
