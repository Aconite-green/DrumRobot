#include "../include/SharedBuffer.hpp"
#include "../include/SensorSignalReadTask.hpp"
#include <string>

void SensorSignalReadTask::operator()(SharedBuffer& buffer) {
    std::string command = "PATH_COMMAND"; // 예시
    buffer.push(command);
}
