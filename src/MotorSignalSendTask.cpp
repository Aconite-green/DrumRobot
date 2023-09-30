#include "../include/MotorPathTask.hpp"
#include "../include/SharedBuffer.hpp"
#include "../include/MotorSignalSendTask.hpp"
#include <string>

void MotorSignalSendTask::operator()(SharedBuffer& buffer) {
    std::string command = buffer.pop();
    // 모터에 신호를 보내는 코드
}
