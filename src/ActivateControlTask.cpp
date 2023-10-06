// ActivateControlTask.cpp
#include "../include/ActivateControlTask.hpp"

ActivateControlTask::ActivateControlTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, const std::map<std::string, int> &sockets)
    : tmotors(tmotors), sockets(sockets)
{
}

void ActivateControlTask::fillCanFrameFromInfo(struct can_frame *frame, const CanFrameInfo &info)
{
    frame->can_id = info.can_id;
    frame->can_dlc = info.can_dlc;
    std::copy(info.data.begin(), info.data.end(), frame->data);
}

int ActivateControlTask::set_socket_timeout(int hsocket, int timeout_sec, int timeout_usec)
{
    struct timeval timeout;
    timeout.tv_sec = timeout_sec;
    timeout.tv_usec = timeout_usec;

    if (setsockopt(hsocket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
    {
        perror("setsockopt failed");         // perror 함수는 실패 원인을 출력해줍니다.
        return ERR_SOCKET_CONFIGURE_FAILURE; // 실패 시 에러 코드 반환
    }
    return 0; // 성공 시 0 반환
}

void ActivateControlTask::sendAndReceive(
    int socket,
    const std::string &name,
    struct can_frame &frame,
    std::function<void(const std::string &, bool)> customOutput)
{
    ssize_t write_status = write(socket, &frame, sizeof(can_frame));
    ssize_t read_status = read(socket, &frame, sizeof(can_frame));
    bool success = write_status > 0 && read_status > 0;

    customOutput(name, success);
}

// operation
void ActivateControlTask::operator()()
{
    struct can_frame frame;

    // sockets 맵의 모든 항목에 대해 set_socket_timeout 설정
    for (const auto &socketPair : sockets)
    {
        int hsocket = socketPair.second;
        if (set_socket_timeout(hsocket, 0, 0) != 0)
        {
            // 타임아웃 설정 실패 처리
            std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }

    // 첫 번째 for문: 모터 상태 확인 및 제어 모드 설정
    for (const auto &motorPair : tmotors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<TMotor> motor = motorPair.second;

        // 상태 확인
        fillCanFrameFromInfo(&frame, motor->getCanFrameForCheckMotor());
        sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                       [](const std::string &motorName, bool success)
                       {
                           if (success)
                           {
                               std::cout << "Motor [" << motorName << "] status check passed." << std::endl;
                           }
                           else
                           {
                               std::cerr << "Motor [" << motorName << "] status check failed." << std::endl;
                           }
                       });

        // 구분자 추가
        std::cout << "---------------------------------------" << std::endl;

        // 제어 모드 설정
        fillCanFrameFromInfo(&frame, motor->getCanFrameForControlMode());
        sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                       [](const std::string &motorName, bool success)
                       {
                           if (success)
                           {
                               std::cout << "Control mode set for motor [" << motorName << "]." << std::endl;
                           }
                           else
                           {
                               std::cerr << "Failed to set control mode for motor [" << motorName << "]." << std::endl;
                           }
                       });

        fillCanFrameFromInfo(&frame, motor->getCanFrameForCheckMotor());
        sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                       [](const std::string &motorName, bool success)
                       {
                           if (success)
                           {
                               std::cout << "Motor [" << motorName << "] status check passed." << std::endl;
                           }
                           else
                           {
                               std::cerr << "Motor [" << motorName << "] status check failed." << std::endl;
                           }
                       });

        // 구분자 추가
        std::cout << "=======================================" << std::endl;
    }
}
