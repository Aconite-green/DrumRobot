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

void ActivateControlTask::sendAndReceive(int socket, const std::string &name, struct can_frame &frame)
{
    ssize_t write_status = write(socket, &frame, sizeof(can_frame));
    if (write_status > 0)
    {
        std::cout << "Command sent to motor [" << name << "]. Awaiting response..." << std::endl;
        ssize_t read_status = read(socket, &frame, sizeof(can_frame));
        if (read_status > 0)
        {
            std::cout << "Motor [" << name << "] successfully processed." << std::endl;
        }
        else
        {
            std::cerr << "Issue receiving response from motor [" << name << "]." << std::endl;
        }
    }
    else
    {
        std::cerr << "Issue sending command to motor [" << name << "]." << std::endl;
    }
}

void ActivateControlTask::operator()()
{
    struct can_frame frame;

    // sockets 맵의 모든 항목에 대해 set_socket_timeout 설정
    for (const auto &socketPair : sockets)
    {
        int hsocket = socketPair.second;
        if (set_socket_timeout(hsocket, 0, 50000) != 0)  
        {
            // 타임아웃 설정 실패 처리
            std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }

    // 첫 번째 for문: 모터 상태 확인
    for (const auto &motorPair : tmotors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<TMotor> motor = motorPair.second;

        fillCanFrameFromInfo(&frame, motor->getCanFrameForCheckMotor());
        std::cout << "Checking motor [" << name << "]..." << std::endl;
        sendAndReceive(sockets.at(motor->interFaceName), name, frame);
    }

    // 두 번째 for문: 제어 모드 설정
    for (const auto &motorPair : tmotors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<TMotor> motor = motorPair.second;

        fillCanFrameFromInfo(&frame, motor->getCanFrameForControlMode());
        std::cout << "Setting control mode for motor [" << name << "]..." << std::endl;
        sendAndReceive(sockets.at(motor->interFaceName), name, frame);
    }

    for (const auto &socketPair : sockets)
    {
        int hsocket = socketPair.second;
        if (set_socket_timeout(hsocket, 0, 5000) != 0)  // 타임아웃을 5ms로 설정
        {
            // 타임아웃 설정 실패 처리
            std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }

   
}
