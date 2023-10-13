// DeactivateControlTask.cpp
#include "../include/DeactivateControlTask.hpp"

DeactivateControlTask::DeactivateControlTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors, const std::map<std::string, int> &sockets)
    : tmotors(tmotors), maxonMotors(maxonMotors), sockets(sockets)
{
}

void DeactivateControlTask::fillCanFrameFromInfo(struct can_frame *frame, const CanFrameInfo &info)
{
    frame->can_id = info.can_id;
    frame->can_dlc = info.can_dlc;
    std::copy(info.data.begin(), info.data.end(), frame->data);
}

int DeactivateControlTask::set_socket_timeout(int hsocket, int timeout_sec, int timeout_usec)
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

void DeactivateControlTask::sendAndReceive(
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

void DeactivateControlTask::sendNotRead(
    int socket,
    const std::string &name,
    struct can_frame &frame,
    std::function<void(const std::string &, bool)> customOutput)
{
    ssize_t write_status = write(socket, &frame, sizeof(can_frame));
    bool success = write_status > 0;

    customOutput(name, success);
}

void DeactivateControlTask::operator()()
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

    // Tmotors
    if (!tmotors.empty())
    {
        for (const auto &motorPair : tmotors)
        {
            std::string name = motorPair.first;
            std::shared_ptr<TMotor> motor = motorPair.second;

            fillCanFrameFromInfo(&frame, motor->getCanFrameForCheckMotor());
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });

            fillCanFrameFromInfo(&frame, motor->getCanFrameForExit());

            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "Exiting control mode for motor [" << motorName << "]." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Failed to exit control mode for motor [" << motorName << "]." << std::endl;
                               }
                           });

            // 구분자 추가
            std::cout << "=======================================" << std::endl;
        }
    }
    else
    {
        std::cout << "No Tmotors to process." << std::endl;
    }

    // MaxonMotor
    if (!maxonMotors.empty())
    {
        for (const auto &motorPair : maxonMotors)
        {
            std::string name = motorPair.first;
            std::shared_ptr<MaxonMotor> motor = motorPair.second;

            fillCanFrameFromInfo(&frame, motor->getCanFrameForQuickStop());
            sendNotRead(sockets.at(motor->interFaceName), name, frame,
                        [](const std::string &motorName, bool success)
                        {
                            if (success)
                            {
                                std::cout << "Quick Stop for motor [" << motorName << "]." << std::endl;
                            }
                            else
                            {
                                std::cerr << "Failed to Quick Stop for motor [" << motorName << "]." << std::endl;
                            }
                        });

            // 구분자 추가
            std::cout << "---------------------------------------" << std::endl;

            fillCanFrameFromInfo(&frame, motor->getCanFrameForSync());
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "Sync Signal for motor [" << motorName << "]." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Failed to send Sync Signal for motor [" << motorName << "]." << std::endl;
                               }
                           });

            fillCanFrameFromInfo(&frame, motor->getCanFrameForExit());
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "Exiting for motor [" << motorName << "]." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Failed to exit for motor [" << motorName << "]." << std::endl;
                               }
                           });
        }
    }
    else
    {
        std::cout << "No Maxon motors to process." << std::endl;
    }
}
