// ActivateControlTask.cpp
#include "../include/ActivateControlTask.hpp"

ActivateControlTask::ActivateControlTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors, const std::map<std::string, int> &sockets)
    : tmotors(tmotors), maxonMotors(maxonMotors), sockets(sockets)
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

void ActivateControlTask::sendNotRead(
    int socket,
    const std::string &name,
    struct can_frame &frame,
    std::function<void(const std::string &, bool)> customOutput)
{
    ssize_t write_status = write(socket, &frame, sizeof(can_frame));
    bool success = write_status > 0;

    customOutput(name, success);
}

void ActivateControlTask::writeAndReadForSync(
    int socket,
    const std::string &name,
    struct can_frame &frame,
    size_t numMaxonMotors,
    std::function<void(const std::string &, bool)> customOutput)
{
    ssize_t write_status = write(socket, &frame, sizeof(can_frame));
    if (write_status <= 0)
    {
        customOutput(name, false);
        return;
    }

    // 응답을 저장할 버퍼
    std::vector<struct can_frame> read_frames(numMaxonMotors);

    bool success = true;
    ssize_t read_status;

    // 모터의 개수만큼 read 실행
    for (size_t i = 0; i < numMaxonMotors; ++i)
    {
        read_status = read(socket, &read_frames[i], sizeof(can_frame));
        if (read_status <= 0)
        {
            success = false;
            break;
        }
    }

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
        if (set_socket_timeout(hsocket, 0, 50000) != 0)
        {
            // 타임아웃 설정 실패 처리
            std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }
    if (!tmotors.empty())
    {
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
            // 상태 확인
            fillCanFrameFromInfo(&frame, motor->getCanFrameForZeroing());
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "zero set for motor [" << motorName << "]." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Failed to set zero for motor [" << motorName << "]." << std::endl;
                               }
                           });

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

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForControlMode());
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForPosOffset());
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });
            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForTorqueOffset());
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForOperational());
            sendNotRead(sockets.at(motor->interFaceName), name, frame,
                        [](const std::string &motorName, bool success) {

                        });

            fillCanFrameFromInfo(&frame, motor->getCanFrameForEnable());
            sendNotRead(sockets.at(motor->interFaceName), name, frame,
                        [](const std::string &motorName, bool success)
                        {
                            if (success)
                            {
                                std::cout << "Enabled for motor [" << motorName << "]." << std::endl;
                            }
                            else
                            {
                                std::cerr << "Failed to Enable for motor [" << motorName << "]." << std::endl;
                            }
                        });

            fillCanFrameFromInfo(&frame, motor->getCanFrameForSync());
            writeAndReadForSync(sockets.at(motor->interFaceName), name, frame, maxonMotors.size(),
                                [](const std::string &motorName, bool success) {

                                });

            // 구분자 추가
            std::cout << "=======================================" << std::endl;
        }
    }
    else
    {
        std::cout << "No Maxon motors to process." << std::endl;
    }
}
