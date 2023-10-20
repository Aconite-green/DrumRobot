#include "../include/MotorSignalSendTask.hpp"
#include <cerrno>  // errno
#include <cstring> // strerror

MotorSignalSendTask::MotorSignalSendTask(
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors,
    std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors,
    const std::map<std::string, int> &sockets,
    std::atomic<bool> &paused,
    std::atomic<bool> &stop)
    : tmotors(tmotors), maxonMotors(maxonMotors), sockets(sockets), paused(paused), stop(stop)
{
}

void MotorSignalSendTask::operator()(SharedBuffer<can_frame> &buffer)
{
    struct can_frame frameToProcess;
    clock_t external = clock();
    stop.store(false);
    while (!stop)
    {
        if (paused.load())
            continue;

        clock_t internal = clock();
        double elapsed_time = ((double)(internal - external)) / CLOCKS_PER_SEC * 1000;
        if (elapsed_time >= 2) // 5ms
        {
            external = clock();

            for (auto &motor_pair : tmotors)
            {
                auto motor_ptr = motor_pair.second;
                auto interface_name = motor_ptr->interFaceName;

                if (buffer.try_pop(frameToProcess))
                {
                    if (sockets.find(interface_name) != sockets.end())
                    {
                        int socket_descriptor = sockets.at(interface_name);
                        ssize_t bytesWritten = write(socket_descriptor, &frameToProcess, sizeof(struct can_frame));
                        if (bytesWritten == -1)
                        {
                            std::cerr << "Failed to write to socket for interface: " << interface_name << std::endl;
                            std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
                        }
                    }
                    else
                    {
                        std::cerr << "Socket not found for interface: " << interface_name << std::endl;
                    }
                }
                else
                {
                    std::cerr << "No CAN frame left in buffer[T]" << std::endl;
                    stop.store(true);
                    break;
                }
            }

            if (!maxonMotors.empty())
            {
                for (auto &motor_pair : maxonMotors)
                {
                    auto motor_ptr = motor_pair.second;
                    auto interface_name = motor_ptr->interFaceName;

                    if (buffer.try_pop(frameToProcess))
                    {
                        if (sockets.find(interface_name) != sockets.end())
                        {
                            int socket_descriptor = sockets.at(interface_name);
                            ssize_t bytesWritten = write(socket_descriptor, &frameToProcess, sizeof(struct can_frame));
                            if (bytesWritten == -1)
                            {
                                std::cerr << "Failed to write to socket for interface: " << interface_name << std::endl;
                                std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
                            }
                        }
                        else
                        {
                            std::cerr << "Socket not found for interface: " << interface_name << std::endl;
                        }
                    }
                    else
                    {
                        std::cerr << "No CAN frame left in buffer[M]" << std::endl;
                        stop.store(true);
                        break;
                    }
                }
                if (buffer.try_pop(frameToProcess))
                {
                    auto interface_name_for_sync = maxonMotors.begin()->second->interFaceName; // 첫 번째 MaxonMotor의 인터페이스 이름을 가져옵니다.
                    int socket_descriptor_for_sync = sockets.at(interface_name_for_sync);      // 모든 MaxonMotor가 같은 소켓을 사용한다고 했으므로, 아무거나 선택해도 됩니다.                                                                         // 이곳에 sync 신호에 대한 정보를 채워주세요.
                    ssize_t bytesWritten = write(socket_descriptor_for_sync, &frameToProcess, sizeof(struct can_frame));
                    if (bytesWritten == -1)
                    {
                        std::cerr << "Failed to write sync signal to socket for interface: " << interface_name_for_sync << std::endl;
                        std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
                    }
                }
            }
        }
    }
}
