#include "../include/MotorPathTask.hpp"
#include "../include/SharedBuffer.hpp"
#include "../include/MotorResponseReadTask.hpp"
#include <string>

MotorResponseReadTask::MotorResponseReadTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, const std::map<std::string, int> &sockets, std::atomic<bool> &paused)
    : tmotors(tmotors), sockets(sockets), paused(paused)
{
    // 포트당 연결된 모터 개수를 계산
    for (const auto& motor_pair : tmotors) {
        const auto& motor = motor_pair.second;
        const auto& interface_name = motor->interFaceName;

        if (motor_count_per_port.find(interface_name) == motor_count_per_port.end()) {
            motor_count_per_port[interface_name] = 0;
        }
        motor_count_per_port[interface_name]++;
    }
}

void MotorResponseReadTask::operator()(SharedBuffer<can_frame> &buffer)
{
    clock_t external = clock();
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 5000; // 5ms timeout

    while (true)
    {
        if (paused.load())
        {
            continue;
        }

        clock_t internal = clock();
        double elapsed_time = ((double)(internal - external)) / CLOCKS_PER_SEC * 1000;
        if (elapsed_time >= 100) // 5ms
        {
            external = clock();

            for (const auto &socket_pair : sockets)
            {
                int socket_descriptor = socket_pair.second;
                int motor_count = motor_count_per_port[socket_pair.first]; // 포트에 연결된 모터 개수를 가져옵니다.

                for (int i = 0; i < motor_count; ++i)
                {
                    can_frame readFrame;
                    setsockopt(socket_descriptor, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
                    ssize_t bytesRead = read(socket_descriptor, &readFrame, sizeof(can_frame));

                    if (bytesRead == -1)
                    {
                        std::cerr << "Failed to read from socket for interface: " << socket_pair.first << std::endl;
                        return; // 타임아웃에 걸리면 루프를 종료합니다.
                    }
                    else
                    {
                        buffer.push(readFrame);
                    }
                }
            }
        }
    }
}

