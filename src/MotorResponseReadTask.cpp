#include "../include/MotorPathTask.hpp"
#include "../include/SharedBuffer.hpp"
#include "../include/MotorResponseReadTask.hpp"
#include <string>

MotorResponseReadTask::MotorResponseReadTask(
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors, 
     std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors, 
    const std::map<std::string, int> &sockets, 
    std::atomic<bool> &paused, 
    std::atomic<bool> &stop)
    : tmotors(tmotors), maxonMotors(maxonMotors), sockets(sockets), paused(paused), stop(stop)
{
    // 포트당 연결된 모터 개수를 계산
    for (const auto &motor_pair : tmotors)
    {
        const auto &motor = motor_pair.second;
        const auto &interface_name = motor->interFaceName;

        if (motor_count_per_port.find(interface_name) == motor_count_per_port.end())
        {
            motor_count_per_port[interface_name] = 0;
        }
        motor_count_per_port[interface_name]++;
    }
}

int MotorResponseReadTask::kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

void MotorResponseReadTask::operator()(SharedBuffer<can_frame> &buffer)
{
    clock_t external = clock();
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 5000; 

    while (!stop)
    {

        if (kbhit())
        {
            char ch = getchar();
            if (ch == 'q' || ch == 'Q')
                paused.store(true);
            if (ch == 'c' || ch == 'C')
                paused.store(false);
            if (ch == 'e' || ch == 'E')
                stop.store(true);
        }
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
                int motor_count = motor_count_per_port[socket_pair.first]; 

                for (int i = 0; i < motor_count; ++i)
                {
                    
                    can_frame readFrame[1000];
                    setsockopt(socket_descriptor, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
                    ssize_t bytesRead = read(socket_descriptor, &readFrame, sizeof(can_frame) * 1000);

                    if (bytesRead == -1)
                    {
                        std::cerr << "Failed to read from socket for interface: " << socket_pair.first << std::endl;
                        return;
                    }
                    else
                    {
                        buffer.push(readFrame[1000]);
                    }
                }
            }
        }
    }
}
