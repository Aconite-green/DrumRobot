#include "../include/MotorSignalSendTask.hpp"
#include <cerrno>  // errno
#include <cstring> // strerror

MotorSignalSendTask::MotorSignalSendTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, const std::map<std::string, int> &sockets, std::atomic<bool> &paused)
    : tmotors(tmotors), sockets(sockets), paused(paused)
{
}

int MotorSignalSendTask::kbhit(void)
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

void MotorSignalSendTask::operator()(SharedBuffer<can_frame> &buffer)
{
    struct can_frame frameToProcess;
    clock_t external = clock();

    while (!buffer.empty())
    {
        while (1)
        {
            if (kbhit())
            {
                char ch = getchar();
                if (ch == 'q' || ch == 'Q')
                    paused.store(true);
                if (ch == 'c' || ch == 'C')
                    paused.store(false);
            }

            if (paused.load())
                continue;

            clock_t internal = clock();
            double elapsed_time = ((double)(internal - external)) / CLOCKS_PER_SEC * 1000;
            if (elapsed_time >= 100) // 5ms
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
                            if (write(socket_descriptor, &frameToProcess, sizeof(struct can_frame)) == -1)
                            {
                                std::cerr << "Failed to write to socket for interface: " << interface_name << std::endl;
                                std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
                            }
                            else
                            {
                                // CAN 프레임의 can_id와 data 출력
                                std::cout << "Successfully sent CAN frame with the following details:" << std::endl;
                                std::cout << "can_id: " << std::hex << frameToProcess.can_id << std::dec << std::endl;
                                std::cout << "data: ";
                                for (int i = 0; i < frameToProcess.can_dlc; ++i)
                                {
                                    std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frameToProcess.data[i]) << ' ';
                                }
                                std::cout << std::dec << std::endl;
                            }
                            ssize_t bytesRead = read(socket_descriptor, &frameToProcess, sizeof(struct can_frame));
                            if (bytesRead == -1)
                            {
                                std::cerr << "Read failed on socket: " << strerror(errno) << std::endl;
                            }
                            else
                            {
                                std::cout << "Successfully read " << bytesRead << " bytes from socket." << std::endl;
                            }
                        }
                        else
                        {
                            std::cerr << "Socket not found for interface: " << interface_name << std::endl;
                        }
                    }
                    else
                    {
                        std::cerr << "Failed to pop CAN frame from buffer" << std::endl;
                    }
                }
            }
        }
    }
}
