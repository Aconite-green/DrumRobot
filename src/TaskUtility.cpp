#include "../include/tasks/TaskUtility.hpp"


void handleError(ssize_t bytesWritten, const std::string &interface_name)
{
    if (bytesWritten == -1)
    {
        std::cerr << "Failed to write to socket for interface: " << interface_name << std::endl;
        std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
    }
}

int kbhit()
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


