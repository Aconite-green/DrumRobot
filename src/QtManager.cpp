#include "../include/managers/QtManager.hpp"

QtManager::QtManager(State &stateRef,
                     CanManager &canManagerRef,
                     std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : state(stateRef),
      canManager(canManagerRef), motors(motorsRef),
      server_fd(-1),
      client_socket(-1)
{
    memset(&address, 0, sizeof(address));
}

QtManager::~QtManager()
{
    if (server_fd != -1)
    {
        close(server_fd);
    }
    if (client_socket != -1)
    {
        close(client_socket);
    }
    unlink("/tmp/unix_socket");
}

void QtManager::initializeServer()
{
    if ((server_fd = socket(AF_UNIX, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
    }
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, "/tmp/unix_socket");

    unlink(address.sun_path);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        perror("bind failed");
    }
    if (listen(server_fd, 1) < 0)
    {
        perror("listen");
    }
}

bool QtManager::sendMessage(const std::string &message)
{
    if (client_socket != -1)
    {
        send(client_socket, message.c_str(), message.size(), 0);
        return true;
    }
    return false;
}

std::string QtManager::receiveMessage()
{
    char buffer[1024] = {0};
    int valread = read(client_socket, buffer, 1024);
    if (valread > 0)
    {
        return std::string(buffer, valread);
    }
    return "";
}

void QtManager::guiThread()
{
    int addrlen = sizeof(address);

    while (state.main != Main::Shutdown)
    {
        switch (state.main.load())
        {
        case Main::SystemInit:
        {
            initializeServer();
            break;
        }
        case Main::Ideal:
        {
            if (client_socket == -1)
            {
                if ((client_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
                {
                    perror("accept error");
                    break;
                }
            }

            std::string message = receiveMessage();
            if (!message.empty())
            {
                std::cout << "Received from GUI: " << message << std::endl;
                // Process the received data
            }
            break;
        }
        case Main::Homing:
        {
            break;
        }
        case Main::Perform:
        {
            break;
        }
        case Main::AddStance:
        {
            break;
        }
        case Main::Check:
        {
            break;
        }
        case Main::Test:
        {
            break;
        }
        case Main::Pause:
        {
            break;
        }
        case Main::Error:
        {
            break;
        }
        case Main::Shutdown:
            break;
        }
    }
}
