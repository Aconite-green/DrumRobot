#ifndef QT_MANAGER_H
#define QT_MANAGER_H

#include <iostream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include "SystemState.hpp"
#include "CanManager.hpp"
#include "Motor.hpp"

class QtManager {
public:

    QtManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);
    ~QtManager();

    void initializeServer();
    bool sendMessage(const std::string &message);
    std::string receiveMessage();

    void guiThread();

private:
    int server_fd;
    int client_socket;
    struct sockaddr_un address;

    State &state;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
};

#endif // QT_MANAGER_H
