#ifndef QT_MANAGER_H
#define QT_MANAGER_H

#include <iostream>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include "SystemState.hpp"
#include "CanManager.hpp"
#include "Motor.hpp"

#define PORT 8080
#define BUF_SIZE 1024

class QtManager {
public:
    QtManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);
    ~QtManager();

    void initializeServer();
    bool sendMessage(const std::string &message);
    std::string receiveMessage();

    void guiThread();

private:
    int sockfd;
    struct sockaddr_in serverAddr, clientAddr;
    char buffer[BUF_SIZE];

    State &state;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
};

#endif // QT_MANAGER_H
