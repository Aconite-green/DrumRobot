#ifndef CAN_SOCKET_UTILS_H
#define CAN_SOCKET_UTILS_H
#include <linux/can.h>
#include <math.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <bits/types.h>
#include <linux/can/raw.h>
#include <sys/time.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <map>
#include <queue>
#include <memory>
#include "Motor.hpp"
#include "CommandParser.hpp"

class CanManager
{
public:
    static const int ERR_SOCKET_CREATE_FAILURE = -1;
    static const int ERR_SOCKET_CONFIGURE_FAILURE = -2;

    // Public Methods
    CanManager(std::queue<can_frame> &sendBufferRef, std::queue<can_frame> &recieveBufferRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);
    ~CanManager();

    void initializeCAN();
    void restartCanPorts();
    void setSocketsTimeout(int sec, int usec);
    void checkCanPortsStatus();
    void setMotorsSocket();

    // Basic Function
    bool sendAndRecv(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);
    bool sendFromBuff(std::shared_ptr<GenericMotor> &motor);
    bool recvToBuff(std::shared_ptr<GenericMotor> &motor, int readCount);
    bool txFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);
    bool rxFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);




    // Buffer Uitility
    void clearReadBuffers();

    std::map<std::string, int> sockets;
    std::map<std::string, bool> isConnected;

private:
    std::vector<std::string> ifnames;
    std::queue<can_frame> &sendBuffer;
    std::queue<can_frame> &recieveBuffer;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;

    

    // Port
    bool getCanPortStatus(const char *port);
    void activateCanPort(const char *port);
    void list_and_activate_available_can_ports();
    void deactivateCanPort(const char *port);

    // Network (Socket)
    int createSocket(const std::string &ifname);
    int setSocketTimeout(int socket, int sec, int usec);
    void releaseBusyResources();

    // Buffer Uitility
    void clearCanBuffer(int canSocket);
};

#endif // CAN_SOCKET_UTILS_H
