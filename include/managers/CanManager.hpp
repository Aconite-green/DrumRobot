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

using namespace std;

class CanManager
{
public:
    static const int ERR_SOCKET_CREATE_FAILURE = -1;
    static const int ERR_SOCKET_CONFIGURE_FAILURE = -2;

    // Public Methods
    CanManager(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);
    ~CanManager();

    void initializeCAN();
    void restartCanPorts();
    void setSocketsTimeout(int sec, int usec);
    void checkCanPortsStatus();
    
    // Motor Functions
    void setMotorsSocket();
    bool checkConnection(std::shared_ptr<GenericMotor> motor);
    bool checkAllMotors();

    // Basic Function
    bool sendAndRecv(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);
    bool sendFromBuff(std::shared_ptr<GenericMotor> &motor);
    bool recvToBuff(std::shared_ptr<GenericMotor> &motor, int readCount);
    bool txFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);
    bool rxFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);
    
    // DrumPlay Functions
    void readFramesFromAllSockets();
    void distributeFramesToMotors();


    // Buffer Uitility
    void clearReadBuffers();

    std::map<std::string, int> sockets;
    std::map<std::string, bool> isConnected;

private:
    std::vector<std::string> ifnames;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;

    std::map<int, std::vector<can_frame>> tempFrames;
    // Port
    bool getCanPortStatus(const char *port);
    void activateCanPort(const char *port);
    void list_and_activate_available_can_ports();
    void deactivateCanPort(const char *port);

    // Network (Socket)
    int createSocket(const std::string &ifname);
    int setSocketTimeout(int socket, int sec, int usec);

    // Buffer Uitility
    void clearCanBuffer(int canSocket);
};

#endif // CAN_SOCKET_UTILS_H
