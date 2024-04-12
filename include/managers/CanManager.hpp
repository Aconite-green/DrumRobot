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

    CanManager(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    ~CanManager();

    void initializeCAN();

    void setSocketsTimeout(int sec, int usec);

    void checkCanPortsStatus();

    void setMotorsSocket();

    bool sendAndRecv(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);

    bool sendFromBuff(std::shared_ptr<GenericMotor> &motor);

    bool sendMotorFrame(std::shared_ptr<GenericMotor> motor);

    bool checkAllMotors_test();

    bool checkAllMotors_Fixed();

    bool sendForCheck(std::shared_ptr<GenericMotor> motor);

    bool sendForCheck_Fixed(std::shared_ptr<GenericMotor> motor);

    bool recvToBuff(std::shared_ptr<GenericMotor> &motor, int readCount);

    bool txFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);

    bool rxFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);

    void readFramesFromAllSockets();

    void distributeFramesToMotors();

    void clearReadBuffers();

    void setSocketNonBlock();
    void setSocketBlock();
    std::map<std::string, int> sockets;      ///< 모터와 통신하는 소켓의 맵.
    std::map<std::string, bool> isConnected; ///< 모터의 연결 상태를 나타내는 맵.
    int maxonCnt = 0;                        ///< 연결된 Maxon 모터의 수.
    std::map<int, int> motorsPerSocket;

    // Functions for Thread Case

    bool safetyCheck(std::string errorMessagePart);

private:
    std::vector<std::string> ifnames;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;

    std::map<int, std::vector<can_frame>> tempFrames;

    bool getCanPortStatus(const char *port);
    void activateCanPort(const char *port);
    void list_and_activate_available_can_ports();

    int createSocket(const std::string &ifname);
    int setSocketTimeout(int socket, int sec, int usec);

    void clearCanBuffer(int canSocket);
};

#endif // CAN_SOCKET_UTILS_H
