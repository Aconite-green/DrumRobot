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



class CanManager
{
public:
    static const int ERR_SOCKET_CREATE_FAILURE = -1;
    static const int ERR_SOCKET_CONFIGURE_FAILURE = -2;

    // Public Methods
    CanManager(std::queue<can_frame> &sendBufferRef,std::queue<can_frame> &recieveBufferRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);
    ~CanManager();

    void initializeCAN(const std::vector<std::string> &ifnames);
    void restart_all_can_ports();
    void set_all_sockets_timeout(int sec, int usec);
    void clear_all_can_buffers();
    void checkCanPortsStatus();

    std::map<std::string, int> sockets;
    std::map<std::string, bool> isConnected;

private:
    std::vector<std::string> ifnames;
    std::queue<can_frame> &sendBuffer;
    std::queue<can_frame> &recieveBuffer;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    // Port
    bool is_port_up(const char *port);
    void activate_port(const char *port);
    void list_and_activate_available_can_ports();
    void down_port(const char *port);

    // Network (Socket)
    int create_socket(const std::string &ifname);
    int set_socket_timeout(int socket, int sec, int usec);
    void releaseBusyResources();

    // Recieve Buffer Uitility
    void clearCanBuffer(int canSocket);
    bool is_port_connected(const char *port);
};

#endif // CAN_SOCKET_UTILS_H
