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

class CanSocketUtils
{
public:
    static const int ERR_SOCKET_CREATE_FAILURE = -1;
    static const int ERR_SOCKET_CONFIGURE_FAILURE = -2;
    // Public Methods
    CanSocketUtils(const std::vector<std::string> &ifnames);
    CanSocketUtils();
    ~CanSocketUtils();

    void restart_all_can_ports();

    std::map<std::string, int> sockets; 

private:
    bool is_port_up(const char *port);
    void activate_port(const char *port);
    std::vector<std::string> ifnames;
    // 각 if_name에 대한 소켓 디스크립터를 저장
    int create_socket(const std::string &ifname);
    void list_and_activate_available_can_ports();
    void down_port(const char *port);
};

#endif // CAN_SOCKET_UTILS_H
