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
    ~CanSocketUtils();

    int create_socket(const std::string &ifname); // 여러 if_name에 대해 소켓을 생성
    int set_socket_timeout(int hsocket, int timeout_sec, int timeout_usec);

    void list_and_activate_available_can_ports();

    const std::map<std::string, int> &getSockets() const
    {
        return sockets;
    }

private:
    void check(int result, const char *errMsg, int errCode);
    bool is_port_up(const char *port);
    void activate_port(const char *port);
    std::vector<std::string> ifnames;
    std::map<std::string, int> sockets; // 각 if_name에 대한 소켓 디스크립터를 저장
};

#endif // CAN_SOCKET_UTILS_H
