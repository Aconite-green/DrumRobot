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

class CanSocketUtils
{
public:
    static const int ERR_SOCKET_CREATE_FAILURE = -1;
    static const int ERR_SOCKET_CONFIGURE_FAILURE = -2;
    // Public Methods
    CanSocketUtils(const char *ifname);
    ~CanSocketUtils();

    int create_socket();
    void send_frame_and_receive_reply(struct can_frame *frame);

    void writeRawCANDataToCSV(struct can_frame *received_frames, int actual_frames, const char *folder_path, const char *file_name);

    void list_and_activate_available_can_ports();
    int hsocket;

private:
    void check(int result, const char *errMsg, int errCode);
    bool is_port_up(const char *port);
    void activate_port(const char *port);
    std::string ifname;
    
};

#endif // CAN_SOCKET_UTILS_H
