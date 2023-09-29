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
    static const int NON_BLOCK_FLAG = O_NONBLOCK;
    static const int BUFFER_SIZE = 256 * 1000;

    // Public Methods

    void check(int result, const char *errMsg, int errCode);
    int create_socket(const char *ifname);
    void configure_sockets(int sockets[], int count, int enable_block, struct timeval tv, struct can_filter rfilter[4], int loopback, int recv_own_msgs);
    void send_frame_and_receive_reply(int hsocket, struct can_frame *frame);
    void writeRawCANDataToCSV(struct can_frame *received_frames, int actual_frames, const char *folder_path, const char *file_name);
    bool is_port_up(const char *port);
    void activate_port(const char *port);
    void list_and_activate_available_can_ports();

private:
    // Private Methods
    int set_blocking_mode(int hsocket, int enable_block);
    int set_timeout(int hsocket, struct timeval tv);
    int configure_socket(int hsocket, int enable_block, struct timeval tv, struct can_filter rfilter[4], int loopback, int recv_own_msgs);
    int set_buffer_size(int hsocket, int size);
    int set_loopback(int hsocket, int loopback);
    int set_recv_own_msgs(int hsocket, int recv_own_msgs);
    int set_can_filter(int hsocket, struct can_filter rfilter[4]);
};

#endif // CAN_SOCKET_UTILS_H
