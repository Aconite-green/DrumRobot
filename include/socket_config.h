#ifndef SOCKET_CONFIG_H
#define SOCKET_CONFIG_H

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <bits/types.h>
#include <linux/can/raw.h>
#include <sys/time.h>
#include <stdio.h>
#include <poll.h>

#define ERR_SOCKET_CREATE_FAILURE -1
#define ERR_SOCKET_CONFIGURE_FAILURE -2
#define NON_BLOCK_FLAG O_NONBLOCK
#define BUFFER_SIZE 256 * 1000

void list_available_can_ports();
void check_can_port_status(const char *port);
void print_activation_instructions();

void check(int result, const char* errMsg, int errCode);
int create_socket(const char* ifname);
void configure_sockets(int sockets[], int count, int enable_block, struct timeval tv, struct can_filter rfilter[4], int loopback, int recv_own_msgs);
int set_blocking_mode(int hsocket, int enable_block);
int set_timeout(int hsocket, struct timeval tv); // 변경된 시그니처
int configure_socket(int hsocket, int enable_block, struct timeval tv, struct can_filter rfilter[4], int loopback, int recv_own_msgs);

int set_buffer_size(int hsocket, int size);
int set_loopback(int hsocket, int loopback);
int set_recv_own_msgs(int hsocket, int recv_own_msgs);
void poll_socket(int hsocket, int timeout_ms);


#endif // SOCKET_CONFIG_H
