#ifndef CAN_UTILS_H
#define CAN_UTILS_H
#include <linux/can.h>
#include <math.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <time.h>
#include <stdlib.h>
#include "motor.hpp"
#include "error_handling.hpp"
#include "socket_config.hpp"
#include "CommandParser.hpp"
#include "motor.hpp"



//MainThread(connect + SetMotor)
void send_frame_and_receive_reply(int hsocket, struct can_frame *frame);
void printframe(struct can_frame *frame);
void send_frame(int hsocket, struct can_frame *frame);


//Thread 1 : Motion Planner
//Thread 2 : 5ms loop
int kbhit(void);
void send_no_read(int hsocket, struct can_frame *frame);
void send_receive(int hsocket, struct can_frame *frame);
//Thread 3 : Read data
//Thread 4 : Read IO (Sensor)
void sendFramesPeriodically(int can_socket, struct can_frame *frames, int num_frames, int period_ms, struct can_frame **received_frames_ptr, int *actual_frames_ptr);
void writeRawCANDataToCSV(struct can_frame *received_frames, int actual_frames, const char *folder_path, const char *file_name);


#endif