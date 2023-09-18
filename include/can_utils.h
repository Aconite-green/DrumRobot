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
#include "motor.h"
#include "error_handling.h"
#include "socket_config.h"


typedef struct {
    int id;
    float position;
    float speed;
    float torque;
} RecvMotorInfo;

void pack_cmd(MotorLimits limits, struct can_frame *frame, int can_id, float p_des, float v_des, float kp, float kd, float t_ff);
void unpack_reply(MotorLimits limits, struct can_frame *frame, RecvMotorInfo *info);
void enterControlmode(struct can_frame *frame, int can_id);
void set_to_zero(struct can_frame *frame, int can_id);
void exitControlmode(struct can_frame *frame, int can_id);
int kbhit(void);
void prepack_frames_from_csv(const char *filepath, MotorLimits limits, struct can_frame **frames_ptr, int *num_frames);
void send_frame_and_receive_reply(int hsocket, struct can_frame *frame);
void sendFramesPeriodically(int can_socket, struct can_frame *frames, int num_frames, int period_ms, MotorLimits limits, struct can_frame **received_frames_ptr, int *actual_frames_ptr);
void postProcessReceivedData(struct can_frame *received_frames, int actual_frames, MotorLimits limits, const char *folder_path, const char *file_name);
void printframe(struct can_frame *frame);

//EPOS CSP
void setNodeCSPMode(struct can_frame *frame, int can_id);
void setNodeTorqueOffset(struct can_frame *frame, int can_id);
void setNodePositionOffset(struct can_frame *frame, int can_id);

void setOperational(struct can_frame *frame, int can_id);

void controlWordShutdown(struct can_frame *frame, int can_id);
void controlWordSwitchOnEnable(struct can_frame *frame, int can_id);
void Sync(struct can_frame *frame, int can_id);

void controlWordTargetPosition(struct can_frame *frame, int can_id, float angle);

//EPOS ppm
void setEpos4ProfilePositionMode(struct can_frame *frame, int can_id);
void setEpos4ShutDown(struct can_frame *frame, int can_id);
void setEpos4Enable(struct can_frame *frame, int can_id);
void setTargetPosition(struct can_frame *frame, int can_id, float angle);
void startEpos4Positioning(struct can_frame *frame, int can_id);
#endif