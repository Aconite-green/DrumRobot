#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>

#include <sys/ioctl.h>
#include <net/if.h>
#include <time.h>

#include <linux/can/raw.h>

// User Defined
#include "../include/socket_config.h"
#include "../include/realtime_config.h"
#include "../include/motor.h"
#include "../include/can_utils.h"

#define PI 3.142592
#define DEG_TO_RAD PI / 180

int main()
{

    list_available_can_ports();
    check_can_port_status("can0");

    // Define the socket for a CAN port
    int can_socket = create_socket("can0");
    if (can_socket == -1)
    {
        printf("Failed to create socket a can port\n");
        return 1;
    }

    // 이 부분을 수정하여 모터의 개수와 CAN ID를 설정
    const int num_motors = 3;    // 모터의 개수
    int motor_ids[] = {2, 3, 4}; // 모터의 CAN ID

    // Motor check
    MotorLimits motorlimits;
    MotorLimits *p_limits;
    p_limits = get_motor_limits("AK10_9");
    motorlimits = *p_limits;

    struct can_frame can_frame;

    for (int i = 0; i < num_motors; ++i)
    {
        MotorCommand command;
        command.motor_id = motor_ids[i];
        command.p_des = 0;
        command.v_des = 0;
        command.kp = 8;
        command.kd = 1;
        command.t_ff = 0;

        // Check available motor
        pack_cmd(motorlimits, &can_frame, command.motor_id, 0, 0, 0, 0, 0); // zero command
        send_frame_and_receive_reply(can_socket, &can_frame);
        printf("motor %d connected\n", motor_ids[i]);
        usleep(10000);

        // Enter Controlmode
        enterControlmode(&can_frame, command.motor_id);
        send_frame_and_receive_reply(can_socket, &can_frame);
        printf("motor %d enter control mode\n", motor_ids[i]);
        usleep(50000);
    }

    struct can_frame *sent_frames = NULL;
    struct can_frame *received_frames = NULL;
    int num_frames = 0;
    int actual_frames = 0;

    // 데이터를 CSV 파일에서 불러오고 sent_frames에 저장
    prepack_frames_from_csv("./tests/command/psine_2sec.csv", motorlimits, &sent_frames, &num_frames);

    // sent_frames를 사용하여 CAN 데이터를 전송하고 received_frames에 수신 데이터를 저장
    sendFramesPeriodically(can_socket, sent_frames, num_frames, 5 /*ms*/, motorlimits, &received_frames, &actual_frames);

    // 수신한 데이터를 처리
    postProcessReceivedData(received_frames, actual_frames, motorlimits, "./tests/recv_data", "psine_2sec.csv");

    // 메모리 해제
    free(sent_frames);
    free(received_frames);

    // 각 모터에 대한 종료 처리
    for (int i = 0; i < num_motors; ++i) {
        // Zero command before exit
        pack_cmd(motorlimits, &can_frame, motor_ids[i], 0, 0, 0, 0, 0); 
        send_frame_and_receive_reply(can_socket, &can_frame);
        usleep(10000);

        // Exit Control Mode
        exitControlmode(&can_frame, motor_ids[i]);
        send_frame_and_receive_reply(can_socket, &can_frame);
        printf("motor %d exit\n", motor_ids[i]);
    }

    // Close socket
    close(can_socket);

    return 0;
}
