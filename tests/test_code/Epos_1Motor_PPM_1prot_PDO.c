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

    struct can_frame frame;

    setOperational(&frame, 0x00);
    send_frame_and_receive_reply(can_socket, &frame);
    usleep(100000);

    setEpos4ProfilePositionMode(&frame, 0x601);
    send_frame_and_receive_reply(can_socket, &frame);
    usleep(100000);

    // Enable
    controlWordSwitchOnEnable(&frame, 0x201);
    send_frame_and_receive_reply(can_socket, &frame);
    usleep(100000);

    // Set Target postion 1
    controlWordTargetPosition(&frame, 0x301, 0);
    send_frame_and_receive_reply(can_socket, &frame);

    // Start postioning
    startEpos4Positioning(&frame, 0x201);
    send_frame_and_receive_reply(can_socket, &frame);


    // Enable
    controlWordSwitchOnEnable(&frame, 0x201);
    send_frame_and_receive_reply(can_socket, &frame);

    // Set Target postion 2
    controlWordTargetPosition(&frame, 0x301, 90);
    send_frame_and_receive_reply(can_socket, &frame);

    // Start postioning
    startEpos4Positioning(&frame, 0x201);
    send_frame_and_receive_reply(can_socket, &frame);

    // Close socket
    close(can_socket);

    return 0;
}