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

    // Enter CSP mode
    setNodeCSPMode(&frame, 601);
    send_frame_and_receive_reply(can_socket, &frame);
    getchar();

    //Set torqueoffset
    setNodeTorqueOffset(&frame, 601);
    send_frame_and_receive_reply(can_socket, &frame);
    getchar();
    
    //Set postionoffset
    setNodePositionOffset(&frame, 601);
    send_frame_and_receive_reply(can_socket, &frame);
    getchar();

    //pre-operation -> operational (PDO enable)
    setOperational(&frame, 0);
    send_frame_and_receive_reply(can_socket, &frame);
    getchar();

    //Shut Down
    controlWordShutdown(&frame, 201);
    send_frame_and_receive_reply(can_socket, &frame);
    getchar();

    //Sync
    Sync(&frame, 80);
    send_frame_and_receive_reply(can_socket, &frame);
    getchar();

    //Enable CSP mode
    controlWordSwitchOnEnable(&frame, 201);
    send_frame_and_receive_reply(can_socket, &frame);
    getchar();

    //Sync
    Sync(&frame, 80);
    send_frame_and_receive_reply(can_socket, &frame);
    getchar();

    //Move to Target postion(90)
    controlWordTargetPosition(&frame, 301, 90);
    send_frame_and_receive_reply(can_socket, &frame);
    getchar();

    Sync(&frame, 80);
    send_frame_and_receive_reply(can_socket, &frame);
    getchar();

    //Move to Target postion(0)
    controlWordTargetPosition(&frame, 301, 0);
    send_frame_and_receive_reply(can_socket, &frame);
    getchar();

    Sync(&frame, 80);
    send_frame_and_receive_reply(can_socket, &frame);
    getchar();



    
    // Close socket
    close(can_socket);

    return 0;
}
