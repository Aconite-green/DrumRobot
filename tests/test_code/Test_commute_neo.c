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

void send_can_frame(int fd, int can_id, int dlc, unsigned char *data)
{
    unsigned char frame[100];
    frame[0] = can_id;
    frame[1] = dlc;
    memcpy(frame + 2, data, dlc);

    ssize_t bytes_written = write(fd, frame, 2 + dlc);

    if (bytes_written != (2 + dlc))
    {
        perror("write failed");
        // 여기에 에러 처리 코드를 추가할 수 있습니다.
    }
}

void receive_can_frame(int fd, int *can_id, int *dlc, unsigned char *data)
{
    unsigned char frame[100];
    int n = read(fd, frame, sizeof(frame));

    if (n > 1)
    {
        *can_id = frame[0];
        *dlc = frame[1];
        memcpy(data, frame + 2, *dlc);
    }
}

int main()
{
    int fd;
    struct termios options;

    fd = open("/dev/ttyS4", O_RDWR | O_NOCTTY);

    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSANOW, &options);

    unsigned char data_to_send[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_can_frame(fd, 1, 8, data_to_send);

    int received_can_id, received_dlc;
    unsigned char received_data[8];

    receive_can_frame(fd, &received_can_id, &received_dlc, received_data);

    printf("Received CAN ID: %d\n", received_can_id);
    printf("Received DLC: %d\n", received_dlc);
    printf("Received Data: ");
    for (int i = 0; i < received_dlc; i++)
    {
        printf("%02X ", received_data[i]);
    }
    printf("\n");

    close(fd);

    return 0;
}