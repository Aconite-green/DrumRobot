#include "socket_config.h"

void check(int result, const char *errMsg, int errCode)
{
    if (result == -1)
    {
        perror(errMsg);
        exit(errCode);
    }
}

int set_blocking_mode(int hsocket, int enable_block)
{
    int flag = fcntl(hsocket, F_GETFL, 0);
    if (flag < 0)
    {
        return -1;
    }

    if (enable_block)
    {
        flag &= ~NON_BLOCK_FLAG;
    }
    else
    {
        flag |= NON_BLOCK_FLAG;
    }

    return fcntl(hsocket, F_SETFL, flag);
}

int set_timeout(int hsocket, struct timeval tv)
{
    int result;

    result = setsockopt(hsocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (result < 0)
    {
        return -1;
    }
    result = setsockopt(hsocket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    if (result < 0)
    {
        return -1;
    }

    return 0;
}

int set_buffer_size(int hsocket, int size)
{
    return setsockopt(hsocket, SOL_SOCKET, SO_SNDBUF, &size, sizeof(size));
}

int set_can_filter(int hsocket, struct can_filter rfilter[4])
{
    socklen_t filter_size = sizeof(struct can_filter) * 4;                         // 배열의 크기를 직접 계산
    return setsockopt(hsocket, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, filter_size); // &rfilter가 아닌 rfilter를 사용
}

int set_loopback(int hsocket, int loopback)
{
    return setsockopt(hsocket, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
}

int set_recv_own_msgs(int hsocket, int recv_own_msgs)
{
    return setsockopt(hsocket, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));
}

int create_socket(const char *ifname)
{
    int result;
    int hsocket;
    struct sockaddr_can addr;
    struct ifreq ifr;

    hsocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (hsocket < 0)
    {
        return ERR_SOCKET_CREATE_FAILURE;
    }

    memset(&ifr, 0, sizeof(struct ifreq));
    memset(&addr, 0, sizeof(struct sockaddr_can));

    strcpy(ifr.ifr_name, ifname);
    result = ioctl(hsocket, SIOCGIFINDEX, &ifr);
    if (result < 0)
    {
        return ERR_SOCKET_CREATE_FAILURE;
    }

    addr.can_ifindex = ifr.ifr_ifindex;
    addr.can_family = AF_CAN;

    if (bind(hsocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        return ERR_SOCKET_CREATE_FAILURE;
    }

    return hsocket;
}

int configure_socket(int hsocket, int enable_block, struct timeval tv, struct can_filter rfilter[4], int loopback, int recv_own_msgs)
{
    int result;

    result = set_blocking_mode(hsocket, enable_block);
    if (result < 0)
    {
        return ERR_SOCKET_CONFIGURE_FAILURE;
    }

    if (enable_block)
    {
        result = set_timeout(hsocket, tv);
        if (result < 0)
        {
            return ERR_SOCKET_CONFIGURE_FAILURE;
        }
    }

    result = set_buffer_size(hsocket, BUFFER_SIZE);
    if (result < 0)
    {
        return ERR_SOCKET_CONFIGURE_FAILURE;
    }

    result = set_can_filter(hsocket, rfilter);
    if (result < 0)
    {
        return ERR_SOCKET_CONFIGURE_FAILURE;
    }

    result = set_loopback(hsocket, loopback);
    if (result < 0)
    {
        return ERR_SOCKET_CONFIGURE_FAILURE;
    }

    result = set_recv_own_msgs(hsocket, recv_own_msgs);
    if (result < 0)
    {
        return ERR_SOCKET_CONFIGURE_FAILURE;
    }

    return 0;
}

void configure_sockets(int sockets[], int count, int enable_block, struct timeval tv, struct can_filter rfilter[4], int loopback, int recv_own_msgs)
{
    for (int i = 0; i < count; i++)
    {
        int result = configure_socket(sockets[i], enable_block, tv, rfilter, loopback, recv_own_msgs);
        check(result, "Failed to configure socket", ERR_SOCKET_CONFIGURE_FAILURE);
    }
}

void print_activation_instructions()
{
    printf("To activate the CAN port, you can use the following commands:\n");
    printf("sudo ip link set can0 type can bitrate 1000000 sample-point 0.75\n");
    printf("sudo ip link set can0 up\n");
}

void check_can_port_status(const char *port)
{
    printf("Checking status of %s:\n", port);
    char command[50];
    snprintf(command, sizeof(command), "ip link show %s", port);

    FILE *fp;
    char output[1024];

    fp = popen(command, "r");
    if (fp == NULL)
    {
        perror("Error opening pipe");
        exit(1);
    }

    // 명령어의 출력을 읽습니다.
    if (fgets(output, sizeof(output) - 1, fp) == NULL)
    {
        perror("fgets failed");
        pclose(fp);
        exit(1);
    }
    pclose(fp);

    if (strstr(output, "DOWN"))
    {
        printf("%s is DOWN\n", port);
        printf("Please activate the CAN port before continuing.\n");
        print_activation_instructions();
        exit(1);
    }
    else if (strstr(output, "UP"))
    {
        printf("%s is UP\n", port);
    }
    else
    {
        printf("Could not determine the status of %s.\n", port);
        exit(1);
    }
}

void list_available_can_ports()
{
    printf("Listing available CAN ports:\n");
    int result = system("ip link show | grep can");
    if (result != 0)
    {
        perror("No available CAN port");
        exit(1);
    }
}

void poll_socket(int hsocket, int timeout_ms)
{
    struct pollfd pfd;
    pfd.fd = hsocket;
    pfd.events = POLLIN; // POLLIN: 데이터를 읽을 수 있는 상태

    int ret = poll(&pfd, 1, timeout_ms);

    if (ret == -1)
    {
        perror("poll");
        printf("poll error\n"); // 적절한 에러 코드를 정의해주세요.
        return;
    }
    else if (ret == 0)
    {
        printf("Timeout occurred!\n");
        return;
    }
    else
    {
        if (pfd.revents & POLLIN) // POLLIN 이벤트가 발생했을 경우
        {
            printf("Socket is ready for reading.\n");
            // 실제 읽기는 여기서 하지 않음. 이 정보를 다른 함수에서 사용할 것.
            return;
        }
    }
}

