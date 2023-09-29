#include "../include/CanSocketUtils.hpp"


void CanSocketUtils::check(int result, const char *errMsg, int errCode)
{
    if (result == -1)
    {
        perror(errMsg);
        exit(errCode);
    }
}

int CanSocketUtils::set_blocking_mode(int hsocket, int enable_block)
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

int CanSocketUtils::set_timeout(int hsocket, struct timeval tv)
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

int CanSocketUtils::set_buffer_size(int hsocket, int size)
{
    return setsockopt(hsocket, SOL_SOCKET, SO_SNDBUF, &size, sizeof(size));
}

int CanSocketUtils::set_can_filter(int hsocket, struct can_filter rfilter[4])
{
    socklen_t filter_size = sizeof(struct can_filter) * 4;                         // 배열의 크기를 직접 계산
    return setsockopt(hsocket, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, filter_size); // &rfilter가 아닌 rfilter를 사용
}

int CanSocketUtils::set_loopback(int hsocket, int loopback)
{
    return setsockopt(hsocket, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
}

int CanSocketUtils::set_recv_own_msgs(int hsocket, int recv_own_msgs)
{
    return setsockopt(hsocket, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));
}

int CanSocketUtils::create_socket(const char *ifname)
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

int CanSocketUtils::configure_socket(int hsocket, int enable_block, struct timeval tv, struct can_filter rfilter[4], int loopback, int recv_own_msgs)
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

void CanSocketUtils::configure_sockets(int sockets[], int count, int enable_block, struct timeval tv, struct can_filter rfilter[4], int loopback, int recv_own_msgs)
{
    for (int i = 0; i < count; i++)
    {
        int result = configure_socket(sockets[i], enable_block, tv, rfilter, loopback, recv_own_msgs);
        check(result, "Failed to configure socket", ERR_SOCKET_CONFIGURE_FAILURE);
    }
}


bool CanSocketUtils::is_port_up(const char *port)
{
    char command[50];
    snprintf(command, sizeof(command), "ip link show %s", port);

    FILE *fp = popen(command, "r");
    if (fp == NULL) {
        perror("Error opening pipe");
        return false;
    }

    char output[1024];
    if (fgets(output, sizeof(output) - 1, fp) == NULL) {
        perror("fgets failed");
        pclose(fp);
        return false;
    }
    pclose(fp);

    if (strstr(output, "DOWN")) {
        return false;
    } else if (strstr(output, "UP")) {
        return true;
    } else {
        return false;
    }
}

void CanSocketUtils::activate_port(const char *port)
{
    char command1[100], command2[100];
    snprintf(command1, sizeof(command1), "sudo ip link set %s type can bitrate 1000000 sample-point 0.75", port);
    snprintf(command2, sizeof(command2), "sudo ip link set %s up", port);
    
    system(command1);
    system(command2);
}

void CanSocketUtils::list_and_activate_available_can_ports()
{
    FILE* fp = popen("ip link show | grep can", "r");
    if (fp == nullptr) {
        perror("No available CAN port");
        exit(1);
    }

    char output[1024];
    while (fgets(output, sizeof(output) - 1, fp) != nullptr) {
        std::string line(output);
        std::istringstream iss(line);
        std::string port;
        iss >> port;
        port = port.substr(0, port.size() - 1); // 콜론 제거

        if (is_port_up(port.c_str())) {
            printf("%s is already UP\n", port.c_str());
        } else {
            printf("%s is DOWN, activating...\n", port.c_str());
            activate_port(port.c_str());
        }
    }
    pclose(fp);
}


void CanSocketUtils::writeRawCANDataToCSV(struct can_frame *received_frames, int actual_frames, const char *folder_path, const char *file_name)
{
    // 파일 경로와 이름을 합친다.
    char full_path[512];
    snprintf(full_path, sizeof(full_path), "%s/%s", folder_path, file_name);

    FILE *file = fopen(full_path, "w");
    if (file == NULL)
    {
        printf("Failed to open file at %s. Attempting to create it.\n", full_path);

        // 폴더가 존재하지 않을 수도 있으니 폴더를 먼저 생성
        char command[512];
        snprintf(command, sizeof(command), "mkdir -p %s", folder_path);

        int ret = system(command);
        if (ret == -1)
        {
            printf("System command failed: %s\n", command);
            return;
        }

        // 다시 파일을 열어본다.
        file = fopen(full_path, "w");
        if (file == NULL)
        {
            // 여전히 실패하면 에러 메시지를 출력하고 함수를 종료한다.
            printf("Failed to create file at %s.\n", full_path);
            return;
        }
    }

    // CSV 파일 헤더 작성
    fprintf(file, "can_id,can_dlc,data\n");

    // 모든 CAN 프레임을 순회하면서 CSV 파일에 쓴다.
    for (int i = 0; i < actual_frames; i++)
    {
        struct can_frame frame = received_frames[i];
        fprintf(file, "%x,%d,", frame.can_id, frame.can_dlc);

        for (int j = 0; j < frame.can_dlc; j++)
        {
            fprintf(file, "%02x", frame.data[j]);
            if (j < frame.can_dlc - 1)
            {
                fprintf(file, " ");
            }
        }
        fprintf(file, "\n");
    }

    fclose(file);
}

void CanSocketUtils::send_frame_and_receive_reply(int hsocket, struct can_frame *frame)
{
    struct timeval timeout;
    timeout.tv_sec = 0;  // 1초
    timeout.tv_usec = 50000; // 0마이크로초

    // 타임아웃 설정
    if (setsockopt(hsocket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
    {
        printf("setsockopt failed\n");
    }

// 현재 can_frame의 ID와 데이터 출력
    printf("Before sending - CAN Frame ID: 0x%X, Data: ", frame->can_id);
    for (int i = 0; i < frame->can_dlc; i++) {
        printf("%02X ", frame->data[i]);
    }
    printf("\n");

    int result;
    result = write(hsocket, frame, sizeof(struct can_frame));
    if (result <= 0)
    {
        printf("send error, errno: %d, strerror: %s\n", errno, strerror(errno));
        return;
    }

    result = read(hsocket, frame, sizeof(struct can_frame));
    if (result <= 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK) // 타임아웃 발생시
        {
            printf("recv timeout\n");
        }
        else
        {
            printf("recv error, errno: %d, strerror: %s\n", errno, strerror(errno));
        }
        return;
    }

    // 수신한 후의 can_frame의 ID와 데이터 출력
    printf("After receiving - CAN Frame ID: 0x%X, Data: ", frame->can_id);
    for (int i = 0; i < frame->can_dlc; i++) {
        printf("%02X ", frame->data[i]);
    }
    printf("\n");
}