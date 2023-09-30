#include "../include/CanSocketUtils.hpp"

void CanSocketUtils::check(int result, const char *errMsg, int errCode)
{
    if (result == -1)
    {
        perror(errMsg);
        exit(errCode);
    }
}

CanSocketUtils::CanSocketUtils(const char* ifname) {
    this->ifname = ifname;

    if (!is_port_up(ifname)) {
        std::cerr << "Port " << ifname << " is not up. Activating..." << std::endl;
        activate_port(ifname);
    }

    hsocket = create_socket();
    if (hsocket < 0) {
        std::cerr << "Failed to create socket. Exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }
}

CanSocketUtils::~CanSocketUtils() {
    close(hsocket);
}



int CanSocketUtils::create_socket() {
    int result;
    struct sockaddr_can addr;
    struct ifreq ifr;

    hsocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (hsocket < 0) {
        return ERR_SOCKET_CREATE_FAILURE;
    }

    memset(&ifr, 0, sizeof(struct ifreq));
    memset(&addr, 0, sizeof(struct sockaddr_can));

    strcpy(ifr.ifr_name, ifname.c_str());
    result = ioctl(hsocket, SIOCGIFINDEX, &ifr);
    if (result < 0) {
        return ERR_SOCKET_CREATE_FAILURE;
    }

    addr.can_ifindex = ifr.ifr_ifindex;
    addr.can_family = AF_CAN;

    if (bind(hsocket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        return ERR_SOCKET_CREATE_FAILURE;
    }

    return hsocket;
}

bool CanSocketUtils::is_port_up(const char *port)
{
    char command[50];
    snprintf(command, sizeof(command), "ip link show %s", port);

    FILE *fp = popen(command, "r");
    if (fp == NULL)
    {
        perror("Error opening pipe");
        return false;
    }

    char output[1024];
    if (fgets(output, sizeof(output) - 1, fp) == NULL)
    {
        perror("fgets failed");
        pclose(fp);
        return false;
    }
    pclose(fp);

    if (strstr(output, "DOWN"))
    {
        return false;
    }
    else if (strstr(output, "UP"))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void CanSocketUtils::activate_port(const char *port)
{
    char command1[100], command2[100];
    snprintf(command1, sizeof(command1), "sudo ip link set %s type can bitrate 1000000 sample-point 0.75", port);
    snprintf(command2, sizeof(command2), "sudo ip link set %s up", port);

    int ret1 = system(command1);
    int ret2 = system(command2);

    if (ret1 != 0 || ret2 != 0)
    {
        fprintf(stderr, "Failed to activate port: %s\n", port);
        exit(1); // 또는 다른 에러 처리
    }
}

void CanSocketUtils::list_and_activate_available_can_ports()
{
    int portCount = 0; // CAN 포트 수를 세기 위한 변수

    FILE *fp = popen("ip link show | grep can", "r");
    if (fp == nullptr)
    {
        perror("No available CAN port");
        exit(1);
    }

    char output[1024];
    while (fgets(output, sizeof(output) - 1, fp) != nullptr)
    {
        portCount++; // CAN 포트를 발견할 때마다 카운트 증가
        std::string line(output);
        std::istringstream iss(line);
        std::string port;
        iss >> port;
        port = port.substr(0, port.size() - 1); // 콜론 제거

        if (is_port_up(port.c_str()))
        {
            printf("%s is already UP\n", port.c_str());
        }
        else
        {
            printf("%s is DOWN, activating...\n", port.c_str());
            activate_port(port.c_str());
        }
    }
    pclose(fp);

    // CAN 포트를 하나도 발견하지 못했으면 프로그램 종료
    if (portCount == 0)
    {
        printf("No CAN port found. Exiting...\n");
        exit(1);
    }
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

void CanSocketUtils::send_frame_and_receive_reply(struct can_frame *frame)
{
    struct timeval timeout;
    timeout.tv_sec = 0;      // 1초
    timeout.tv_usec = 50000; // 0마이크로초

    // 타임아웃 설정
    if (setsockopt(hsocket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
    {
        printf("setsockopt failed\n");
    }

    // 현재 can_frame의 ID와 데이터 출력
    printf("Before sending - CAN Frame ID: 0x%X, Data: ", frame->can_id);
    for (int i = 0; i < frame->can_dlc; i++)
    {
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
    for (int i = 0; i < frame->can_dlc; i++)
    {
        printf("%02X ", frame->data[i]);
    }
    printf("\n");
}