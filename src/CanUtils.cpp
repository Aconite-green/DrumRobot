// can_utils.c 파일
#include "../include/CanUtils.hpp"

void printframe(struct can_frame *frame)
{
    printf("recv : %d ", frame->can_id);

    for (int i = 0; i < frame->can_dlc; i++)
    {
        printf("0x%02X ", frame->data[i]);
    }

    printf("\n");
}

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

void send_frame_and_receive_reply(int hsocket, struct can_frame *frame)
{
    struct timeval timeout;
    timeout.tv_sec = 0;      // 1초
    timeout.tv_usec = 5000; // 0마이크로초

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

void send_receive(int hsocket, struct can_frame *frame)
{
    struct timeval timeout;
    timeout.tv_sec = 0;      // 1초
    timeout.tv_usec = 5000; // 0마이크로초

    // 타임아웃 설정
    if (setsockopt(hsocket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
    {
        printf("setsockopt failed\n");
    }

    

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

}

void send_frame(int hsocket, struct can_frame *frame)
{

    // 현재 can_frame의 ID와 데이터 출력
    printf("Only Send - CAN Frame ID: 0x%X, Data: ", frame->can_id);
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
}

void send_no_read(int hsocket, struct can_frame *frame)
{

    int result;
    result = write(hsocket, frame, sizeof(struct can_frame));
    if (result <= 0)
    {
        printf("send error, errno: %d, strerror: %s\n", errno, strerror(errno));
        return;
    }
}

// 각각의 명령어를 나타내는 구조체 배열을 반환
// CSV 파일에서 각 줄의 값을 읽어 struct can_frame으로 저장하는 함수

// sendFramesPeriodically 함수
void sendFramesPeriodically(int can_socket, struct can_frame *frames, int num_frames, int period_ms, struct can_frame **received_frames_ptr, int *actual_frames_ptr)
{
    *received_frames_ptr = (struct can_frame *)malloc(num_frames * sizeof(struct can_frame));
    if (*received_frames_ptr == NULL)
    {
        printf("Memory allocation for received_frames failed.\n");
        return;
    }

    struct can_frame *received_frames = *received_frames_ptr;
    *actual_frames_ptr = 0;

    clock_t external = clock();
    int current_frame = 0;
    int should_exit = 0;

    while (!should_exit)
    {
        while (1)
        {
            clock_t internal = clock();
            double elapsed_time = (double)(internal - external) / (double)CLOCKS_PER_SEC * 1000;
            if (elapsed_time > period_ms)
            {
                // 데이터 전송과 수신
                send_frame_and_receive_reply(can_socket, &frames[current_frame]);

                // 수신 데이터 저장
                received_frames[*actual_frames_ptr] = frames[current_frame];
                (*actual_frames_ptr)++;

                current_frame++;
                if (current_frame >= num_frames)
                {
                    should_exit = 1;
                    break;
                }

                external = clock();
                break;
            }
        }

        if (kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
                printf("Exit control mode.\n");
                break;
            }
        }
    }
}


void writeRawCANDataToCSV(struct can_frame *received_frames, int actual_frames, const char *folder_path, const char *file_name)
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






