// can_utils.c 파일
#include "../include/can_utils.hpp"

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
void prepack_frames_from_csv(const char *filepath, MotorLimits limits, struct can_frame **frames_ptr, int *num_frames)
{
    FILE *file = fopen(filepath, "r");
    if (file == NULL)
    {
        printf("Failed to open file\n");
        return;
    }

    char ch;
    int lines = 0;
    while (!feof(file))
    {
        ch = fgetc(file);
        if (ch == '\n')
        {
            lines++;
        }
    }
    rewind(file);

    *num_frames = lines;
    *frames_ptr = (struct can_frame *)malloc(sizeof(struct can_frame) * (*num_frames));
    if (*frames_ptr == NULL)
    {
        printf("Memory allocation failed\n");
        return;
    }

    struct can_frame *prepacked_frames = *frames_ptr;

    char motor_type[100];
    float p_des, v_des, kp, kd, t_ff;
    int can_id, targetPosition;

    int sync_frame_count = 0; // Maxon 모터에 대한 sync frame을 카운트하기 위한 변수

    for (int i = 0; i < *num_frames; i++)
    {
        if (fscanf(file, "%s,%d,", motor_type, &can_id) != 2)
        {
            printf("Failed to read a line from the CSV file\n");
            free(prepacked_frames);
            return;
        }

        if (strcmp(motor_type, "Tmotor") == 0)
        {
            if (fscanf(file, "%f,%f,%f,%f,%f\n", &p_des, &v_des, &kp, &kd, &t_ff) != 5)
            {
                printf("Failed to read Tmotor parameters from the CSV file\n");
                free(prepacked_frames);
                return;
            }
            pack_cmd(limits, &(prepacked_frames[i]), can_id, p_des, v_des, kp, kd, t_ff);
        }
        else if (strcmp(motor_type, "Maxon") == 0)
        {
            if (fscanf(file, "%d\n", &targetPosition) != 1)
            {
                printf("Failed to read Maxon parameters from the CSV file\n");
                free(prepacked_frames);
                return;
            }
            controlWordTargetPosition(&(prepacked_frames[i]), can_id, targetPosition);

            // Maxon 모터에 대한 sync frame 추가
            sync_frame_count++;
            *frames_ptr = (struct can_frame *)realloc(*frames_ptr, sizeof(struct can_frame) * (*num_frames + sync_frame_count));
            struct can_frame *sync_frame = &((*frames_ptr)[*num_frames + sync_frame_count - 1]);
            sync_frame->can_id = 0x80;
            sync_frame->can_dlc = 1;
            sync_frame->data[0] = 0x00;
        }
        else
        {
            printf("Unknown motor type\n");
            free(prepacked_frames);
            return;
        }
    }

    // 전체 frame 수 업데이트
    *num_frames += sync_frame_count;

    fclose(file);
}

void postProcessReceivedData(struct can_frame *received_frames, int actual_frames, MotorLimits limits, const char *folder_path, const char *file_name)
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

    RecvMotorInfo info;
    for (int i = 0; i < actual_frames; i++)
    {
        unpack_reply(limits, &received_frames[i], &info);
        fprintf(file, "%d,%f,%f,%f\n", info.id, info.position, info.speed, info.torque);
    }

    fclose(file);
}

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

#include <stdio.h>
#include <stdlib.h>

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

////////////////////////////Epos4 CSP

void setOperational(struct can_frame *frame, int can_id)
{
    frame->can_id = can_id & CAN_SFF_MASK; // Replace YOUR_CAN_ID with the appropriate id
    frame->can_dlc = 8;                    // Data Length Code is set to maximum allowed length
    frame->data[0] = 0x01;                 // Data[0]
    frame->data[1] = 0x00;                 // Data[1]
    frame->data[2] = 0x00;                 // Data[2]
    frame->data[3] = 0x00;                 // Data[3]
    frame->data[4] = 0x00;                 // Data[4]
    frame->data[5] = 0x00;                 // Data[5]
    frame->data[6] = 0x00;                 // Data[6]
    frame->data[7] = 0x00;                 // Data[7]
}

void controlWordShutdown(struct can_frame *frame, int can_id)
{
    frame->can_id = can_id & CAN_SFF_MASK;
    frame->can_dlc = 8;
    frame->data[0] = 0x06;
    frame->data[1] = 0x00;
    frame->data[2] = 0x00;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void controlWordSwitchOnEnable(struct can_frame *frame, int can_id)
{
    frame->can_id = can_id & CAN_SFF_MASK;
    frame->can_dlc = 8;
    frame->data[0] = 0x0F;
    frame->data[1] = 0x00;
    frame->data[2] = 0x00;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void controlWordTargetPosition(struct can_frame *frame, int can_id, int targetPosition)
{

    // 10진수 targetPosition을 16진수로 변환
    unsigned char posByte0 = targetPosition & 0xFF;         // 하위 8비트
    unsigned char posByte1 = (targetPosition >> 8) & 0xFF;  // 다음 8비트
    unsigned char posByte2 = (targetPosition >> 16) & 0xFF; // 다음 8비트
    unsigned char posByte3 = (targetPosition >> 24) & 0xFF; // 최상위 8비트

    frame->can_id = can_id & CAN_SFF_MASK;
    frame->can_dlc = 4;
    frame->data[0] = posByte0;
    frame->data[1] = posByte1;
    frame->data[2] = posByte2;
    frame->data[3] = posByte3;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void Sync(struct can_frame *frame)
{
    frame->can_id = 0x80;
    frame->can_dlc = 1;
    frame->data[0] = 0x00;
    frame->data[1] = 0x00;
    frame->data[2] = 0x00;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void setNodeCSPMode(struct can_frame *frame, int can_id)
{
    frame->can_id = can_id;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0x60;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x08;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void setNodeTorqueOffset(struct can_frame *frame, int can_id)
{
    frame->can_id = can_id;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0xB2;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void setNodePositionOffset(struct can_frame *frame, int can_id)
{
    frame->can_id = can_id;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0xB0;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

///////////////////////////Epos4 PPM

// Epos4 2EA Profile Position Mode로 변경
void setEpos4ProfilePositionMode(struct can_frame *frame, int can_id)
{
    frame->can_id = can_id;
    frame->can_dlc = 8; // Data length
    frame->data[0] = 0x22;
    frame->data[1] = 0x60;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x01;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

// Epos4 2EA Shut down
void setEpos4ShutDown(struct can_frame *frame, int can_id)
{
    frame->can_id = can_id;
    frame->can_dlc = 8; // Data length
    frame->data[0] = 0x22;
    frame->data[1] = 0x40;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x06;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

// Epos4 2EA Enable
void setEpos4Enable(struct can_frame *frame, int can_id)
{
    frame->can_id = can_id;
    frame->can_dlc = 8; // Data length
    frame->data[0] = 0x22;
    frame->data[1] = 0x40;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x0F;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void setTargetPosition(struct can_frame *frame, int can_id, float angle)
{
    // 각도를 입력받아 모터가 이동해야 할 엔코더 값으로 변환
    int encoder_per_rotation = 4096;
    float gear_ratio = 35.0;
    int targetPosition = (int)(angle / 360.0 * encoder_per_rotation * gear_ratio);

    // 10진수 targetPosition을 16진수로 변환
    unsigned char posByte0 = targetPosition & 0xFF;         // 하위 8비트
    unsigned char posByte1 = (targetPosition >> 8) & 0xFF;  // 다음 8비트
    unsigned char posByte2 = (targetPosition >> 16) & 0xFF; // 다음 8비트
    unsigned char posByte3 = (targetPosition >> 24) & 0xFF; // 최상위 8비트

    frame->can_id = can_id;
    frame->can_dlc = 8; // Data length
    frame->data[0] = 0x22;
    frame->data[1] = 0x7A;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = posByte0;
    frame->data[5] = posByte1;
    frame->data[6] = posByte2;
    frame->data[7] = posByte3;
}

// Start Positioning
void startEpos4Positioning(struct can_frame *frame, int can_id)
{
    frame->can_id = can_id & CAN_SFF_MASK;
    frame->can_dlc = 8;
    frame->data[0] = 0x1F;
    frame->data[1] = 0x00;
    frame->data[2] = 0x00;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}
