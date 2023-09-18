// can_utils.c 파일
#include "can_utils.h"

int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    if (x < x_min)
        x = x_min;
    else if (x > x_max)
        x = x_max;
    return (int)((x - x_min) * ((float)((1 << bits) / span)));
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// Sends routine code
void pack_cmd(MotorLimits limits, struct can_frame *frame, int can_id, float p_des, float v_des, float kp, float kd, float t_ff)
{
    p_des = fminf(fmaxf(limits.P_MIN, p_des), limits.P_MAX);
    v_des = fminf(fmaxf(limits.V_MIN, v_des), limits.V_MAX);
    kp = fminf(fmaxf(limits.Kp_MIN, kp), limits.Kp_MAX);
    kd = fminf(fmaxf(limits.Kd_MIN, kd), limits.Kd_MAX);
    t_ff = fminf(fmaxf(limits.T_MIN, t_ff), limits.T_MAX);

    /// convert floats to unsigned ints ///
    int p_int = float_to_uint(p_des, limits.P_MIN, limits.P_MAX, 16);
    int v_int = float_to_uint(v_des, limits.V_MIN, limits.V_MAX, 12);
    int kp_int = float_to_uint(kp, limits.Kp_MIN, limits.Kp_MAX, 12);
    int kd_int = float_to_uint(kd, limits.Kd_MIN, limits.Kd_MAX, 12);
    int t_int = float_to_uint(t_ff, limits.T_MIN, limits.T_MAX, 12);

    // Set CAN frame id and data length code
    frame->can_id = can_id & CAN_SFF_MASK; // Replace YOUR_CAN_ID with the appropriate id
    frame->can_dlc = 8;                    // Data Length Code is set to maximum allowed length

    /// pack ints into the can buffer ///
    frame->data[0] = p_int >> 8;                           // Position 8 higher
    frame->data[1] = p_int & 0xFF;                         // Position 8 lower
    frame->data[2] = v_int >> 4;                           // Speed 8 higher
    frame->data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); // Speed 4 bit lower KP 4bit higher
    frame->data[4] = kp_int & 0xFF;                        // KP 8 bit lower
    frame->data[5] = kd_int >> 4;                          // Kd 8 bit higher
    frame->data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); // KP 4 bit lower torque 4 bit higher
    frame->data[7] = t_int & 0xff;                         // torque 4 bit lower
}

void unpack_reply(MotorLimits limits, struct can_frame *frame, RecvMotorInfo *info)
{
    /// unpack ints from can buffer ///
    info->id = frame->data[0];
    int p_int = (frame->data[1] << 8) | frame->data[2];
    int v_int = (frame->data[3] << 4) | (frame->data[4] >> 4);
    int i_int = ((frame->data[4] & 0xF) << 8) | frame->data[5];

    /// convert ints to floats ///
    info->position = uint_to_float(p_int, limits.P_MIN, limits.P_MAX, 16);
    info->speed = uint_to_float(v_int, limits.V_MIN, limits.V_MAX, 12);
    info->torque = uint_to_float(i_int, limits.T_MIN, limits.T_MAX, 12);
}

void enterControlmode(struct can_frame *frame, int can_id)
{
    // Set CAN frame id and data length code
    frame->can_id = can_id & CAN_SFF_MASK; // Replace YOUR_CAN_ID with the appropriate id

    frame->can_dlc = 8; // Data Length Code is set to maximum allowed length
    /// pack ints into the can buffer ///
    frame->data[0] = 0xFF; // Position 8 higher
    frame->data[1] = 0xFF; // Position 8 lower
    frame->data[2] = 0xFF; // Speed 8 higher
    frame->data[3] = 0xFF; // Speed 4 bit lower KP 4bit higher
    frame->data[4] = 0xFF; // KP 8 bit lower
    frame->data[5] = 0xFF; // Kd 8 bit higher
    frame->data[6] = 0xFF; // KP 4 bit lower torque 4 bit higher
    frame->data[7] = 0xFC; // torque 4 bit lower
}

void set_to_zero(struct can_frame *frame, int can_id)
{
    /// pack ints into the can buffer ///
    frame->data[0] = 0xFF; // Position 8 higher
    frame->data[1] = 0xFF; // Position 8 lower
    frame->data[2] = 0xFF; // Speed 8 higpaher
    frame->data[3] = 0xFF; // Speed 4 bit lower KP 4bit higher
    frame->data[4] = 0xFF; // KP 8 bit lower
    frame->data[5] = 0xFF; // Kd 8 bit higher
    frame->data[6] = 0xFF; // KP 4 bit lower torque 4 bit higher
    frame->data[7] = 0xFE; // torque 4 bit lower

    // Set CAN frame id and data length code
    frame->can_id = can_id & CAN_SFF_MASK; // Replace YOUR_CAN_ID with the appropriate id
    frame->can_dlc = 8;                    // Data Length Code is set to maximum allowed length
}

void exitControlmode(struct can_frame *frame, int can_id)
{
    // Set CAN frame id and data length code
    frame->can_id = can_id & CAN_SFF_MASK; // Replace YOUR_CAN_ID with the appropriate id
    frame->can_dlc = 8;                    // Data Length Code is set to maximum allowed length
    /// pack ints into the can buffer ///
    frame->data[0] = 0xFF; // Position 8 higher
    frame->data[1] = 0xFF; // Position 8 lower
    frame->data[2] = 0xFF; // Speed 8 higher
    frame->data[3] = 0xFF; // Speed 4 bit lower KP 4bit higher
    frame->data[4] = 0xFF; // KP 8 bit lower
    frame->data[5] = 0xFF; // Kd 8 bit higher
    frame->data[6] = 0xFF; // KP 4 bit lower torque 4 bit higher
    frame->data[7] = 0xFD; // torque 4 bit lower
}

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

#include <sys/socket.h> // setsockopt을 위한 헤더
#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

void send_frame_and_receive_reply(int hsocket, struct can_frame *frame)
{
    struct timeval timeout;
    timeout.tv_sec = 1;  // 1초
    timeout.tv_usec = 0; // 0마이크로초

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

    // 정상적으로 데이터 수신
    // 여기에 로직 추가
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

    // 줄 수 세기
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
    rewind(file); // 파일 포인터를 처음으로 돌림

    *num_frames = lines;
    *frames_ptr = (struct can_frame *)malloc(sizeof(struct can_frame) * (*num_frames));
    if (*frames_ptr == NULL)
    {
        printf("Memory allocation failed\n");
        return;
    }

    struct can_frame *prepacked_frames = *frames_ptr;

    // CSV 파일에서 각 줄을 읽어와서 can_frame에 저장
    float p_des, v_des, kp, kd, t_ff;
    int can_id;
    for (int i = 0; i < *num_frames; i++)
    {
        if (fscanf(file, "%d,%f,%f,%f,%f,%f\n", &can_id, &p_des, &v_des, &kp, &kd, &t_ff) != 6)
        {
            printf("Failed to read a line from the CSV file\n");
            free(prepacked_frames); // 이미 할당된 메모리 해제
            return;
        }
        pack_cmd(limits, &(prepacked_frames[i]), can_id, p_des, v_des, kp, kd, t_ff);
    }

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
void sendFramesPeriodically(int can_socket, struct can_frame *frames, int num_frames, int period_ms, MotorLimits limits, struct can_frame **received_frames_ptr, int *actual_frames_ptr)
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

void controlWordTargetPosition(struct can_frame *frame, int can_id, float angle)
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

void Sync(struct can_frame *frame, int can_id)
{
    frame->can_id = can_id & CAN_SFF_MASK;
    frame->can_dlc = 0;
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
