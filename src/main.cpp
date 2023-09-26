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

    // Motor check
    MotorLimits motorlimits;
    MotorLimits *p_limits;
    p_limits = get_motor_limits("AK10_9");
    motorlimits = *p_limits;

    struct can_frame can_frame;

    int tmotor_ids[] = {2, 3};
    int t_num_motor = sizeof(tmotor_ids) / sizeof(tmotor_ids[0]);

    for (int i = 0; i < t_num_motor; ++i)
    {
        MotorCommand command;
        command.motor_id = tmotor_ids[i];
        command.p_des = 0;
        command.v_des = 0;
        command.kp = 8;
        command.kd = 1;
        command.t_ff = 0;

        // Check available motor
        pack_cmd(motorlimits, &can_frame, command.motor_id, 0, 0, 0, 0, 0); // zero command
        send_frame_and_receive_reply(can_socket, &can_frame);
        printf("motor %d connected\n", tmotor_ids[i]);
        usleep(10000);

        // Enter Controlmode
        enterControlmode(&can_frame, command.motor_id);
        send_frame_and_receive_reply(can_socket, &can_frame);
        printf("motor %d enter control mode\n", tmotor_ids[i]);
        usleep(50000);
    }

    // Maxon Motor initialization

    // Enter CSP mode
    setNodeCSPMode(&can_frame, 0x603);
    send_frame_and_receive_reply(can_socket, &can_frame);
    usleep(50000);

    // Set torqueoffset
    setNodeTorqueOffset(&can_frame, 0x603);
    send_frame_and_receive_reply(can_socket, &can_frame);
    usleep(50000);

    // Set postionoffset
    setNodePositionOffset(&can_frame, 0x603);
    send_frame_and_receive_reply(can_socket, &can_frame);
    usleep(50000);

    // pre-operation -> operational (PDO enable)
    setOperational(&can_frame, 0x00);
    send_frame_and_receive_reply(can_socket, &can_frame);
    usleep(50000);

    // Shut Down
    controlWordShutdown(&can_frame, 0x203);
    send_frame_and_receive_reply(can_socket, &can_frame);
    usleep(50000);

    // Sync
    Sync(&can_frame);
    send_frame_and_receive_reply(can_socket, &can_frame);
    usleep(50000);

    // Enable CSP mode
    controlWordSwitchOnEnable(&can_frame, 0x203);
    send_frame_and_receive_reply(can_socket, &can_frame);
    usleep(50000);

    // Sync
    Sync(&can_frame);
    send_frame_and_receive_reply(can_socket, &can_frame);
    usleep(50000);

    // 공통 변수 및 설정
    clock_t external = clock();
    int should_exit = 0;

    int can_ids[] = {0x303, 2, 3}; // 예시 CAN ID. 실제 ID로 교체 필요
    int num_motors = sizeof(can_ids) / sizeof(can_ids[0]);
    int cycle_count = 0;
    float sample_time = 0.005; // 5ms

    float total_times[] = {2.0, 2.0, 2.0}; // 각 모터의 총 주기 시간
                                           // 모터의 개수
    float local_times[num_motors];         // 각 모터의 로컬 시간을 저장할 배열
    float target_positions[num_motors];    // 각 모터의 타겟 위치를 저장할 배열

    // 초기화 코드...
    for (int i = 0; i < num_motors; ++i)
    {
        local_times[i] = 0.0;
    }

    char c = 'a';

    while (!should_exit)
    {
        while (1)
        {

            clock_t internal = clock();
            double elapsed_time = ((double)(internal - external)) / CLOCKS_PER_SEC * 1000;
            if (elapsed_time >= (sample_time * 1000))
            {
                // 시간 초기화
                external = clock();
                for (int i = 0; i < num_motors; ++i)
                {
                    // 각 모터의 로컬 시간 업데이트
                    local_times[i] = fmod(local_times[i] + sample_time, total_times[i]);

                    // 주기가 끝났으면 cycle_count 증가
                    if (fabs(local_times[i] - total_times[i]) < 1e-5)
                    {
                        cycle_count++;
                    }
                    // 사인 함수 계산
                    target_positions[i] = sinf(2 * PI * local_times[i] / total_times[i]);

                    // 패키지 생성 및 전송
                    // Maxon의 경우
                    if (can_ids[i] == 0x303)
                    {
                        controlWordTargetPosition(&can_frame, can_ids[i], (int)(target_positions[i] * 35840));
                        send_no_read(can_socket, &can_frame);
                        Sync(&can_frame);
                        send_receive(can_socket, &can_frame);
                    }
                    // Tmotor의 경우
                    else
                    {
                        pack_cmd(motorlimits, &can_frame, can_ids[i], (float)(target_positions[i] * PI / 2), 0, 8, 1, 0);
                        send_receive(can_socket, &can_frame);
                    }
                }
            }
            // 3번 주기를 완료했으면 should_exit를 1로 설정해 루프 탈출
            if (cycle_count >= 10)
            {
                should_exit = 1;
                break;
            }

            // 종료 조건 체크 (예: 키보드 입력)
            if (kbhit())
            {
                c = getchar();
                if (c == 'q' || c == 'Q')
                {
                    printf("Exit control mode.\n");
                    should_exit = 1;
                    break;
                }
            }
        }

    }
    // Exit Tmotor
    for (int i = 0; i < t_num_motor; ++i)
    {
        // Zero command before exit
        pack_cmd(motorlimits, &can_frame, tmotor_ids[i], 0, 0, 0, 0, 0);
        send_frame_and_receive_reply(can_socket, &can_frame);
        usleep(10000);

        // Exit Control Mode
        exitControlmode(&can_frame, tmotor_ids[i]);
        send_frame_and_receive_reply(can_socket, &can_frame);
        printf("motor %d exit\n", tmotor_ids[i]);
    }

    // Close socket
    close(can_socket);

    return 0;
}
