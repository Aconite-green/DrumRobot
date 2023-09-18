#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main()
{
    // 모터의 개수와 CAN ID 설정
    int num_motors = 5;
    int can_ids[] = {2, 3, 4, 5, 6}; // 예시 CAN ID
    float total_times[] = {8.0, 4.0, 4.0, 2.0, 2.0}; // 각 모터별 sine wave 주기

    struct stat st = {0};
    if (stat("testpath", &st) == -1)
    {
        mkdir("testpath", 0700);
    }

    FILE *file = fopen("./command/5_tomtor.csv", "w");
    if (file == NULL)
    {
        printf("Failed to open file\n");
        return 1;
    }

    float kp = 8.0;
    float kd = 2.0;
    float t_ff = 0.0;
    float sample_time = 0.005;
    int cycles = 5;

    // 가장 긴 주기를 최종 동작 시간으로 설정
    float max_time = 8.0; 
    int max_samples = (int)(max_time / sample_time);

    for (int cycle = 0; cycle < cycles; cycle++)
    {
        for (int i = 0; i < max_samples; i++)
        {
            float time = i * sample_time;

            for (int j = 0; j < num_motors; ++j)
            {
                float local_time = fmod(time, total_times[j]); // 주기에 맞게 시간을 조정
                float p_des = sinf(2 * M_PI * local_time / total_times[j]) * M_PI/2;
                float v_des = cosf(2 * M_PI * local_time / total_times[j]) * M_PI/2;

                fprintf(file, "%d,%.4f,%.4f,%.4f,%.4f,%.4f\n", can_ids[j], p_des, v_des, kp, kd, t_ff);
            }
        }
    }

    fclose(file);

    return 0;
}
