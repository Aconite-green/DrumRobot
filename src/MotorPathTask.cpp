#include <iostream>    // for std::cerr
#include <cmath>       // for sinf(), fmod(), M_PI
#include <ctime>       // for clock(), CLOCKS_PER_SEC/
#include <linux/can.h> // for can_frame
#include "../include/MotorPathTask.hpp"
#include "../include/SharedBuffer.hpp"

void MotorPathTask::operator()(SharedBuffer<can_frame> &buffer)
{

    // TMotor 객체 배열 생성
    TMotor motors[] = {
        TMotor(1, "AK10_9", "Waist"),
    };

    // 배열 길이를 numMotor 변수에 저장
    int numMotor = sizeof(motors) / sizeof(TMotor);

    float total_times[] = {8}; // 각 모터별 sine wave 주기
    int numTotalTimes = sizeof(total_times) / sizeof(float);

    if (numMotor != numTotalTimes)
    {
    
        std::cerr << "Error: The number of motors does not match the number of total_times entries.\n";
        return;
    }


    float sample_time = 0.005;
    int cycles = 5;
    float max_time = 8.0;
    int max_samples = static_cast<int>(max_time / sample_time);

    for (int cycle = 0; cycle < cycles; cycle++)
    {
        for (int i = 0; i < max_samples; i++)
        {
            float time = i * sample_time;

            for (int j = 0; j < numMotor; ++j)
            {

                float local_time = std::fmod(time, total_times[j]); // 배열 인덱스 j를 사용
                float p_des = sinf(2 * M_PI * local_time / total_times[j]) * M_PI / 2;
                float v_des = cosf(2 * M_PI * local_time / total_times[j]) * M_PI / 2;

                Parser.parseSendCommand(motors[j], &frame, motors[j].nodeId, 8, p_des, v_des, 8, 1, 0);

                buffer.push(frame);
            }
        }
    }
}
