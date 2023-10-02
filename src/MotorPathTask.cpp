#include <iostream>    // for std::cerr
#include <cmath>       // for sinf(), fmod(), M_PI
#include <ctime>       // for clock(), CLOCKS_PER_SEC/
#include <linux/can.h> // for can_frame
#include "../include/MotorPathTask.hpp"
#include "../include/SharedBuffer.hpp"
#include <map>
#include <memory>

void MotorPathTask::operator()(SharedBuffer<can_frame> &buffer, std::map<std::string, std::shared_ptr<TMotor>> &tmotors)
{
    // total_times와 그에 해당하는 모터의 이름
    std::map<std::string, float> total_times = {{"waist", 8}};

    if (tmotors.size() != total_times.size())
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

            for (auto &entry : tmotors)
            {
                const std::string &motor_name = entry.first;
                std::shared_ptr<TMotor> &motor = entry.second;
                
                if (total_times.find(motor_name) == total_times.end())
                {
                    std::cerr << "Error: total_time for motor " << motor_name << " not found.\n";
                    continue;
                }

                float local_time = std::fmod(time, total_times[motor_name]);
                float p_des = sinf(2 * M_PI * local_time / total_times[motor_name]) * M_PI / 2;
                float v_des = cosf(2 * M_PI * local_time / total_times[motor_name]) * M_PI / 2;

                Parser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, v_des, 8, 1, 0);

                buffer.push(frame);
            }
        }
    }
}
