#include <iostream>    // for std::cerr
#include <cmath>       // for sinf(), fmod(), M_PI
#include <ctime>       // for clock(), CLOCKS_PER_SEC/
#include <linux/can.h> // for can_frame
#include "../include/MotorPathTask.hpp"
#include "../include/SharedBuffer.hpp"
#include <map>
#include <memory>

#define Tdegree_180 M_PI
#define Tdegree_90 M_PI/2
#define Mdegree_180 2048*35
#define Mdegree_90 1024*35



MotorPathTask::MotorPathTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors)
    : tmotors(tmotors), maxonMotors(maxonMotors)
{
}

void MotorPathTask::operator()(SharedBuffer<can_frame> &buffer)
{

    std::map<std::string, std::pair<float, float>> motor_configurations = {
        {"1_waist", {8, Tdegree_180}},
        {"2_R_arm1", {8, Tdegree_180}},
        {"3_L_arm1", {8, Tdegree_180}},
        {"4_R_arm2", {8, Tdegree_180}},
        {"5_R_arm3", {8, Tdegree_180}},
        {"6_L_arm2", {8, Tdegree_180}},
        {"7_L_arm3", {8, Tdegree_180}},
        {"a_maxon", {8, Mdegree_180}},
        {"b_maxon", {8, Mdegree_180}}};

    struct can_frame frame;
    if ((tmotors.size() + maxonMotors.size()) != motor_configurations.size())
    {
        std::cerr << "Error: The number of motors does not match the number of total_times entries.\n";
        return;
    }

    float sample_time = 0.002;
    int cycles = 5;
    float max_time = std::max_element(motor_configurations.begin(), motor_configurations.end(),
                                      [](const auto &a, const auto &b)
                                      {
                                          return a.second.first < b.second.first;
                                      })
                         ->second.first; // 모든 모터 중 가장 긴 주기를 max_time으로 설정합니다.

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

                if (motor_configurations.find(motor_name) == motor_configurations.end())
                {
                    std::cerr << "Error: Configuration for motor " << motor_name << " not found.\n";
                    continue;
                }

                float local_time = std::fmod(time, motor_configurations[motor_name].first); // 주기
                float amplitude = motor_configurations[motor_name].second;                  // 진폭
                float p_des = (1 - cosf(2 * M_PI * local_time / motor_configurations[motor_name].first))/2 * amplitude;

                TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 50, 1, 0);

                buffer.push(frame);
            }
            if (!maxonMotors.empty())
            {
                for (auto &entry : maxonMotors)
                {
                    const std::string &motor_name = entry.first;
                    std::shared_ptr<MaxonMotor> &motor = entry.second;

                    if (motor_configurations.find(motor_name) == motor_configurations.end())
                    {
                        std::cerr << "Error: Configuration for motor " << motor_name << " not found.\n";
                        continue;
                    }

                    float local_time = std::fmod(time, motor_configurations[motor_name].first); // 주기
                    float amplitude = motor_configurations[motor_name].second;
                    int p_des = (1 - cosf(2 * M_PI * local_time / motor_configurations[motor_name].first))/2 * amplitude;
                    MParser.parseSendCommand(*motor, &frame, p_des);
                    buffer.push(frame);
                }
                MParser.makeSync(&frame);

                buffer.push(frame);
            }
        }
    }
}
