#include <iostream>    // for std::cerr
#include <cmath>       // for sinf(), fmod(), M_PI
#include <ctime>       // for clock(), CLOCKS_PER_SEC/
#include <linux/can.h> // for can_frame
#include "../include/MotorPathTask.hpp"
#include "../include/SharedBuffer.hpp"
#include <map>
#include <memory>

MotorPathTask::MotorPathTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors)
    : tmotors(tmotors), maxonMotors(maxonMotors)
{
}

void MotorPathTask::operator()(SharedBuffer<can_frame> &buffer)
{

    // CSV 파일을 쓰기 모드로 열기
    std::ofstream csvFile("motor_send_data.csv");
    csvFile << "CAN_ID,p_des\n"; // CSV 헤더

    // total_times는 동적으로 설정 가능하며 모터 이름과 그에 해당하는 주기(초)를 맵핑합니다.
    std::map<std::string, float> total_times = {
        {"1_waist", 8}, {"2_R_arm1", 8}, {"3_L_arm1", 8}, {"4_R_arm2", 8}, {"a_maxon", 8}, {"b_maxon", 8}

    };
    struct can_frame frame;
    if ((tmotors.size() + maxonMotors.size()) != total_times.size())
    {
        std::cerr << "Error: The number of motors does not match the number of total_times entries.\n";
        return;
    }

    float sample_time = 0.005;
    int cycles = 1;
    float max_time = std::max_element(total_times.begin(), total_times.end(),
                                      [](const auto &a, const auto &b)
                                      {
                                          return a.second < b.second;
                                      })
                         ->second; // 모든 모터 중 가장 긴 주기를 max_time으로 설정합니다.

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
                float p_des = (1 - cosf(2 * M_PI * local_time / total_times[motor_name])) * M_PI / 2;
                csvFile << std::hex << motor->nodeId << ',' << p_des << '\n';
                TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 50, 1, 0);

                buffer.push(frame);
            }
            if (!maxonMotors.empty())
            {
                for (auto &entry : maxonMotors)
                {
                    const std::string &motor_name = entry.first;
                    std::shared_ptr<MaxonMotor> &motor = entry.second;

                    if (total_times.find(motor_name) == total_times.end())
                    {
                        std::cerr << "Error: total_time for motor " << motor_name << " not found.\n";
                        continue;
                    }

                    float local_time = std::fmod(time, total_times[motor_name]);
                    int p_des = (1 - cosf(2 * M_PI * local_time / total_times[motor_name])) * (4096 * 35) / 2;
                    csvFile << std::hex << motor->nodeId << ',' << p_des << '\n';
                    MParser.parseSendCommand(*motor, &frame, p_des);
                    buffer.push(frame);
                }
                MParser.makeSync(&frame);
                csvFile << "sync : "<<std::hex << frame.can_id << '\n';
                buffer.push(frame);
            }
        }
    }

    csvFile.close();
}
