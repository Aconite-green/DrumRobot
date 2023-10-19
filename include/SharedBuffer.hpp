#ifndef SHAREDBUFFER_HPP
#define SHAREDBUFFER_HPP

#include <mutex>              // 뮤텍스를 사용하기 위한 헤더
#include <condition_variable> // 조건 변수를 사용하기 위한 헤더
#include <iostream>
#include <iomanip>
#include <linux/can.h>
#include <queue> // 큐를 사용하기 위한 헤더
#include "CommandParser.hpp"
#include <fstream>
#include <map>

template <typename T>
class SharedBuffer
{
public:
    void push(const T &item)
    {
        std::unique_lock<std::mutex> lock(buffer_mutex); // 동기화를 위해 뮤텍스 잠금
        buffer.push(item);                               // 큐에 아이템 추가
        cond_var.notify_one();                           // 조건 변수를 통해 대기 중인 스레드에게 알림
    }

    // 버퍼에서 아이템을 가져오는 함수. 성공하면 true 반환
    bool try_pop(T &item)
    {
        std::unique_lock<std::mutex> lock(buffer_mutex); // 동기화를 위해 뮤텍스 잠금
        if (buffer.empty())                              // 큐가 비어 있는지 확인
        {
            return false; // 큐가 비어있다면 false 반환
        }
        item = buffer.front(); // 큐의 첫 번째 아이템을 가져옴
        buffer.pop();          // 큐의 첫 번째 아이템 제거
        return true;           // 성공적으로 아이템을 가져왔으므로 true 반환
    }

    // 버퍼가 비어 있지 않을 때까지 대기하고, 아이템을 가져오는 함수
    void wait_and_pop(T &item)
    {
        std::unique_lock<std::mutex> lock(buffer_mutex); // 동기화를 위해 뮤텍스 잠금
        while (buffer.empty())                           // 큐가 비어 있을 경우
        {
            cond_var.wait(lock); // 조건 변수로 대기
        }
        item = buffer.front(); // 큐의 첫 번째 아이템을 가져옴
        buffer.pop();          // 큐의 첫 번째 아이템 제거
    }

    // 버퍼가 비어 있는지 확인하는 함수
    bool empty() const
    {
        std::unique_lock<std::mutex> lock(buffer_mutex); // 동기화를 위해 뮤텍스 잠금
        return buffer.empty();                           // 큐가 비어 있는지 확인 후 반환
    }

    // 버퍼의 내용을 출력하는 함수
    void print_buffer() const
    {
        std::unique_lock<std::mutex> lock(buffer_mutex);
        std::queue<T> temp_buffer = buffer;
        int count = 0;
        std::cout << "Buffer contents: \n"
                  << std::endl;

        while (!temp_buffer.empty() && count < 30)
        {
            const can_frame &frame = temp_buffer.front();
            std::cout << "can_id: " << std::hex << frame.can_id
                      << ", can_dlc: " << std::dec << static_cast<int>(frame.can_dlc)
                      << ", data: ";

            for (int i = 0; i < frame.can_dlc; ++i)
            {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frame.data[i]) << ' ';
            }
            std::cout << std::endl;
            temp_buffer.pop();
            count++;
        }
    }

    void parse_and_save_to_csv(const std::string &csv_file_name,
                               TMotorCommandParser &Tparser,
                               MaxonCommandParser &Mparser,
                               std::map<std::string, std::shared_ptr<TMotor>> &tmotors,
                               std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors)
    {
        std::unique_lock<std::mutex> lock(buffer_mutex); // 동기화를 위한 뮤텍스 잠금

        // CSV 파일 열기. 파일이 없으면 새로 생성됩니다.
        std::ofstream ofs(csv_file_name, std::ios::app);

        if (!ofs.is_open())
        {
            std::cerr << "Failed to open or create the CSV file: " << csv_file_name << std::endl;
            return;
        }

        // 파일이 새로 생성되었으면 CSV 헤더를 추가
        ofs.seekp(0, std::ios::end);
        if (ofs.tellp() == 0)
        {
            ofs << "MotorName,ID,Position,Speed,Torque\n";
        }

        std::queue<T> temp_buffer = buffer; // 버퍼를 임시 큐에 복사

        while (!temp_buffer.empty())
        {
            can_frame frame = temp_buffer.front();
            temp_buffer.pop();
            bool isMaxonMotorFound = false;

            // MaxonMotor 처리
            if (!maxonMotors.empty()) // maxonMotor가 존재할 때만 실행
            {
                for (const auto &pair : maxonMotors)
                {
                    std::shared_ptr<MaxonMotor> motor = pair.second;
                    if (motor->rxPdoIds[0] == frame.can_id)
                    {
                        isMaxonMotorFound = true;
                        auto [id, position] = Mparser.parseRecieveCommand(&frame);
                        ofs << "0x" << std::hex << std::setw(4) << std::setfill('0') << id << ","
                            << std::dec
                            << position << "\n";
                        break; // 해당하는 모터를 찾았으므로 for문을 종료
                    }
                }
            }

            // MaxonMotor가 없거나 해당 canframe이 MaxonMotor가 아니면 TMotor 처리
            if (!isMaxonMotorFound)
            {
                for (const auto &pair : tmotors)
                {
                    std::shared_ptr<TMotor> motor = pair.second;
                    if (motor->nodeId == frame.data[0])
                    {
                        auto [id, position, speed, torque] = Tparser.parseRecieveCommand(*motor, &frame);
                        ofs << "0x" << std::hex << std::setw(4) << std::setfill('0') << id << ","
                            << std::dec
                            << position << ","
                            << speed << ","
                            << torque << "\n";
                        break; // 해당하는 모터를 찾았으므로 for문을 종료
                    }
                }
            }
        }

        ofs.close();
    }

private:
    std::queue<T>
        buffer;                       // 데이터를 저장할 큐
    mutable std::mutex buffer_mutex;  // 동기화를 위한 뮤텍스
    std::condition_variable cond_var; // 대기 중인 스레드를 깨우기 위한 조건 변수
};

#endif // SHAREDBUFFER_HPP
