#ifndef SHAREDBUFFER_HPP
#define SHAREDBUFFER_HPP

#include <mutex> // 뮤텍스를 사용하기 위한 헤더
#include <condition_variable> // 조건 변수를 사용하기 위한 헤더
#include <iostream>
#include <iomanip>
#include <linux/can.h>
#include <queue> // 큐를 사용하기 위한 헤더

template <typename T>
class SharedBuffer
{
public:
    // 아이템을 버퍼에 넣는 함수
    void push(const T &item)
    {
        std::unique_lock<std::mutex> lock(buffer_mutex); // 동기화를 위해 뮤텍스 잠금
        buffer.push(item); // 큐에 아이템 추가
        cond_var.notify_one(); // 조건 변수를 통해 대기 중인 스레드에게 알림
    }

    // 버퍼에서 아이템을 가져오는 함수. 성공하면 true 반환
    bool try_pop(T &item)
    {
        std::unique_lock<std::mutex> lock(buffer_mutex); // 동기화를 위해 뮤텍스 잠금
        if (buffer.empty()) // 큐가 비어 있는지 확인
        {
            return false; // 큐가 비어있다면 false 반환
        }
        item = buffer.front(); // 큐의 첫 번째 아이템을 가져옴
        buffer.pop(); // 큐의 첫 번째 아이템 제거
        return true; // 성공적으로 아이템을 가져왔으므로 true 반환
    }

    // 버퍼가 비어 있지 않을 때까지 대기하고, 아이템을 가져오는 함수
    void wait_and_pop(T &item)
    {
        std::unique_lock<std::mutex> lock(buffer_mutex); // 동기화를 위해 뮤텍스 잠금
        while (buffer.empty()) // 큐가 비어 있을 경우
        {
            cond_var.wait(lock); // 조건 변수로 대기
        }
        item = buffer.front(); // 큐의 첫 번째 아이템을 가져옴
        buffer.pop(); // 큐의 첫 번째 아이템 제거
    }

    // 버퍼가 비어 있는지 확인하는 함수
    bool empty() const
    {
        std::unique_lock<std::mutex> lock(buffer_mutex); // 동기화를 위해 뮤텍스 잠금
        return buffer.empty(); // 큐가 비어 있는지 확인 후 반환
    }

    // 버퍼의 내용을 출력하는 함수
    void print_buffer() const
    {
        std::unique_lock<std::mutex> lock(buffer_mutex); // 동기화를 위해 뮤텍스 잠금
        std::queue<T> temp_buffer = buffer; // 현재 큐의 상태를 복사
        int count = 0;

        std::cout << "Buffer contents: \n" << std::endl;
        while (!temp_buffer.empty() && count < 30) // 큐가 비어있지 않을 동안 또는 30회까지
        {
            // (이 부분은 CAN 프레임 출력과 관련된 부분이므로 생략)
            temp_buffer.pop(); // 복사한 큐의 첫 번째 아이템 제거
            count++; // 카운트 증가
        }
    }

private:
    std::queue<T> buffer; // 데이터를 저장할 큐
    mutable std::mutex buffer_mutex; // 동기화를 위한 뮤텍스
    std::condition_variable cond_var; // 대기 중인 스레드를 깨우기 위한 조건 변수
};

#endif // SHAREDBUFFER_HPP
