#ifndef SHAREDBUFFER_HPP
#define SHAREDBUFFER_HPP

#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class SharedBuffer {
public:
    void push(const T& item) {
        std::unique_lock<std::mutex> lock(buffer_mutex);
        buffer.push(item);
        cond_var.notify_one();  // 데이터가 들어왔으므로 대기 중인 쓰레드에 알림
    }

    bool try_pop(T& item) {
        std::unique_lock<std::mutex> lock(buffer_mutex);
        if (buffer.empty()) {
            return false;
        }
        item = buffer.front();
        buffer.pop();
        return true;
    }

    // 빈 버퍼에서 데이터를 기다림
    void wait_and_pop(T& item) {
        std::unique_lock<std::mutex> lock(buffer_mutex);
        while (buffer.empty()) {
            cond_var.wait(lock);
        }
        item = buffer.front();
        buffer.pop();
    }

    bool empty() const {
        std::unique_lock<std::mutex> lock(buffer_mutex);
        return buffer.empty();
    }

private:
    std::queue<T> buffer;
    mutable std::mutex buffer_mutex;
    std::condition_variable cond_var;
};

#endif // SHAREDBUFFER_HPP
