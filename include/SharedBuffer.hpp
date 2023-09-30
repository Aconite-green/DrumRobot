#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>

class SharedBuffer {
private:
    std::queue<std::string> buffer;
    std::mutex mtx;
    std::condition_variable cv;
public:
    void push(const std::string& command);
    std::string pop();
};
