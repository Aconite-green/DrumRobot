#include "../include/SharedBuffer.hpp"

void SharedBuffer::push(const std::string& command) {
    std::unique_lock<std::mutex> lock(mtx);
    buffer.push(command);
    lock.unlock();
    cv.notify_one();
}

std::string SharedBuffer::pop() {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [&](){ return !buffer.empty(); });
    std::string command = buffer.front();
    buffer.pop();
    return command;
}
