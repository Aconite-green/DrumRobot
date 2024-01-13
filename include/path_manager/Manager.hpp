#pragma once

#include <queue> // std::queue 사용을 위해 필요
#include <map>
#include <memory>
#include <string>
#include "../motors/Motor.hpp"
#include "../usbio/Global.hpp"
#include <linux/can/raw.h>

class Manager {
protected:
    std::queue<can_frame> &sendBuffer;
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
    std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors;

public:
    Manager(std::queue<can_frame> &sendBufferRef, std::map<std::string, std::shared_ptr<TMotor>> &tmotorsRef, std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotorsRef)
        : sendBuffer(sendBufferRef), tmotors(tmotorsRef), maxonMotors(maxonMotorsRef) {}

    virtual void initializeMotors() = 0; // 순수 가상 함수
    // 다른 공통 메소드들...
};
