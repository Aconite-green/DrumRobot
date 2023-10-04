#pragma once

#include <stdio.h>
#include "../include/CanSocketUtils.hpp"
#include "../include/CommandParser.hpp"
#include "../include/ErrorHandle.hpp"
#include "../include/Motor.hpp"
#include <map>
#include <memory>
#include <string>

class DeactivateControlTask
{
public:
    DeactivateControlTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, const std::map<std::string, int> &sockets);
    void operator()();
    static const int ERR_SOCKET_CONFIGURE_FAILURE = -1;

private:
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
    const std::map<std::string, int> &sockets;
    void fillCanFrameFromInfo(struct can_frame *frame, const CanFrameInfo &info);
    int set_socket_timeout(int hsocket, int timeout_sec, int timeout_usec);
    void sendAndReceive(int socket, const std::string &name, struct can_frame &frame);
};
