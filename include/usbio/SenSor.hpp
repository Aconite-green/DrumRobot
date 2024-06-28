#pragma once

#include "Global.hpp"
#include <stdio.h>
#include <time.h>
#include "../motors/Motor.hpp"
#include <memory>
using namespace std;

class Sensor
{
public:
    Sensor(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    ~Sensor();

    DWORD ReadVal();
    bool OpenDeviceUntilSuccess();
    void closeDevice();
    void connect();
    void writeVal(std::shared_ptr<TMotor> tMotor, bool status);

    bool connected = false;

private:
    
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    // int DeviceID = USB2051_32;
    int DeviceID = USB2055_32;
    BYTE BoardID = 0x02;
    BYTE total_di;
    int DevNum, res;
    char module_name[15];
    DWORD DIValue = 0;
    DWORD o_dwDICntValue[USBIO_DI_MAX_CHANNEL];

    struct timespec start, end;
    long duration;
};
