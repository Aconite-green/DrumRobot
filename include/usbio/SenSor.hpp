#pragma once

#include "Global.hpp"
#include <stdio.h>
#include <time.h>


using namespace std;

class Sensor
{
public:
    // 생성자 선언
    Sensor();
    ~Sensor();

    DWORD ReadVal();
    bool OpenDeviceUntilSuccess();
    void closeDevice();
    void connect();
    bool connected = false;

private:
    int DeviceID = USB2051_32;
    BYTE BoardID = 0x02;
    BYTE total_di;
    int DevNum, res;
    char module_name[15];
    DWORD DIValue = 0, o_dwDICntValue[USBIO_DI_MAX_CHANNEL];

    struct timespec start, end;
	long duration;
};