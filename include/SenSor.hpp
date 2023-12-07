#pragma once

#include "Global.hpp"

using namespace std;

class Sensor
{
public:
    // 생성자 선언
    Sensor();
    ~Sensor();

    DWORD ReadVal();
    bool IsInitialized() const { return isInitialized; }
    bool OpenDeviceUntilSuccess();
    void closeDevice();

private:
    bool isInitialized;
    int DeviceID = USB2051_32;
    BYTE BoardID = 0x02;
    BYTE total_di;
    int DevNum, res;
    char module_name[15];
    DWORD DIValue = 0, o_dwDICntValue[USBIO_DI_MAX_CHANNEL];
};