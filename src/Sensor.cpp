#include "../include/SenSor.hpp"

Sensor::Sensor()
{
}

Sensor::~Sensor()
{
}

DWORD Sensor::ReadVal()
{
    clock_gettime(CLOCK_MONOTONIC, &start);

    USBIO_DI_ReadValue(DevNum, &DIValue);
    clock_gettime(CLOCK_MONOTONIC, &end);
    duration = get_nano_seconds(&start, &end);
    printf("USBIO_DI_ReadValue execution time: %ld ns\n", duration);

    return DIValue;
}

long Sensor::get_nano_seconds(struct timespec *start, struct timespec *end)
{
    return (end->tv_sec - start->tv_sec) * 1000000000 + (end->tv_nsec - start->tv_nsec);
}

bool Sensor::OpenDeviceUntilSuccess()
{
    printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());

    while (true)
    {
        res = USBIO_OpenDevice(DeviceID, BoardID, &DevNum);
        if (res == 0)
        {
            printf("Device opened successfully.\n");
            return true;
        }
        else
        {
            printf("Open Device failed! Error : 0x%x. Retrying...\n", res);
            usleep(100000); // 잠시 대기 후 재시도
        }
    }

    printf("Demo usbio_di DevNum = %d\n", DevNum);

    USBIO_ModuleName(DevNum, module_name);

    USBIO_GetDITotal(DevNum, &total_di);
    printf("%s DI number: %d\n\n", module_name, total_di);
}

void Sensor::closeDevice()
{
    res = USBIO_CloseDevice(DevNum);

    if (res)
    {
        printf("close %s with Board iD %d failed! Erro : %d\r\n", module_name, BoardID, res);
    }
}

void Sensor::connect()
{
    printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());

    res = USBIO_OpenDevice(DeviceID, BoardID, &DevNum);
    if (res == 0)
    {
        printf("Device opened successfully.\n");
        connected = true;
    }
    else
    {
        printf("Open Device failed! Error : 0x%x. Retrying...\n", res);
        connected = false;
    }

    printf("Demo usbio_di DevNum = %d\n", DevNum);

    USBIO_ModuleName(DevNum, module_name);

    USBIO_GetDITotal(DevNum, &total_di);
    printf("%s DI number: %d\n\n", module_name, total_di);
}
