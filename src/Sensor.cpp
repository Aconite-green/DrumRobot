#include "../include/SenSor.hpp"

Sensor::Sensor()
{
    printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());
    res = USBIO_OpenDevice(DeviceID, BoardID, &DevNum);

    if (res)
    {
        printf("open Device failed! Erro : 0x%x\r\n", res);
    }

    printf("Demo usbio_di DevNum = %d\n", DevNum);

    USBIO_ModuleName(DevNum, module_name);

    USBIO_GetDITotal(DevNum, &total_di);
    printf("%s DI number: %d\n\n", module_name, total_di);
}

Sensor::~Sensor(){
    res = USBIO_CloseDevice(DevNum);

    if (res)
    {
        printf("close %s with Board iD %d failed! Erro : %d\r\n", module_name, BoardID, res);
    }
}

DWORD Sensor::ReadVal(){
    USBIO_DI_ReadValue(DevNum, &DIValue);

    return DIValue;
}



