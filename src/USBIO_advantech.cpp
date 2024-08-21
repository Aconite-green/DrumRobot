#include "../include/USBIO_advantech/USBIO_advantech.hpp"

USBIO::USBIO()
{
}

USBIO::~USBIO()
{
}

bool USBIO::USBIO_4761_init()
{
    DeviceInformation devInfo(deviceDescription);

    do
    {
        ret = instantDoCtrl->setSelectedDevice(devInfo);
        CHK_RESULT(ret);
        ret = instantDoCtrl->LoadProfile(profilePath);//Loads a profile to initialize the device.
        CHK_RESULT(ret);
    } while (false);

    if(BioFailed(ret))
    {
        wchar_t enumString[256];
        AdxEnumToString(L"ErrorCode", (int32)ret, 256, enumString);
        std::cout << "Some error occurred. And the last error code is 0x" << ret << ". [" << enumString << "]\n";

        return false;
    }
    else
    {
        return true;
    }
}

bool USBIO::USBIO_4761_output(unsigned int inputVal)
{
    uint8 bufferForWriting[1] = {0x00};

    if (inputVal == 0)
    {
        bufferForWriting[0] = 0x00;
    }
    else
    {
        bufferForWriting[0] = 0xff;
    }
    
    do
    {
        ret = instantDoCtrl->Write(0, 1, bufferForWriting);
        CHK_RESULT(ret);
    } while (false);
    

    if(BioFailed(ret))
    {
        wchar_t enumString[256];
        AdxEnumToString(L"ErrorCode", (int32)ret, 256, enumString);
        std::cout << "Some error occurred. And the last error code is 0x" << ret << ". [" << enumString << "]\n";

        return false;
    }
    else
    {
        return true;
    }
}

void USBIO::USBIO_4761_exit()
{
    instantDoCtrl->Dispose();
}