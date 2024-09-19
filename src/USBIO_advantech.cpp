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

        useUSBIO = false;

        return false;
    }
    else
    {
        return true;
    }
}

void USBIO::USBIO_4761_set(int num, bool state)
{
    uint8 current_output = bufferForWriting[0];

    if (state)
    {
        current_output |= 1<<num;
    }
    else
    {
        current_output &= ~(1<<num);
    }

    bufferForWriting[0] = current_output;
}

bool USBIO::USBIO_4761_output()
{
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

void USBIO::USBIO_4761_testset(int val)
{
    if(val == 0)
    {
        bufferForWriting[0] = 0x00;
    }
    else if (val == 1)
    {
        bufferForWriting[0] = 0x01;
    }
    else if (val == 2)
    {
        bufferForWriting[0] = 0x02;
    }
    else if (val == 3)
    {
        bufferForWriting[0] = 0x03;
    }
    else if (val == 4)
    {
        bufferForWriting[0] = 0x04;
    }
    else if (val == 5)
    {
        bufferForWriting[0] = 0x05;
    }
    else if (val == 6)
    {
        bufferForWriting[0] = 0x06;
    }
    else if (val == 7)
    {
        bufferForWriting[0] = 0x07;
    }
    else if (val == 8)
    {
        bufferForWriting[0] = 0x08;
    }
    else if (val == 9)
    {
        bufferForWriting[0] = 0x09;
    }
    else if (val == 10)
    {
        bufferForWriting[0] = 0x0A;
    }
    else if (val == 11)
    {
        bufferForWriting[0] = 0x0B;
    }
    else if (val == 12)
    {
        bufferForWriting[0] = 0x0C;
    }
    else if (val == 13)
    {
        bufferForWriting[0] = 0x0D;
    }
    else if (val == 14)
    {
        bufferForWriting[0] = 0x0E;
    }
    else if (val == 15)
    {
        bufferForWriting[0] = 0x0F;
    }
}
