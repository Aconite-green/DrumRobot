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

void USBIO::USBIO_test()
{
    using namespace Automation::BDaq;

    const wchar_t* profilePath = L"../../profile/DemoDevice.xml";

    // Step 1: Create a instantDoCtrl for DO function.
    ErrorCode ret = Automation::BDaq::Success;
    InstantDoCtrl * instantDoCtrl = InstantDoCtrl::Create();

    do
    {
        // Step 2: Select a device by device number or device description and specify the access mode.
        DeviceInformation devInfo(deviceDescription);
        ret = instantDoCtrl->setSelectedDevice(devInfo);
        CHK_RESULT(ret);
        ret = instantDoCtrl->LoadProfile(profilePath);//Loads a profile to initialize the device.
        CHK_RESULT(ret);

        // Step 3: Write DO ports
        
        // uint8  bufferForWriting[64] = {0};
        // uint32 inputVal = 0;
        // cout << "Input a 16 hex number for DO port " << 0 << " to output(for example, 0x00): ";
        // cin >> inputVal;
        // bufferForWriting[0] = inputVal;  

        // // Set the 'startPort'as the first port for Do .
        // // Set the 'portCount'to decide how many sequential ports to operate Do.
        // int32    startPort = 0;
        // int32    portCount = 1;
        // ret = instantDoCtrl->Write(startPort, portCount, bufferForWriting);

        uint8 portNum;
        uint32 inputVal;
        uint8 bufferForWriting[1] = {0x00};
        // std::cout << "Input a number for DO port : ";
        // std::cin >> portNum;
        // std::cout << "Input a value for DO port : ";
        // std::cin >> inputVal;

        bufferForWriting[0] = 0x01;
        ret = instantDoCtrl->Write(0, 1, bufferForWriting);
        CHK_RESULT(ret);
        // std::cout << "\n DO output completed !\n\n";

        // Step 4: Close device and release any allocated resource.
        instantDoCtrl->Dispose();

        if(BioFailed(ret))
        {
            wchar_t enumString[256];
            AdxEnumToString(L"ErrorCode", (int32)ret, 256, enumString);
            std::cout << "Some error occurred. And the last error code is 0x" << ret << ". [" << enumString << "]\n";
        }

    } while (false);
}