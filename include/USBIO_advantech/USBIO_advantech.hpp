#pragma once

#include <stdlib.h>
#include <iostream>
#include "../USBIO_advantech/compatibility.h"
#include "../USBIO_advantech/bdaqctrl.h"

using namespace Automation::BDaq;

typedef unsigned char byte;

#define  deviceDescription  L"USB-4761,BID#0"

class USBIO
{
public:
    USBIO();

    ~USBIO();

    bool USBIO_4761_init();

    bool USBIO_4761_output(unsigned int inputVal);

    void USBIO_4761_exit();

    void USBIO_test();

private:
    const wchar_t* profilePath = L"../../../profile/DemoDevice.xml";

    ErrorCode ret;// = Success;
    InstantDoCtrl *instantDoCtrl = InstantDoCtrl::Create();

};

