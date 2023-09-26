// motor.c 파일
#include "../include/motor.hpp" // Include header file
#include <iostream>

TMotor::TMotor(int id, const std::string &motorType, const std::string &roboticSection) : motorType(motorType), id(id), roboticSection(roboticSection)
{
    // 공통된 초기값 설정
    pMin = -12.5;
    pMax = 12.5;
    kpMin = 0;
    kpMax = 500;
    kdMin = 0;
    kdMax = 5;

    // 타입에 따른 초기값 설정
    setLimits();
}

void TMotor::setLimits()
{
    if (motorType == "AK10_9")
    {
        vMin = -50;
        vMax = 50;
        tMin = -65;
        tMax = 65;
    }
    else if (motorType == "AK70_10")
    {
        vMin = -50;
        vMax = 50;
        tMin = -25;
        tMax = 25;
    }
    else if (motorType == "AK60_6")
    {
        vMin = -45;
        vMax = 45;
        tMin = -15;
        tMax = 15;
    }
    else if (motorType == "AK80_6")
    {
        vMin = -76;
        vMax = 76;
        tMin = -12;
        tMax = 12;
    }
    else if (motorType == "AK80_9")
    {
        vMin = -50;
        vMax = 50;
        tMin = -18;
        tMax = 18;
    }
    else if (motorType == "AK80_80" || motorType == "AK80_64")
    {
        vMin = -8;
        vMax = 8;
        tMin = -144;
        tMax = 144;
    }
    else if (motorType == "AK80_8")
    {
        vMin = -37.5;
        vMax = 37.5;
        tMin = -32;
        tMax = 32;
    }
    else
    {
        std::cout << "Error: Invalid motor motorType entered!" << std::endl;
    }
}

void TMotor::fillCanFrameForCheckMotor(struct can_frame *frame)
{
    int canId = this->id;
    int dlc = 8;

    this->parser.parseSendCommand(*this, frame, canId, dlc, 0, 0, 0, 0, 0);
}

void TMotor::fillCanFrameForControlMode(struct can_frame *frame)
{
    frame->can_id = this->id;
    frame->can_dlc = 8;

    frame->data[0] = 0xFF;
    frame->data[1] = 0xFF;
    frame->data[2] = 0xFF;
    frame->data[3] = 0xFF;
    frame->data[4] = 0xFF;
    frame->data[5] = 0xFF;
    frame->data[6] = 0xFF;
    frame->data[7] = 0xFC;
}

void TMotor::fillCanFrameForExit(struct can_frame *frame)
{
    frame->can_id = this->id;
    frame->can_dlc = 8;

    /// pack ints into the can buffer ///
    frame->data[0] = 0xFF; // Position 8 higher
    frame->data[1] = 0xFF; // Position 8 lower
    frame->data[2] = 0xFF; // Speed 8 higher
    frame->data[3] = 0xFF; // Speed 4 bit lower KP 4bit higher
    frame->data[4] = 0xFF; // KP 8 bit lower
    frame->data[5] = 0xFF; // Kd 8 bit higher
    frame->data[6] = 0xFF; // KP 4 bit lower torque 4 bit higher
    frame->data[7] = 0xFD; // torque 4 bit lower
}

void TMotor::fillCanFrameForZeroing(struct can_frame *frame)
{
    frame->can_id = this->id;
    frame->can_dlc = 8;

    frame->data[0] = 0xFF; // Position 8 higher
    frame->data[1] = 0xFF; // Position 8 lower
    frame->data[2] = 0xFF; // Speed 8 higpaher
    frame->data[3] = 0xFF; // Speed 4 bit lower KP 4bit higher
    frame->data[4] = 0xFF; // KP 8 bit lower
    frame->data[5] = 0xFF; // Kd 8 bit higher
    frame->data[6] = 0xFF; // KP 4 bit lower torque 4 bit higher
    frame->data[7] = 0xFE; // torque 4 bit lower
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// maxonMotor 클래스 생성자 구현
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


MaxonMotor::MaxonMotor(int nodeId, const std::string &roboticSection, std::initializer_list<int> pdoIds)
    : nodeId(nodeId), roboticSection(roboticSection)
{
    // canId 값 설정
    canSendId = 0x600 + nodeId;
    canReceiveId = 0x580 + nodeId;

    // pdoId 배열 초기화
    int index = 0;
    for (const auto &pdo : pdoIds)
    {
        if (index >= 4)
        {
            std::cout << "Warning: More than 4 PDO IDs provided. Ignoring extras." << std::endl;
            break;
        }
        pdoId[index] = pdo;
        ++index;
    }
}

