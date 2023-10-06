// motor.c 파일
#include "../include/Motor.hpp" // Include header file
#include <iostream>

TMotor::TMotor(uint32_t nodeId, const std::string &motorType, const std::string &interFaceName)
    : nodeId(nodeId), motorType(motorType), interFaceName(interFaceName) // 멤버 변수 초기화
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

CanFrameInfo TMotor::getCanFrameForCheckMotor() {
    return {this->nodeId, 8, {0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00}};
}

CanFrameInfo TMotor::getCanFrameForControlMode() {
    return {this->nodeId, 8, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}};
}

CanFrameInfo TMotor::getCanFrameForExit() {
    return {this->nodeId, 8, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}};
}

CanFrameInfo TMotor::getCanFrameForZeroing() {
    return {this->nodeId, 8, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}};
}

CanFrameInfo TMotor::getCanFrameForQuickStop() {
    return {this->nodeId, 8, {0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00}};
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// maxonMotor 클래스 구현
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MaxonMotor::MaxonMotor(uint32_t nodeId, const std::vector<uint32_t>& pdoIds)
: nodeId(nodeId)
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

CanFrameInfo MaxonMotor::getCanFrameForCheckMotor() {
    return {this->canSendId, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForControlMode() {
    return {this->canSendId, 8, {0x22, 0x60, 0x60, 0x00, 0x08, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForZeroing() {
    return {this->canSendId, 8, {0x22, 0xB0, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForExit() {
    return {0x00, 8, {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForOperational() {
    return {0x00, 8, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForEnable() {
    return {this->pdoId[0], 8, {0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForTorqueOffset() {
    return {this->canSendId, 8, {0x22, 0xB2, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}};
}


CanFrameInfo MaxonMotor::getCanFrameForTargetPosition(int targetPosition) {
    // 10진수 targetPosition을 16진수로 변환
    // 1[revolve] = 4096[inc]
    unsigned char posByte0 = targetPosition & 0xFF;         // 하위 8비트
    unsigned char posByte1 = (targetPosition >> 8) & 0xFF;  // 다음 8비트
    unsigned char posByte2 = (targetPosition >> 16) & 0xFF; // 다음 8비트
    unsigned char posByte3 = (targetPosition >> 24) & 0xFF; // 최상위 8비트

    return {this->pdoId[1], 4, {posByte0, posByte1, posByte2, posByte3, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForSync() {
    return {0x80, 1, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForQuickStop() {
    return {this->pdoId[0], 8, {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
}
