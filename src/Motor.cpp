// motor.c 파일
#include "../include/Motor.hpp" // Include header file
#include <iostream>

TMotor::TMotor(uint32_t nodeId, const std::string &motorType, const std::string &interFaceName)
    : nodeId(nodeId), motorType(motorType), interFaceName(interFaceName)
{
}

CanFrameInfo TMotor::getCanFrameForCheckMotor()
{
    return {this->nodeId, 8, {0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00}};
}

CanFrameInfo TMotor::getCanFrameForControlMode()
{
    return {this->nodeId, 8, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}};
}

CanFrameInfo TMotor::getCanFrameForExit()
{
    return {this->nodeId, 8, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}};
}

CanFrameInfo TMotor::getCanFrameForZeroing()
{
    return {this->nodeId, 8, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}};
}

CanFrameInfo TMotor::getCanFrameForQuickStop()
{
    return {this->nodeId, 8, {0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00}};
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// maxonMotor 클래스 구현
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MaxonMotor::MaxonMotor(uint32_t nodeId, const std::vector<uint32_t> &txPdoIds, const std::vector<uint32_t> &rxPdoIds, const std::string &interFaceName)
    : nodeId(nodeId), interFaceName(interFaceName)
{
    // canId 값 설정
    canSendId = 0x600 + nodeId;
    canReceiveId = 0x580 + nodeId;

    // pdoId 배열 초기화
    int index = 0;
    for (const auto &pdo : txPdoIds)
    {
        if (index >= 4)
        {
            std::cout << "Warning: More than 4 txPDO IDs provided. Ignoring extras." << std::endl;
            break;
        }
        this->txPdoIds[index] = pdo;
        ++index;
    }
    index = 0;
    for (const auto &pdo : rxPdoIds)
    {
        if (index >= 4)
        {
            std::cout << "Warning: More than 4 rxPDO IDs provided. Ignoring extras." << std::endl;
            break;
        }
        this->rxPdoIds[index] = pdo;
        ++index;
    }
}

CanFrameInfo MaxonMotor::getCanFrameForCheckMotor()
{
    return {this->canSendId, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForControlMode()
{
    return {this->canSendId, 8, {0x22, 0x60, 0x60, 0x00, 0x08, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForPosOffset()
{
    return {this->canSendId, 8, {0x22, 0xB0, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForExit()
{
    return {0x00, 8, {0x02, this->nodeId, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForOperational()
{
    return {0x00, 8, {0x01, this->nodeId, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForEnable()
{
    return {this->txPdoIds[0], 8, {0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForTorqueOffset()
{
    return {this->canSendId, 8, {0x22, 0xB2, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForTargetPosition(int targetPosition)
{
    // 10진수 targetPosition을 16진수로 변환
    // 1[revolve] = 4096[inc] * 35[gear ratio]
    unsigned char posByte0 = targetPosition & 0xFF;         // 하위 8비트
    unsigned char posByte1 = (targetPosition >> 8) & 0xFF;  // 다음 8비트
    unsigned char posByte2 = (targetPosition >> 16) & 0xFF; // 다음 8비트
    unsigned char posByte3 = (targetPosition >> 24) & 0xFF; // 최상위 8비트

    return {this->txPdoIds[1], 4, {posByte0, posByte1, posByte2, posByte3, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForSync()
{
    return {0x80, 1, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForQuickStop()
{
    return {this->txPdoIds[0], 8, {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

////////////////////Homing

CanFrameInfo MaxonMotor::getCanFrameForHomeMode()
{
    return {this->canSendId, 8, {0x22, 0x60, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getFlowingErrorWindow()
{
    return {this->canSendId, 8, {0x22, 0x65, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getHomeoffsetDistance()
{
    return {this->canSendId, 8, {0x22, 0xB1, 0x30, 0x00, 0x00, 0x8C, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getMaxProfileVelocity()
{
    return {this->canSendId, 8, {0x22, 0x7F, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getQuickStopDeceleration()
{
    return {this->canSendId, 8, {0x22, 0x85, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getSpeedForSwitchSearch()
{
    return {this->canSendId, 8, {0x22, 0x99, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getSpeedForZeroSearch()
{
    return {this->canSendId, 8, {0x22, 0x99, 0x60, 0x02, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getHomingAcceleration()
{
    return {this->canSendId, 8, {0x22, 0x9A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getHomePosition()
{
    return {this->canSendId, 8, {0x22, 0xB0, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getHomingMethod()
{
    return {this->canSendId, 8, {0x22, 0x98, 0x60, 0x00, 0xFD, 0xFF, 0xFF, 0xFF}};
    /*-4로 하고 싶은 경우 FD -> FC로 바꾸기*/
}

CanFrameInfo MaxonMotor::getStartHoming()
{
    return {this->canSendId, 8, {0x22, 0x40, 0x60, 0x00, 0x1F, 0x00, 0x00, 0x00}};
}

CanFrameInfo MaxonMotor::getCanFrameForCurrentThreshold()
{
    return {this->canSendId, 8, {0x23, 0xB2, 0x30, 0x00, 0xF4, 0x01, 0x00, 0x00}};
}