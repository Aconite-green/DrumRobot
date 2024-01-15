// motor.c 파일
#include "../include/motors/Motor.hpp" // Include header file
#include <iostream>

TMotor::TMotor(uint32_t nodeId, const std::string &motorType, const std::string &interFaceName)
    : GenericMotor(nodeId), motorType(motorType), interFaceName(interFaceName)
{
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// maxonMotor 클래스 구현
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MaxonMotor::MaxonMotor(uint32_t nodeId, const std::string &interFaceName)
    : GenericMotor(nodeId), interFaceName(interFaceName)
{
    // canId 값 설정
    canSendId = 0x600 + nodeId;
    canReceiveId = 0x580 + nodeId;
}

