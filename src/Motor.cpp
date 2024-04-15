
#include "../include/motors/Motor.hpp" // Include header file

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GenericMotor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GenericMotor::GenericMotor(uint32_t nodeId)
    :nodeId(nodeId)
    {}


void GenericMotor::clearSendBuffer()
{
    while (!sendBuffer.empty())
    {
        sendBuffer.pop();
    }
}

void GenericMotor::clearReceiveBuffer()
{
    while (!recieveBuffer.empty())
    {
        recieveBuffer.pop();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TMotor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TMotor::TMotor(uint32_t nodeId, const std::string &motorType)
    : GenericMotor(nodeId), motorType(motorType)
{
}

void TMotor::clearCommandBuffer(){
    while (!commandBuffer.empty())
    {
        commandBuffer.pop();
    }
}

void TMotor::clearInRecordBuffer(){
    while (!InRecordBuffer.empty())
    {
        InRecordBuffer.pop();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// maxonMotor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MaxonMotor::MaxonMotor(uint32_t nodeId)
    : GenericMotor(nodeId)
{
    // canId 값 설정
    canSendId = 0x600 + nodeId;
    canReceiveId = 0x580 + nodeId;
}

void MaxonMotor::clearCommandBuffer(){
    while (!commandBuffer.empty())
    {
        commandBuffer.pop();
    }
}

void MaxonMotor::clearInRecordBuffer(){
    while (!InRecordBuffer.empty())
    {
        InRecordBuffer.pop();
    }
}

void MaxonMotor::clearWrist_BackArr(){
    while (!wrist_BackArr.empty())
    {
        wrist_BackArr.pop();
    }
}