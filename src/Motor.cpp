
#include "../include/motors/Motor.hpp" // Include header file

// For Qt
// #include "../motors/Motor.hpp"
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

float TMotor::jointAngleToMotorPosition(float jointAngle)
{
    float motorPosition;
    
    motorPosition = (jointAngle - initialJointAngle) * cwDir / timingBeltRatio;

    return motorPosition;
}

float TMotor::motorPositionToJointAngle(float motorPosition)
{
    float jointAngle;
    
    jointAngle = motorPosition * cwDir * timingBeltRatio + initialJointAngle;

    return jointAngle;
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

void MaxonMotor::clearWrist_BackArr(){
    while (!wrist_BackArr.empty())
    {
        wrist_BackArr.pop();
    }
}

float MaxonMotor::jointAngleToMotorPosition(float jointAngle)
{
    float motorPosition;
    
    motorPosition = (jointAngle - initialJointAngle) * cwDir;

    return motorPosition;
}

float MaxonMotor::motorPositionToJointAngle(float motorPosition)
{
    float jointAngle;
    
    jointAngle = motorPosition * cwDir + initialJointAngle;

    return jointAngle;
}