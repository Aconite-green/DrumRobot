#pragma once

#include <atomic>


enum class Main
{
    SystemInit, 
    Ideal,      
    Homing,     
    Tune,       
    Perform,    
    Check,      
    Shutdown,   
    Ready,      
    Back,       
    Pause,
    AddStance       
};

enum class HomeSub {
    Start,
    MoveToSensor,
    StopAtSensor,
    MoveToZeroPositionInit,
    MoveToZeroPositionCheck,
    StopAtZeroPosition,
    SetZero,
    Done,
    SafetyCheck
};

enum class PerformSub {
    Start,
    CheckBuf,
    FillBuf,
    SafetyCheck,
    SendCANFrame,
    Done,
};

enum class AddStanceSub {
    Start,
    CheckBuf,
    FillBuf,
    SafetyCheck,
    SendCANFrame,
    Done,
};

enum class ReadSub{
    TimeCheck,
    ReadCANFrame,
    UpdateMotorInfo
};


enum class Maxon
{
    Position,     
    Torque_Move,  
    Torque_Touch, 
    Torque_Back,  
    Torque_InPos  
};


struct MaxonState
{
    std::atomic<Maxon> state; 
    MaxonState() : state(Maxon::Position) {}
};


struct State
{
    std::atomic<Main> main;         
    std::atomic<HomeSub> home;
    std::atomic<PerformSub> perform;
    std::atomic<AddStanceSub> addstance;
    std::atomic<ReadSub> read;
    MaxonState leftMaxon;           
    MaxonState rightMaxon;          
    
    State() : main(Main::SystemInit),
              home(HomeSub::Start),
              perform(PerformSub::Start),
              addstance(AddStanceSub::Start),
              read(ReadSub::TimeCheck)
    {
    }
};
