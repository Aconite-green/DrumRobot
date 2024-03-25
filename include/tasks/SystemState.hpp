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
    Pause,
    AddStance,
    Error
};

enum class HomeSub
{
    SelectMotorByUser,
    MakeHomingOrderBuf,
    GetSelectedMotor,
    HomeTmotor,
    HomeMaxon,
    Done
};

enum class HomeTmotor
{
    MoveToSensor,
    SensorCheck,
    FillBuf,
    CheckBuf,
    SafetyCheck,
    SendCANFrameForZeroPos,
    SetZero,
    Done
};

enum class HomeMaxon
{

};

enum class PerformSub
{
    TimeCheck,
    CheckBuf,
    GeneratePath,
    SafetyCheck,
    SendCANFrame
};

enum class AddStanceSub
{
    TimeCheck,
    CheckCommand,
    CheckBuf,
    FillBuf,
    SafetyCheck,
    SendCANFrame
};

enum class ReadSub
{
    TimeCheck,
    ReadCANFrame,
    UpdateMotorInfo,
    CheckMaxonControl,
    CheckDrumHit,
    CheckReachedPosition
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
    std::atomic<HomeTmotor> homeTmotor;
    std::atomic<PerformSub> perform;
    std::atomic<AddStanceSub> addstance;
    std::atomic<ReadSub> read;
    MaxonState leftMaxon;
    MaxonState rightMaxon;

    State() : main(Main::SystemInit),
              home(HomeSub::SelectMotorByUser),
              homeTmotor(HomeTmotor::MoveToSensor),
              perform(PerformSub::TimeCheck),
              addstance(AddStanceSub::CheckCommand),
              read(ReadSub::TimeCheck)
    {
    }
};
