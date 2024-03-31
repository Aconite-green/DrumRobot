#pragma once

#include <atomic>

enum class Main
{
    SystemInit,
    Ideal,
    Homing,
    Test,
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
    Done
};

enum class HomeMaxon
{
    StartHoming,
    CheckHomeStatus,
    Done
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

enum class TestSub
{
    SelectParamByUser,
    FillBuf,
    CheckBuf,
    TimeCheck,
    SafetyCheck,
    SendCANFrame,
    Done
};

struct State
{
    std::atomic<Main> main;
    std::atomic<HomeSub> home;
    std::atomic<HomeTmotor> homeTmotor;
    std::atomic<HomeMaxon> homeMaxon;
    std::atomic<PerformSub> perform;
    std::atomic<AddStanceSub> addstance;
    std::atomic<ReadSub> read;
    std::atomic<TestSub> test;

    State() : main(Main::SystemInit),
              home(HomeSub::SelectMotorByUser),
              homeTmotor(HomeTmotor::MoveToSensor),
              homeMaxon(HomeMaxon::StartHoming),
              perform(PerformSub::TimeCheck),
              addstance(AddStanceSub::CheckCommand),
              read(ReadSub::TimeCheck),
              test(TestSub::SelectParamByUser)
    {
    }
};
