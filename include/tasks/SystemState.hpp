#pragma once

#include <atomic>

enum class Main
{
    SystemInit,
    Ideal,
    Test,
    Perform,
    Play,
    Shutdown,
    Pause,
    AddStance,
    Error
};

enum class PerformSub
{
    TimeCheck,
    CheckBuf,
    GeneratePath,
    SetCANFrame,
    SendCANFrame
};

enum class PlaySub
{
    TimeCheck,
    GeneratePath,
    SolveIK,
    SetCANFrame,
    SendCANFrame
};

enum class AddStanceSub
{
    TimeCheck,
    CheckCommand,
    CheckBuf,
    FillBuf,
    SetCANFrame,
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
    SetQValue,
    SetXYZ,
    FillBuf,
    CheckBuf,
    TimeCheck,
    SetCANFrame,
    SendCANFrame,
    Done
};



struct State
{
    std::atomic<Main> main;
    std::atomic<PerformSub> perform;
    std::atomic<PlaySub> play;
    std::atomic<AddStanceSub> addstance;
    std::atomic<ReadSub> read;
    std::atomic<TestSub> test;

    State() : main(Main::SystemInit),
              perform(PerformSub::TimeCheck),
              play(PlaySub::TimeCheck),
              addstance(AddStanceSub::CheckCommand),
              read(ReadSub::TimeCheck),
              test(TestSub::SelectParamByUser)
    {
    }
};
