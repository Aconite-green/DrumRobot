

#pragma once

#include <atomic>

enum class Main
{
    SystemInit, // 시스템 시작: CAN 포트 열기 및 모터 연결 상태 확인
    Ideal,
    Homing, // Home : Homing 동작 제어
    Ready,  // 로봇 준비 동작
    Tune,   // 모터 뮤닝상태
    Perform,
    Check,   // 드럼 연주 진행중
    Shutdown // 시스템 종료 및 모든 작업 마무리
};

enum class HomeMode
{
    NotHome,
    Homing,    // Home 시작
    HomeReady, // Home 완료
    HomeError
};

enum class RunMode
{
    Ready,
    Running,
    Pause,
    Stop,
    RunError
};

struct SystemState
{
    std::atomic<Main> main;
    std::atomic<HomeMode> homeMode;
    std::atomic<RunMode> runMode;

    SystemState() : main(Main::SystemInit),
                    homeMode(HomeMode::NotHome),
                    runMode(RunMode::Stop) {}
};
