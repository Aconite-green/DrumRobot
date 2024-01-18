

#pragma once

#include <atomic>

enum class Main
{
    SystemInit, // 시스템 시작: CAN 포트 열기 및 모터 연결 상태 확인
    Ideal,      // 기본 상태
    Homing,     // Home : Homing 동작 제어
    Tune,       // 모터 뮤닝상태
    Perform,    // 드럼 연주 모드
    Check,      // 현재 모터들의 포지션을 체크하는 상태
    Shutdown,   // 시스템 종료 및 모든 작업 마무리
    Back,
    Ready
};

enum class HomeMode
{
    NotHome,  // Homing이 완료되춤지 않은 초기상태
    HomeDone, // Home 완료
    HomeError
};

enum class RunMode
{
    PrePreparation,
    Ready,
    Running, // 연주중
    Pause,   // 일시정지
    Stop,    // 아예 멈춤
    RunError
};

enum class TestMode
{
    SingleMode,
    MultiMode,
    StickMode,
    Ideal,
    Exit
};

struct SystemState
{
    std::atomic<Main> main;
    std::atomic<HomeMode> homeMode;
    std::atomic<RunMode> runMode;
    std::atomic<TestMode> testMode;

    SystemState() : main(Main::SystemInit),
                    homeMode(HomeMode::NotHome),
                    runMode(RunMode::PrePreparation),
                    testMode(TestMode::SingleMode) {}
};
