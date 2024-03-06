#pragma once

#include <atomic>

/**
 * @enum Main
 * @brief 시스템의 주요 상태를 정의합니다.
 */
enum class Main
{
    SystemInit, ///< 시스템 시작: CAN 포트 열기 및 모터 연결 상태 확인.
    Ideal,      ///< 기본 상태.
    Homing,     ///< Home: Homing 동작 제어.팅
    Tune,       ///< 모터 개별 혹은 다축 테스팅
    Perform,    ///< 드럼 연주 모드.치
    Check,      ///< 현재 모터들의 포지션을 체크하는 상태.
    Shutdown,   ///< 시스템 종료 및 모든 작업 마무리.
    Ready,      ///< 드럼 로봇 레디 포지션 위치
    Back,       ///< 드럼 로봇 제로 포지션 위치
    Pause       ///< 연주 중 일시정지
};

/**
 * @enum HomeMode
 * @brief 홈 모드의 상태를 정의합니다.
 */
enum class HomeMode
{
    NotHome,  ///< Homing이 완료되지 않은 초기상태.
    HomeDone, ///< Home 완료.
    HomeError ///< Homing 중 오류 발생.
};
/**
 * @enum Maxon
 * @brief Maxon 모터의 제어 모드 상태를 관리다 합니다
 */
enum class Maxon
{
    Position,     ///< 맥슨 모터 위치 제어중
    Torque_Move,  ///< 맥슨 모터 토크 제어 시작
    Torque_Touch, ///< 맥슨 모터 드럼 감지
    Torque_Back,  ///< 맥슨 모터 감지 후 반대 방향 토크 제어 시작
    Torque_InPos  ///< 맥슨 모터 고정 위치 도달(위치 제어 시작 요청)
};

/**
 * @struct MaxonState
 * @brief 각각의 맥슨 모터 상태를 관리합니다.
 */
struct MaxonState
{
    std::atomic<Maxon> state; ///< 맥슨 모터의 현재 상태.

    /**
     * @brief MaxonState의 기본 생성자.
     *
     * 맥슨 모터를 위치 제어 상태(`Position`)로 초기화합니다.
     */
    MaxonState() : state(Maxon::Position) {}
};

/**
 * @struct SystemState
 * @brief 시스템의 전반적인 상태를 관리합니다.
 *
 * 이 구조체는 시스템의 주 상태(`Main`)와 홈 모드 상태(`HomeMode`)를 관리합니다.
 * 각 상태는 std::atomic을 사용하여 멀티스레딩 환경에서 안전하게 접근됩니다.
 */
struct SystemState
{
    std::atomic<Main> main;         ///< 시스템의 주 상태.
    std::atomic<HomeMode> homeMode; ///< 홈 모드의 상태.
    MaxonState leftMaxon;           ///< 왼쪽 맥슨 모터 상태.
    MaxonState rightMaxon;          ///< 오른쪽 맥슨 모터 상태
    /**
     * @brief SystemState의 기본 생성자.
     *
     * 시스템을 시작 상태(`SystemInit`), 홈 모드를 미완료 상태(`NotHome`),
     * 맥슨 모터('Position 제어')로 초기화합니다.
     */
    SystemState() : main(Main::SystemInit),
                    homeMode(HomeMode::NotHome)
    {
    }
};
