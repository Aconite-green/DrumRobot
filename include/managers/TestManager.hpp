#pragma once

#include <stdio.h>
#include "../include/managers/CanManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/SystemState.hpp"
#include <map>
#include <memory>
#include <string>
#include <functional>
#include <queue>
#include <algorithm>
#include <thread>
#include <cerrno>  // errno
#include <cstring> // strerror
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <iostream>
#include <vector>
#include <limits>
#include <ctime>
#include <fstream>
#include <atomic>
#include <cmath>
#include <chrono>
#include <set>

using namespace std;

/**
 * @class TestManager
 * @brief 모터의 성능 테스트 및 파라미터 튜닝을 위한 클래스입니다.
 *
 * TestManager 클래스는 다양한 테스트 시나리오를 실행하여 모터의 성능을 평가하고,
 * 최적의 운영 파라미터를 결정하기 위한 메서드를 제공합니다.
 */
class TestManager
{
public:
    /**
     * @brief TestManager 클래스의 생성자.
     * @param systemStateRef 시스템 상태에 대한 참조입니다.
     * @param canManagerRef CAN 통신을 관리하는 CanManager 클래스의 참조입니다.
     * @param motorsRef 연결된 모터들의 정보를 담고 있는 맵입니다.
     */
    TestManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    /**
     * @brief 테스트 매니저의 메인 루프를 실행하는 함수입니다.
     * 주요 테스트 루틴을 실행하고 결과를 분석합니다.
     */
    void mainLoop();

    /**
     * @brief 다중 테스트 루프를 실행하는 함수입니다.
     * 여러 테스트 케이스를 동시에 실행하여 모터의 동작을 평가합니다.
     */
    void multiTestLoop();

    /**
     * @brief 테스트 배열을 생성하고 실행하는 함수입니다.
     * @param t 시간 주기입니다.
     * @param cycles 반복 횟수입니다.
     * @param type 테스트 유형입니다.
     * @param LnR 왼쪽 또는 오른쪽 모터를 선택합니다.
     * @param amp 진폭 배열입니다.
     */
    void TestArr(double t, int cycles, int type, int LnR, double amp[]);

private:
    State &state; ///< 시스템의 현재 상태입니다.
    CanManager &canManager; ///< CAN 통신을 통한 모터 제어를 담당합니다.
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors; ///< 연결된 모터들의 정보입니다.

    TMotorCommandParser tmotorcmd; ///< T 모터 명령어 파서입니다.
    MaxonCommandParser maxoncmd; ///< Maxon 모터 명령어 파서입니다.

    vector<string> InputData; ///< 테스트 입력 데이터입니다.

    /**
     * @brief 모터를 이동시키는 함수입니다.
     * 지정된 명령에 따라 모터를 이동시킵니다.
     */
    void move();

    /**
     * @brief 테스트 배열을 생성하는 함수입니다.
     * @param motorName 모터의 이름입니다.
     * @param time 시간 주기입니다.
     * @param cycles 반복 횟수입니다.
     * @param LnR 왼쪽 또는 오른쪽 모터를 선택합니다.
     * @param amp 진폭입니다.
     */
    void mkArr(vector<string> &motorName, int time, int cycles, int LnR, double amp);

    /**
     * @brief 생성된 배열을 모터에 전송하는 루프를 실행하는 함수입니다.
     * 설정된 파라미터에 따라 모터에 명령을 전송합니다.
     */
    void SendLoop();

    /**
     * @brief 테스트 결과를 CSV 파일로 파싱하고 저장하는 함수입니다.
     * @param csv_file_name 저장할 CSV 파일의 이름입니다.
     */
    void parse_and_save_to_csv(const std::string &csv_file_name);

        // Single Mode Test
    /**
     * @brief 단일 모터의 위치를 고정하는 함수입니다.
     * @param motor 모터 객체의 공유 포인터입니다.
     * 모터를 고정된 위치에 정확하게 유지하기 위해 사용됩니다.
     */
    void FixMotorPosition(std::shared_ptr<GenericMotor> &motor);

    /**
     * @brief T 모터의 파라미터를 튜닝하는 함수입니다.
     * @param kp 비례 제어 계수입니다.
     * @param kd 미분 제어 계수입니다.
     * @param sine_t 사인 파형의 주기입니다.
     * @param selectedMotor 선택된 모터의 이름입니다.
     * @param cycles 실행할 사이클 수입니다.
     * @param peakAngle 최대 회전 각도입니다.
     * @param pathType 경로 유형입니다.
     * T 모터의 응답성과 정확도를 개선하기 위해 사용됩니다.
     */
    void TuningTmotor(float kp, float kd, float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType);

    /**
     * @brief 튜닝 루프 작업을 실행하는 함수입니다.
     * 사용자에게 입력을 받아 모터의 튜닝 과정을 관리합니다.
     */
    void TuningLoopTask();

    /**
     * @brief 선택된 모터와 테스트 파라미터를 초기화하는 함수입니다.
     * @param selectedMotor 선택된 모터의 이름입니다.
     * @param kp 비례 제어 계수입니다.
     * @param kd 미분 제어 계수입니다.
     * @param peakAngle 최대 회전 각도입니다.
     * @param pathType 경로 유형입니다.
     * @param controlType 제어 유형입니다.
     * @param des_vel 목표 속도입니다.
     * @param des_tff 목표 토크 피드포워드 값입니다.
     * @param direction 회전 방향입니다.
     * 튜닝 과정에서 사용될 파라미터를 사용자로부터 받아 설정합니다.
     */
    void InitializeParameters(const std::string selectedMotor, float &kp, float &kd, float &peakAngle, int &pathType, int &controlType, int &des_vel, int &des_tff, int &direction);

    /**
     * @brief Maxon 모터의 CSP 모드를 튜닝하는 함수입니다.
     * @param sine_t 사인 파형의 주기입니다.
     * @param selectedMotor 선택된 모터의 이름입니다.
     * @param cycles 실행할 사이클 수입니다.
     * @param peakAngle 최대 회전 각도입니다.
     * @param pathType 경로 유형입니다.
     * Maxon 모터의 위치 제어 성능을 최적화하기 위해 사용됩니다.
     */
    void TuningMaxonCSP(float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType);

    /**
     * @brief Maxon 모터의 CSV 모드를 튜닝하는 함수입니다.
     * @param selectedMotor 선택된 모터의 이름입니다.
     * @param des_vel 목표 속도입니다.
     * @param direction 회전 방향입니다.
     * 속도 제어 성능을 평가하고 최적화하기 위해 사용됩니다.
     */
    void TuningMaxonCSV(const std::string selectedMotor, int des_vel, int direction);

    /**
     * @brief Maxon 모터의 CST 모드를 튜닝하는 함수입니다.
     * @param selectedMotor 선택된 모터의 이름입니다.
     * @param des_tff 목표 토크 피드포워드 값입니다.
     * @param direction 회전 방향입니다.
     * 토크 제어 성능을 평가하고 최적화하기 위해 사용됩니다.
     */
    void TuningMaxonCST(const std::string selectedMotor, int des_tff, int direction);

    /**
     * @brief Maxon 모터의 작동 모드를 설정하는 함수입니다.
     * @param targetMode 설정할 모드의 이름입니다.
     * 모터의 작동 모드를 변경하기 위해 사용됩니다.
     */
    void setMaxonMode(std::string targetMode);

    /**
     * @brief 키보드 입력이 있는지 확인하는 함수입니다.
     * 사용자로부터의 입력을 비동기적으로 확인하기 위해 사용됩니다.
     * @return 키보드 입력이 있으면 1, 없으면 0을 반환합니다.
     */
    int kbhit();

    // Stick Mode Test
    /**
     * @brief 스틱 모드 테스트 루프를 실행하는 함수입니다.
     * 스틱 모드의 성능을 테스트하기 위해 사용됩니다.
     */
    void TestStickLoop();

    /**
     * @brief 특정 모터에 대한 스틱 모드 테스트를 실행하는 함수입니다.
     * @param selectedMotor 선택된 모터의 이름입니다.
     * @param des_tff 목표 토크 피드포워드 값입니다.
     * @param tffThreshold 토크 피드포워드 임계값입니다.
     * @param posThreshold 위치 임계값입니다.
     * @param backTorqueUnit 역토크 단위입니다.
     * 모터의 스틱 모드 성능을 평가하기 위해 사용됩니다.
     */
    void TestStick(const std::string selectedMotor, int des_tff, float tffThreshold, float posThreshold, int backTorqueUnit);

    /**
     * @brief 위치와 속도 임계값을 기반으로 DCT(Discrete Cosine Transform) 함수를 실행하는 함수입니다.
     * @param positions 모터 위치 데이터 배열입니다.
     * @param vel_th 속도 임계값입니다.
     * 위치와 속도 데이터를 분석하여 모터의 동작 품질을 평가합니다.
     * @return 분석 결과에 따라 true 또는 false를 반환합니다.
     */
    bool dct_fun(float positions[], float vel_th);

};
