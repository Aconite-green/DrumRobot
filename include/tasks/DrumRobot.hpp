#pragma once

#include <stdio.h>
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

#include "SystemState.hpp"
#include "../include/usbio/SenSor.hpp"
#include "../include/managers/CanManager.hpp"
#include "../include/managers/PathManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/managers/TestManager.hpp"
#include "../include/managers/HomeManager.hpp"

using namespace std;

/**
 * @class DrumRobot
 * @brief 드럼 로봇의 메인 제어 클래스.
 *
 * 이 클래스는 드럼 로봇 시스템의 메인 제어 로직을 포함하며, 다양한 매니저 클래스와 상태를 관리합니다.
 */
class DrumRobot
{
public:
    /**
     * @brief DrumRobot 클래스의 생성자.
     * @param systemStateRef 시스템 상태에 대한 참조.
     * @param canManagerRef CAN 매니저에 대한 참조.
     * @param pathManagerRef 경로 매니저에 대한 참조.
     * @param homeManagerRef 홈 매니저에 대한 참조.
     * @param testManagerRef 테스트 매니저에 대한 참조.
     * @param motorsRef 모터 객체들의 맵 참조.
     */
    DrumRobot(SystemState &systemStateRef,
              CanManager &canManagerRef,
              PathManager &pathManagerRef,
              HomeManager &homeManagerRef,
              TestManager &testManagerRef,
              std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    void stateMachine(); ///< 상태 머신을 실행하는 메소드.
    void sendLoopForThread(); ///< 송신 루프를 별도의 스레드로 실행하는 메소드.
    void recvLoopForThread(); ///< 수신 루프를 별도의 스레드로 실행하는 메소드.

private:
    SystemState &systemState; ///< 시스템 상태 참조.
    CanManager &canManager; ///< CAN 매니저 참조.
    PathManager &pathManager; ///< 경로 매니저 참조.
    HomeManager &homeManager; ///< 홈 매니저 참조.
    TestManager &testManager; ///< 테스트 매니저 참조.
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors; ///< 모터 객체들의 맵 참조.

    TMotorCommandParser tmotorcmd; ///< T 모터 명령 파서.
    MaxonCommandParser maxoncmd; ///< Maxon 명령 파서.
    Sensor sensor; ///< 센서 객체.

    // State Utility 메소드들
    void displayAvailableCommands() const; ///< 사용 가능한 명령어를 표시하는 메소드.
    bool processInput(const std::string &input); ///< 사용자 입력을 처리하는 메소드.
    void idealStateRoutine(); ///< 이상 상태 루틴을 실행하는 메소드.
    void checkUserInput(); ///< 사용자 입력을 확인하는 메소드.
    void printCurrentPositions(); ///< 현재 모터 위치를 출력하는 메소드.
    int kbhit(); ///< 키보드 입력이 있는지 확인하는 메소드.

    bool isReady; ///< 준비 상태 플래그.
    bool isBack; ///< 되돌아가기 플래그.

    // System Initialize 메소드들
    void initializeMotors(); ///< 모터 초기화 메소드.
    void initializecanManager(); ///< CAN 매니저 초기화 메소드.
    void DeactivateControlTask(); ///< 제어 태스크 비활성화 메소드.
    void motorSettingCmd(); ///< 모터 설정 명령어 메소드.
    void setMaxonMode(std::string targetMode); ///< Maxon 모드 설정 메소드.
    void MaxonEnable(); ///< Maxon 모터 활성화 메소드.
    void MaxonDisable(); ///< Maxon 모터 비활성화 메소드.

    // Send Thread Loop 메소드들
    void SendLoop(); ///< 송신 루프 메소드.
    void save_to_txt_inputData(const string &csv_file_name); ///< 입력 데이터를 txt 파일로 저장하는 메소드.
    void SendReadyLoop(); ///< 준비 상태 송신 루프 메소드.
    int writeFailCount; ///< 송신 실패 카운트.
    void initializePathManager(); ///< 경로 매니저 초기화 메소드.
    void clearMotorsSendBuffer(); ///< 모터 송신 버퍼 클리어 메소드.

    // Receive Thread Loop 메소드들
    const int TIME_THRESHOLD_MS = 5; ///< 시간 임계값.
    void RecieveLoop(); ///< 수신 루프 메소드.
    void parse_and_save_to_csv(const std::string &csv_file_name); ///< 파싱 후 CSV 파일로 저장하는 메소드.

    
};
