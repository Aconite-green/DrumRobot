#pragma once

#include <stdio.h>
#include "../include/managers/CanManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/SystemState.hpp"
#include "../include/usbio/SenSor.hpp"
#include <map>
#include <memory>
#include <string>
#include <functional>
#include <queue>
#include <algorithm>
#include <thread>
#include <cerrno>
#include <cstring>
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
 * @class HomeManager
 * @brief 모터의 홈 위치 설정 및 관리를 담당하는 클래스입니다.
 *
 * 이 클래스는 모터의 홈 위치를 설정하고, 모터와 관련된 다양한 작업을 관리합니다.
 */
class HomeManager
{
public:
    /**
     * @brief HomeManager 클래스의 생성자입니다.
     * @param systemStateRef 시스템 상태에 대한 참조입니다.
     * @param canManagerRef CAN 통신 관리자에 대한 참조입니다.
     * @param motorsRef 모터 객체에 대한 참조를 매핑합니다.
     */
    HomeManager(State &stateRef,
                CanManager &canManagerRef,
                std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    /**
     * @brief 모터 홈 위치 설정의 메인 루프를 실행합니다.
     */
    void mainLoop();
    void SendHomeProcess();

private:
    State &state;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    Sensor sensor;

    // 홈
    /**
     * @brief 모터의 홈 위치 설정 상태를 표시합니다.
     */
    void displayHomingStatus();

    /**
     * @brief 모터의 홈 위치 설정 상태를 업데이트합니다.
     */
    void UpdateHomingStatus();

    /* Tmotor */
    /**
     * @brief T모터를 홈 위치로 설정합니다.
     * @param motors 모터 객체 목록입니다.
     * @param motorNames 모터 이름 목록입니다.
     */
    void SetTmotorHome(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames);

    /**
     * @brief T모터의 홈 위치를 설정하는 작업을 수행합니다.
     * @param motors 모터 객체 목록입니다.
     * @param motorNames 모터 이름 목록입니다.
     */
    void HomeTMotor(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames);

    /**
     * @brief T모터를 센서 위치로 이동시킵니다.
     * @param motors 모터 객체 목록입니다.
     * @param motorNames 모터 이름 목록입니다.
     * @param sensorsBit 센서 비트 값 목록입니다.
     * @return vector<float> 각 모터의 중간 위치를 나타내는 벡터입니다.
     */
    vector<float> MoveTMotorToSensorLocation(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames, vector<int> &sensorsBit);

    /**
     * @brief T모터를 회전시킵니다.
     * @param motors 모터 객체 목록입니다.
     * @param motorNames 모터 이름 목록입니다.
     * @param directions 회전 방향 목록입니다.
     * @param degrees 회전 각도 목록입니다.
     * @param midpoints 모터의 중간 위치 목록입니다.
     */
    void RotateTMotor(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames, vector<double> &directions, vector<double> &degrees, vector<float> &midpoints);

    /**
     * @brief 사용자에게 홈 위치 설정을 위한 확인을 요청합니다.
     * @param motorName 모터의 이름입니다.
     * @return bool 사용자가 홈 위치 설정을 진행할지 여부를 반환합니다.
     */
    bool PromptUserForHoming(const std::string &motorName);

    /* Maxon */
    /**
     * @brief Maxon 모터를 홈 위치로 설정합니다.
     * @param motors 모터 객체 목록입니다.
     */
    void SetMaxonHome(vector<std::shared_ptr<GenericMotor>> &motors);

    /**
     * @brief Maxon 모터의 작동 모드를 설정합니다.
     * @param targetMode 목표 모드입니다.
     */
    void setMaxonMode(std::string targetMode);

    /**
     * @brief Maxon 모터를 활성화합니다.
     */
    void MaxonEnable();

    /**
     * @brief 모터의 위치를 수정합니다.
     * @param motor 모터 객체에 대한 참조입니다.
     */
    void FixMotorPosition(std::shared_ptr<GenericMotor> &motor);

    /**
     * @brief Maxon 모터를 비활성화합니다.
     */
    void MaxonDisable();
};
