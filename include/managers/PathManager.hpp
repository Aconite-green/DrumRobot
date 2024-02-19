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
 * @class PathManager
 * @brief 드럼 로봇의 연주 경로를 생성하는 부분을 담당하는 클래스입니다.
 * 
 * 이 클래스는 주어진 악보를 분석하여 정해진 알고리즘을 따라 알맞은 경로를 생성하도록 합니다.
 */

class PathManager
{

public:
    /**
     * @brief PathManager 클래스의 생성자입니다.
     * @param systemStateRef 시스템 상태에 대한 참조입니다.
     * @param canManagerRef CAN 통신 관리자에 대한 참조입니다.
     * @param motorsRef 모터 객체에 대한 참조를 매핑합니다.
     */
    PathManager(SystemState &systemStateRef,
                CanManager &canManagerRef,
                std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    // 드럼 위치 확인
    /**
     * @brief 각 모터의 각도값을 불러와 스틱 끝 좌표를 계산하는 Foward Kinematics을 진행합니다.
     * @return vector<double> 왼팔 / 오른팔의 스틱 끝 좌표를 나타내는 벡터입니다.
     */
    vector<double> fkfun();

    /**
     * @brief 각 모터의 회전방향에 따라 경로 방향을 적용시킵니다.
    */
    void ApplyDir();

    /**
     * @brief rT.txtㅇ에 저장되어 있는 드럼의 위치정보를 불러옵니다.
    */
    void GetDrumPositoin();

    /**
     * @brief codeConfession.txtㅇ에 저장되어 있는 악보 정보를 불러옵니다.
    */
    void GetMusicSheet();

    /**
     * @brief 연주를 진행하고 있는 line에 대한 연주 경로를 생성합니다.
    */
    void PathLoopTask();

    /**
     * @brief 현재 위치부터 원하는 위치까지 이동시키는 경로를 생성합니다.
     * @param arr 이동하고자하는 위치 정보값입니다.
    */
    void GetArr(vector<double> &arr);


    int total = 0;  ///< 악보의 전체 줄 수.
    int line = 0;   ///< 연주를 진행하고 있는 줄.

    // 악보에 따른 position & velocity 값 저장 (5ms 단위)
    vector<vector<double>> p;   ///< 위치 경로 벡터
    vector<vector<double>> v;   ///< 속도 경로 벡터

    //     Ready Array      : waist, R_arm1, L_arm1, R_arm2, R_arm3, L_arm2, L_arm3, R_wrist, L_wrist
    //                        0    , 90    , 90    , 45    , 75    , 45    , 75    , 0      , 0        [deg]
    vector<double> standby = {0, M_PI / 2, M_PI / 2, M_PI / 4, M_PI / 2.4, M_PI / 4, M_PI / 2.4, 0, 0};

    //     Back Array      : waist, R_arm1, L_arm1, R_arm2, R_arm3, L_arm2, L_arm3, R_wrist, L_wrist
    //                        0    , 90    , 90    , 0    , 0     , 0     , 0     , 60      , 60        [deg]
    vector<double> backarr = {0, M_PI / 2, M_PI / 2, 0, 0, 0, 0, M_PI / 3, M_PI / 3};

private:
    TMotorCommandParser TParser;    ///< T 모터 명령어 파서.
    MaxonCommandParser MParser;     ///< Maxon 모터 명령어 파서
    
    SystemState &systemState;   ///< 시스템의 현재 상태입니다.
    CanManager &canManager;     ///< CAN 통신을 통한 모터 제어를 담당합니다.
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;   ///< 연결된 모터들의 정보입니다.

    // Functions for DrumRobot PathGenerating
    vector<double> c_MotorAngle = {0, 0, 0, 0, 0, 0, 0, 0, 0};  ///< 경로 생성 시 사용되는 현재 모터 위치 값.
    vector<vector<double>> right_inst;  ///< 오른팔의 각 악기별 위치 좌표 벡터.
    vector<vector<double>> left_inst;   ///< 왼팔의 각 악기별 위치 좌표 벡터.

    int n_inst = 10;    ///< 총 악기의 수.
    double bpm = 10;    /// 악보의 BPM 정보.
    vector<double> time_arr;    ///< 악보의 시간간격 정보.
    vector<double> time_arr_F;    ///< 악보의 시간간격 정보.
    vector<vector<int>> RA, LA; ///< 오른팔 / 왼팔이 치는 악기.
    vector<int> RF, LF;         ///< 오른발 / 왼발이 치는 악기.;

    double p_R = 0; ///< 오른손 이전 악기 유무.
    double p_L = 0; ///< 왼손 이전 악기 유무.
    double c_R = 0; ///< 오른손 현재 악기 유무.
    double c_L = 0; ///< 왼손 현재 악기 유무.

    double r_wrist = 0.0;   ///< 오른손목 각도.
    double l_wrist = 0.0;   ///< 왼손목 각도.

    /* 실측값 */
    vector<double> P1 = {0.3, 0.94344, 1.16582};       ///< 오른팔 준비자세 좌표.
    vector<double> P2 = {-0.3, 0.94344, 1.16582};      ///< 왼팔 준비자세 좌표.
    vector<double> R = {0.363, 0.793, 0.363, 0.793};     ///< [오른팔 상완, 오른팔 하완+스틱, 왼팔 상완, 왼팔 하완+스틱]의 길이.
    double s = 0.600;   ///< 허리 길이.
    double z0 = 1.026;  ///< 바닥부터 허리까지의 높이.

    vector<double> Q1, Q2, Q3, Q4;  ///< Q1, Q3 : 악기를 연주하기 전 들어올린 상태 / Q2 : 이번에 치는 악기 위치 / Q4 : 다음에 치는 악기 위치

    double ElbowAngle_ready = M_PI / 36;    ///< -0.5일 때 들어올리는 팔꿈치 각도 : 5deg
    double ElbowAngle_hit = M_PI / 18;      ///< -1일 때 들어올리는 팔꿈치 각도 : 10deg
    double WristAngle_ready = M_PI / 4;     ///< -0.5일 때 들어올리는 손목 각도 : 45deg
    double WristAngle_hit = M_PI / 2;       ///< -1일 때 들어올리는 손목 각도 : 90deg

    vector<double> wrist = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  ///< 각 악기별 치는 손목 각도

    map<std::string, int> motor_mapping = { ///< 각 관절에 해당하는 열 정보.
        {"waist", 0}, {"R_arm1", 1}, {"L_arm1", 2}, {"R_arm2", 3}, {"R_arm3", 4}, {"L_arm2", 5}, {"L_arm3", 6}, {"R_wrist", 7}, {"L_wrist", 8}};
    
    map<int, int> motor_dir = { ///< 각 열에 해당하는 관절방향 [ 1 : CW , -1 : CCW ]
        {0, 1},
        {1, 1},
        {2, 1},
        {3, 1},
        {4, 1},
        {5, 1},
        {6, 1},
        {7, 1},
        {8, 1}};

    /**
     * @brief 두 위치를 (1-cos)함수로 연결합니다.
     * @param Q1 현재 위치 벡터입니다.
     * @param Q2 이동하려는 위치 벡터입니다.
     * @param k 벡터의 인덱스 값입니다.
     * @param n 연결하는데 사용되는 총 벡터 수입니다.
     * @return vector<double> 총 n개의 벡터 중 k번째 벡터를 나타냅니다.
    */
    vector<double> connect(vector<double> &Q1, vector<double> &Q2, int k, int n);

    /**
     * @brief 두 위치를 5차함수로 연결합니다.
     * @param P0 현재 위치 벡터입니다.
     * @param P1 첫번째 위치 벡터입니다.
     * @param P2 두번째 위치 벡터입니다.
     * @param V0 현재 속도 벡터입니다.
     * @param t1 첫번재 시간간격입니다.
     * @param t2 두번째 시간간격입니다.
     * @param t 시간 인덱스입니다.
    */
    void iconnect(vector<double> &P0, vector<double> &P1, vector<double> &P2, vector<double> &V0, double t1, double t2, double t);

    /**
     * @brief 오른팔 / 왼팔의 좌표에 따라 Inverse Kinematics을 진행합니다.
     * @param P1 오른팔 스틱 끝의 좌표입니다.
     * @param P2 왼팔 스틱 끝의 좌표입니다.
     * @return vector<double> 해당 좌표에 알맞는 각 모터의 위치 벡터입니다.
    */
    vector<double> IKfun(vector<double> &P1, vector<double> &P2);

    /**
     * @brief 현재 line에서 연주하는 악기에 따른 오른팔 / 왼팔의 좌표를 불러옵니다.
    */
    void getDrummingPosAndAng();

    /**
     * @brief 각 모터의 현재위치 값을 불러옵니다.
    */
    void getMotorPos();

    /**
     * @brief 처음 시작하는 line인 경우, 위치벡터 Q1 / Q2 벡터를 생성합니다.
    */
    void getQ1AndQ2();

    /**
     * @brief line이 진행됨에 따라 Q3 / Q4 벡터를 생성합니다.
    */
    void getQ3AndQ4();

    /**
     * @brief 생성한 경로를 각 모터의 버퍼에 쌓아줍니다.
    */
    void Motors_sendBuffer();
};