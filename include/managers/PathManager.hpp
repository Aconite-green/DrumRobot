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
#include <numeric>
#include "../include/eigen-3.4.0/Eigen/Dense"

//For Qt
/*
#include "CanManager.hpp"
#include "../motors/CommandParser.hpp"
#include "../motors/Motor.hpp"
#include "../tasks/SystemState.hpp"
*/

using namespace std;
using namespace Eigen;

/**
 * @class PathManager
 * @brief 드럼 로봇의 연주 경로를 생성하는 부분을 담당하는 클래스입니다.
 *
 * 이 클래스는 주어진 악보를 분석하여 정해진 알고리즘을 따라 알맞은 경로를 생성하도록 합니다.
 */

class PathManager
{

public:
    
    PathManager(State &stateRef,
                CanManager &canManagerRef,
                std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    
    /////////////////////////////////////////////////////////////////////////// Init
    void GetDrumPositoin();
    void GetMusicSheet();
    void SetReadyAng();


    /////////////////////////////////////////////////////////////////////////// Perform
    void PathLoopTask();
    
    int total = 0; ///< 악보의 전체 줄 수.
    int line = 0;  ///< 연주를 진행하고 있는 줄.
    float bpm = 50;         /// txt 악보의 BPM 정보.


    /////////////////////////////////////////////////////////////////////////// AddStance
    void GetArr(vector<float> &arr);

    //   Ready Pos Array   : waist, R_arm1, L_arm1, R_arm2, R_arm3, L_arm2, L_arm3, R_wrist, L_wrist
    //                      { 0    , 90    , 90    , 45    , 75    , 45    , 75    , 30      , 30 }      [deg]
    vector<float> readyArr = {0, M_PI / 2.0, M_PI / 2.0, M_PI * 0.25, M_PI / 2.4, M_PI * 0.25, M_PI / 2.4, M_PI / 6.0, M_PI / 6.0};

    //   Home Pos Array    : waist, R_arm1, L_arm1, R_arm2, R_arm3, L_arm2, L_arm3, R_wrist, L_wrist
    //                      { 0    , 135    , 45    , -45    , 90     , -45     , 90     , 90      , 90 }      [deg]
    vector<float> homeArr = {0, M_PI * 0.75, M_PI * 0.25, -M_PI / 4.0, M_PI / 2.0, -M_PI / 4.0, M_PI / 2.0, M_PI / 2.0, M_PI / 2.0};

    //   Back Pos Array    : waist, R_arm1, L_arm1, R_arm2, R_arm3, L_arm2, L_arm3, R_wrist, L_wrist
    //                      { 0    , 135    , 45    , 0    , 0     , 0     , 0     , 90      , 90 }      [deg]
    vector<float> backArr = {0, M_PI * 0.75, M_PI * 0.25, 0, 0, 0, 0, M_PI / 2.0, M_PI / 2.0};


    ///////////////////////////////////////////////////////////////////////////
    vector<float> fkfun();

    /*토크 제어에서 사용됨*/
    float wrist_targetPos = M_PI / 18.0;    // 타격 후 제어 변환 기준 각도
    float wrist_hit_time = 0.1;     // 타격하는데 걸리는 시간

private:
    TMotorCommandParser TParser; ///< T 모터 명령어 파서.
    MaxonCommandParser MParser;  ///< Maxon 모터 명령어 파서

    State &state;                                                 ///< 시스템의 현재 상태입니다.
    CanManager &canManager;                                       ///< CAN 통신을 통한 모터 제어를 담당합니다.
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors; ///< 연결된 모터들의 정보입니다.

    map<std::string, int> motor_mapping = { ///< 각 관절에 해당하는 열 정보.
        {"waist", 0},
        {"R_arm1", 1},
        {"L_arm1", 2},
        {"R_arm2", 3},
        {"R_arm3", 4},
        {"L_arm2", 5},
        {"L_arm3", 6},
        {"R_wrist", 7},
        {"L_wrist", 8},
        {"maxonForTest", 8}};


    /////////////////////////////////////////////////////////////////////////// Init
    string score_path = "../include/codes/test_1106.txt";        /// 악보 txt 파일 주소
    vector<float> time_arr; /// txt 악보의 시간간격 정보.
    MatrixXd inst_arr;      /// txt 악보의 오른팔 / 왼팔이 치는 악기.
    VectorXd default_right; /// 오른팔 시작 위치
    VectorXd default_left;  /// 왼팔 시작 위치
    MatrixXd right_drum_position;                               ///< 오른팔의 각 악기별 위치 좌표 벡터.
    MatrixXd left_drum_position;                                ///< 왼팔의 각 악기별 위치 좌표 벡터.
    VectorXd part_length;
    float s = 0.520;  ///< 허리 길이.
    float z0 = 0.890-0.0605; ///< 바닥부터 허리까지의 높이.


    ///////////////////////////////////////////////////////////////////////////
    // q1[rad], q2[rad], acc[rad/s^2], t2[s]
    VectorXd cal_Vmax(VectorXd &q1, VectorXd &q2, float acc, float t2);
    // q1[rad], q2[rad], Vmax[rad/s], acc[rad/s^2], t[s], t2[s]
    VectorXd makeProfile(VectorXd &q1, VectorXd &q2, VectorXd &Vmax, float acc, float t, float t2);

    
    ///////////////////////////////////////////////////////////////////////////
    vector<float> connect(vector<float> &Q1, vector<float> &Q2, int k, int n);  // 안쓰고 있음
    pair<float, float> q78_fun(MatrixXd &t_madi, float t_now);
    float con_fun_pos(float th_a, float th_b, float k, float n);
    MatrixXd tms_fun(float t2_a, float t2_b, VectorXd &inst2_a, VectorXd &inst2_b);
    void itms0_fun(vector<float> &t2, MatrixXd &inst2, MatrixXd &A30, MatrixXd &A31, MatrixXd &AA40, MatrixXd &AA41);
    void itms_fun(vector<float> &t2, MatrixXd &inst2, MatrixXd &B, MatrixXd &BB, VectorXd &pre_inst);
    VectorXd pos_madi_fun(VectorXd &A);
    MatrixXd sts2wrist_fun(MatrixXd &AA);
    MatrixXd sts2elbow_fun(MatrixXd &AA);
    VectorXd ikfun_final(VectorXd &pR, VectorXd &pL, VectorXd &part_length, float s, float z0);
    float con_fun(float th_a, float th_b, int k, int n);
    pair<float, float> iconf_fun(float qk1_06, float qk2_06, float qk3_06, float qv_in, float t1, float t2, float t);
    pair<float, float> qRL_fun(MatrixXd &t_madi, float t_now);
    pair<float, float> SetTorqFlag(MatrixXd &State, float t_now);


    /////////////////////////////////////////////////////////////////////////// Perform
    VectorXd inst_now;      /// 연주 중 현재 위치하는 악기
    float wrist_ready = 30 * M_PI / 180.0;                // 타격 시 들어올리는 손목 각도 (-1)
    float wrist_stanby = 10 * M_PI / 180.0;                 // 대기 시 들어올리는 손목 각도 (-0.5)
    float elbow_ready = 5 * M_PI / 180.0;                  // 타격 시 들어올리는 팔꿈치 각도 (-1)
    float elbow_stanby = 5 * M_PI / 180.0;                  // 대기 시 들어올리는 팔꿈치 각도 (-0.5)

    void Motors_sendBuffer(VectorXd &Qi, VectorXd &Vi, pair<float, float> Si, bool brake_state);


    /////////////////////////////////////////////////////////////////////////// AddStance
    void getMotorPos();
    vector<float> c_MotorAngle = {0, 0, 0, 0, 0, 0, 0, 0, 0}; ///< 경로 생성 시 사용되는 현재 모터 위치 값.
};