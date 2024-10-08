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

   
    vector<float> fkfun();

    void GetDrumPositoin();
    void GetMusicSheet();
    void SetReadyAng();
    void PathLoopTask();
    void GetArr(vector<float> &arr);

    int total = 0; ///< 악보의 전체 줄 수.
    int line = 0;  ///< 연주를 진행하고 있는 줄.

    //     Ready Array      : waist, R_arm1, L_arm1, R_arm2, R_arm3, L_arm2, L_arm3, R_wrist, L_wrist
    //                      { 0    , 90    , 90    , 45    , 75    , 45    , 75    , 30      , 30 }      [deg]
    vector<float> readyarr = {0, M_PI / 2, M_PI / 2, M_PI * 0.25, M_PI / 2.4, M_PI * 0.25, M_PI / 2.4, M_PI / 6, M_PI / 6};
    // Standby(Home) Array  : waist, R_arm1, L_arm1, R_arm2, R_arm3, L_arm2, L_arm3, R_wrist, L_wrist
    //                      { 0    , 135    , 45    , -45    , 90     , -45     , 90     , 90      , 90 }      [deg]
    vector<float> standby = {0, M_PI * 0.75, M_PI * 0.25, -M_PI / 4, M_PI / 2, -M_PI / 4, M_PI / 2, M_PI / 2, M_PI / 2};
    //     Back Array       : waist, R_arm1, L_arm1, R_arm2, R_arm3, L_arm2, L_arm3, R_wrist, L_wrist
    //                      { 0    , 135    , 45    , 0    , 0     , 0     , 0     , 90      , 90 }      [deg]
    vector<float> backarr = {0, M_PI * 0.75, M_PI * 0.25, 0, 0, 0, 0, M_PI / 2, M_PI / 2};

    /*하이브리드 제어때 사용됨*/
    float wrist_targetPos = M_PI / 18.0;    // 타격 후 제어 변환 기준 각도
    float wrist_hit_time = 0.1;     // 타격하는데 걸리는 시간

    float wrist_backPos = M_PI / 18.0;  // 대기 시 들어올리는 손목 각도 (-0.5) : 10deg
    float wrist_hitPos = M_PI / 9.0;    // 타격 시 들어올리는 손목 각도 (-1)   : 20deg
    float wrist_back_time = 0.04;       // 타격 후 들어올리는 궤적시간

    float elbow_backPos = M_PI / 36.0;  // 대기 시 들어올리는 팔꿈치 각도 (-0.5) : 5deg
    float elbow_hitPos = M_PI / 15.0;   // 타격 시 들어올리는 팔꿈치 각도 (-1)   : 12deg

private:
    TMotorCommandParser TParser; ///< T 모터 명령어 파서.
    MaxonCommandParser MParser;  ///< Maxon 모터 명령어 파서

    State &state;                                                 ///< 시스템의 현재 상태입니다.
    CanManager &canManager;                                       ///< CAN 통신을 통한 모터 제어를 담당합니다.
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors; ///< 연결된 모터들의 정보입니다.

    string score_path = "../include/codes/codeMeaningOfYou.txt";
    // string score_path = "../include/codes/codeTest.txt";

    // Functions for DrumRobot PathGenerating
    vector<float> c_MotorAngle = {0, 0, 0, 0, 0, 0, 0, 0, 0}; ///< 경로 생성 시 사용되는 현재 모터 위치 값.
    MatrixXd right_inst;                                       ///< 오른팔의 각 악기별 위치 좌표 벡터.
    MatrixXd left_inst;                                        ///< 왼팔의 각 악기별 위치 좌표 벡터.

    int n_inst = 10; ///< 총 악기의 수.
    float bpm = 50; /// 악보의 BPM 정보.

    vector<float> time_arr; ///< 악보의 시간간격 정보.
    MatrixXd inst_arr;       ///< 오른팔 / 왼팔이 치는 악기.
    VectorXd default_right;
    VectorXd default_left;
    VectorXd inst_00;

    // MatrixXd RF, LF;         ///< 오른발 / 왼발이 치는 악기.

    /* 실측값 */
    // vector<float> P1 = {0.3, 0.94344, 1.16582};       ///< 오른팔 준비자세 좌표.
    // vector<float> P2 = {-0.3, 0.94344, 1.16582};      ///< 왼팔 준비자세 좌표.
    // vector<float> R = {0.363, 0.793, 0.363, 0.793};     ///< [오른팔 상완, 오른팔 하완+스틱, 왼팔 상완, 왼팔 하완+스틱]의 길이.
    
    VectorXd part_length;
    float s = 0.600;  ///< 허리 길이.
    float z0 = 1.026; ///< 바닥부터 허리까지의 높이.

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
    
    vector<float> connect(vector<float> &Q1, vector<float> &Q2, int k, int n);  // 안쓰고 있음

    // vector<float> cal_Vmax_add(vector<float> &q1, vector<float> &q2, float acc, float t2);  // q1[rad], q2[rad], acc[rad/s^2], t2[s]
    // vector<float> makeProfile_add(vector<float> &q1, vector<float> &q2, vector<float> &Vmax, float acc, float t, float t2);  // q1[rad], q2[rad], Vmax[rad/s], acc[rad/s^2], t[s], t2[s]

    // q1[rad], q2[rad], acc[rad/s^2], t2[s]
    VectorXd cal_Vmax(VectorXd &q1, VectorXd &q2, float acc, float t2);
    // q1[rad], q2[rad], Vmax[rad/s], acc[rad/s^2], t[s], t2[s]
    VectorXd makeProfile(VectorXd &q1, VectorXd &q2, VectorXd &Vmax, float acc, float t, float t2);
    pair<float, float> q78_fun(MatrixXd &t_madi, float t_now);
    // POS LOOP mode
    float con_fun_pos(float th_a, float th_b, float k, float n);

    void getMotorPos();

    void Motors_sendBuffer(VectorXd &Qi, VectorXd &Vi, pair<float, float> Si, bool brake_state);

    ////////////////////////////// New Motor Generation ///////////////////////////////
    /**
     * @brief
     */
    MatrixXd tms_fun(float t2_a, float t2_b, VectorXd &inst2_a, VectorXd &inst2_b);
    void itms0_fun(vector<float> &t2, MatrixXd &inst2, MatrixXd &A30, MatrixXd &A31, MatrixXd &AA40, MatrixXd &AA41);
    void itms_fun(vector<float> &t2, MatrixXd &inst2, MatrixXd &B, MatrixXd &BB);
    VectorXd pos_madi_fun(VectorXd &A);
    MatrixXd sts2wrist_fun(MatrixXd &AA, float v_wrist);
    MatrixXd sts2elbow_fun(MatrixXd &AA, float v_elbow);
    VectorXd ikfun_final(VectorXd &pR, VectorXd &pL, VectorXd &part_length, float s, float z0);
    // POS-SPD LOOP mode
    float con_fun(float th_a, float th_b, int k, int n);
    pair<float, float> iconf_fun(float qk1_06, float qk2_06, float qk3_06, float qv_in, float t1, float t2, float t);
    pair<float, float> qRL_fun(MatrixXd &t_madi, float t_now);
    pair<float, float> SetTorqFlag(MatrixXd &State, float t_now);
    // void SetTargetPos(MatrixXd &State, MatrixXd &t_madi);
};