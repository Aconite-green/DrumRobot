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
    /**
     * @brief PathManager 클래스의 생성자입니다.
     * @param systemStateRef 시스템 상태에 대한 참조입니다.
     * @param canManagerRef CAN 통신 관리자에 대한 참조입니다.
     * @param motorsRef 모터 객체에 대한 참조를 매핑합니다.
     */
    PathManager(State &stateRef,
                CanManager &canManagerRef,
                std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    // 드럼 위치 확인
    /**
     * @brief 각 모터의 각도값을 불러와 스틱 끝 좌표를 계산하는 Foward Kinematics을 진행합니다.
     * @return vector<double> 왼팔 / 오른팔의 스틱 끝 좌표를 나타내는 벡터입니다.
     */
    vector<double> fkfun();

    /**
     * @brief rT.txtㅇ에 저장되어 있는 드럼의 위치정보를 불러옵니다.
     */
    void GetDrumPositoin();

    /**
     * @brief codeConfession.txtㅇ에 저장되어 있는 악보 정보를 불러옵니다.
     */
    void GetMusicSheet();

    /**
     * @brief 원하는 준비 동작에 따라 준비동작 각도를 추출합니다.
     */
    void SetReadyAng();

    /**
     * @brief 연주를 진행하고 있는 line에 대한 연주 경로를 생성합니다.
     */
    void PathLoopTask();

    /**
     * @brief 현재 위치부터 원하는 위치까지 이동시키는 경로를 생성합니다.
     * @param arr 이동하고자하는 위치 정보값입니다.
     */
    void GetArr(vector<double> &arr);

    void Get_wrist_BackArr(string MotorName, float &A, float &B, double t);

    int total = 0; ///< 악보의 전체 줄 수.
    int line = 0;  ///< 연주를 진행하고 있는 줄.

    // 악보에 따른 position & velocity 값 저장 (5ms 단위)
    vector<vector<double>> pos; ///< 위치 경로 벡터
    vector<vector<double>> vel; ///< 속도 경로 벡터

    //     Ready Array      : waist, R_arm1, L_arm1, R_arm2, R_arm3, L_arm2, L_arm3, R_wrist, L_wrist
    //                      { 0    , 90    , 90    , 45    , 75    , 45    , 75    , 30      , 30 }      [deg]
    vector<double> readyarr = {0, M_PI / 2, M_PI / 2, M_PI / 4, M_PI / 2.4, M_PI / 4, M_PI / 2.4, M_PI / 6, M_PI / 6};

    //     Back Standby      : waist, R_arm1, L_arm1, R_arm2, R_arm3, L_arm2, L_arm3, R_wrist, L_wrist
    //                      { 0    , 120    , 60    , -45    , 90     , -45     , 90     , 90      , 90 }      [deg]
    vector<double> standby = {0, M_PI / 3 * 2, M_PI / 3, -M_PI / 4, M_PI / 2, -M_PI / 4, M_PI / 2, M_PI / 2, M_PI / 2};
    //     Back Array      : waist, R_arm1, L_arm1, R_arm2, R_arm3, L_arm2, L_arm3, R_wrist, L_wrist
    //                      { 0    , 80    , 80    , 0    , 0     , 0     , 0     , 90      , 90 }      [deg]
    vector<double> backarr = {0, M_PI * 0.4, M_PI * 0.4, 0, 0, 0, 0, M_PI / 2, M_PI / 2};

    float wrist_targetPos = M_PI / 18.0;
    double wrist_hit_time = 0.1;

    float wrist_backPos = M_PI / 6.0;
    double wrist_back_time = 0.1;

private:
    TMotorCommandParser TParser; ///< T 모터 명령어 파서.
    MaxonCommandParser MParser;  ///< Maxon 모터 명령어 파서

    State &state;                                                 ///< 시스템의 현재 상태입니다.
    CanManager &canManager;                                       ///< CAN 통신을 통한 모터 제어를 담당합니다.
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors; ///< 연결된 모터들의 정보입니다.

    // Functions for DrumRobot PathGenerating
    vector<double> c_MotorAngle = {0, 0, 0, 0, 0, 0, 0, 0, 0}; ///< 경로 생성 시 사용되는 현재 모터 위치 값.
    MatrixXd right_inst;                                       ///< 오른팔의 각 악기별 위치 좌표 벡터.
    MatrixXd left_inst;                                        ///< 왼팔의 각 악기별 위치 좌표 벡터.

    int n_inst = 10; ///< 총 악기의 수.
    double bpm = 50; /// 악보의 BPM 정보.

    vector<double> time_arr; ///< 악보의 시간간격 정보.
    MatrixXd inst_arr;       ///< 오른팔 / 왼팔이 치는 악기.
    VectorXd default_right;
    VectorXd default_left;

    // MatrixXd RF, LF;         ///< 오른발 / 왼발이 치는 악기.

    /* 실측값 */
    // vector<double> P1 = {0.3, 0.94344, 1.16582};       ///< 오른팔 준비자세 좌표.
    // vector<double> P2 = {-0.3, 0.94344, 1.16582};      ///< 왼팔 준비자세 좌표.
    // vector<double> R = {0.363, 0.793, 0.363, 0.793};     ///< [오른팔 상완, 오른팔 하완+스틱, 왼팔 상완, 왼팔 하완+스틱]의 길이.
    VectorXd part_length;
    double s = 0.600;  ///< 허리 길이.
    double z0 = 1.026; ///< 바닥부터 허리까지의 높이.

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
     * @brief 각 모터의 현재위치 값을 불러옵니다.
     */
    void getMotorPos();

    /**
     * @brief 생성한 경로를 각 모터의 버퍼에 쌓아줍니다.
     */
    void Motors_sendBuffer(VectorXd &Qi, VectorXd &Vi, pair<double, double> Si);

    ////////////////////////////// New Motor Generation ///////////////////////////////
    /**
     * @brief
     */
    MatrixXd tms_fun(double t2_a, double t2_b, VectorXd &inst2_a, VectorXd &inst2_b);
    void itms0_fun(vector<double> &t2, MatrixXd &inst2, MatrixXd &A30, MatrixXd &A31, MatrixXd &AA40, MatrixXd &AA41);
    void itms_fun(vector<double> &t2, MatrixXd &inst2, MatrixXd &B, MatrixXd &BB);
    VectorXd pos_madi_fun(VectorXd &A);
    MatrixXd sts2wrist_fun(MatrixXd &AA, double v_wrist);
    MatrixXd sts2elbow_fun(MatrixXd &AA, double v_elbow);
    VectorXd ikfun_final(VectorXd &pR, VectorXd &pL, VectorXd &part_length, double s, double z0);
    double con_fun(double t_a, double t_b, double th_a, double th_b, double t_now);
    pair<double, double> iconf_fun(double qk1_06, double qk2_06, double qk3_06, double qv_in, double t1, double t2, double t);
    pair<double, double> qRL_fun(MatrixXd &t_madi, double t_now);
    pair<double, double> SetTorqFlag(MatrixXd &State, double t_now);
    void SetTargetPos(MatrixXd &State, MatrixXd &t_madi);
};