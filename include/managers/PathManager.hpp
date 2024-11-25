#pragma once

#include <stdio.h>
#include "../include/managers/CanManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/SystemState.hpp"
#include "../include/tasks/Functions.hpp"

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
                std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
                Functions &funRef);

    
    /////////////////////////////////////////////////////////////////////////// Init

    void GetDrumPositoin();
    void GetMusicSheet();
    void SetReadyAng();

    int total = 0; ///< 악보의 전체 줄 수.
    int line = 0;  ///< 연주를 진행하고 있는 줄.
    float bpm = 0;         /// txt 악보의 BPM 정보.

    /////////////////////////////////////////////////////////////////////////// Perform

    void PathLoopTask();

    /////////////////////////////////////////////////////////////////////////// Play

    void generateTrajectory();
    void solveIK(VectorXd &pR1, VectorXd &pL1);
    void solveIKFixedWaist(VectorXd &pR1, VectorXd &pL1, VectorXd &q_lin);

    // x, y, z 저장할 구조체
    typedef struct {

        // 오른팔 좌표
        VectorXd pR; // 0: x, 1: y, 2: z

        // 왼팔 좌표
        VectorXd pL; // 0: x, 1: y, 2: z

        VectorXd qLin;

    }Pos;

    // 타격 각도 저장할 구조체
    typedef struct {
        float hitR; // 오른손 타격 각도
        float hitL; // 왼손 타격 각도
    }HitRL;

    // 이전에 타격이었는지 확인
    bool prevR = 0; 
    bool prevL = 0;

    float wristReadyAng = 0.2;

    queue<Pos> P; // 구조체 담아놓을 큐
    queue<HitRL> Hit; // 타격 시 각도값 받는 큐

    /////////////////////////////////////////////////////////////////////////// AddStance

    void GetArr(vector<float> &arr);

    vector<float> makeHomeArr(int cnt);

    //   Ready Pos Array   :  waist         , R_arm1        , L_arm1        , R_arm2        , R_arm3        , L_arm2        , L_arm3        , R_wrist       , L_wrist
    //                       { 0            , 90            , 90            , 45            , 75            , 45            , 75            , 30            , 30         } [deg]
    vector<float> readyArr = { 0            , M_PI / 2.0    , M_PI / 2.0    , M_PI * 0.25   , M_PI / 2.4    , M_PI * 0.25   , M_PI / 2.4    , M_PI / 6.0    , M_PI / 6.0 };

    //   Home Pos Array    : waist          , R_arm1        , L_arm1        , R_arm2    , R_arm3            , L_arm2    , L_arm3            , R_wrist               , L_wrist
    //                      { 10            , 90            , 90            , 0         , 120               , 0         , 120               , 95                    , 95                    } [deg]
    vector<float> homeArr = { M_PI / 18.0   , M_PI / 2.0    , M_PI / 2.0    , 0         , M_PI * (2.0/3.0)  , 0         , M_PI * (2.0/3.0)  , M_PI * (95.0/180.0)   , M_PI * (95.0/180.0)   };

    //   Back Pos Array    : waist      , R_arm1        , L_arm1        , R_arm2    , R_arm3    , L_arm2    , L_arm3    , R_wrist       , L_wrist
    //                      { 0         , 135           , 45            , 0         , 0         , 0         , 0         , 90            , 90         } [deg]
    vector<float> backArr = { 0         ,M_PI * 0.75    , M_PI * 0.25   , 0         , 0         , 0         , 0         , M_PI / 2.0    , M_PI / 2.0 };

    /////////////////////////////////////////////////////////////////////////// 기타

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
    Functions &fun;

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


    typedef struct{

        float upperArm = 0.250;         ///< 상완 길이.
        float lowerArm = 0.328;         ///< 하완 길이.
        float stick = 0.325+0.048;      ///< 스틱 길이 + 브라켓 길이.
        float waist = 0.520;            ///< 허리 길이.
        float height = 1.020-0.0605;    ///< 바닥부터 허리까지의 높이.

    }PartLength;


    /////////////////////////////////////////////////////////////////////////// Init
    string trimWhitespace(const std::string &str);

    string score_path = "../include/codes/testTrajectory_2.txt";        /// 악보 txt 파일 주소
    vector<float> time_arr; /// txt 악보의 시간간격 정보.
    MatrixXd inst_arr;      /// txt 악보의 오른팔 / 왼팔이 치는 악기.
    VectorXd default_right; /// 오른팔 시작 위치
    VectorXd default_left;  /// 왼팔 시작 위치
    MatrixXd right_drum_position;                               ///< 오른팔의 각 악기별 위치 좌표 벡터.
    MatrixXd left_drum_position;                                ///< 왼팔의 각 악기별 위치 좌표 벡터.

    /////////////////////////////////////////////////////////////////////////// AddStance
    // q1[rad], q2[rad], acc[rad/s^2], t2[s]
    VectorXd cal_Vmax(VectorXd &q1, VectorXd &q2, float acc, float t2);
    // q1[rad], q2[rad], Vmax[rad/s], acc[rad/s^2], t[s], t2[s]
    VectorXd makeProfile(VectorXd &q1, VectorXd &q2, VectorXd &Vmax, float acc, float t, float t2);
    void makeHitPath(float ti, float tf, float t, MatrixXd &AA);
    void makeHitPath_test(float ti, float tf, float t, MatrixXd &AA, float intensity);
    void getMotorPos();

    vector<float> c_MotorAngle = {0, 0, 0, 0, 0, 0, 0, 0, 0}; ///< 경로 생성 시 사용되는 현재 모터 위치 값

    /////////////////////////////////////////////////////////////////////////// Play
    float timeScaling_33(float ti, float tf, float t, float tm, float sm);
    float timeScaling_3(float ti, float tf, float t);
    VectorXd makePath_1(VectorXd Pi, VectorXd Pf, float s[], float sm, float h);
    VectorXd makePath_2(VectorXd Pi, VectorXd Pf, float s[], float sm, float h);

    VectorXd ikfun_fixed_waist(VectorXd &pR, VectorXd &pL, float theta0);

    void getState(vector<float> &t3, MatrixXd &inst3, MatrixXd &state);
    VectorXd getInstrumentPosition(VectorXd &A);

    VectorXd inst_now_R;
    VectorXd inst_now_L;      /// 연주 중 현재 위치하는 악기 저장

    const bool XYZm = false; // 궤적 생성 중 정지 여부

    /////////////////////////////////////////////////////////////////////////// Perform & Play
    MatrixXd tms_fun(float t2_a, float t2_b, VectorXd &inst2_a, VectorXd &inst2_b);
    void itms0_fun(vector<float> &t2, MatrixXd &inst2, MatrixXd &A30, MatrixXd &A31, MatrixXd &AA40, MatrixXd &AA41);
    void itms_fun(vector<float> &t2, MatrixXd &inst2, MatrixXd &B, MatrixXd &BB, VectorXd &pre_inst);
    VectorXd pos_madi_fun(VectorXd &A);
    VectorXd ikfun_final(VectorXd &pR, VectorXd &pL);
    void pushConmmandBuffer(VectorXd &Qi, bool brake_state);
    
    VectorXd inst_now;      /// 연주 중 현재 위치하는 악기 저장

    /////////////////////////////////////////////////////////////////////////// 손목
    MatrixXd sts2wrist_fun(MatrixXd &AA);
    MatrixXd sts2elbow_fun(MatrixXd &AA);
    pair<float, float> q78_fun(MatrixXd &t_madi, float t_now);
    float con_fun_pos(float th_a, float th_b, float k, float n);

    float wrist_ready = 30 * M_PI / 180.0;                // 타격 시 들어올리는 손목 각도 (-1)
    float wrist_stanby = 10 * M_PI / 180.0;                 // 대기 시 들어올리는 손목 각도 (-0.5)
    float elbow_ready = 15 * M_PI / 180.0;                  // 타격 시 들어올리는 팔꿈치 각도 (-1)
    float elbow_stanby = 5 * M_PI / 180.0;                  // 대기 시 들어올리는 팔꿈치 각도 (-0.5)


    ///////////////////////////////////////////////////////////////////////////
};