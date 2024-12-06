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
    void SetReadyAngle();

    int line = 0;  ///< 연주를 진행하고 있는 줄.
    
    /////////////////////////////////////////////////////////////////////////// Read & Parse Measure
    bool readMeasure(ifstream& inputFile, bool &BPMFlag, double &timeSum);
    void parseMeasure(double &timeSum);

    float bpm = 0;         /// txt 악보의 BPM 정보.
    queue<vector<string>> Q; // 읽은 악보 저장한 큐
    
    /////////////////////////////////////////////////////////////////////////// Play
    
    void seonwoo_solveIK(VectorXd &pR1, VectorXd &pL1);
    void solveIKFixedWaist(VectorXd &pR1, VectorXd &pL1, VectorXd &q_lin);

    // SeonWoo
    void seonwoo_generateTrajectory();

    // x, y, z 저장할 구조체
    typedef struct {

        // 오른팔 좌표
        VectorXd pR; // 0: x, 1: y, 2: z

        // 왼팔 좌표
        VectorXd pL; // 0: x, 1: y, 2: z

        VectorXd qLin;

        // 허리 각도
        float waist_q;

        // 손목 각도, 팔꿈치 추가 각도
        VectorXd add_qR;
        VectorXd add_qL;

        // 브레이크
        bool brake_state[8];

    }Pos;

    // 타격 궤적 생성 파라미터
    typedef struct {

        float wristStayAngle = 10.0 * M_PI / 180.0;
        float wristHitAngle = -5.0 * M_PI / 180.0;
        float wristLiftAngle = 25.0 * M_PI / 180.0;

        float elbowStayAngle = 3.0 * M_PI / 180.0;
        float elbowLiftAngle = 6.0 * M_PI / 180.0;

    }HitParameter;

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
    
    VectorXd default_right; /// 오른팔 시작 위치
    VectorXd default_left;  /// 왼팔 시작 위치
    MatrixXd right_drum_position;                               ///< 오른팔의 각 악기별 위치 좌표 벡터.
    MatrixXd left_drum_position;                                ///< 왼팔의 각 악기별 위치 좌표 벡터.

    /////////////////////////////////////////////////////////////////////////// AddStance
    // q1[rad], q2[rad], acc[rad/s^2], t2[s]
    VectorXd cal_Vmax(VectorXd &q1, VectorXd &q2, float acc, float t2);
    // q1[rad], q2[rad], Vmax[rad/s], acc[rad/s^2], t[s], t2[s]
    VectorXd makeProfile(VectorXd &q1, VectorXd &q2, VectorXd &Vmax, float acc, float t, float t2);
    void getMotorPos();

    vector<float> c_MotorAngle = {0, 0, 0, 0, 0, 0, 0, 0, 0}; ///< 경로 생성 시 사용되는 현재 모터 위치 값

    /////////////////////////////////////////////////////////////////////////// Play
    double timeScaling(double ti, double tf, double t);
    VectorXd ikfun_final(VectorXd &pR, VectorXd &pL);

    VectorXd ikfun_fixed_waist(VectorXd &pR, VectorXd &pL, float theta0);

    void getState(vector<float> &t3, MatrixXd &inst3, MatrixXd &state);
    VectorXd getTargetPosition(VectorXd &inst_vector);

    const bool XYZm = false; // 궤적 생성 중 정지 여부
    
    // SeonWoo
    VectorXd seonwoo_makePath(VectorXd Pi, VectorXd Pf, double s);
    VectorXd makeHitTrajetory(int state, float ti, float tf, float t, HitParameter parameters);
    float makeWristAngleCLH(float ti, float tf, float t, HitParameter parameters);
    float makeWristAngleSLH(float ti, float tf, float t, HitParameter parameters);
    float makeWristAngleCS(float ti, float tf, float t, HitParameter parameters);
    float makeElbowAngle(float ti, float tf, float t, HitParameter parameters);
    void seonwoo_getInstrument();

    float seonwoo_q0_t1, seonwoo_q0_t2;
    VectorXd seonwoo_inst_now_R;
    VectorXd seonwoo_inst_now_L;      /// 연주 중 현재 위치하는 악기 저장

    VectorXd seonwoo_inst_i = VectorXd::Zero(18);
    VectorXd seonwoo_inst_f = VectorXd::Zero(18);
    VectorXd seonwoo_state = VectorXd::Zero(2);

    float seonwoo_tR_i, seonwoo_tR_f;
    float seonwoo_tL_i, seonwoo_tL_f;

    float seonwoo_t1, seonwoo_t2;
    
    VectorXd seonwoo_hitR = VectorXd::Zero(2);
    VectorXd seonwoo_hitL = VectorXd::Zero(2);

    
    void pushConmmandBuffer(VectorXd &Qi);

    /////////////////////////////////////////////////////////////////////////// Read & Parse Measure
    string trimWhitespace(const std::string &str);

    double threshold = 2.4;
    double total_time = 0.0;
    double detect_time_R = 0;
    double detect_time_L = 0;
    double current_time = 0;
    double moving_start_R = 0;
    double moving_start_L = 0;
    int line_n = 0; 
    vector<string> prev_col = { "0","0","0","0","0","0","0","0" };
};