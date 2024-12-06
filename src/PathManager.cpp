#include "../include/managers/PathManager.hpp" // 적절한 경로로 변경하세요.

// For Qt
// #include "../managers/PathManager.hpp"
PathManager::PathManager(State &stateRef,
                         CanManager &canManagerRef,
                         std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
                         Functions &funRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef), fun(funRef)
{
}

////////////////////////////////////////////////////////////////////////////////
/*                              Initialization                                */
////////////////////////////////////////////////////////////////////////////////

void PathManager::GetDrumPositoin()
{
    ifstream inputFile("../include/managers/rT.txt");

    if (!inputFile.is_open())
    {
        cerr << "Failed to open the file."
             << "\n";
    }

    // Read data into a 2D vector
    MatrixXd inst_xyz(6, 8);

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 8; ++j)
        {
            inputFile >> inst_xyz(i, j);
            if (i == 1 || i == 4)
                inst_xyz(i, j) *= 1.0;
        }
    }

    right_drum_position.resize(3, 9);
    left_drum_position.resize(3, 9);

    // Extract the desired elements
    Vector3d right_S;
    Vector3d right_FT;
    Vector3d right_MT;
    Vector3d right_HT;
    Vector3d right_HH;
    Vector3d right_R;
    Vector3d right_RC;
    Vector3d right_LC;

    Vector3d left_S;
    Vector3d left_FT;
    Vector3d left_MT;
    Vector3d left_HT;
    Vector3d left_HH;
    Vector3d left_R;
    Vector3d left_RC;
    Vector3d left_LC;

    for (int i = 0; i < 3; ++i)
    {
        right_S(i) = inst_xyz(i, 0);
        right_FT(i) = inst_xyz(i, 1);
        right_MT(i) = inst_xyz(i, 2);
        right_HT(i) = inst_xyz(i, 3);
        right_HH(i) = inst_xyz(i, 4);
        right_R(i) = inst_xyz(i, 5);
        right_RC(i) = inst_xyz(i, 6);
        right_LC(i) = inst_xyz(i, 7);

        left_S(i) = inst_xyz(i + 3, 0);
        left_FT(i) = inst_xyz(i + 3, 1);
        left_MT(i) = inst_xyz(i + 3, 2);
        left_HT(i) = inst_xyz(i + 3, 3);
        left_HH(i) = inst_xyz(i + 3, 4);
        left_R(i) = inst_xyz(i + 3, 5);
        left_RC(i) = inst_xyz(i + 3, 6);
        left_LC(i) = inst_xyz(i + 3, 7);
    }

    right_drum_position << right_RC, right_R, right_S, right_HH, right_HH, right_FT, right_MT, right_LC, right_HT;
    left_drum_position << left_RC, left_R, left_S, left_HH, left_HH, left_FT, left_MT, left_LC, left_HT;
}

void PathManager::SetReadyAngle()
{
    VectorXd inst_p(18);

    default_right.resize(9);
    default_left.resize(9);
    default_right << 0, 0, 1, 0, 0, 0, 0, 0, 0;
    default_left << 0, 0, 1, 0, 0, 0, 0, 0, 0;

    inst_p << default_right,
        default_left;

    MatrixXd combined(6, 18);
    combined << right_drum_position, MatrixXd::Zero(3, 9), MatrixXd::Zero(3, 9), left_drum_position;
    MatrixXd p = combined * inst_p;

    VectorXd pR = VectorXd::Map(p.data(), 3, 1);
    VectorXd pL = VectorXd::Map(p.data() + 3, 3, 1);
    VectorXd qk = ikfun_final(pR, pL);

    for (int i = 0; i < qk.size(); ++i)
    {
        readyArr[i] = qk(i);
    }
    readyArr[7] = wristReadyAng;
    readyArr[8] = wristReadyAng;
}

////////////////////////////////////////////////////////////////////////////////    
/*                       Play (Task Space Trajectory)                         */
////////////////////////////////////////////////////////////////////////////////

void PathManager::seonwoo_generateTrajectory()
{
    // position
    VectorXd Pi(6), Pf(6);
    VectorXd Pi_R(3);
    VectorXd Pi_L(3);
    VectorXd Pf_R(3);
    VectorXd Pf_L(3);

    double n, s_R, s_L;
    double delta_t_measure_R = seonwoo_tR_f - seonwoo_tR_i;
    double delta_t_measure_L = seonwoo_tL_f - seonwoo_tL_i;
    double dt = canManager.deltaT;

    // state
    seonwoo_getInstrument();

    // position
    Pi = getTargetPosition(seonwoo_inst_i);
    Pf = getTargetPosition(seonwoo_inst_f);

    Pi_R << Pi(0), Pi(1), Pi(2);
    Pi_L << Pi(3), Pi(4), Pi(5);
    Pf_R << Pf(0), Pf(1), Pf(2);
    Pf_L << Pf(3), Pf(4), Pf(5);

    std::cout << "\nPi_R\n" << Pi_R
    << "\nPi_L\n" << Pi_L
    << "\nPf_R\n" << Pf_R
    << "\nPf_L\n" << Pf_L << std::endl;

    // trajectory
    n = (seonwoo_t2 - seonwoo_t1) / dt;
    for (int i = 0; i < n; i++)
    {
        Pos Pt;
        double t_R = dt * i + seonwoo_t1 - seonwoo_tR_i;
        double t_L = dt * i + seonwoo_t1 - seonwoo_tL_i;

        s_R = timeScaling(0.0f, delta_t_measure_R, t_R);
        s_L = timeScaling(0.0f, delta_t_measure_L, t_L);

        Pt.pR = seonwoo_makePath(Pi_R, Pf_R, s_R);
        Pt.pL = seonwoo_makePath(Pi_L, Pf_L, s_L);

        if (i == 0)
        {
            VectorXd q_t1 = ikfun_final(Pt.pR, Pt.pL);
            seonwoo_q0_t1 = q_t1(0);
        }
        else if (i == n - 1)
        {
            VectorXd q_t2 = ikfun_final(Pt.pR, Pt.pL);
            seonwoo_q0_t2 = q_t2(0);
        }

        Pt.waist_q = 0.0;

        HitParameter param;
        Pt.add_qR = makeHitTrajetory(seonwoo_state(0), 0.0f, delta_t_measure_R, t_R, param);
        Pt.add_qL = makeHitTrajetory(seonwoo_state(1), 0.0f, delta_t_measure_L, t_L, param);

        // brake
        for (int j = 0; j < 8; j++)
        {
            Pt.brake_state[j] = false;
        }
        
        P.push(Pt);

        std::string fileName;
        fileName = "Trajectory_R";
        fun.appendToCSV_DATA(fileName, Pt.pR[0], Pt.pR[1], Pt.pR[2]);
        fileName = "Trajectory_L";
        fun.appendToCSV_DATA(fileName, Pt.pL[0], Pt.pL[1], Pt.pL[2]);
        // fileName = "S";
        // fun.appendToCSV_DATA(fileName, s[0], s[1], 0);
    }
}

void PathManager::seonwoo_solveIK(VectorXd &pR1, VectorXd &pL1)
{
    // HitRL CurRL;

    VectorXd q(9);

    VectorXd q_06 = ikfun_final(pR1, pL1);

    for (int i = 0; i < 7; i++)
    {
        q(i) = q_06(i);
    }

    // CurRL = Hit.front(); Hit.pop();
    
    // q(7) = CurRL.hitR;
    // q(8) = CurRL.hitL;

    pushConmmandBuffer(q);

    // 데이터 기록
    for (int m = 0; m < 9; m++)
    {
        std::string fileName = "solveIK_q" + to_string(m);
        fun.appendToCSV_DATA(fileName, m, q(m), 0);
    }
}

void PathManager::solveIKFixedWaist(VectorXd &pR1, VectorXd &pL1, VectorXd &q_lin)
{
    VectorXd q = ikfun_fixed_waist(pR1, pL1, q_lin(0));
    HitRL CurRL;

    CurRL = Hit.front(); Hit.pop();
    
    q(7) = CurRL.hitR;
    q(8) = CurRL.hitL;

    pushConmmandBuffer(q);

    // // 데이터 기록
    // for (int m = 0; m < 9; m++)
    // {
    //     std::string fileName = "solveIK_q" + to_string(m);
    //     // fun.appendToCSV_DATA(fileName, q_ik(m), q(m), q_lin(m));
    //     fun.appendToCSV_DATA(fileName, m, q(m), q_lin(m));
    // }
}

////////////////////////////////////////////////////////////////////////////////
/*                               AddStance                                    */
////////////////////////////////////////////////////////////////////////////////

void PathManager::GetArr(vector<float> &arr)
{
    const float acc_max = 100.0;    // rad/s^2
    //vector<float> Qi;
    //vector<float> Vmax;
    VectorXd Q1 = VectorXd::Zero(9);
    VectorXd Q2 = VectorXd::Zero(9);
    VectorXd Qi = VectorXd::Zero(9);
    VectorXd Vmax = VectorXd::Zero(9);

    float dt = canManager.deltaT;   // 0.005
    float t = 3.0;                  // 3초동안 실행
    float extra_time = 1.0;         // 추가 시간 1초
    int n = (int)(t / dt);   
    int n_p = (int)(extra_time / dt); 

    getMotorPos(); 

    for (int i = 0; i < 9; i++)
    {
        Q1(i) = c_MotorAngle[i];
        Q2(i) = arr[i];
    }

    Vmax = cal_Vmax(Q1, Q2, acc_max, t);

    for (int k = 0; k < 9; k++)
    {
        cout << "Q1[" << k << "] : " << Q1[k]*180.0/M_PI <<  " [deg] -> Q2[" << k << "] : " << Q2[k]*180.0/M_PI << " [deg]" << endl;
        cout << "Vmax[" << k << "] : " << Vmax(k) << "[rad/s]\n\n";
    }

    for (int k = 1; k <= n + n_p; ++k)
    {
        // Make Array
        Qi = makeProfile(Q1, Q2, Vmax, acc_max, t*k/n, t);

        // Send to Buffer
        for (auto &entry : motors)
        {
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
            {
                TMotorData newData;
                newData.position = Qi[motor_mapping[entry.first]];
                newData.spd = 0;
                newData.acl = 0;
                newData.isBrake = false;
                tMotor->commandBuffer.push(newData);
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
            {
                MaxonData newData;
                newData.position = Qi[motor_mapping[entry.first]];
                newData.WristState = 0.5;
                maxonMotor->commandBuffer.push(newData);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                              Play FUNCTION                                 */
////////////////////////////////////////////////////////////////////////////////

float PathManager::timeScaling_33(float ti, float tf, float t, float tm, float sm)
{
    float s;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd x;

    A.resize(4,4);
    b.resize(4,1);

    if (t < tm)
    {
        A << 1, ti, ti*ti, ti*ti*ti,
        1, tm, tm*tm, tm*tm*tm,
        0, 1, 2*ti, 3*ti*ti,
        0, 1, 2*tm, 3*tm*tm;

        b << 0, sm, 0, 0;

        A_1 = A.inverse();
        x = A_1 * b;

        s = x(0,0) + x(1,0) * t + x(2,0) * t * t + x(3,0) * t * t * t;
    }
    else
    {
        A << 1, tm, tm*tm, tm*tm*tm,
        1, tf, tf*tf, tf*tf*tf,
        0, 1, 2*tm, 3*tm*tm,
        0, 1, 2*tf, 3*tf*tf;

        b << sm, 1, 0, 0;

        A_1 = A.inverse();
        x = A_1 * b;

        s = x(0,0) + x(1,0) * t + x(2,0) * t * t + x(3,0) * t * t * t;
    }

    return s;
}

double PathManager::timeScaling(double ti, double tf, double t)
{
    double s;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    A.resize(4,4);
    b.resize(4,1);

    A << 1, ti, ti*ti, ti*ti*ti,
    1, tf, tf*tf, tf*tf*tf,
    0, 1, 2*ti, 3*ti*ti,
    0, 1, 2*tf, 3*tf*tf;

    b << 0, 1, 0, 0;

    A_1 = A.inverse();
    sol = A_1 * b;

    s = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;

    return s;
}

VectorXd PathManager::makePath_1(VectorXd Pi, VectorXd Pf, float s[], float sm, float h)    // 경로 1차 함수
{
    float xi = Pi(0), xf = Pf(0);
    float yi = Pi(1), yf = Pf(1);
    float zi = Pi(2), zf = Pf(2);

    float xm = xi + (xf - xi) * sm;
    float ym = yi + (yf - yi) * sm;
    float zm = std::max(zi, zf) + h;

    VectorXd Ps;
    Ps.resize(3);

    if (Pi == Pf)
    {
        Ps(0) = xi;
        Ps(1) = yi;
        Ps(2) = zi;
    }
    else
    {
        if(XYZm)
        {
            // x, y, z 모두 중간점에서 정지

            if (s[0] < sm)
            {
                // x
                Ps(0) = xi + (xm - xi) * s[0] / sm;
                // y
                Ps(1) = yi + (ym - yi) * s[0] / sm;
                // z
                Ps(2) = zi + (zm - zi) * s[0] / sm;
            }
            else
            {
                // x
                Ps(0) = xm + (xf - xm) * (s[0] - sm) / (1 - sm);
                // y
                Ps(1) = ym + (yf - ym) * (s[0] - sm) / (1 - sm);
                // z
                Ps(2) = zm + (zf - zm) * (s[0] - sm) / (1 - sm);
            }
        }
        else
        {
            // z만 중간점에서 정지

            // x
            Ps(0) = xi + (xf - xi) * s[1];
            // y
            Ps(1) = yi + (yf - yi) * s[1];
            if (s[0] < sm)
            {
                // z
                Ps(2) = zi + (zm - zi) * s[0] / sm;
            }
            else
            {
                // z
                Ps(2) = zm + (zf - zm) * (s[0] - sm) / (1 - sm);
            }
        }
    }

    return Ps;
}

VectorXd PathManager::makePath_2(VectorXd Pi, VectorXd Pf, float s[], float sm, float h)    // 경로 2차 함수
{
    float xi = Pi(0), xf = Pf(0);
    float yi = Pi(1), yf = Pf(1);
    float zi = Pi(2), zf = Pf(2);

    float xm = xi + (xf - xi) * sm;
    float ym = yi + (yf - yi) * sm;
    float zm = std::max(zi, zf) + h;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd x;

    VectorXd Ps;
    Ps.resize(3);

    A.resize(3,3);
    b.resize(3,1);

    if (Pi == Pf)
    {
        Ps(0) = xi;
        Ps(1) = yi;
        Ps(2) = zi;
    }
    else
    {
        if (s[0] < sm)
        {
            if(XYZm)
            {
                // x, y, z 모두 중간점에서 정지
                // x
                Ps(0) = xi + (xm - xi) * s[0] / sm;
                // y
                Ps(1) = yi + (ym - yi) * s[0] / sm;
            }
            else
            {
                // z 만 중간점에서 정지
                // x
                Ps(0) = xi + (xf - xi) * s[1];
                // y
                Ps(1) = yi + (yf - yi) * s[1];
            }

            A << 1, 0, 0,
            1, sm, sm*sm,
            0, 1, 2*sm;

            b << zi, zm, 0;

            A_1 = A.inverse();
            x = A_1 * b;

            Ps(2) = x(0,0) + x(1,0) * s[0] + x(2,0) * s[0] * s[0];
        }
        else
        {
            if(XYZm)
            {
                // x, y, z 모두 중간점에서 정지
                // x
                Ps(0) = xm + (xf - xm) * (s[0] - sm) / (1 - sm);
                // y
                Ps(1) = ym + (yf - ym) * (s[0] - sm) / (1 - sm);
            }
            else
            {
                // z 만 중간점에서 정지
                // x
                Ps(0) = xi + (xf - xi) * s[1];
                // y
                Ps(1) = yi + (yf - yi) * s[1];
            }

            A << 1, sm, sm*sm,
            1, 1, 1,
            0, 1, 2*sm;

            b << zm, zf, 0;

            A_1 = A.inverse();
            x = A_1 * b;

            Ps(2) = x(0,0) + x(1,0) * s[0] + x(2,0) * s[0] * s[0];
        }
    }

    return Ps;
}

VectorXd PathManager::seonwoo_makePath(VectorXd Pi, VectorXd Pf, double s)
{
    double degree = 2.0;

    double xi = Pi(0), xf = Pf(0);
    double yi = Pi(1), yf = Pf(1);
    double zi = Pi(2), zf = Pf(2);

    VectorXd Ps;
    Ps.resize(3);

    if (Pi == Pf)
    {
        Ps(0) = xi;
        Ps(1) = yi;
        Ps(2) = zi;
    }
    else
    {
        Ps(0) = xi + s * (xf - xi);
        Ps(1) = yi + s * (yf - yi);

        if (zi > zf)
        {
            double a = zf - zi;
            double b = zi;

            Ps(2) = a*std::pow(s, degree) + b;
        }
        else
        {
            double a = (zi - zf) * std::pow(-1, degree);
            double b = zf;

            Ps(2) = a*std::pow(s-1, degree) + b;
        }
    }

    return Ps;
}

VectorXd PathManager::makeHitTrajetory(int state, float ti, float tf, float t, HitParameter param)
{
    VectorXd addAngle;
    addAngle.resize(2);    // wrist, elbow

    if (state == 1)
    {
        // Contact - Lift - Hit
        addAngle(0) = makeWristAngleCLH(ti, tf, t, param);
        addAngle(1) = makeElbowAngle(ti, tf, t, param);
    }
    else if (state == 2)
    {
        // Stay - Lift - Hit
        addAngle(0) = makeWristAngleSLH(ti, tf, t, param);
        addAngle(1) = makeElbowAngle(ti, tf, t, param);
    }
    else if (state == 3)
    {
        // Contact - Stay
        addAngle(0) = makeWristAngleCS(ti, tf, t, param);
        addAngle(1) = makeElbowAngle(ti, tf, t, param);
    }
    else
    {
        // Stay
        addAngle(0) = param.wristStayAngle;
        addAngle(1) = param.elbowStayAngle;
    }
   
    return addAngle;
}

float PathManager::makeWristAngleCLH(float ti, float tf, float t, HitParameter param)
{
    float wrist_q;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    float t_contact = 0.2 * (tf - ti);
    float t_lift = 0.6 * (tf - ti);
    float t_end = tf - ti;    // t_hit
    float hit_angle = param.wristHitAngle;
    float lift_angle = param.wristLiftAngle;

    if (t < t_contact)
    {
        A.resize(3,3);
        b.resize(3,1);

        A << 1, 0, 0,
            1, t_contact, t_contact*t_contact,
            0, 1, 2*t_contact;

        b << 0, hit_angle, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t;
    }
    else if (t < t_lift)
    {
        A.resize(4,4);
        b.resize(4,1);

        A << 1, t_contact, t_contact*t_contact, t_contact*t_contact*t_contact,
            1, t_lift, t_lift*t_lift, t_lift*t_lift*t_lift,
            0, 1, 2*t_contact, 3*t_contact*t_contact,
            0, 1, 2*t_lift, 3*t_lift*t_lift;

        b << hit_angle, lift_angle, 0, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
    }
    else if (t < t_end)
    {
        A.resize(3,3);
        b.resize(3,1);

        A << 1, t_lift, t_lift*t_lift,
            1, t_end, t_end*t_end,
            0, 1, 2*t_lift;

        b << lift_angle, 0, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t;
    }
    else
    {
        wrist_q = 0.0;
    }

    return wrist_q;
}

float PathManager::makeWristAngleSLH(float ti, float tf, float t, HitParameter param)
{
    float wrist_q;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    float t_lift = 0.6 * (tf - ti);
    float t_end = tf - ti;    // t_hit
    float stay_angle = param.wristStayAngle;
    float lift_angle = param.wristLiftAngle;

    if (t < t_lift)
    {
        A.resize(4,4);
        b.resize(4,1);

        A << 1, 0, 0, 0,
            1, t_lift, t_lift*t_lift, t_lift*t_lift*t_lift,
            0, 1, 0, 0,
            0, 1, 2*t_lift, 3*t_lift*t_lift;

        b << stay_angle, lift_angle, 0, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
    }
    else if (t < t_end)
    {
        A.resize(3,3);
        b.resize(3,1);

        A << 1, t_lift, t_lift*t_lift,
            1, t_end, t_end*t_end,
            0, 1, 2*t_lift;

        b << lift_angle, 0, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t;
    }
    else
    {
        wrist_q = 0.0;
    }

    return wrist_q;
}

float PathManager::makeWristAngleCS(float ti, float tf, float t, HitParameter param)
{
    float wrist_q;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;
    
    float t_contact = 0.2 * (tf - ti);
    float t_lift = 0.6 * (tf - ti);   // t_stay
    float hit_angle = param.wristHitAngle;
    float stay_angle = param.wristStayAngle;

    if (t < t_contact)
    {
        A.resize(3,3);
        b.resize(3,1);

        A << 1, 0, 0,
            1, t_contact, t_contact*t_contact,
            0, 1, 2*t_contact;

        b << 0, hit_angle, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t;
    }
    else if (t < t_lift)
    {
        A.resize(4,4);
        b.resize(4,1);

        A << 1, t_contact, t_contact*t_contact, t_contact*t_contact*t_contact,
            1, t_lift, t_lift*t_lift, t_lift*t_lift*t_lift,
            0, 1, 2*t_contact, 3*t_contact*t_contact,
            0, 1, 2*t_lift, 3*t_lift*t_lift;

        b << hit_angle, stay_angle, 0, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
    }
    else
    {
        wrist_q = stay_angle;
    }

    return wrist_q;
}

float PathManager::makeElbowAngle(float ti, float tf, float t, HitParameter param)
{
    float wrist_q;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    float t_lift = 0.5 * (tf - ti);
    float t_end = tf - ti;    // t_hit
    float lift_angle = param.elbowLiftAngle;
    float start_angle = 0.0;
    float end_angle = 0.0;

    if (t < t_lift)
    {
        A.resize(4,4);
        b.resize(4,1);

        A << 1, 0, 0, 0,
            1, t_lift, t_lift*t_lift, t_lift*t_lift*t_lift,
            0, 1, 0, 0,
            0, 1, 2*t_lift, 3*t_lift*t_lift;

        b << start_angle, lift_angle, 0, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
    }
    else if (t < t_end)
    {
        A.resize(4,4);
        b.resize(4,1);

        A << 1, t_lift, t_lift*t_lift, t_lift*t_lift*t_lift,
            1, t_end, t_end*t_end, t_end*t_end*t_end,
            0, 1, 2*t_lift, 3*t_lift*t_lift,
            0, 1, 2*t_end, 3*t_end*t_end;

        b << lift_angle, end_angle, 0, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
    }
    else
    {
        wrist_q = end_angle;
    }

    return wrist_q;
}

void PathManager::seonwoo_getInstrument()
{
    float norm_R_i = seonwoo_inst_i.block(0, 0, 9, 1).norm();
    float norm_L_i = seonwoo_inst_i.block(9, 0, 9, 1).norm();
    float norm_R_f = seonwoo_inst_f.block(0, 0, 9, 1).norm();
    float norm_L_f = seonwoo_inst_f.block(9, 0, 9, 1).norm();

    if (norm_R_f == 0 && norm_R_i == 0)
    {
        seonwoo_inst_i.block(0, 0, 9, 1) = seonwoo_inst_now_R;
        seonwoo_inst_f.block(0, 0, 9, 1) = seonwoo_inst_now_R;
    }
    else if (norm_R_f == 0 && norm_R_i == 1)
    {
        seonwoo_inst_f.block(0, 0, 9, 1) = seonwoo_inst_now_R;
    }
    else if (norm_R_f == 1 && norm_R_i == 0)
    {
        seonwoo_inst_i.block(0, 0, 9, 1) = seonwoo_inst_now_R;
        seonwoo_inst_now_R = seonwoo_inst_f.block(0, 0, 9, 1);
    }
    else if (norm_R_f == 1 && norm_R_i == 1)
    {
        seonwoo_inst_now_R = seonwoo_inst_f.block(0, 0, 9, 1);
    }

    if (norm_L_f == 0 && norm_L_i == 0)
    {
        seonwoo_inst_i.block(9, 0, 9, 1) = seonwoo_inst_now_L;
        seonwoo_inst_f.block(9, 0, 9, 1) = seonwoo_inst_now_L;
    }
    else if (norm_L_f == 0 && norm_L_i == 1)
    {
        seonwoo_inst_f.block(9, 0, 9, 1) = seonwoo_inst_now_L;
    }
    else if (norm_L_f == 1 && norm_L_i == 0)
    {
        seonwoo_inst_i.block(9, 0, 9, 1) = seonwoo_inst_now_L;
        seonwoo_inst_now_L = seonwoo_inst_f.block(9, 0, 9, 1);
    }
    else if (norm_L_f == 1 && norm_L_i == 1)
    {
        seonwoo_inst_now_L = seonwoo_inst_f.block(9, 0, 9, 1);
    }
}

void PathManager::getState(vector<float> &t3, MatrixXd &inst3, MatrixXd &state)
{
    float norm_R0 = inst3.block(0, 0, 9, 1).norm();
    float norm_L0 = inst3.block(9, 0, 9, 1).norm();
    float norm_R1 = inst3.block(0, 1, 9, 1).norm();
    float norm_L1 = inst3.block(9, 1, 9, 1).norm();

    state.resize(3, 2);
    state << t3[0], t3[1], norm_R0, norm_R1, norm_L0, norm_L1;
}

VectorXd PathManager::getTargetPosition(VectorXd &inst_vector)
{
    VectorXd inst_right = inst_vector.segment(0, 9);
    VectorXd inst_left = inst_vector.segment(9, 9);

    if (inst_right.sum() == 0)
    {
        std::cout << "Right Instrument Vector Error!!\n";
        state.main = Main::Error;
    }

    if (inst_left.sum() == 0)
    {
        std::cout << "Left Instrument Vector Error!!\n";
        state.main = Main::Error;
    }

    VectorXd inst_p(18);
    inst_p << inst_right,
        inst_left;

    MatrixXd combined(6, 18);
    combined << right_drum_position, MatrixXd::Zero(3, 9), MatrixXd::Zero(3, 9), left_drum_position;
    MatrixXd p = combined * inst_p;

    return p;
}

VectorXd PathManager::ikfun_fixed_waist(VectorXd &pR, VectorXd &pL, float theta0)
{
    VectorXd Qf;
    PartLength part_length;

    float XR = pR(0), YR = pR(1), ZR = pR(2);
    float XL = pL(0), YL = pL(1), ZL = pL(2);
    float R1 = part_length.upperArm;
    float R2 = part_length.lowerArm + part_length.stick;
    float L1 = part_length.upperArm;
    float L2 = part_length.lowerArm + part_length.stick;
    float s = part_length.waist;
    float z0 = part_length.height;

    float shoulderXR = 0.5 * s * cos(theta0);
    float shoulderYR = 0.5 * s * sin(theta0);
    float shoulderXL = -0.5 * s * cos(theta0);
    float shoulderYL = -0.5 * s * sin(theta0);

    float theta01 = atan2(YR - shoulderYR, XR - shoulderXR);
    float theta1 = theta01 - theta0;

    if (theta1 < 0 || theta1 > 150.0 * M_PI / 180.0) // the1 범위 : 0deg ~ 150deg
    {
        std::cout << "IKFUN (q1) is not solved!!\n";
        state.main = Main::Error;
    }

    float theta02 = atan2(YL - shoulderYL, XL - shoulderXL);
    float theta2 = theta02 - theta0;

    if (theta2 < 30 * M_PI / 180.0 || theta2 > M_PI) // the2 범위 : 30deg ~ 180deg
    {
        std::cout << "IKFUN (q2) is not solved!!\n";
        state.main = Main::Error;
    }

    float zeta = z0 - ZR;
    float r2 = (YR - shoulderYR)*(YR - shoulderYR) + (XR - shoulderXR)*(XR - shoulderXR); // r^2

    float x = zeta*zeta + r2 - R1*R1 - R2*R2;
    float y = sqrt(4.0*R1*R1*R2*R2 - x*x);

    float theta4 = atan2(y,x);

    if (theta4 < 0 || theta4 > 140.0 * M_PI / 180.0) // the4 범위 : 0deg ~ 120deg
    {
        std::cout << "IKFUN (q4) is not solved!!\n";
        state.main = Main::Error;
    }

    float theta34 = atan2(sqrt(r2), zeta);
    float theta3 = theta34 - atan2(R2*sin(theta4), R1 + R2*cos(theta4));

    if (theta3 < -45.0 * M_PI / 180.0 || theta3 > 90.0 * M_PI / 180.0) // the3 범위 : -45deg ~ 90deg
    {
        std::cout << "IKFUN (q3) is not solved!!\n";
        state.main = Main::Error;
    }

    zeta = z0 - ZL;
    r2 = (YL - shoulderYL)*(YL - shoulderYL) + (XL - shoulderXL)*(XL - shoulderXL); // r^2

    x = zeta*zeta + r2 - L1*L1 - L2*L2;
    y = sqrt(4.0*L1*L1*L2*L2 - x*x);

    float theta6 = atan2(y,x);

    if (theta6 < 0 || theta6 > 140.0 * M_PI / 180.0) // the6 범위 : 0deg ~ 120deg
    {
        std::cout << "IKFUN (q6) is not solved!!\n";
        state.main = Main::Error;
    }

    float theta56 = atan2(sqrt(r2), zeta);
    float theta5 = theta56 - atan2(L2*sin(theta6), L1 + L2*cos(theta6));

    if (theta5 < -45.0 * M_PI / 180.0 || theta5 > 90.0 * M_PI / 180.0) // the5 범위 : -45deg ~ 90deg
    {
        std::cout << "IKFUN (q5) is not solved!!\n";
        state.main = Main::Error;
    }

    Qf.resize(9);
    Qf << theta0, theta1, theta2, theta3, theta4, theta5, theta6, 0.0, 0.0;

    return Qf;
}

bool PathManager::readMeasure(ifstream& inputFile, bool &BPMFlag, double &timeSum)
{
    string line;

    while(getline(inputFile, line))
    {
        istringstream iss(line);
        string item;
        int cnt = 0;

        vector<string> columns;
        while (getline(iss, item, '\t'))
        {
            if(cnt >= 8) break;
            item = trimWhitespace(item);
            columns.push_back(item);
            cnt++;
        }

        if (!BPMFlag)
        { // 첫번째 행엔 bpm에 대한 정보
            cout << "music";
            bpm = stod(columns[0].substr(4));
            cout << " bpm = " << bpm << "\n";
            BPMFlag = 1;
        }
        else
        {
            timeSum += stod(columns[1]);
            columns.push_back(to_string(total_time));
            total_time += stod(columns[1]);

            // 큐에 저장
            Q.push(columns);

            if(timeSum >= threshold)
            {
                // Q에 있는 요소들 출력
                queue<vector<string>> tempQ = Q; // 큐 복사본 사용
                while (!tempQ.empty())
                {
                    vector<string> current = tempQ.front();
                    tempQ.pop();

                    // 요소 출력 (탭으로 구분)
                    for (size_t i = 0; i < current.size(); ++i)
                    {
                        cout << current[i];
                        if (i != current.size() - 1) // 마지막 요소가 아니라면 탭 추가
                            cout << '\t';
                    }
                    cout << '\n'; // 다음 요소로 넘어갈 때 개행
                }
                cout << '\n'; // 한 반복 끝날 때 개행 두 번

                return true;
            }
        }
    }
    return false;
}

void PathManager::parseMeasure(double &timeSum)
{

    map<string, int> instrument_mapping = {
    {"1", 2}, {"2", 5}, {"3", 6}, {"4", 8}, {"5", 3}, {"6", 1}, {"7", 0}, {"8", 7}, {"11", 2}, {"51", 2}, {"61", 2}, {"71", 2}, {"81", 2}, {"91", 2}};
    // S        FT          MT       HT        HH        R         RC        LC         S          S          S          S          S           S
    
    // 지금 들어온 Q 맨 앞에 값이 현재 시간임
    vector<string> curLine = Q.front(); 
    current_time = stod(curLine[8]);  
    
    // 이전 위치에 대한 업데이트
    if(prev_col[2] != "0" || prev_col[3] != "0")
        {
            VectorXd inst_R_prev = VectorXd::Zero(9), inst_L_prev = VectorXd::Zero(9);
            if (prev_col[2] != "0")
            {
                inst_R_prev(instrument_mapping[prev_col[2]]) = 1.0; // 해당 악기 상태 활성화
            }

            if (prev_col[3] != "0")
            {
                inst_L_prev(instrument_mapping[prev_col[3]]) = 1.0; // 해당 악기 상태 활성화
            }
                seonwoo_inst_i << inst_R_prev, inst_L_prev;
        }

    // 들어왔을 때 현재 시간이 detect_timeR이나 detect_timeL보다 크거나 같으면 움직이기 시작하는 시간을 현재 시간으로 설정
    if (detect_time_R <= current_time)
    {
        moving_start_R = current_time;

    }
    if (detect_time_L <= current_time)
    {
        moving_start_L = current_time;
    }

    float sum = 0;
    float make_time = 0;
    //threshold/2
    VectorXd inst_R = VectorXd::Zero(9), inst_L = VectorXd::Zero(9);
    // VectorXd inst_next = VectorXd::Zero(18);

    // 큐를 전부 순회
    for (size_t i = 0; i < Q.size(); ++i)
    {
        curLine = Q.front();
        Q.pop();
        Q.push(curLine); // 현재 데이터를 다시 큐 끝에 삽입
        sum += stod(curLine[1]); // 합계 갱신

        // 오른손 타격 감지
        if (curLine[2] != "0" && !(inst_R.array() != 0).any())
        {
            inst_R(instrument_mapping[curLine[2]]) = 1.0; // 해당 악기 상태 활성화
            detect_time_R = stof(curLine[1]) + stof(curLine[8]); // 오른손 타격 시간 갱신
        }

        // 왼손 타격 감지
        if (curLine[3] != "0" && !(inst_L.array() != 0).any())
        {
            inst_L(instrument_mapping[curLine[3]]) = 1.0; // 해당 악기 상태 활성화
            detect_time_L = stof(curLine[1]) + stof(curLine[8]); // 왼손 타격 시간 갱신
        }

        // 양손 모두 타격 감지
        if ((inst_R.array() != 0).any() && (inst_L.array() != 0).any())
        {
            continue; // 둘 다 타격이 감지되면 다음 루프로 넘어감
        }
    }

    //결론적으로 움직일 위치와 현재 오른손 시간 현재 왼손 시간 타격할 오른손 시간 타격할 왼손 시간 움직이기 시작한 왼손 시간 움직이긴 시작한 오른손 시간 정보를 다음 함수에 넘겨주는 구조가 될 예정
    seonwoo_inst_f << inst_R, inst_L;
    prev_col = Q.front();
    timeSum -= stod(prev_col[1]);

    make_time = current_time + stod(prev_col[1]);

    seonwoo_tR_i = moving_start_R;
    seonwoo_tL_i = moving_start_L;
    seonwoo_tR_f = detect_time_R;
    seonwoo_tL_f = detect_time_L;
    seonwoo_t1 = current_time;
    seonwoo_t2 = make_time;

    Q.pop();
    line_n++;
    // std::cout << "-----------------------------------------------------현재라인 : " << line_n << "----------------------------------------------------" << std::endl;
    // std::cout << "// 오른손 타격할 악기 --> \t" << inst_R.transpose() << "    움직임 시작 시간 --> " << moving_start_R << " \t현재시간 --> "  << current_time <<  " \t궤적시간 --> "  << make_time << " \t타격감지시간 --> " << detect_time_R << " \t //" << std::endl;
    // std::cout << "// 왼손 타격할 악기 --> \t" << inst_L.transpose() << "    움직임 시작 시간 --> " << moving_start_L << " \t현재시간 --> "  << current_time <<  " \t궤적시간 --> "  << make_time << " \t타격감지시간 --> " << detect_time_L << " \t //" <<  std::endl;
    // std::cout << "// 이전 악기 --> \t" << seonwoo_inst_i.transpose() << "    다음 악기 --> " << seonwoo_inst_f.transpose() << " \t\t //" << std::endl;
    // std::cout << "--------------------------------------------------------------------------------------------------------------------" << std::endl << std::endl;

}

VectorXd PathManager::ikfun_final(VectorXd &pR, VectorXd &pL)
{
    // float direction = 0.0 * M_PI;
    PartLength part_length;

    float X1 = pR(0), Y1 = pR(1), z1 = pR(2);
    float X2 = pL(0), Y2 = pL(1), z2 = pL(2);
    float r1 = part_length.upperArm;
    float r2 = part_length.lowerArm + part_length.stick;
    float L1 = part_length.upperArm;
    float L2 = part_length.lowerArm + part_length.stick;
    float s = part_length.waist;
    float z0 = part_length.height;

    int j = 0, m = 0;
    float the3[1351];
    float zeta = z0 - z2;
    VectorXd Qf(9);
    MatrixXd Q_arr(7,1);
    // float the0_f = 0;

    // the3 배열 초기화
    for (int i = 0; i < 1351; ++i)
        the3[i] = -M_PI / 4.0 + i * M_PI / 1350.0 * (3.0 / 4.0); // the3 범위 : -45deg ~ 90deg

    for (int i = 0; i < 1351; ++i)
    {
        float det_the4 = (z0 - z1 - r1 * cos(the3[i])) / r2;

        if (det_the4 < 1 && det_the4 > -1)
        {
            float the34 = acos((z0 - z1 - r1 * cos(the3[i])) / r2);
            float the4 = the34 - the3[i];

            if (the4 >= 0 && the4 < 120.0 * M_PI / 180.0) // the4 범위 : 0deg ~ 120deg
            {
                float r = r1 * sin(the3[i]) + r2 * sin(the34);
                float det_the1 = (X1 * X1 + Y1 * Y1 - r * r - s * s / 4.0) / (s * r);

                if (det_the1 < 1 && det_the1 > -1)
                {
                    float the1 = acos(det_the1);
                    if (the1 > 0 && the1 < 150.0 * M_PI / 180.0) // the1 범위 : 0deg ~ 150deg
                    {
                        float alpha = asin(X1 / sqrt(X1 * X1 + Y1 * Y1));
                        float det_the0 = (s / 4.0 + (X1 * X1 + Y1 * Y1 - r * r) / s) / sqrt(X1 * X1 + Y1 * Y1);

                        if (det_the0 < 1 && det_the0 > -1)
                        {
                            float the0 = asin(det_the0) - alpha;
                            if (the0 > -M_PI / 3.0 && the0 < M_PI / 3.0) // the0 범위 : -60deg ~ 60deg
                            {
                                float L = sqrt((X2 - 0.5 * s * cos(the0 + M_PI)) * (X2 - 0.5 * s * cos(the0 + M_PI)) + (Y2 - 0.5 * s * sin(the0 + M_PI)) * (Y2 - 0.5 * s * sin(the0 + M_PI)));
                                float det_the2 = (X2 - 0.5 * s * cos(the0 + M_PI)) / L;

                                if (det_the2 < 1 && det_the2 > -1)
                                {
                                    float the2 = acos(det_the2) - the0;
                                    if (the2 > 30 * M_PI / 180.0 && the2 < M_PI) // the2 범위 : 30deg ~ 180deg
                                    {
                                        float Lp = sqrt(L * L + zeta * zeta);
                                        float det_the6 = (Lp * Lp - L1 * L1 - L2 * L2) / (2 * L1 * L2);

                                        if (det_the6 < 1 && det_the6 > -1)
                                        {
                                            float the6 = acos(det_the6);
                                            if (the6 >= 0 && the6 < 120.0 * M_PI / 180.0) // the6 범위 : 0deg ~ 120deg
                                            {
                                                float T = (zeta * zeta + L * L + L1 * L1 - L2 * L2) / (L1 * 2);
                                                float det_the5 = L * L + zeta * zeta - T * T;

                                                if (det_the5 > 0)
                                                {
                                                    float sol = T * L - zeta * sqrt(L * L + zeta * zeta - T * T);
                                                    sol /= (L * L + zeta * zeta);
                                                    float the5 = asin(sol);
                                                    if (the5 > -M_PI / 4 && the5 < M_PI / 2) // the5 범위 : -45deg ~ 90deg
                                                    {
                                                        // if (j == 0 || fabs(the0 - direction) < fabs(the0_f - direction))
                                                        // {
                                                        //     Qf(0) = the0;
                                                        //     Qf(1) = the1;
                                                        //     Qf(2) = the2;
                                                        //     Qf(3) = the3[i];
                                                        //     Qf(4) = the4;
                                                        //     Qf(5) = the5;
                                                        //     Qf(6) = the6;
                                                        //     the0_f = the0;
                                                        //     j = 1;
                                                        // }
                                                        if (j == 0)
                                                        {
                                                            Q_arr(0,0) = the0;
                                                            Q_arr(1,0) = the1;
                                                            Q_arr(2,0) = the2;
                                                            Q_arr(3,0) = the3[i];
                                                            Q_arr(4,0) = the4;
                                                            Q_arr(5,0) = the5;
                                                            Q_arr(6,0) = the6;

                                                            j = 1;
                                                        }
                                                        else
                                                        {
                                                            Q_arr.conservativeResize(Q_arr.rows(), Q_arr.cols() + 1);

                                                            Q_arr(0,j) = the0;
                                                            Q_arr(1,j) = the1;
                                                            Q_arr(2,j) = the2;
                                                            Q_arr(3,j) = the3[i];
                                                            Q_arr(4,j) = the4;
                                                            Q_arr(5,j) = the5;
                                                            Q_arr(6,j) = the6;

                                                            j++;
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (j == 0)
    {
        cout << "IKFUN is not solved!!\n";
        state.main = Main::Error;
    }
    else
    {
        m = j/2;
        // std::cout << "j = " << j << ", m = " << m << std::endl;
        for (int i = 0; i < 7; i++)
        {
            Qf(i) = Q_arr(i,m);

            // std::cout << "Q(" << i << ") = " << Qf(i) << std::endl;
        }
    }

    Qf(7) = 0.0;
    Qf(8) = 0.0;

    return Qf;
}

void PathManager::pushConmmandBuffer(VectorXd &Qi)
{
    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            TMotorData newData;
            newData.position = Qi[motor_mapping[entry.first]];
            newData.spd = 0;
            newData.acl = 0;
            tMotor->commandBuffer.push(newData);
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            MaxonData newData;
            newData.position = Qi[motor_mapping[entry.first]];
            newData.WristState = 0; // 토크 제어 시 WristState 사용
            maxonMotor->commandBuffer.push(newData);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                           AddStance FUNCTION                               */
////////////////////////////////////////////////////////////////////////////////

VectorXd PathManager::cal_Vmax(VectorXd &q1, VectorXd &q2, float acc, float t2)
{
    VectorXd Vmax = VectorXd::Zero(9);

    for (int i = 0; i < 9; i++)
    {
        float val;
        float S = q2(i) - q1(i);

        // 이동거리 양수로 변경
        if (S < 0)
        {
            S = -1 * S;
        }

        if (S > t2*t2*acc/4)
        {
            // 가속도로 도달 불가능
            // -1 반환
            val = -1;
        }
        else
        {
            // 2차 방정식 계수
            float A = 1/acc;
            float B = -1*t2;
            float C = S;

            // 2차 방정식 해
            float sol1 = (-B+sqrt(B*B-4*A*C))/2/A;
            float sol2 = (-B-sqrt(B*B-4*A*C))/2/A;

            if (sol1 >= 0 && sol1 <= acc*t2/2)
            {
                val = sol1;
            }
            else if (sol2 >= 0 && sol2 <= acc*t2/2)
            {
                val = sol2;
            }
            else
            {
                // 해가 범위 안에 없음
                // -2 반환
                val = -2;
            }
        }

        Vmax(i) = val;
    }

    return Vmax;
}

VectorXd PathManager::makeProfile(VectorXd &q1, VectorXd &q2, VectorXd &Vmax, float acc, float t, float t2)
{
    VectorXd Qi = VectorXd::Zero(9);

    for(int i = 0; i < 9; i++)
    {
        float val, S;
        int sign;

        S = q2(i) - q1(i);
        
        // 부호 확인, 이동거리 양수로 변경
        if (S < 0)
        {
            S = -1 * S;
            sign = -1;
        }
        else
        {
            sign = 1;
        }


        // 궤적 생성
        if (S == 0)
        {
            // 정지
            val = q1(i);
        }
        else if (Vmax(i) < 0)
        {
            // Vmax 값을 구하지 못했을 때 삼각형 프로파일 생성
            float acc_tri = 4 * S / t2 / t2;

            if (t < t2/2)
            {
                val = q1(i) + sign * 0.5 * acc_tri * t * t;
            }
            else if (t < t2)
            {
                val = q2(i) - sign * 0.5 * acc_tri * (t2 - t) * (t2 - t);
            }
            else
            {
                val = q2(i);
            }
        }
        else
        {
            // 사다리꼴 프로파일
            if (t < Vmax(i) / acc)
            {
                // 가속
                val = q1(i) + sign * 0.5 * acc * t * t;
            }
            else if (t < S / Vmax(i))
            {
                // 등속
                val = q1(i) + (sign * 0.5 * Vmax(i) * Vmax(i) / acc) + (sign * Vmax(i) * (t - Vmax(i) / acc));          
            }
            else if (t < Vmax(i) / acc + S / Vmax(i))
            {
                // 감속
                val = q2(i) - sign * 0.5 * acc * (S / Vmax(i) + Vmax(i) / acc - t) * (S / Vmax(i) + Vmax(i) / acc - t);              
            }
            else 
            {
                val = q2(i);              
            }
        }

        Qi(i) = val;
    }

    return  Qi;
}

void PathManager::getMotorPos()
{
    // 각 모터의 현재위치 값 불러오기 ** CheckMotorPosition 이후에 해야함(변수값을 불러오기만 해서 갱신 필요)
    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            c_MotorAngle[motor_mapping[entry.first]] = tMotor->jointAngle;
        }
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            c_MotorAngle[motor_mapping[entry.first]] = maxonMotor->jointAngle;
        }
    }
}

vector<float> PathManager::makeHomeArr(int cnt)
{
    vector<float> home_arr = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    if (cnt == 1)
    {
        getMotorPos();

        for (int i = 0; i < 9; i++)
        {
            home_arr[i] = c_MotorAngle[i];
        }
        home_arr[1] = 135 * M_PI / 180.0;
        home_arr[2] = 45 * M_PI / 180.0;
        home_arr[7] = 90 * M_PI / 180.0;
        home_arr[8] = 90 * M_PI / 180.0;
    }
    // else if (cnt == 2)
    // {
    //     getMotorPos();

    //     for (int i = 0; i < 9; i++)
    //     {
    //         home_arr[i] = c_MotorAngle[i];
    //     }
    //     home_arr[1] = 135 * M_PI / 180.0;
    //     home_arr[2] = 45 * M_PI / 180.0;
    //     home_arr[7] = 90 * M_PI / 180.0;
    //     home_arr[8] = 90 * M_PI / 180.0;
    // }
    else if (cnt == 2)
    {
        for (int i = 0; i < 9; i++)
        {
            home_arr[i] = homeArr[i];
        }
    }
    else
    {
        std::cout << "Invalid Home Cnt";
    }

    return home_arr;
}

////////////////////////////////////////////////////////////////////////////////
/*                            SYSTEM FUNCTION                                 */
////////////////////////////////////////////////////////////////////////////////

string PathManager::trimWhitespace(const std::string &str)
{
    size_t first = str.find_first_not_of(" \t");
    if (std::string::npos == first)
    {
        return str;
    }
    size_t last = str.find_last_not_of(" \t");
    return str.substr(first, (last - first + 1));
}

vector<float> PathManager::fkfun()
{
    getMotorPos();

    PartLength part_length;
    vector<float> P;
    vector<float> theta(9);
    for (auto &motorPair : motors)
    {
        auto &name = motorPair.first;
        auto &motor = motorPair.second;
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            theta[motor_mapping[name]] = tMotor->jointAngle;
            cout << name << " : " << theta[motor_mapping[name]] << "\n";
        }
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            theta[motor_mapping[name]] = maxonMotor->jointAngle;
            cout << name << " : " << theta[motor_mapping[name]] << "\n";
        }
    }
    float r1 = part_length.upperArm, r2 = part_length.lowerArm, l1 = part_length.upperArm, l2 = part_length.lowerArm, stick = part_length.stick;
    float s = part_length.waist, z0 = part_length.height;

    P.push_back(0.5 * s * cos(theta[0]) + r1 * sin(theta[3]) * cos(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * cos(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * cos(theta[0] + theta[1]));
    P.push_back(0.5 * s * sin(theta[0]) + r1 * sin(theta[3]) * sin(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * sin(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * sin(theta[0] + theta[1]));
    P.push_back(z0 - r1 * cos(theta[3]) - r2 * cos(theta[3] + theta[4]) - stick * cos(theta[3] + theta[4] + theta[7]));
    P.push_back(-0.5 * s * cos(theta[0]) + l1 * sin(theta[5]) * cos(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * cos(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * cos(theta[0] + theta[2]));
    P.push_back(-0.5 * s * sin(theta[0]) + l1 * sin(theta[5]) * sin(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * sin(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * sin(theta[0] + theta[2]));
    P.push_back(z0 - l1 * cos(theta[5]) - l2 * cos(theta[5] + theta[6]) - stick * cos(theta[5] + theta[6] + theta[8]));

    return P;
}
