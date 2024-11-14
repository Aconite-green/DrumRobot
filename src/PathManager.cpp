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

/////////////////////////////////////////////////////////////////////////////////
/*                            SEND BUFFER TO MOTOR                            */
///////////////////////////////////////////////////////////////////////////////

void PathManager::Motors_sendBuffer(VectorXd &Qi, bool brake_state)
{
    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            TMotorData newData;
            newData.position = Qi[motor_mapping[entry.first]];
            newData.spd = 0;
            newData.acl = 0;
            newData.isBrake = brake_state;
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

/////////////////////////////////////////////////////////////////////////////////
/*                               SYSTEM FUNCTION                              */
///////////////////////////////////////////////////////////////////////////////

float PathManager::timeScaling(float ti, float tf, float t, float tm, float sm)
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

float PathManager::timeScaling_only3(float ti, float tf, float t)
{
    float s;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd x;

    A.resize(4,4);
    b.resize(4,1);

    A << 1, ti, ti*ti, ti*ti*ti,
    1, tf, tf*tf, tf*tf*tf,
    0, 1, 2*ti, 3*ti*ti,
    0, 1, 2*tf, 3*tf*tf;

    b << 0, 1, 0, 0;

    A_1 = A.inverse();
    x = A_1 * b;

    s = x(0,0) + x(1,0) * t + x(2,0) * t * t + x(3,0) * t * t * t;

    return s;
}

VectorXd PathManager::makePath(VectorXd Pi, VectorXd Pf, float s[], float sm, float h)
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


// 손목 토크 제어 시 필요
pair<float, float> PathManager::SetTorqFlag(MatrixXd &State, float t_now)
{
    float q7_isTorq = 10.0;
    float q8_isTorq = 10.0;

    VectorXd time_madi = State.row(0);
    VectorXd q7_state = State.row(1);
    VectorXd q8_state = State.row(2);

    if (time_madi(0) == 0) // 연주 시작 시
    {
        if (t_now < time_madi(1))
        {
            if (abs(t_now - time_madi(0)) < 0.001)
            {
                q7_isTorq = -0.5;
                q8_isTorq = -0.5;
            }
            else
            {
                q7_isTorq = 0;
                q8_isTorq = 0;
            }
        }
        else if (t_now < time_madi(2))
        {
            if (q7_state(1) == -1)
            {
                if (abs(t_now - time_madi(1)) < 0.001) // 타격 전 대기
                    q7_isTorq = 2;
                else if (abs(t_now - (time_madi(2) - wrist_hit_time)) < 0.001) // 타격 시작
                    q7_isTorq = -1;
                else
                    q7_isTorq = 0;
            }
            else
            {
                if (abs(t_now - time_madi(1)) < 0.001)
                    q7_isTorq = q7_state(1);
                else
                    q7_isTorq = 0;
            }

            if (q8_state(1) == -1)
            {
                if (abs(t_now - time_madi(1)) < 0.001) // 타격 전 대기
                    q8_isTorq = 2;
                else if (abs(t_now - (time_madi(2) - wrist_hit_time)) < 0.001) // 타격 시작
                    q8_isTorq = -1;
                else
                    q8_isTorq = 0;
            }
            else
            {
                if (abs(t_now - time_madi(1)) < 0.001)
                    q8_isTorq = q8_state(1);
                else
                    q8_isTorq = 0;
            }
        }
    }
    else
    {
        if (t_now < time_madi(1))
        {
            if (q7_state(0) == -1)
            {
                if (abs(t_now - time_madi(0)) < 0.001) // 타격 전 대기
                    q7_isTorq = 2;
                else if (abs(t_now - (time_madi(1) - wrist_hit_time)) < 0.001) // 타격 시작
                    q7_isTorq = -1;
                else
                    q7_isTorq = 0;
            }
            else
            {
                if (abs(t_now - time_madi(0)) < 0.001)
                    q7_isTorq = q7_state(0);
                else
                    q7_isTorq = 0;
            }

            if (q8_state(0) == -1)
            {
                if (abs(t_now - time_madi(0)) < 0.001) // 타격 전 대기
                    q8_isTorq = 2;
                else if (abs(t_now - (time_madi(1) - wrist_hit_time)) < 0.001) // 타격 시작
                    q8_isTorq = -1;
                else
                    q8_isTorq = 0;
            }
            else
            {
                if (abs(t_now - time_madi(0)) < 0.001)
                    q8_isTorq = q8_state(0);
                else
                    q8_isTorq = 0;
            }
        }
        else if (t_now < time_madi(2))
        {
            if (q7_state(1) == -1)
            {
                if (abs(t_now - time_madi(1)) < 0.001) // 타격 전 대기
                    q7_isTorq = 2;
                else if (abs(t_now - (time_madi(2) - wrist_hit_time)) < 0.001) // 타격 시작
                    q7_isTorq = -1;
                else
                    q7_isTorq = 0;
            }
            else
            {
                if (abs(t_now - time_madi(1)) < 0.001)
                    q7_isTorq = q7_state(1);
                else
                    q7_isTorq = 0;
            }

            if (q8_state(1) == -1)
            {
                if (abs(t_now - time_madi(1)) < 0.001) // 타격 전 대기
                    q8_isTorq = 2;
                else if (abs(t_now - (time_madi(2) - wrist_hit_time)) < 0.001) // 타격 시작
                    q8_isTorq = -1;
                else
                    q8_isTorq = 0;
            }
            else
            {
                if (abs(t_now - time_madi(1)) < 0.001)
                    q8_isTorq = q8_state(1);
                else
                    q8_isTorq = 0;
            }
        }
    }

    return std::make_pair(q7_isTorq, q8_isTorq);
}


vector<float> PathManager::connect(vector<float> &Q1, vector<float> &Q2, int k, int n)
{
    vector<float> Qi;
    std::vector<float> A, B;

    // Compute A and Bk
    for (long unsigned int i = 0; i < Q1.size(); ++i)
    {
        A.push_back(0.5 * (Q1[i] - Q2[i]));
        B.push_back(0.5 * (Q1[i] + Q2[i]));
    }

    // Compute Qi using the provided formula
    for (long unsigned int i = 0; i < Q1.size(); ++i)
    {
        float val = A[i] * cos(M_PI * k / n) + B[i];
        Qi.push_back(val);
    }

    return Qi;
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

string trimWhitespace(const std::string &str)
{
    size_t first = str.find_first_not_of(" \t");
    if (std::string::npos == first)
    {
        return str;
    }
    size_t last = str.find_last_not_of(" \t");
    return str.substr(first, (last - first + 1));
}

MatrixXd PathManager::tms_fun(float t2_a, float t2_b, VectorXd &inst2_a, VectorXd &inst2_b)
{
    int flag = 0;

    VectorXd inst_c = VectorXd::Zero(18);

    float t3;
    MatrixXd t3_inst3;

    // 1번 룰: 1이 연속되면 t3와 inst3를 생성하고, t2 0.2초 앞에 inst2를 타격할 준비(-1)를 함
    if ((round(inst2_a.segment(0, 9).norm()) == 1) && (round(inst2_b.segment(0, 9).norm()) == 1))
    {
        // 오른손
        inst_c.segment(0, 9) = -inst2_b.segment(0, 9);
        t3 = 0.5 * (t2_b + t2_a);
        t3_inst3.resize(19, 3);
        t3_inst3.row(0) << t2_a, t3, t2_b;
        t3_inst3.block(1, 0, 18, 1) = inst2_a;
        t3_inst3.block(1, 1, 18, 1) = inst_c;
        t3_inst3.block(1, 2, 18, 1) = inst2_b;
        flag = 1;
    }

    if ((round(inst2_a.segment(9, 9).norm()) == 1) && (round(inst2_b.segment(9, 9).norm()) == 1))
    {
        // 왼손
        inst_c.segment(9, 9) = -inst2_b.segment(9, 9);
        t3 = 0.5 * (t2_a + t2_b);
        t3_inst3.resize(19, 3);
        t3_inst3.row(0) << t2_a, t3, t2_b;
        t3_inst3.block(1, 0, 18, 1) = inst2_a;
        t3_inst3.block(1, 1, 18, 1) = inst_c;
        t3_inst3.block(1, 2, 18, 1) = inst2_b;
        flag = 1;
    }

    // 2번 룰: 1번 룰에 의해 새로운 시점/악기가 정의되어 있고(flag = 1) inst2에 1이 있으면 inst3에 -1을 넣는다.
    if ((round(inst2_a.segment(0, 9).norm()) == 0) && (round(inst2_b.segment(0, 9).norm()) == 1) && (flag == 1))
    {
        // 오른손
        inst_c.segment(0, 9) = -inst2_b.segment(0, 9);
        t3 = 0.5 * (t2_a + t2_b);
        t3_inst3.resize(19, 3);
        t3_inst3.row(0) << t2_a, t3, t2_b;
        t3_inst3.block(1, 0, 18, 1) = inst2_a;
        t3_inst3.block(1, 1, 18, 1) = inst_c;
        t3_inst3.block(1, 2, 18, 1) = inst2_b;
    }

    if ((round(inst2_a.segment(9, 9).norm()) == 0) && (round(inst2_b.segment(9, 9).norm()) == 1) && (flag == 1))
    {
        // 왼손
        inst_c.segment(9, 9) = -inst2_b.segment(9, 9);
        t3 = 0.5 * (t2_a + t2_b);
        t3_inst3.resize(19, 3);
        t3_inst3.row(0) << t2_a, t3, t2_b;
        t3_inst3.block(1, 0, 18, 1) = inst2_a;
        t3_inst3.block(1, 1, 18, 1) = inst_c;
        t3_inst3.block(1, 2, 18, 1) = inst2_b;
    }

    // 3번 룰: 1번 룰을 거치지 않았고 inst1이 0 이고 inst2에 1이 있으면, inst1에 -1을 넣는다.
    if ((round(inst2_a.segment(0, 9).norm()) == 0) && (round(inst2_b.segment(0, 9).norm()) == 1) && (flag == 0))
    {
        inst2_a.segment(0, 9) = -inst2_b.segment(0, 9);
        t3_inst3.resize(19, 2);
        t3_inst3.row(0) << t2_a, t2_b;
        t3_inst3.block(1, 0, 18, 1) = inst2_a;
        t3_inst3.block(1, 1, 18, 1) = inst2_b;
    }

    if ((round(inst2_a.segment(9, 9).norm()) == 0) && (round(inst2_b.segment(9, 9).norm()) == 1) && (flag == 0))
    {
        inst2_a.segment(9, 9) = -inst2_b.segment(9, 9);
        t3_inst3.resize(19, 2);
        t3_inst3.row(0) << t2_a, t2_b;
        t3_inst3.block(1, 0, 18, 1) = inst2_a;
        t3_inst3.block(1, 1, 18, 1) = inst2_b;
    }

    // 악보 끝까지 연주하기 위해서 inst2_b에 0 행렬이 인위적으로 들어가는 경우, 그대로 내보냄
    if (round((inst2_b.norm()) == 0) && (flag == 0))
    {
        t3_inst3.resize(19, 2);
        t3_inst3.row(0) << t2_a, t2_b;
        t3_inst3.block(1, 0, 18, 1) = inst2_a;
        t3_inst3.block(1, 1, 18, 1) = inst2_b;
    }

    return t3_inst3;
}

void PathManager::itms0_fun(vector<float> &t2, MatrixXd &inst2, MatrixXd &A30, MatrixXd &A31, MatrixXd &AA40, MatrixXd &AA41)
{
    MatrixXd T(0, 0);

    for (int k = 0; k < 4; ++k)
    {
        VectorXd inst_0 = inst2.col(k);
        VectorXd inst_1 = inst2.col(k + 1);
        MatrixXd inst3 = tms_fun(t2[k], t2[k + 1], inst_0, inst_1);

        if (T.cols() == 0)
        {
            T.resize(inst3.rows(), inst3.cols());
            T = inst3;
        }
        else
        {
            MatrixXd temp = T.leftCols(T.cols() - 1);
            T.resize(T.rows(), T.cols() - 1 + inst3.cols());
            T << temp, inst3;
        }
    }

    /* 빈 자리에 -0.5 집어넣기:  */
    int nn = T.cols();

    for (int k = 1; k < nn; ++k)
    {
        if (round(T.block(1, k, 9, 1).sum()) == 0)
        {
            float norm_val = T.block(1, k - 1, 9, 1).norm();
            MatrixXd block = T.block(1, k - 1, 9, 1);
            T.block(1, k, 9, 1) = -0.5 * block.cwiseAbs() / norm_val;
        }

        if (round(T.block(10, k, 9, 1).sum()) == 0)
        {
            float norm_val = T.block(10, k - 1, 9, 1).norm();
            MatrixXd block = T.block(10, k - 1, 9, 1);
            T.block(10, k, 9, 1) = -0.5 * block.cwiseAbs() / norm_val;
        }

        // 다음 악기 저장
        if (T(0, k) == t2[1])
        {
            inst_now.resize(19, 1);
            inst_now(0) = 0.0;
            inst_now = T.col(k);
        }
    }

    /* 일단 0=t2(1)에서부터 t2(4)까지 정의함 */
    int j = 0;
    for (int k = 0; k < nn; ++k)
    {
        if (T(0, k) <= t2[3])
        {
            j++;
        }
    }

    MatrixXd t4_inst4 = MatrixXd::Zero(T.rows(), j);
    j = 0; // j 초기화
    for (int k = 0; k < nn; ++k)
    {
        if (T(0, k) <= t2[3])
        {
            t4_inst4.col(j) = T.col(k);
            j++;
        }
    }

    /* hit31: t2(2)에서 t2(4)까지 중에서 타격을 포함하는 t4_inst4를 골라냄  */
    int kk = 0;
    for (int k = 0; k < j; ++k)
    {
        for (int i = 1; i <= 3; ++i)
        {
            if (t4_inst4(0, k) == t2[i])
            {
                if (kk == 0)
                {
                    A31.resize(t4_inst4.rows(), 1);
                    A31 = t4_inst4.col(k);
                    kk = 1;
                }
                else
                {
                    A31.conservativeResize(A31.rows(), A31.cols() + 1);
                    A31.col(A31.cols() - 1) = t4_inst4.col(k);
                }
            }
        }
    }

    /* state31: t2(2)에서 시작해서 3개의 연속된 t4_inst4를 골라냄 */
    for (int k = 0; k < j; ++k)
    {
        if (t4_inst4(0, k) == t2[1])
        {
            AA41.resize(3, 3);
            AA41 << t4_inst4(0, k), t4_inst4(0, k + 1), t4_inst4(0, k + 2),// t4_inst4(0, k + 3),
                t4_inst4.block(1, k, 9, 1).sum(), t4_inst4.block(1, k + 1, 9, 1).sum(), t4_inst4.block(1, k + 2, 9, 1).sum(),// t4_inst4.block(1, k + 3, 9, 1).sum(),
                t4_inst4.block(10, k, 9, 1).sum(), t4_inst4.block(10, k + 1, 9, 1).sum(), t4_inst4.block(10, k + 2, 9, 1).sum();//, t4_inst4.block(10, k + 3, 9, 1).sum();
            break;
        }
    }

    kk = 0;
    for (int k = 0; k < j; ++k)
    {
        for (int i = 0; i < 3; ++i)
        {
            if (t4_inst4(0, k) == t2[i])
            {
                if (kk == 0)
                {
                    A30.resize(t4_inst4.rows(), 1);
                    A30 = t4_inst4.col(k);
                    kk = 1;
                }
                else
                {
                    A30.conservativeResize(A30.rows(), A30.cols() + 1);
                    A30.col(A30.cols() - 1) = t4_inst4.col(k);
                }
            }
        }
    }

    AA40.resize(3, 4);
    AA40 << t4_inst4(0, 0), t4_inst4(0, 1), t4_inst4(0, 2), t4_inst4(0, 3),
        t4_inst4.block(1, 0, 9, 1).sum(), t4_inst4.block(1, 1, 9, 1).sum(), t4_inst4.block(1, 2, 9, 1).sum(), t4_inst4.block(1, 3, 9, 1).sum(),
        t4_inst4.block(10, 0, 9, 1).sum(), t4_inst4.block(10, 1, 9, 1).sum(), t4_inst4.block(10, 2, 9, 1).sum(), t4_inst4.block(10, 3, 9, 1).sum();
}

void PathManager::itms_fun(vector<float> &t2, MatrixXd &inst2, MatrixXd &B, MatrixXd &BB, VectorXd &pre_inst)
{
    MatrixXd T(0, 0);

    for (int k = 0; k < 4; ++k)
    {
        VectorXd inst_0 = inst2.col(k);
        VectorXd inst_1 = inst2.col(k + 1);
        MatrixXd inst3 = tms_fun(t2[k], t2[k + 1], inst_0, inst_1);

        if (T.cols() == 0)
        {
            T.resize(inst3.rows(), inst3.cols());
            T = inst3;
        }
        else
        {
            MatrixXd temp = T.leftCols(T.cols() - 1);
            T.resize(T.rows(), T.cols() - 1 + inst3.cols());
            T << temp, inst3;
        }
    }

    /* 빈 자리에 -0.5 집어넣기:  */
    int nn = T.cols();

    if (round(T.block(1, 0, 9, 1).sum()) == 0)
    {
        float norm_val = pre_inst.block(1, 0, 9, 1).norm();
        MatrixXd block = pre_inst.block(1, 0, 9, 1);
        T.block(1, 0, 9, 1) = -0.5 * block.cwiseAbs() / norm_val;
    }

    if (round(T.block(10, 0, 9, 1).sum()) == 0)
    {
        float norm_val = pre_inst.block(10, 0, 9, 1).norm();
        MatrixXd block = pre_inst.block(10, 0, 9, 1);
        T.block(10, 0, 9, 1) = -0.5 * block.cwiseAbs() / norm_val;
    }

    for (int k = 1; k < nn; ++k)
    {
        if (round(T.block(1, k, 9, 1).sum()) == 0)
        {
            float norm_val = T.block(1, k - 1, 9, 1).norm();
            MatrixXd block = T.block(1, k - 1, 9, 1);
            T.block(1, k, 9, 1) = -0.5 * block.cwiseAbs() / norm_val;
        }

        if (round(T.block(10, k, 9, 1).sum()) == 0)
        {
            float norm_val = T.block(10, k - 1, 9, 1).norm();
            MatrixXd block = T.block(10, k - 1, 9, 1);
            T.block(10, k, 9, 1) = -0.5 * block.cwiseAbs() / norm_val;
        }

        // 다음 악기 저장
        if (T(0, k) == t2[1])
        {
            inst_now = T.col(k);
        }
    }

    /* 일단 0=t2(1)에서부터 t2(4)까지 정의함 */
    int j = 0;
    for (int k = 0; k < nn; ++k)
    {
        if (T(0, k) >= t2[1] && T(0, k) <= t2[4])
        {
            j++;
        }
    }

    MatrixXd t4_inst4(T.rows(), j);
    j = 0; // j 초기화
    for (int k = 0; k < nn; ++k)
    {
        if (T(0, k) >= t2[1] && T(0, k) <= t2[4])
        {
            t4_inst4.col(j) = T.col(k);
            j++;
        }
    }

    /* B: t2(2)에서 t2(4)까지 중에서 타격을 포함하는 t4_inst4를 골라냄  */
    int kk = 0;
    for (int k = 0; k < j; ++k)
    {
        for (int i = 1; i <= 3; ++i)
        {
            if (t4_inst4(0, k) == t2[i])
            {
                if (kk == 0)
                {
                    B.resize(t4_inst4.rows(), 1);
                    B = t4_inst4.col(k);
                    kk = 1;
                }
                else
                {
                    B.conservativeResize(B.rows(), B.cols() + 1);
                    B.col(B.cols() - 1) = t4_inst4.col(k);
                }
            }
        }
    }

    /* state31: t2(2)에서 시작해서 3개의 연속된 t4_inst4를 골라냄 */
    for (int k = 0; k < j; ++k)
    {
        if (t4_inst4(0, k) == t2[1])
        {
            BB.resize(3, 4);
            BB << t4_inst4(0, k), t4_inst4(0, k + 1), t4_inst4(0, k + 2), t4_inst4(0, k + 3),
                t4_inst4.block(1, k, 9, 1).sum(), t4_inst4.block(1, k + 1, 9, 1).sum(), t4_inst4.block(1, k + 2, 9, 1).sum(), t4_inst4.block(1, k + 3, 9, 1).sum(),
                t4_inst4.block(10, k, 9, 1).sum(), t4_inst4.block(10, k + 1, 9, 1).sum(), t4_inst4.block(10, k + 2, 9, 1).sum(), t4_inst4.block(10, k + 3, 9, 1).sum();
            break;
        }
    }
}

VectorXd PathManager::pos_madi_fun(VectorXd &A)
{
    float time = A(0);

    VectorXd inst_right = A.segment(1, 9);
    VectorXd inst_left = A.segment(10, 9);

    VectorXd inst_right_01 = inst_right / inst_right.sum();
    VectorXd inst_left_01 = inst_left / inst_left.sum();

    float inst_right_state = inst_right.sum();
    float inst_left_state = inst_left.sum();

    VectorXd inst_p(18);
    inst_p << inst_right_01,
        inst_left_01;

    MatrixXd combined(6, 18);
    combined << right_drum_position, MatrixXd::Zero(3, 9), MatrixXd::Zero(3, 9), left_drum_position;
    MatrixXd p = combined * inst_p;

    VectorXd output(9);
    output << time,
        p,
        inst_right_state,
        inst_left_state;

    return output;
}

MatrixXd PathManager::sts2wrist_fun(MatrixXd &AA)
{
    MatrixXd t_madi = AA.row(0);
    MatrixXd sts_R = AA.row(1);
    MatrixXd sts_L = AA.row(2);

    MatrixXd theta_R = MatrixXd::Zero(1, 3);
    MatrixXd theta_L = MatrixXd::Zero(1, 3);

    for (int i = 0; i < 3; ++i)
    {
        if (sts_R(0, i) == 1)
        {
            theta_R(0, i) = 0;
        }
        else if (sts_R(0, i) == -0.5)
        {
            theta_R(0, i) = wrist_stanby;
        }
        else if (sts_R(0,i) == -1)
        {
            theta_R(0, i) = wrist_ready;
        }

        if (sts_L(0, i) == 1)
        {
            theta_L(0, i) = 0;
        }
        else if (sts_L(0, i) == -0.5)
        {
            theta_L(0, i) = wrist_stanby;
        }
        else if (sts_L(0,i) == -1)
        {
            theta_L(0, i) = wrist_ready;
        }
    }

    MatrixXd t_wrist_madi(3, 3);
    t_wrist_madi << t_madi.block(0, 0, 1, 3),
        theta_R.block(0, 0, 1, 3),
        theta_L.block(0, 0, 1, 3);

    return t_wrist_madi;
}

MatrixXd PathManager::sts2elbow_fun(MatrixXd &AA)
{
    MatrixXd t_madi = AA.row(0);
    MatrixXd sts_R = AA.row(1);
    MatrixXd sts_L = AA.row(2);

    MatrixXd theta_R = MatrixXd::Zero(1, 3);
    MatrixXd theta_L = MatrixXd::Zero(1, 3);

    for (int i = 0; i < 3; ++i)
    {
        if (sts_R(0, i) == 1)
        {
            theta_R(0, i) = 0;
        }
        else if (sts_R(0, i) == -0.5)
        {
            theta_R(0, i) = elbow_stanby;
        }
        else if (sts_R(0, i) == -1)
        {
            theta_R(0, i) = elbow_ready;
        }

        if (sts_L(0, i) == 1)
        {
            theta_L(0, i) = 0;
        }
        else if (sts_L(0, i) == -0.5)
        {
            theta_L(0, i) = elbow_stanby;
        }
        else if (sts_L(0, i) == -1)
        {
            theta_L(0, i) = elbow_ready;
        }
    }

    MatrixXd t_elbow_madi(3, 3);
    t_elbow_madi << t_madi.block(0, 0, 1, 3),
        theta_R.block(0, 0, 1, 3),
        theta_L.block(0, 0, 1, 3);

    return t_elbow_madi;
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

    P.push_back(0.5 * s * cos(theta[0]) + r1 * sin(theta[3]) * cos(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * cos(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * cos(theta[0] + theta[1]));
    P.push_back(0.5 * s * sin(theta[0]) + r1 * sin(theta[3]) * sin(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * sin(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * sin(theta[0] + theta[1]));
    P.push_back(z0 - r1 * cos(theta[3]) - r2 * cos(theta[3] + theta[4]) - stick * cos(theta[3] + theta[4] + theta[7]));
    P.push_back(-0.5 * s * cos(theta[0]) + l1 * sin(theta[5]) * cos(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * cos(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * cos(theta[0] + theta[2]));
    P.push_back(-0.5 * s * sin(theta[0]) + l1 * sin(theta[5]) * sin(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * sin(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * sin(theta[0] + theta[2]));
    P.push_back(z0 - l1 * cos(theta[5]) - l2 * cos(theta[5] + theta[6]) - stick * cos(theta[5] + theta[6] + theta[8]));

    return P;
}

float PathManager::con_fun(float th_a, float th_b, int k, int n)
{
    float A, B;
    A = 0.5 * (th_a - th_b);
    B = 0.5 * (th_a + th_b);

    return (A * cos(M_PI * k / n) + B);
}

float PathManager::con_fun_pos(float th_a, float th_b, float k, float n)
{
    float A, B;
    A = 0.5 * (th_a - th_b);
    B = 0.5 * (th_a + th_b);

    return (A * cos(M_PI * k / n) + B);
}

pair<float, float> PathManager::iconf_fun(float qk1_06, float qk2_06, float qk3_06, float qv_in, float t1, float t2, float t)
{
    float p_out, v_out /*, V1_out*/;

    if ((qk2_06 - qk1_06) / (qk3_06 - qk2_06) > 0)
    { // 방향 지속의 경우, 2차 함수
        float c = qk1_06;
        float b = qv_in;
        float a = (qk2_06 - qk1_06 - qv_in * t1) / (t1 * t1);

        p_out = a * t * t + b * t + c; // 위치
        v_out = 2 * a * t + b;         // 속도
        // V1_out = 2 * a * t1 + b; // t1 시점에서의 속도
    }
    else
    { // 방향 전환의 경우, 3차 함수
        float c = qv_in;
        float d = qk1_06;

        float T11 = t1 * t1 * t1;
        float T12 = t1 * t1;
        float T21 = 3 * t1 * t1;
        float T22 = 2 * t1;

        // 역행렬 계산을 위한 수식 처리
        float det = T11 * T22 - T12 * T21;
        float invT11 = T22 / det;
        float invT12 = -T12 / det;
        float invT21 = -T21 / det;
        float invT22 = T11 / det;

        float ANS1 = -c * t1 - d + qk2_06;
        float ANS2 = -c;

        // 행렬식을 이용한 계산
        float a = invT11 * ANS1 + invT12 * ANS2;
        float b = invT21 * ANS1 + invT22 * ANS2;

        p_out = a * t * t * t + b * t * t + c * t + d; // 위치
        v_out = 3 * a * t * t + 2 * b * t + c;         // 속도
        // V1_out = 3 * a * t1 * t1 + 2 * b * t1 + c; // t1 시점에서의 속도
    }

    return std::make_pair(p_out, v_out);
}

pair<float, float> PathManager::qRL_fun(MatrixXd &t_madi, float t_now)
{
    float qR_t, qL_t;

    VectorXd time_madi = t_madi.row(0);
    VectorXd q7_madi = t_madi.row(1);
    VectorXd q8_madi = t_madi.row(2);

    if (t_now >= time_madi(0) && t_now < time_madi(1))
    {
        qR_t = q7_madi(1);
        qL_t = q8_madi(1);
    }
    else if (t_now >= time_madi(1) && t_now < time_madi(2))
    {
        qR_t = q7_madi(2);
        qL_t = q8_madi(2);
    }
    else
    {
        qR_t = q7_madi(2);
        qL_t = q8_madi(2);
    }

    return std::make_pair(qR_t, qL_t);
}

pair<float, float> PathManager::q78_fun(MatrixXd &t_madi, float t_now)
{
    float qR_t, qL_t;
    VectorXd time_madi = t_madi.row(0);
    VectorXd q7_madi = t_madi.row(1);
    VectorXd q8_madi = t_madi.row(2);
    float t1 = time_madi(1) - time_madi(0);
    float t2 = time_madi(2) - time_madi(1);
    float t_i;

    if (t_now >= time_madi(0) && t_now < time_madi(1))
    {
        t_i = t_now - time_madi(0);
        qR_t = con_fun_pos(q7_madi(0), q7_madi(1), t_i, t1);
        qL_t = con_fun_pos(q8_madi(0), q8_madi(1), t_i, t1);
    }
    else if (t_now >= time_madi(1) && t_now < time_madi(2))
    {
        t_i = t_now - time_madi(1);
        qR_t = con_fun_pos(q7_madi(1), q7_madi(2), t_i, t2);
        qL_t = con_fun_pos(q8_madi(1), q8_madi(2), t_i, t2);
    }
    else
    {
        qR_t = q7_madi(2);
        qL_t = q8_madi(2);
    }

    return std::make_pair(qR_t, qL_t);
}

/////////////////////////////////////////////////////////////////////////////////
/*                         POSITION LOOP MODE FUNCTION                        */
///////////////////////////////////////////////////////////////////////////////

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

void PathManager::solveIK(VectorXd &pR1, VectorXd &pL1)
{
    VectorXd q(9);

    VectorXd q_06 = ikfun_final(pR1, pL1);

    for (int i = 0; i < 7; i++)
    {
        q(i) = q_06(i);
    }

    q(7) = 0;
    q(8) = 0;

    Motors_sendBuffer(q, false);

    // 데이터 기록
    for (int m = 0; m < 9; m++)
    {
        std::string fileName = "solveIK_q" + to_string(m);
        fun.appendToCSV_DATA(fileName, m, q(m), 0);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                                  MAKE PATH                                 */
///////////////////////////////////////////////////////////////////////////////

void PathManager::GetDrumPositoin()
{
    part_length.resize(6);
    part_length << 0.250, 0.328, 0.250, 0.328, 0.325+0.048, 0.325+0.048; ///< [오른팔 상완, 오른팔 하완, 왼팔 상완, 왼팔 하완, 스틱, 스틱]의 길이.

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

void PathManager::GetMusicSheet()
{
    /////////// 드럼로봇 악기정보 텍스트 -> 딕셔너리 변환
    map<string, int> instrument_mapping = {
        {"1", 2}, {"2", 5}, {"3", 6}, {"4", 8}, {"5", 3}, {"6", 1}, {"7", 0}, {"8", 7}, {"11", 2}, {"51", 2}, {"61", 2}, {"71", 2}, {"81", 2}, {"91", 2}};
        // S        FT          MT      HT        HH        R           RC      LC          S           S       S           S           S           S
    default_right.resize(9);
    default_left.resize(9);
    default_right << 0, 0, 1, 0, 0, 0, 0, 0, 0;
    default_left << 0, 0, 1, 0, 0, 0, 0, 0, 0;

    ifstream file(score_path);
    if (!file.is_open())
        cerr << "Error opening file." << endl;

    string row;
    int lineIndex = 0;
    float time = 0.0;
    time_arr.push_back(time);
    inst_arr.resize(18, 1);
    inst_arr.block(0, 0, 9, 1) = default_right;
    inst_arr.block(9, 0, 9, 1) = default_left;
    while (getline(file, row))
    {
        istringstream iss(row);
        string item;

        vector<string> columns;
        while (getline(iss, item, '\t'))
        {
            item = trimWhitespace(item);
            columns.push_back(item);
        }

        if (lineIndex == 0)
        { // 첫번째 행엔 bpm에 대한 정보
            bpm = stod(columns[0].substr(4));
            // cout << "bpm = " << bpm << "\n";
        }
        else
        {
            VectorXd inst_arr_R = VectorXd::Zero(9), inst_arr_L = VectorXd::Zero(9);
            VectorXd inst_col = VectorXd::Zero(18);

            if (columns[2] != "0")
                inst_arr_R(instrument_mapping[columns[2]]) = 1.0;
            if (columns[3] != "0")
                inst_arr_L(instrument_mapping[columns[3]]) = 1.0;

            time += stod(columns[1]) * 100.0 / bpm;
            time_arr.push_back(time);
            inst_col << inst_arr_R, inst_arr_L;
            inst_arr.conservativeResize(inst_arr.rows(), inst_arr.cols() + 1);
            inst_arr.col(inst_arr.cols() - 1) = inst_col;
        }

        lineIndex++;
    }

    file.close();

    time_arr.push_back(time + 1);
    time_arr.push_back(time + 2);
    time_arr.push_back(time + 3);

    VectorXd inst_col = VectorXd::Zero(18);
    inst_arr.conservativeResize(inst_arr.rows(), inst_arr.cols() + 3);
    inst_arr.col(inst_arr.cols() - 1) = inst_col;
    inst_arr.col(inst_arr.cols() - 2) = inst_col;
    inst_arr.col(inst_arr.cols() - 3) = inst_col;

    total = time_arr.size() - 3;
}

void PathManager::SetReadyAng()
{
    VectorXd inst_p(18);
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
    readyArr[7] = 0.0;
    readyArr[8] = 0.0;
}

void PathManager::PathLoopTask()
{
    // float t_now = time_arr[line];

    MatrixXd A30;  // 크기가 19x3인 2차원 벡터
    MatrixXd A31;  // 크기가 19x3인 2차원 벡터
    MatrixXd AA40; // 크기가 3x4인 2차원 벡터
    MatrixXd AA41; // 크기가 3x4인 2차원 벡터 // 3x3
    MatrixXd B;    // 크기가 19x3인 2차원 벡터
    MatrixXd BB;   // 크기가 3x4인 2차원 벡터
    MatrixXd State(3, 4);

    VectorXd p1(9), p2(9), p3(9);
    MatrixXd wrist_addAngle(3, 3), elbow_addAngle(3, 3);

    // 연주 처음 시작할 때 Q1, Q2 계산
    if (line == 0)
    {
        std::vector<float> t2(time_arr.begin(), time_arr.begin() + 5);
        MatrixXd inst2 = inst_arr.middleCols(0, 5);
        itms0_fun(t2, inst2, A30, A31, AA40, AA41);

        VectorXd A1 = A30.col(0);
        VectorXd A2 = A30.col(1);
        VectorXd A3 = A30.col(2);
        p1 = pos_madi_fun(A1);
        p2 = pos_madi_fun(A2);
        p3 = pos_madi_fun(A3);

        State = AA40;
    }
    else if (line == 1)
    {
        std::vector<float> t2(time_arr.begin(), time_arr.begin() + 5);
        MatrixXd inst2 = inst_arr.middleCols(0, 5);
        itms0_fun(t2, inst2, A30, A31, AA40, AA41);

        VectorXd A1 = A31.col(0);
        VectorXd A2 = A31.col(1);
        VectorXd A3 = A31.col(2);
        p1 = pos_madi_fun(A1);
        p2 = pos_madi_fun(A2);
        p3 = pos_madi_fun(A3);

        State = AA41;
    }
    else if (line > 1)
    {
        std::vector<float> t2(time_arr.begin() + line - 1, time_arr.begin() + line + 4);
        MatrixXd inst2 = inst_arr.middleCols(line - 1, 5);
        itms_fun(t2, inst2, B, BB, inst_now);

        VectorXd B1 = B.col(0);
        VectorXd B2 = B.col(1);
        VectorXd B3 = B.col(2);
        p1 = pos_madi_fun(B1);
        p2 = pos_madi_fun(B2);
        p3 = pos_madi_fun(B3);

        State = BB;
    }

    float dt = canManager.deltaT;   // 0.005
    float t = p2(0) - p1(0);
    int n = t / dt;
    VectorXd qt = VectorXd::Zero(9);
    VectorXd Vmax = VectorXd::Zero(9);
    const float acc_max = 100.0;    // rad/s^2
    pair<float, float> qElbow;
    pair<float, float> qWrist;

    // ik함수삽입, p1, p2, p3가 ik로 각각 들어가고, q0~ q6까지의 마디점이 구해짐, 마디점이 바뀔때만 계산함
    VectorXd pR1 = VectorXd::Map(p1.data() + 1, 3, 1);
    VectorXd pL1 = VectorXd::Map(p1.data() + 4, 3, 1);
    VectorXd qk1_06 = ikfun_final(pR1, pL1);


    VectorXd pR2 = VectorXd::Map(p2.data() + 1, 3, 1);
    VectorXd pL2 = VectorXd::Map(p2.data() + 4, 3, 1);
    VectorXd qk2_06 = ikfun_final(pR2, pL2);

    VectorXd pR3 = VectorXd::Map(p3.data() + 1, 3, 1);
    VectorXd pL3 = VectorXd::Map(p3.data() + 4, 3, 1);
    VectorXd qk3_06 = ikfun_final(pR3, pL3);
    
    wrist_addAngle = sts2wrist_fun(State);
    elbow_addAngle = sts2elbow_fun(State);

    Vmax = cal_Vmax(qk1_06, qk2_06, acc_max, t);

    // 출력
    std::cout << "State :\n"
         << State << "\n";
    std::cout << "wrist_addAngle :\n"
         << wrist_addAngle << "\n";
    std::cout << "elbow_addAngle :\n"
         << elbow_addAngle << "\n";

    std::cout << "R : p1 -> p2\n"
         << "(" << p1(1) << "," << p1(2) << "," << p1(3) << ") -> (" << p2(1) << "," << p2(2) << "," << p2(3) << ")\n";
    std::cout << "L : p1 -> p2\n"
         << "(" << p1(4) << "," << p1(5) << "," << p1(6) << ") -> (" << p2(4) << "," << p2(5) << "," << p2(6) << ")\n";

    for (int k = 0; k < 9; k++)
    {
        std::cout << "Q1[" << k << "] : " << qk1_06[k]*180.0/M_PI <<  " [deg] -> Q2[" << k << "] : " << qk2_06[k]*180.0/M_PI << " [deg],\t";
        std::cout << "Vmax[" << k << "] : " << Vmax(k) << "[rad/s]\n";
    }

    for (int i = 0; i < n; i++)
    {
        float t_step = dt*(i+1);
        VectorXd qi = VectorXd::Zero(9);
        
        qi = makeProfile(qk1_06, qk2_06, Vmax, acc_max, t_step, t);

        for (int m = 0; m < 7; m++)
        {
            qt(m) = qi(m);
        }

        qElbow = q78_fun(elbow_addAngle, t_step + p1(0));
        qWrist = q78_fun(wrist_addAngle, t_step + p1(0));
        
        qt(4) = qt(4) + qElbow.first;
        qt(6) = qt(6) + qElbow.second;
        qt(7) = qWrist.first;
        qt(8) = qWrist.second;

        // // 데이터 기록
        // for (int m = 0; m < 9; m++)
        // {
        //     std::string file_name = "desired_path";
        //     fun.appendToCSV_DATA(file_name, m, t_step + p1(0), qt(m));
        // }
        
        // Command Buffer 쌓기
        Motors_sendBuffer(qt, false);
    }
}

void PathManager::makeTrajectory()
{
    MatrixXd A30;  // 크기가 19x3인 2차원 벡터
    MatrixXd A31;  // 크기가 19x3인 2차원 벡터
    MatrixXd AA40; // 크기가 3x4인 2차원 벡터
    MatrixXd AA41; // 크기가 3x4인 2차원 벡터 // 3x3
    MatrixXd B;    // 크기가 19x3인 2차원 벡터
    MatrixXd BB;   // 크기가 3x4인 2차원 벡터

    VectorXd output1, output2;
    VectorXd Pi_R = VectorXd::Zero(3);
    VectorXd Pi_L = VectorXd::Zero(3);
    VectorXd Pf_R = VectorXd::Zero(3);
    VectorXd Pf_L = VectorXd::Zero(3);
    float ti = 0, tf = 0;

    float dt = canManager.deltaT;   // 0.005
    int n;
    float tm, sm, h;
    
    // 연주 처음 시작할 때 Q1, Q2 계산
    if (line == 0)
    {
        std::vector<float> t2(time_arr.begin(), time_arr.begin() + 5);
        MatrixXd inst2 = inst_arr.middleCols(0, 5);
        itms0_fun(t2, inst2, A30, A31, AA40, AA41);

        VectorXd A1 = A30.col(0);
        VectorXd A2 = A30.col(1);
        output1 = pos_madi_fun(A1);
        output2 = pos_madi_fun(A2);

        ti = output1(0);
        tf = output2(0);
        Pi_R << output1(1), output1(2), output1(3);
        Pi_L << output1(4), output1(5), output1(6);
        Pf_R << output2(1), output2(2), output2(3);
        Pf_L << output1(4), output1(5), output1(6);
    }
    else if (line == 1)
    {
        std::vector<float> t2(time_arr.begin(), time_arr.begin() + 5);
        MatrixXd inst2 = inst_arr.middleCols(0, 5);
        itms0_fun(t2, inst2, A30, A31, AA40, AA41);

        VectorXd A1 = A31.col(0);
        VectorXd A2 = A31.col(1);
        output1 = pos_madi_fun(A1);
        output2 = pos_madi_fun(A2);

        ti = output1(0);
        tf = output2(0);
        Pi_R << output1(1), output1(2), output1(3);
        Pi_L << output1(4), output1(5), output1(6);
        Pf_R << output2(1), output2(2), output2(3);
        Pf_L << output1(4), output1(5), output1(6);
    }
    else if (line > 1)
    {
        std::vector<float> t2(time_arr.begin() + line - 1, time_arr.begin() + line + 4);
        MatrixXd inst2 = inst_arr.middleCols(line - 1, 5);
        itms_fun(t2, inst2, B, BB, inst_now);

        VectorXd B1 = B.col(0);
        VectorXd B2 = B.col(1);
        output1 = pos_madi_fun(B1);
        output2 = pos_madi_fun(B2);

        ti = output1(0);
        tf = output2(0);
        Pi_R << output1(1), output1(2), output1(3);
        Pi_L << output1(4), output1(5), output1(6);
        Pf_R << output2(1), output2(2), output2(3);
        Pf_L << output1(4), output1(5), output1(6);
    }

    n = (tf - ti) / dt;
    tm = ti + 0.5*(tf - ti);
    h = 0.1;
    sm = 0.5;

    for (int i = 0; i < n; i++)
    {
        Pos Pt;
        VectorXd Pt_R;
        VectorXd Pt_L;
        float t = ti + dt * i;
        float s[2] = {0};   // s[0] : 3차 + 3차
                            // s[1] : 3차

        s[0] = timeScaling(ti, tf, t, tm, sm);
        s[1] = timeScaling_only3(ti, tf, t);
        Pt_R = makePath(Pi_R, Pf_R, s, sm, h);
        Pt_L = makePath(Pi_L, Pf_L, s, sm, h);

        for (int i = 0; i < 3; i++)
        {
            Pt.pR[i] = Pt_R(i);
            Pt.pL[i] = Pt_L(i);
        }
        P.push(Pt);

        std::string fileName;
        fileName = "Trajectory_R";
        fun.appendToCSV_DATA(fileName, Pt.pR[0], Pt.pR[1], Pt.pR[2]);
        fileName = "Trajectory_L";
        fun.appendToCSV_DATA(fileName, Pt.pR[0], Pt.pR[1], Pt.pR[2]);
    }
}


/////////////////////////////////////////////////////////////////////////////////
/*                           AddStance FUNCTION                               */
///////////////////////////////////////////////////////////////////////////////

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
