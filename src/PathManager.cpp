#include "../include/managers/PathManager.hpp" // 적절한 경로로 변경하세요.

PathManager::PathManager(State &stateRef,
                         CanManager &canManagerRef,
                         std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef)
{
}

/////////////////////////////////////////////////////////////////////////////////
/*                            SEND BUFFER TO MOTOR                            */
///////////////////////////////////////////////////////////////////////////////

void PathManager::Motors_sendBuffer(VectorXd &Qi, VectorXd &Vi, pair<double, double> Si)
{
    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            TMotorData newData;
            newData.position = Qi[motor_mapping[entry.first]] * tMotor->cwDir - tMotor->homeOffset;
            newData.velocity = Vi(motor_mapping[entry.first]) * tMotor->cwDir;

            tMotor->commandBuffer.push(newData);
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            MaxonData newData;
            newData.position = Qi(motor_mapping[entry.first]) * maxonMotor->cwDir;
            if (entry.first == "R_wrist")
                newData.WristState = Si.first;
            else if (entry.first == "L_wrist")
                newData.WristState = Si.second;
            else if (entry.first == "maxonForTest")
                newData.WristState = Si.second;

            maxonMotor->commandBuffer.push(newData);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                               SYSTEM FUNCTION                              */
///////////////////////////////////////////////////////////////////////////////

vector<double> PathManager::connect(vector<double> &Q1, vector<double> &Q2, int k, int n)
{
    vector<double> Qi;
    std::vector<double> A, B;

    // Compute A and Bk
    for (long unsigned int i = 0; i < Q1.size(); ++i)
    {
        A.push_back(0.5 * (Q1[i] - Q2[i]));
        B.push_back(0.5 * (Q1[i] + Q2[i]));
    }

    // Compute Qi using the provided formula
    for (long unsigned int i = 0; i < Q1.size(); ++i)
    {
        double val = A[i] * cos(M_PI * k / n) + B[i];
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
            if (entry.first == "R_arm1" || entry.first == "L_arm1" || entry.first == "R_arm3" || entry.first == "L_arm3")
            {
                tMotor->homeOffset = M_PI / 2 * tMotor->cwDir - tMotor->sensorLocation;
            }
            else if (entry.first == "R_arm2" || entry.first == "L_arm2")
            {
                tMotor->homeOffset = -M_PI / 6 * tMotor->cwDir - tMotor->sensorLocation;
            }

            c_MotorAngle[motor_mapping[entry.first]] = (tMotor->currentPos + tMotor->homeOffset) * tMotor->cwDir;
        }
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            c_MotorAngle[motor_mapping[entry.first]] = maxonMotor->currentPos * maxonMotor->cwDir;
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

MatrixXd PathManager::tms_fun(double t2_a, double t2_b, VectorXd &inst2_a, VectorXd &inst2_b)
{
    int flag = 0;

    VectorXd inst_c = VectorXd::Zero(18);

    double t3;
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

void PathManager::itms0_fun(vector<double> &t2, MatrixXd &inst2, MatrixXd &A30, MatrixXd &A31, MatrixXd &AA40, MatrixXd &AA41)
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
            double norm_val = T.block(1, k - 1, 9, 1).norm();
            MatrixXd block = T.block(1, k - 1, 9, 1);
            T.block(1, k, 9, 1) = -0.5 * block.cwiseAbs() / norm_val;
        }

        if (round(T.block(10, k, 9, 1).sum()) == 0)
        {
            double norm_val = T.block(10, k - 1, 9, 1).norm();
            MatrixXd block = T.block(10, k - 1, 9, 1);
            T.block(10, k, 9, 1) = -0.5 * block.cwiseAbs() / norm_val;
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
            AA41.resize(3, 4);
            AA41 << t4_inst4(0, k), t4_inst4(0, k + 1), t4_inst4(0, k + 2), t4_inst4(0, k + 3),
                t4_inst4.block(1, k, 9, 1).sum(), t4_inst4.block(1, k + 1, 9, 1).sum(), t4_inst4.block(1, k + 2, 9, 1).sum(), t4_inst4.block(1, k + 3, 9, 1).sum(),
                t4_inst4.block(10, k, 9, 1).sum(), t4_inst4.block(10, k + 1, 9, 1).sum(), t4_inst4.block(10, k + 2, 9, 1).sum(), t4_inst4.block(10, k + 3, 9, 1).sum();
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

void PathManager::itms_fun(vector<double> &t2, MatrixXd &inst2, MatrixXd &B, MatrixXd &BB)
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
            double norm_val = T.block(1, k - 1, 9, 1).norm();
            MatrixXd block = T.block(1, k - 1, 9, 1);
            T.block(1, k, 9, 1) = -0.5 * block.cwiseAbs() / norm_val;
        }

        if (round(T.block(10, k, 9, 1).sum()) == 0)
        {
            double norm_val = T.block(10, k - 1, 9, 1).norm();
            MatrixXd block = T.block(10, k - 1, 9, 1);
            T.block(10, k, 9, 1) = -0.5 * block.cwiseAbs() / norm_val;
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
    double time = A(0);

    VectorXd inst_right = A.segment(1, 9);
    VectorXd inst_left = A.segment(10, 9);

    VectorXd inst_right_01 = inst_right / inst_right.sum();
    VectorXd inst_left_01 = inst_left / inst_left.sum();

    double inst_right_state = inst_right.sum();
    double inst_left_state = inst_left.sum();

    VectorXd inst_p(18);
    inst_p << inst_right_01,
        inst_left_01;

    MatrixXd combined(6, 18);
    combined << right_inst, MatrixXd::Zero(3, 9), MatrixXd::Zero(3, 9), left_inst;
    MatrixXd p = combined * inst_p;

    VectorXd output(9);
    output << time,
        p,
        inst_right_state,
        inst_left_state;

    return output;
}

MatrixXd PathManager::sts2wrist_fun(MatrixXd &AA, double v_wrist)
{
    MatrixXd t_madi = AA.row(0);
    MatrixXd sts_R = AA.row(1);
    MatrixXd sts_L = AA.row(2);

    MatrixXd theta_R = MatrixXd::Zero(1, 4);
    MatrixXd theta_L = MatrixXd::Zero(1, 4);

    for (int i = 0; i < 4; ++i)
    {
        if (sts_R(0, i) == 1)
            theta_R(0, i) = 0;
        else if (sts_R(0, i) == -0.5)
            theta_R(0, i) = 0.15 * v_wrist;

        if (sts_L(0, i) == 1)
            theta_L(0, i) = 0;
        else if (sts_L(0, i) == -0.5)
            theta_L(0, i) = 0.15 * v_wrist;
    }

    MatrixXd t_wrist_madi(3, 3);
    for (int i = 0; i < 3; ++i)
    {
        if (sts_L(0, i) == -1)
        {
            double dt = t_madi(0, i + 1) - t_madi(0, i);
            theta_L(0, i) = dt * v_wrist;
            if (theta_L(0, i) > (M_PI / 2) * 0.8)
                theta_L(0, i) = (M_PI / 2) * 0.8;
        }

        if (sts_R(0, i) == -1)
        {
            double dt = t_madi(0, i + 1) - t_madi(0, i);
            theta_R(0, i) = dt * v_wrist;
            if (theta_R(0, i) > (M_PI / 2) * 0.8)
                theta_R(0, i) = (M_PI / 2) * 0.8;
        }
    }

    t_wrist_madi << t_madi.block(0, 0, 1, 3),
        theta_R.block(0, 0, 1, 3),
        theta_L.block(0, 0, 1, 3);

    return t_wrist_madi;
}

MatrixXd PathManager::sts2elbow_fun(MatrixXd &AA, double v_elbow)
{
    MatrixXd t_madi = AA.row(0);
    MatrixXd sts_R = AA.row(1);
    MatrixXd sts_L = AA.row(2);

    MatrixXd theta_R = MatrixXd::Zero(1, 4);
    MatrixXd theta_L = MatrixXd::Zero(1, 4);

    for (int i = 0; i < 4; ++i)
    {
        if (sts_R(0, i) == 1)
            theta_R(0, i) = 0;
        else if (sts_R(0, i) == -0.5)
            theta_R(0, i) = 0.15 * v_elbow;

        if (sts_L(0, i) == 1)
            theta_L(0, i) = 0;
        else if (sts_L(0, i) == -0.5)
            theta_L(0, i) = 0.15 * v_elbow;
    }

    MatrixXd t_elbow_madi(3, 3);
    for (int i = 0; i < 3; ++i)
    {
        if (sts_L(0, i) == -1)
        {
            double dt = t_madi(0, i + 1) - t_madi(0, i);
            theta_L(0, i) = dt * v_elbow;
            if (theta_L(0, i) > (M_PI / 2) * 0.8)
                theta_L(0, i) = (M_PI / 2) * 0.8;
        }

        if (sts_R(0, i) == -1)
        {
            double dt = t_madi(0, i + 1) - t_madi(0, i);
            theta_R(0, i) = dt * v_elbow;
            if (theta_R(0, i) > (M_PI / 2) * 0.8)
                theta_R(0, i) = (M_PI / 2) * 0.8;
        }
    }

    t_elbow_madi << t_madi.block(0, 0, 1, 3),
        theta_R.block(0, 0, 1, 3),
        theta_L.block(0, 0, 1, 3);

    return t_elbow_madi;
}

VectorXd PathManager::ikfun_final(VectorXd &pR, VectorXd &pL, VectorXd &part_length, double s, double z0)
{
    double direction = 0.0 * M_PI;

    double X1 = pR(0), Y1 = pR(1), z1 = pR(2);
    double X2 = pL(0), Y2 = pL(1), z2 = pL(2);
    double r1 = part_length(0);
    double r2 = part_length(1) + part_length(4);
    double L1 = part_length(2);
    double L2 = part_length(3) + part_length(5);

    int j = 0;
    double the3[1351];
    double zeta = z0 - z2;
    VectorXd Qf(7);
    double the0_f = 0;

    // the3 배열 초기화
    for (int i = 0; i < 1351; ++i)
        the3[i] = -M_PI / 4.0 + i * M_PI / 1350.0 * (3.0 / 4.0); // the3 범위 : -45deg ~ 90deg

    for (int i = 0; i < 1351; ++i)
    {
        double det_the4 = (z0 - z1 - r1 * cos(the3[i])) / r2;

        if (det_the4 < 1 && det_the4 > -1)
        {
            double the34 = acos((z0 - z1 - r1 * cos(the3[i])) / r2);
            double the4 = the34 - the3[i];

            if (the4 >= 0 && the4 < M_PI * (3.0 / 4.0)) // the4 범위 : 0deg ~ 135deg
            {
                double r = r1 * sin(the3[i]) + r2 * sin(the34);
                double det_the1 = (X1 * X1 + Y1 * Y1 - r * r - s * s / 4.0) / (s * r);

                if (det_the1 < 1 && det_the1 > -1)
                {
                    double the1 = acos(det_the1);
                    if (the1 > 0 && the1 < M_PI * (4.0 / 5.0)) // the1 범위 : 0deg ~ 144deg
                    {
                        double alpha = asin(X1 / sqrt(X1 * X1 + Y1 * Y1));
                        double det_the0 = (s / 4.0 + (X1 * X1 + Y1 * Y1 - r * r) / s) / sqrt(X1 * X1 + Y1 * Y1);

                        if (det_the0 < 1 && det_the0 > -1)
                        {
                            double the0 = asin(det_the0) - alpha;
                            if (the0 > -M_PI / 2 && the0 < M_PI / 2) // the0 범위 : -90deg ~ 90deg
                            {
                                double L = sqrt((X2 - 0.5 * s * cos(the0 + M_PI)) * (X2 - 0.5 * s * cos(the0 + M_PI)) + Y2 * Y2);
                                double det_the2 = (X2 - 0.5 * s * cos(the0 + M_PI)) / L;

                                if (det_the2 < 1 && det_the2 > -1)
                                {
                                    double the2 = acos(det_the2) - the0;
                                    if (the2 > M_PI / 5.0 && the2 < M_PI) // the2 범위 : 36deg ~ 180deg
                                    {
                                        double Lp = sqrt(L * L + zeta * zeta);
                                        double det_the6 = (Lp * Lp - L1 * L1 - L2 * L2) / (2 * L1 * L2);

                                        if (det_the6 < 1 && det_the6 > -1)
                                        {
                                            double the6 = acos(det_the6);
                                            if (the6 >= 0 && the6 < M_PI * (3.0 / 4.0)) // the6 범위 : 0deg ~ 135deg
                                            {
                                                double T = (zeta * zeta + L * L + L1 * L1 - L2 * L2) / (L1 * 2);
                                                double det_the5 = L * L + zeta * zeta - T * T;

                                                if (det_the5 > 0)
                                                {
                                                    double sol = T * L - zeta * sqrt(L * L + zeta * zeta - T * T);
                                                    sol /= (L * L + zeta * zeta);
                                                    double the5 = asin(sol);
                                                    if (the5 > -M_PI / 4 && the5 < M_PI / 2) // the5 범위 : -45deg ~ 90deg
                                                    {
                                                        if (j == 0 || fabs(the0 - direction) < fabs(the0_f - direction))
                                                        {
                                                            Qf << the0, the1, the2, the3[i], the4, the5, the6;
                                                            the0_f = the0;
                                                            j = 1;
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

    return Qf;
}

vector<double> PathManager::fkfun()
{
    getMotorPos();

    vector<double> P;
    vector<double> theta(9);
    for (auto &motorPair : motors)
    {
        auto &name = motorPair.first;
        auto &motor = motorPair.second;
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            theta[motor_mapping[name]] = (tMotor->currentPos + tMotor->homeOffset) * tMotor->cwDir;
            cout << name << " : " << theta[motor_mapping[name]] << "\n";
        }
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            theta[motor_mapping[name]] = maxonMotor->currentPos * maxonMotor->cwDir;
            cout << name << " : " << theta[motor_mapping[name]] << "\n";
        }
    }
    double r1 = part_length(0), r2 = part_length(1), l1 = part_length(2), l2 = part_length(3), stick = part_length(4);
    // double r, l;
    // r = r1 * sin(theta[3]) + r2 * sin(theta[3] + theta[4]);
    // l = l1 * sin(theta[5]) + l2 * sin(theta[5] + theta[6]);

    /*P.push_back(0.5 * s * cos(theta[0]) + r * cos(theta[0] + theta[1]));
    P.push_back(0.5 * s * sin(theta[0]) + r * sin(theta[0] + theta[1]));
    P.push_back(z0 - r1 * cos(theta[3]) - r2 * cos(theta[3] + theta[4]));
    P.push_back(0.5 * s * cos(theta[0] + M_PI) + l * cos(theta[0] + theta[2]));
    P.push_back(0.5 * s * sin(theta[0] + M_PI) + l * sin(theta[0] + theta[2]));
    P.push_back(z0 - l1 * cos(theta[5]) - l2 * cos(theta[5] + theta[6]));*/

    P.push_back(0.5 * s * cos(theta[0]) + r1 * sin(theta[3]) * cos(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * cos(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * cos(theta[0] + theta[1]));
    P.push_back(0.5 * s * sin(theta[0]) + r1 * sin(theta[3]) * sin(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * sin(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * sin(theta[0] + theta[1]));
    P.push_back(z0 - r1 * cos(theta[3]) - r2 * cos(theta[3] + theta[4]) - stick * cos(theta[3] + theta[4] + theta[7]));
    P.push_back(-0.5 * s * cos(theta[0]) + l1 * sin(theta[5]) * cos(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * cos(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * cos(theta[0] + theta[2]));
    P.push_back(-0.5 * s * sin(theta[0]) + l1 * sin(theta[5]) * sin(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * sin(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * sin(theta[0] + theta[2]));
    P.push_back(z0 - l1 * cos(theta[5]) - l2 * cos(theta[5] + theta[6]) - stick * cos(theta[5] + theta[6] + theta[8]));

    return P;
}

double PathManager::con_fun(double t_a, double t_b, double th_a, double th_b, double t_now)
{
    return (th_b - th_a) * (t_now - t_a) / (t_b - t_a) + th_a;
}

pair<double, double> PathManager::iconf_fun(double qk1_06, double qk2_06, double qk3_06, double qv_in, double t1, double t2, double t)
{
    double p_out, v_out /*, V1_out*/;

    if ((qk2_06 - qk1_06) / (qk3_06 - qk2_06) > 0)
    { // 방향 지속의 경우, 2차 함수
        double c = qk1_06;
        double b = qv_in;
        double a = (qk2_06 - qk1_06 - qv_in * t1) / (t1 * t1);

        p_out = a * t * t + b * t + c; // 위치
        v_out = 2 * a * t + b;         // 속도
        // V1_out = 2 * a * t1 + b; // t1 시점에서의 속도
    }
    else
    { // 방향 전환의 경우, 3차 함수
        double c = qv_in;
        double d = qk1_06;

        double T11 = t1 * t1 * t1;
        double T12 = t1 * t1;
        double T21 = 3 * t1 * t1;
        double T22 = 2 * t1;

        // 역행렬 계산을 위한 수식 처리
        double det = T11 * T22 - T12 * T21;
        double invT11 = T22 / det;
        double invT12 = -T12 / det;
        double invT21 = -T21 / det;
        double invT22 = T11 / det;

        double ANS1 = -c * t1 - d + qk2_06;
        double ANS2 = -c;

        // 행렬식을 이용한 계산
        double a = invT11 * ANS1 + invT12 * ANS2;
        double b = invT21 * ANS1 + invT22 * ANS2;

        p_out = a * t * t * t + b * t * t + c * t + d; // 위치
        v_out = 3 * a * t * t + 2 * b * t + c;         // 속도
        // V1_out = 3 * a * t1 * t1 + 2 * b * t1 + c; // t1 시점에서의 속도
    }

    return std::make_pair(p_out, v_out);
}

pair<double, double> PathManager::qRL_fun(MatrixXd &t_madi, double t_now)
{
    double qR_t, qL_t;

    VectorXd time_madi = t_madi.row(0);
    VectorXd q7_madi = t_madi.row(1);
    VectorXd q8_madi = t_madi.row(2);

    if (t_now >= time_madi(0) && t_now < time_madi(1))
    {
        qR_t = con_fun(time_madi(0), time_madi(1), q7_madi(0), q7_madi(1), t_now);
        qL_t = con_fun(time_madi(0), time_madi(1), q8_madi(0), q8_madi(1), t_now);
    }
    else if (t_now >= time_madi(1) && t_now < time_madi(2))
    {
        qR_t = con_fun(time_madi(1), time_madi(2), q7_madi(1), q7_madi(2), t_now);
        qL_t = con_fun(time_madi(1), time_madi(2), q8_madi(1), q8_madi(2), t_now);
    }
    else
    {
        qR_t = q7_madi(2);
        qL_t = q8_madi(2);
    }

    return std::make_pair(qR_t, qL_t);
}

pair<double, double> PathManager::SetTorqFlag(MatrixXd &State, double t_now)
{
    double q7_isTorq = 0.0;
    double q8_isTorq = 0.0;

    VectorXd time_madi = State.row(0);
    VectorXd q7_state = State.row(1);
    VectorXd q8_state = State.row(2);

    if (time_madi(0) == 0.0)
    {
        if (t_now >= time_madi(0) && t_now < time_madi(1))
        {
            q7_isTorq = -0.5;
            q8_isTorq = -0.5;
        }
        else if (t_now >= time_madi(1) && t_now < time_madi(2))
        {
            q7_isTorq = time_madi(1);
            q8_isTorq = time_madi(1);
        }
    }
    else if (t_now >= time_madi(0) && t_now < time_madi(1))
    {
        q7_isTorq = time_madi(0);
        q8_isTorq = time_madi(0);
    }
    else if (t_now >= time_madi(1) && t_now < time_madi(2))
    {
        q7_isTorq = time_madi(1);
        q8_isTorq = time_madi(1);
    }

    return std::make_pair(q7_isTorq, q8_isTorq);
}

void PathManager::SetTargetPos(MatrixXd &State, MatrixXd &t_madi)
{
    VectorXd q7_state = State.row(1);
    VectorXd q8_state = State.row(2);
    VectorXd q7_madi = t_madi.row(1);
    VectorXd q8_madi = t_madi.row(2);

    if (q7_state(0) == 1)
    {
        for (auto &entry : motors)
        {
            if (entry.first == "R_wrist")
            {
                std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second);
                maxonMotor->targetPos = q7_madi(1);
            }
        }
    }

    if (q8_state(0) == 1)
    {
        for (auto &entry : motors)
        {
            if (entry.first == "L_wrist" || entry.first == "maxonForTest")
            {
                std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second);
                maxonMotor->targetPos = q8_madi(1);
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                                  MAKE PATH                                 */
///////////////////////////////////////////////////////////////////////////////

void PathManager::GetDrumPositoin()
{
    getMotorPos();
    part_length.resize(6);
    part_length << 0.363, 0.3835, 0.363, 0.3835, 0.417, 0.417; ///< [오른팔 상완, 오른팔 하완, 왼팔 상완, 왼팔 하완, 스틱, 스틱]의 길이.

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

    right_inst.resize(3, 9);
    left_inst.resize(3, 9);

    // Extract the desired elements
    // Vector3d right_B = {0, 0, 0};
    Vector3d right_S;
    Vector3d right_FT;
    Vector3d right_MT;
    Vector3d right_HT;
    Vector3d right_HH;
    Vector3d right_R;
    Vector3d right_RC;
    Vector3d right_LC;

    // Vector3d left_B = {0, 0, 0};
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

    right_inst << right_RC, right_R, right_S, right_HH, right_HH, right_FT, right_MT, right_LC, right_HT;
    left_inst << left_RC, left_R, left_S, left_HH, left_HH, left_FT, left_MT, left_LC, left_HT;
}

void PathManager::GetMusicSheet()
{
    /////////// 드럼로봇 악기정보 텍스트 -> 딕셔너리 변환
    map<string, int> instrument_mapping = {
        {"1", 2}, {"2", 5}, {"3", 6}, {"4", 8}, {"5", 3}, {"6", 1}, {"7", 0}, {"8", 7}, {"11", 2}, {"51", 2}, {"61", 2}, {"71", 2}, {"81", 2}, {"91", 2}};

    default_right.resize(9);
    default_left.resize(9);
    default_right << 1, 0, 0, 0, 0, 0, 0, 0, 0;
    default_left << 1, 0, 0, 0, 0, 0, 0, 0, 0;

    string score_path = "../include/managers/codeConfession copy.txt";

    ifstream file(score_path);
    if (!file.is_open())
        cerr << "Error opening file." << endl;

    string row;
    int lineIndex = 0;
    double time = 0.0;
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
            cout << "bpm = " << bpm << "\n";
        }
        else
        {
            VectorXd inst_arr_R = VectorXd::Zero(9), inst_arr_L = VectorXd::Zero(9);
            VectorXd inst_col = VectorXd::Zero(18);

            if (columns[2] == "0" && columns[3] == "0")
                continue;
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
    combined << right_inst, MatrixXd::Zero(3, 9), MatrixXd::Zero(3, 9), left_inst;
    MatrixXd p = combined * inst_p;

    VectorXd pR = VectorXd::Map(p.data(), 3, 1);
    VectorXd pL = VectorXd::Map(p.data() + 3, 3, 1);
    VectorXd qk = ikfun_final(pR, pL, part_length, s, z0);

    for (int i = 0; i < qk.size(); ++i)
    {
        standby[i] = qk(i);
    }
    standby[7] = 0.0;
    standby[8] = 0.0;
}

void PathManager::PathLoopTask()
{
    double v_wrist = M_PI;
    double v_elbow = 0.5 * M_PI;
    double t_now = time_arr[line];

    VectorXd qt = VectorXd::Zero(9);
    VectorXd qv_in = VectorXd::Zero(7);
    MatrixXd A30;  // 크기가 19x3인 2차원 벡터
    MatrixXd A31;  // 크기가 19x3인 2차원 벡터
    MatrixXd AA40; // 크기가 3x4인 2차원 벡터
    MatrixXd AA41; // 크기가 3x4인 2차원 벡터
    MatrixXd B;    // 크기가 19x3인 2차원 벡터
    MatrixXd BB;   // 크기가 3x4인 2차원 벡터
    MatrixXd State(3, 4);

    VectorXd p1(9), p2(9), p3(9);
    MatrixXd t_wrist_madi(3, 3), t_elbow_madi(3, 3);

    // 연주 처음 시작할 때 Q1, Q2 계산
    if (line == 0)
    {
        std::vector<double> t2(time_arr.begin(), time_arr.begin() + 5);
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
        std::vector<double> t2(time_arr.begin(), time_arr.begin() + 5);
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
        std::vector<double> t2(time_arr.begin() + line - 1, time_arr.begin() + line + 4);
        MatrixXd inst2 = inst_arr.middleCols(line - 1, 5);
        itms_fun(t2, inst2, B, BB);

        VectorXd B1 = B.col(0);
        VectorXd B2 = B.col(1);
        VectorXd B3 = B.col(2);
        p1 = pos_madi_fun(B1);
        p2 = pos_madi_fun(B2);
        p3 = pos_madi_fun(B3);

        State = BB;
    }

    // ik함수삽입, p1, p2, p3가 ik로 각각 들어가고, q0~ q6까지의 마디점이 구해짐, 마디점이 바뀔때만 계산함
    VectorXd pR1 = VectorXd::Map(p1.data() + 1, 3, 1);
    VectorXd pL1 = VectorXd::Map(p1.data() + 4, 3, 1);
    VectorXd qk1_06 = ikfun_final(pR1, pL1, part_length, s, z0);

    VectorXd pR2 = VectorXd::Map(p2.data() + 1, 3, 1);
    VectorXd pL2 = VectorXd::Map(p2.data() + 4, 3, 1);
    VectorXd qk2_06 = ikfun_final(pR2, pL2, part_length, s, z0);

    VectorXd pR3 = VectorXd::Map(p3.data() + 1, 3, 1);
    VectorXd pL3 = VectorXd::Map(p3.data() + 4, 3, 1);
    VectorXd qk3_06 = ikfun_final(pR3, pL3, part_length, s, z0);

    t_wrist_madi = sts2wrist_fun(State, v_wrist);
    t_elbow_madi = sts2elbow_fun(State, v_elbow);

    SetTargetPos(State, t_wrist_madi); // 타격시 새로운 손목 targetPos값 전달

    double t1 = p2(0) - p1(0);
    double t2 = p3(0) - p1(0);
    double dt = 0.005;
    int n = t1 / dt;

    for (int i = 0; i < n; i++)
    {
        for (int m = 0; m < 7; m++)
        {
            pair<double, double> p = iconf_fun(qk1_06(m), qk2_06(m), qk3_06(m), qv_in(m), t1, t2, t_now - p1(0) + dt * i);
            qt(m) = p.first;
            qv_in(m) = p.second;
        }

        pair<double, double> qElbow = qRL_fun(t_elbow_madi, t_now + dt * i);
        pair<double, double> qWrist = qRL_fun(t_wrist_madi, t_now + dt * i);
        pair<double, double> wrist_state = SetTorqFlag(State, t_now + dt * i); // -1. 1. 0.5 값 5ms단위로 전달

        qt(4) += qElbow.first;
        qt(6) += qElbow.second;
        qt(7) = qWrist.first;
        qt(8) = qWrist.second;

        Motors_sendBuffer(qt, qv_in, wrist_state);
        vector<double> qt_vector(qt.data(), qt.data() + qt.size());
        pos.push_back(qt_vector);
        vector<double> qv_in_vector(qv_in.data(), qv_in.data() + qv_in.size());
        vel.push_back(qv_in_vector);
    }
}

void PathManager::GetArr(vector<double> &arr)
{
    cout << "Get Array...\n";

    vector<double> Qi;
    vector<vector<double>> q_setting;

    getMotorPos();

    int n = 800; // 4초동안 실행
    for (int k = 0; k < n; ++k)
    {
        // Make GetBack Array
        Qi = connect(c_MotorAngle, arr, k, n);
        q_setting.push_back(Qi);

        // Send to Buffer
        for (auto &entry : motors)
        {
            if (std::shared_ptr<TMotor> tmotor = std::dynamic_pointer_cast<TMotor>(entry.second))
            {
                TMotorData newData;
                newData.position = Qi[motor_mapping[entry.first]] * tmotor->cwDir - tmotor->homeOffset;
                newData.velocity = 0.5;
                tmotor->commandBuffer.push(newData);
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
            {
                MaxonData newData;
                newData.position = Qi[motor_mapping[entry.first]] * maxonMotor->cwDir;
                newData.WristState = 0.5;
                maxonMotor->commandBuffer.push(newData);
            }
        }
    }
}