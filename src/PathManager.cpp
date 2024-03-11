#include "../include/managers/PathManager.hpp" // 적절한 경로로 변경하세요.

PathManager::PathManager(SystemState &systemStateRef,
                         CanManager &canManagerRef,
                         std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : systemState(systemStateRef), canManager(canManagerRef), motors(motorsRef)
{
}

/////////////////////////////////////////////////////////////////////////////////
/*                            SEND BUFFER TO MOTOR                            */
///////////////////////////////////////////////////////////////////////////////

void PathManager::Motors_sendBuffer(MatrixXd &Qi, MatrixXd &Vi)
{
    struct can_frame frame;

    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            float p_des = Qi[motor_mapping[entry.first]];
            float v_des = Vi[motor_mapping[entry.first]];

            TParser.parseSendCommand(*tMotor, &frame, tMotor->nodeId, 8, p_des, v_des, tMotor->Kp, tMotor->Kd, 0.0);
            entry.second->sendBuffer.push(frame);
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            float p_des = Qi[motor_mapping[entry.first]];
            MParser.getTargetPosition(*maxonMotor, &frame, p_des);
            entry.second->sendBuffer.push(frame);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                               SYSTEM FUNCTION                              */
///////////////////////////////////////////////////////////////////////////////

void PathManager::ApplyDir()
{ // CW / CCW에 따른 방향 적용
    for (auto &entry : motors)
    {
        shared_ptr<GenericMotor> motor = entry.second;
        standby[motor_mapping[entry.first]] *= motor->cwDir;
        backarr[motor_mapping[entry.first]] *= motor->cwDir;
        motor_dir[motor_mapping[entry.first]] = motor->cwDir;
    }
}

void PathManager::getMotorPos()
{
    // 각 모터의 현재위치 값 불러오기 ** CheckMotorPosition 이후에 해야함(변수값을 불러오기만 해서 갱신 필요)
    for (auto &entry : motors)
    {
        c_MotorAngle[motor_mapping[entry.first]] = entry.second->currentPos;
        // 각 모터의 현재 위치 출력
        cout << "Motor " << entry.first << " current position: " << entry.second->currentPos << "\n";
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


MatrixXd PathManager::tms_fun(double t2_a, double t2_b, MatrixXd &inst2_a, MatrixXd &inst2_b)
{
    // flag 변수 초기화
    int flag = 0;

    // inst_c 행렬 초기화
    MatrixXd inst_c = -100 * MatrixXd::Ones(18, 1);

    // t3와 t3_inst3 행렬 선언 (double과 MatrixXd의 조합)
    double t3;
    MatrixXd t3_inst3(36, 3);

    // 1번 룰: 1이 연속되면 t3와 inst3를 생성하고, t2 0.2초 앞에 inst2를 타격할 준비(-1)를 함
    if ((inst2_a.block(0, 0, 9, 1).norm() == 1) && (inst2_b.block(0, 0, 9, 1).norm() == 1))
    {
        // 오른손
        inst_c.block(0, 0, 9, 1) = -inst2_b.block(0, 0, 9, 1);
        t3 = 0.5 * (t2_b + t2_a);
        t3_inst3 << t2_a, t3, t2_b,
            inst2_a, inst_c, inst2_b; // left
        flag = 1;
    }

    if ((inst2_a.block(9, 0, 9, 1).norm() == 1) && (inst2_b.block(9, 0, 9, 1).norm() == 1))
    {
        // 왼손
        inst_c.block(9, 0, 9, 1) = -inst2_b.block(9, 0, 9, 1);
        t3 = 0.5 * (t2_a + t2_b);
        t3_inst3 << t2_a, t3, t2_b,
            inst2_a, inst_c, inst2_b;
        flag = 1;
    }

    // 2번 룰: 1번 룰에 의해 새로운 시점/악기가 정의되어 있고(flag = 1) inst2에 1이 있으면 inst3에 -1을 넣는다.
    if ((inst2_a.block(0, 0, 9, 1).norm() == 0) && (inst2_b.block(0, 0, 9, 1).norm() == 1) && (flag == 1))
    {
        // 오른손
        inst_c.block(0, 0, 9, 1) = -inst2_b.block(0, 0, 9, 1);
        t3 = 0.5 * (t2_a + t2_b);
        t3_inst3 << t2_a, t3, t2_b,
            inst2_a, inst_c, inst2_b;
    }

    if ((inst2_a.block(9, 0, 9, 1).norm() == 0) && (inst2_b.block(9, 0, 9, 1).norm() == 1) && (flag == 1))
    {
        // 왼손
        inst_c.block(9, 0, 9, 1) = -inst2_b.block(9, 0, 9, 1);
        t3 = 0.5 * (t2_a + t2_b);
        t3_inst3 << t2_a, t3, t2_b,
            inst2_a, inst_c, inst2_b;
    }

    // 3번 룰: 1번 룰을 거치지 않았고 inst1이 0 이고 inst2에 1이 있으면, inst1에 -1을 넣는다.
    if ((inst2_a.block(0, 0, 9, 1).norm() == 0) && (inst2_b.block(0, 0, 9, 1).norm() == 1) && (flag == 0))
    {
        inst2_a.block(0, 0, 9, 1) = -inst2_b.block(0, 0, 9, 1);
        t3_inst3 << t2_a, t2_b,
            inst2_a, inst2_b;
    }

    if ((inst2_a.block(9, 0, 9, 1).norm() == 0) && (inst2_b.block(9, 0, 9, 1).norm() == 1) && (flag == 0))
    {
        inst2_a.block(9, 0, 9, 1) = -inst2_b.block(9, 0, 9, 1);
        t3_inst3 << t2_a, t2_b,
            inst2_a, inst2_b;
    }

    // 악보 끝까지 연주하기 위해서 inst2_b에 0 행렬이 인위적으로 들어가는 경우, 그대로 내보냄
    if ((inst2_b.norm() == 0) && (flag == 0))
    {
        t3_inst3 << t2_a, t2_b,
            inst2_a, inst2_b;
    }

    return t3_inst3;
}

void PathManager::itms0_fun(vector<double> &t2, MatrixXd &inst2, MatrixXd &A30, MatrixXd &A31, MatrixXd &AA40, MatrixXd &AA41)
{
    MatrixXd T(18, 1);

    for (int k = 0; k < 4; ++k)
    {
        MatrixXd inst_0 = inst2.col(k);
        MatrixXd inst_1 = inst2.col(k + 1);
        MatrixXd inst3 = tms_fun(t2[k], t2[k + 1], inst_0, inst_1);

        int n = T.cols();

        if (n == 1)
            T = inst3;
        else
        {
            for (size_t i = 0; i < T.size(); ++i)
            {
                MatrixXd temp = T.block(0, 0, T.rows(), n - 1);
                T.resize(T.rows(), n);
                T << temp, inst3;
            }
            T = {T[0], inst3};
        }
    }

    /* 빈 자리에 -0.5 집어넣기:  */
    int nn = T.cols();

    for (int k = 1; k < nn; ++k)
    {
        // T의 2행부터 10행까지의 합이 0인지 확인하고 조건에 따라 업데이트
        if (T.block(1, k, 9, 1).sum() == 0)
        {
            double norm_val = T.block(1, k - 1, 9, 1).norm();
            MatrixXd block = T.block(1, k - 1, 9, 1);
            T.block(1, k, 9, 1) = -0.5 * block.array().abs() / norm_val;
        }

        // T의 11행부터 19행까지의 합이 0인지 확인하고 조건에 따라 업데이트
        if (T.block(10, k, 9, 1).sum() == 0)
        {
            double norm_val = T.block(10, k - 1, 9, 1).norm();
            MatrixXd block = T.block(10, k - 1, 9, 1);
            T.block(10, k, 9, 1) = -0.5 * block.array().abs() / norm_val;
        }
    }

    /* 일단 0=t2(1)에서부터 t2(4)까지 정의함 */
    int j = 0;

    // t4_inst4의 크기를 결정하기 위해 먼저 반복문을 실행하여 t2(4)보다 작거나 같은 열의 개수를 계산
    for (int k = 0; k < nn; ++k)
    {
        if (T(0, k) <= t2[3])
        {
            j++;
        }
    }

    // t4_inst4 행렬 선언 및 크기 지정
    MatrixXd t4_inst4(T.rows(), j);

    // 다시 반복하여 T의 열을 t4_inst4에 복사
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
    j = t4_inst4.cols();
    // 반복문을 사용하여 t4_inst4의 각 열에 대해 검사
    for (int k = 0; k < j; ++k)
    {
        // t4_inst4의 첫 번째 행이 t2의 두 번째부터 네 번째 요소 중 하나와 같은지 확인
        for (int i = 1; i <= 3; ++i)
        {
            if (t4_inst4(0, k) == t2[i])
            {
                if (kk == 0)
                {
                    // kk가 0이면 A31에 현재 열을 복사
                    A31 = t4_inst4.col(k);
                    kk = 1;
                }
                else
                {
                    // kk가 0이 아니면 A31에 현재 열을 추가
                    A31.conservativeResize(A31.rows(), A31.cols() + 1);
                    A31.col(A31.cols() - 1) = t4_inst4.col(k);
                }
            }
        }
    }

    /* state31: t2(2)에서 시작해서 3개의 연속된 t4_inst4를 골라냄 */
    for (int k = 0; k < j; ++k)
    {
        // t4_inst4의 두 번째 행이 t2의 두 번째 요소와 같은지 확인
        if (t4_inst4(0, k) == t2[1])
        {
            // AA41에 값을 할당
            AA41.resize(3, 4);
            AA41 << t4_inst4(0, k), t4_inst4(0, k + 1), t4_inst4(0, k + 2), t4_inst4(0, k + 3),
                t4_inst4.block(1, k, 9, 1).sum(), t4_inst4.block(1, k + 1, 9, 1).sum(), t4_inst4.block(1, k + 2, 9, 1).sum(), t4_inst4.block(1, k + 3, 9, 1).sum(),
                t4_inst4.block(10, k, 9, 1).sum(), t4_inst4.block(10, k + 1, 9, 1).sum(), t4_inst4.block(10, k + 2, 9, 1).sum(), t4_inst4.block(10, k + 3, 9, 1).sum();
            break; // 조건을 만족하는 첫 번째 열을 찾으면 반복문 종료
        }
    }

    int kk = 0;
    for (int k = 0; k < j; ++k)
    {
        // t4_inst4의 첫 번째 행이 t2의 첫 번째부터 세 번째 요소 중 하나와 같은지 확인
        for (int i = 0; i < 3; ++i)
        {
            if (t4_inst4(0, k) == t2[i])
            {
                if (kk == 0)
                {
                    // kk가 0이면 A30에 현재 열을 복사
                    A30 = t4_inst4.col(k);
                    kk = 1;
                }
                else
                {
                    // kk가 0이 아니면 A30에 현재 열을 추가
                    A30.conservativeResize(A30.rows(), A30.cols() + 1);
                    A30.col(A30.cols() - 1) = t4_inst4.col(k);
                }
            }
        }
    }

    AA40.resize(4, 4);
    AA40 << t4_inst4.col(0), t4_inst4.col(1), t4_inst4.col(2), t4_inst4.col(3),
        t4_inst4.block(1, 0, 9, 1).colwise().sum(), t4_inst4.block(1, 1, 9, 1).colwise().sum(),
        t4_inst4.block(10, 0, 9, 1).colwise().sum(), t4_inst4.block(10, 1, 9, 1).colwise().sum();
}

void PathManager::itms_fun(vector<double> &t2, MatrixXd &inst2, MatrixXd &B, MatrixXd &BB)
{
    MatrixXd T(18, 1);

    for (int k = 0; k < 4; ++k)
    {
        MatrixXd inst_0 = inst2.col(k);
        MatrixXd inst_1 = inst2.col(k + 1);
        MatrixXd inst3 = tms_fun(t2[k], t2[k + 1], inst_0, inst_1);

        int n = T.cols();

        if (n == 1)
            T = inst3;
        else
        {
            for (size_t i = 0; i < T.size(); ++i)
            {
                MatrixXd temp = T.block(0, 0, T.rows(), n - 1);
                T.resize(T.rows(), n);
                T << temp, inst3;
            }
            T = {T[0], inst3};
        }
    }

    /* 빈 자리에 -0.5 집어넣기:  */
    int nn = T.cols();

    for (int k = 1; k < nn; ++k)
    {
        // T의 2행부터 10행까지의 합이 0인지 확인하고 조건에 따라 업데이트
        if (T.block(1, k, 9, 1).sum() == 0)
        {
            double norm_val = T.block(1, k - 1, 9, 1).norm();
            MatrixXd block = T.block(1, k - 1, 9, 1);
            T.block(1, k, 9, 1) = -0.5 * block.array().abs() / norm_val;
        }

        // T의 11행부터 19행까지의 합이 0인지 확인하고 조건에 따라 업데이트
        if (T.block(10, k, 9, 1).sum() == 0)
        {
            double norm_val = T.block(10, k - 1, 9, 1).norm();
            MatrixXd block = T.block(10, k - 1, 9, 1);
            T.block(10, k, 9, 1) = -0.5 * block.array().abs() / norm_val;
        }
    }

    /* 일단 0=t2(1)에서부터 t2(4)까지 정의함 */
    int j = 0;

    // t4_inst4의 크기를 결정하기 위해 먼저 반복문을 실행하여 t2(4)보다 작거나 같은 열의 개수를 계산
    for (int k = 0; k < nn; ++k)
    {
        if (T(0, k) >= t2[1] && T(0, k) <= t2[4])
        {
            j++;
        }
    }

    // t4_inst4 행렬 선언 및 크기 지정
    MatrixXd t4_inst4(T.rows(), j);

    // 다시 반복하여 T의 열을 t4_inst4에 복사
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
    j = t4_inst4.cols();
    // 반복문을 사용하여 t4_inst4의 각 열에 대해 검사
    for (int k = 0; k < j; ++k)
    {
        // t4_inst4의 첫 번째 행이 t2의 두 번째부터 네 번째 요소 중 하나와 같은지 확인
        for (int i = 1; i <= 3; ++i)
        {
            if (t4_inst4(0, k) == t2[i])
            {
                if (kk == 0)
                {
                    // kk가 0이면 B에 현재 열을 복사
                    B = t4_inst4.col(k);
                    kk = 1;
                }
                else
                {
                    // kk가 0이 아니면 B에 현재 열을 추가
                    B.conservativeResize(B.rows(), B.cols() + 1);
                    B.col(B.cols() - 1) = t4_inst4.col(k);
                }
            }
        }
    }

    /* state31: t2(2)에서 시작해서 3개의 연속된 t4_inst4를 골라냄 */
    for (int k = 0; k < j; ++k)
    {
        // t4_inst4의 두 번째 행이 t2의 두 번째 요소와 같은지 확인
        if (t4_inst4(0, k) == t2[1])
        {
            // BB에 값을 할당
            BB.resize(3, 4);
            BB << t4_inst4(0, k), t4_inst4(0, k + 1), t4_inst4(0, k + 2), t4_inst4(0, k + 3),
                t4_inst4.block(1, k, 9, 1).sum(), t4_inst4.block(1, k + 1, 9, 1).sum(), t4_inst4.block(1, k + 2, 9, 1).sum(), t4_inst4.block(1, k + 3, 9, 1).sum(),
                t4_inst4.block(10, k, 9, 1).sum(), t4_inst4.block(10, k + 1, 9, 1).sum(), t4_inst4.block(10, k + 2, 9, 1).sum(), t4_inst4.block(10, k + 3, 9, 1).sum();
            break; // 조건을 만족하는 첫 번째 열을 찾으면 반복문 종료
        }
    }
}

MatrixXd PathManager::pos_madi_fun(MatrixXd &A)
{
    double time = A(0, 0);

    MatrixXd inst_right = A.block(1, 0, 9, 1);
    MatrixXd inst_left = A.block(10, 0, 9, 1);

    MatrixXd inst_right_01 = inst_right / inst_right.sum();
    MatrixXd inst_left_01 = inst_left / inst_left.sum();

    double inst_right_state = inst_right.sum();
    double inst_left_state = inst_left.sum();

    MatrixXd inst_p(18, 1);
    inst_p << inst_right_01,
        inst_left_01;

    MatrixXd combined(6, 18);
    combined << right_inst, MatrixXd::Zero(3, 9), MatrixXd::Zero(3, 9), left_inst;
    MatrixXd p = combined * inst_p;

    MatrixXd output(6, 1);
    output << time,
        p,
        inst_right_state,
        inst_left_state;
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

MatrixXd PathManager::ikfun_final(MatrixXd &pR, MatrixXd &pL, MatrixXd &part_length, double s, double z0)
{
    double direction = 0.0 * M_PI;

    double X1 = pR(0), Y1 = pR(1), z1 = pR(2);
    double X2 = pL(0), Y2 = pL(1), z2 = pL(2);
    double r1 = part_length(0);
    double r2 = part_length(1) + part_length(4);
    double L1 = part_length(2);
    double L2 = part_length(3) + part_length(5);

    int j = 0;
    double the3[1801];
    double zeta = z0 - z2;
    MatrixXd Qf(7, 1);
    double the0_f = 0;

    // the3 배열 초기화
    for (int i = 0; i < 1801; ++i)
        the3[i] = -M_PI / 2.0 + i * M_PI / 1800.0;

    for (int i = 0; i < 1800; ++i)
    {
        double det_the4 = (z0 - z1 - r1 * cos(the3[i])) / r2;

        if (det_the4 < 1 && det_the4 > -1)
        {
            double the34 = acos((z0 - z1 - r1 * cos(the3[i])) / r2);
            double the4 = the34 - the3[i];

            if (the4 > 0)
            {
                double r = r1 * sin(the3[i]) + r2 * sin(the34);
                double det_the1 = (X1 * X1 + Y1 * Y1 - r * r - s * s / 4.0) / (s * r);

                if (det_the1 < 1 && det_the1 > -1)
                {
                    double the1 = acos(det_the1);
                    if (the1 > 0 && the1 < M_PI * (5.0 / 6.0))
                    {
                        double alpha = asin(X1 / sqrt(X1 * X1 + Y1 * Y1));
                        double det_the0 = (s / 4.0 + (X1 * X1 + Y1 * Y1 - r * r) / s) / sqrt(X1 * X1 + Y1 * Y1);

                        if (det_the0 < 1 && det_the0 > -1)
                        {
                            double the0 = asin(det_the0) - alpha;
                            double L = sqrt((X2 - 0.5 * s * cos(the0 + M_PI)) * (X2 - 0.5 * s * cos(the0 + M_PI)) + Y2 * Y2);
                            double det_the2 = (X2 - 0.5 * s * cos(the0 + M_PI)) / L;

                            if (det_the2 < 1 && det_the2 > -1)
                            {
                                double the2 = acos(det_the2) - the0;
                                if (the2 > M_PI / 6.0 && the2 < M_PI)
                                {
                                    double Lp = sqrt(L * L + zeta * zeta);
                                    double det_the6 = (Lp * Lp - L1 * L1 - L2 * L2) / (2 * L1 * L2);

                                    if (det_the6 < 1 && det_the6 > -1)
                                    {
                                        double the6 = acos(det_the6);
                                        double T = (zeta * zeta + L * L + L1 * L1 - L2 * L2) / (L1 * 2);
                                        double det_the5 = L * L + zeta * zeta - T * T;

                                        if (det_the5 > 0)
                                        {
                                            double sol = T * L - zeta * sqrt(L * L + zeta * zeta - T * T);
                                            sol /= (L * L + zeta * zeta);
                                            double the5 = asin(sol);

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

    return Qf;
}

double PathManager::con_fun(double t_a, double t_b, double th_a, double th_b, double t_now)
{
    return (th_b - th_a) * (t_now - t_a) / (t_b - t_a) + th_a;
}

pair<double, double> PathManager::iconf_fun(double qk1_06, double qk2_06, double qk3_06, double qv_in, double t1, double t2, double t)
{
    double p_out, v_out, V1_out;

    if ((qk2_06 - qk1_06) / (qk3_06 - qk2_06) > 0)
    { // 방향 지속의 경우, 2차 함수
        // p(t) = a*t^2 + b*t + c;  position
        // v(t) = 2*a*t + b;        velocity

        double c = qk1_06;                                     // 초기 위치
        double b = qv_in;                                      // 초기 속도로 정해짐.
        double a = (qk2_06 - qk1_06 - qv_in * t1) / (t1 * t1); // 이것에 제한을 둠, |a| < k 로 함, 만약 a가 k보다 크면, k로 한점을 둠.

        p_out = a * t * t + b * t + c; // position
        v_out = 2 * a * t + b;         // velocity
        V1_out = 2 * a * t1 + b;       // t1 시점에서의 속도
    }
    else
    { // 방향 전환의 경우, 3차 함수
        // p(t) = a*t^3 + b*t^2 + c*t + d;  position
        // v(t) = 3*a*t^2 + 2*b*t + c;      velocity

        double c = qv_in;
        double d = qk1_06;

        double t1_squared = t1 * t1;
        double t1_cubed = t1_squared * t1;
        double t1_squared_times_3 = 3 * t1_squared;
        double t1_squared_times_2 = 2 * t1_squared;

        double T[2][2] = {{t1_cubed, t1_squared},
                          {t1_squared_times_3, t1_squared_times_2}};

        double ANS[2] = {-(c * t1 + d) + qk2_06,
                         -c};

        double det_T = T[0][0] * T[1][1] - T[0][1] * T[1][0];
        double inv_T[2][2] = {{T[1][1] / det_T, -T[0][1] / det_T},
                              {-T[1][0] / det_T, T[0][0] / det_T}};

        double a = inv_T[0][0] * ANS[0] + inv_T[0][1] * ANS[1];
        double b = inv_T[1][0] * ANS[0] + inv_T[1][1] * ANS[1];

        p_out = a * t * t * t + b * t * t + c * t + d;
        v_out = 3 * a * t * t + 2 * b * t + c;
        V1_out = 3 * a * t1_squared + 2 * b * t1 + c;
    }

    return std::make_pair(p_out, v_out);
}

pair<double, double> PathManager::qRL_fun(MatrixXd &t_madi, double t_now)
{
    double qR_t, qL_t;

    VectorXd time_madi = t_madi.row(0);
    VectorXd q7_madi = t_madi.row(1);
    VectorXd q8_madi = t_madi.row(2);

    if (t_now >= time_madi(0) && t_now < q7_madi(1))
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

/////////////////////////////////////////////////////////////////////////////////
/*                                  MAKE PATH                                 */
///////////////////////////////////////////////////////////////////////////////

void PathManager::GetDrumPositoin()
{
    getMotorPos();

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
    Vector3d right_B = {0, 0, 0};
    Vector3d right_S;
    Vector3d right_FT;
    Vector3d right_MT;
    Vector3d right_HT;
    Vector3d right_HH;
    Vector3d right_R;
    Vector3d right_RC;
    Vector3d right_LC;

    Vector3d left_B = {0, 0, 0};
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

    string score_path = "../include/managers/codeConfession.txt";

    ifstream file(score_path);
    if (!file.is_open())
        cerr << "Error opening file." << endl;

    string line;
    int lineIndex = 0;
    double time = 0.0;
    while (getline(file, line))
    {
        istringstream iss(line);
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
            MatrixXd inst_arr_R(9, 1), inst_arr_L(9, 1);
            MatrixXd inst_col(18, 1);

            if (columns[2] == "0" && columns[3] == "0")
                continue;
            if (columns[2] != "0")
                inst_arr_R[instrument_mapping[columns[2]]] = 1;
            if (columns[3] != "0")
                inst_arr_L[instrument_mapping[columns[3]]] = 1;

            time += stod(columns[1]) * 100 / bpm;
            time_arr.push_back(time);
            inst_col << inst_arr_R, inst_arr_L;
            inst_arr.col(inst_arr.cols()) = inst_col;
        }

        lineIndex++;
    }

    file.close();

    total = time_arr.size();
}

void PathManager::PathLoopTask()
{
    double v_wrist = M_PI;
    double v_elbow = 0.5 * M_PI;
    double t_now = time_arr[line];

    MatrixXd qt = MatrixXd::Zero(7, 1);
    MatrixXd qv_in = MatrixXd::Zero(7, 1);
    MatrixXd A30 = MatrixXd::Zero(19, 3); // 크기가 19x3인 2차원 벡터
    MatrixXd A31 = MatrixXd::Zero(19, 3); // 크기가 19x3인 2차원 벡터
    MatrixXd AA40 = MatrixXd::Zero(3, 4); // 크기가 3x4인 2차원 벡터
    MatrixXd AA41 = MatrixXd::Zero(3, 4); // 크기가 3x4인 2차원 벡터
    MatrixXd B = MatrixXd::Zero(19, 3);   // 크기가 19x3인 2차원 벡터
    MatrixXd BB = MatrixXd::Zero(3, 4);   // 크기가 3x4인 2차원 벡터

    MatrixXd p1, p2, p3;
    MatrixXd t_wrist_madi, t_elbow_madi;

    // 연주 처음 시작할 때 Q1, Q2 계산
    if (line == 0)
    {
        std::vector<double> t2(time_arr.begin(), time_arr.begin() + 5);
        MatrixXd inst2 = inst_arr.middleCols(0, 5);
        itms0_fun(t2, inst2, A30, A31, AA40, AA41);

        MatrixXd A1 = A30.col(0);
        MatrixXd A2 = A30.col(1);
        MatrixXd A3 = A30.col(2);
        p1 = pos_madi_fun(A1);
        p2 = pos_madi_fun(A2);
        p3 = pos_madi_fun(A3);

        t_wrist_madi = sts2wrist_fun(AA40, v_wrist);
        t_elbow_madi = sts2elbow_fun(AA40, v_elbow);
    }
    else if (line == 1)
    {
        MatrixXd A1 = A31.col(0);
        MatrixXd A2 = A31.col(1);
        MatrixXd A3 = A31.col(2);
        p1 = pos_madi_fun(A1);
        p2 = pos_madi_fun(A2);
        p3 = pos_madi_fun(A3);

        t_wrist_madi = sts2wrist_fun(AA41, v_wrist);
        t_elbow_madi = sts2elbow_fun(AA41, v_elbow);
    }
    else
    {
        std::vector<double> t2(time_arr.begin() + line - 1, time_arr.begin() + line + 4);
        MatrixXd inst2 = inst_arr.middleCols(line - 1, line + 4);
        itms_fun(t2, inst2, B, BB);

        MatrixXd B1 = B.col(0);
        MatrixXd B2 = B.col(1);
        MatrixXd B3 = B.col(2);
        p1 = pos_madi_fun(B1);
        p2 = pos_madi_fun(B2);
        p3 = pos_madi_fun(B3);

        t_wrist_madi = sts2wrist_fun(BB, v_wrist);
        t_elbow_madi = sts2elbow_fun(BB, v_elbow);
    }

    line++;

    // ik함수삽입, p1, p2, p3가 ik로 각각 들어가고, q0~ q6까지의 마디점이 구해짐, 마디점이 바뀔때만 계산함
    MatrixXd pR1 = MatrixXd::Map(p1.data() + 1, 3, 1);
    MatrixXd pL1 = MatrixXd::Map(p1.data() + 4, 3, 1);
    MatrixXd qk1_06 = ikfun_final(pR1, pL1, part_length, s, z0);

    MatrixXd pR2 = MatrixXd::Map(p2.data() + 1, 3, 1);
    MatrixXd pL2 = MatrixXd::Map(p2.data() + 4, 3, 1);
    MatrixXd qk2_06 = ikfun_final(pR2, pL2, part_length, s, z0);

    MatrixXd pR3 = MatrixXd::Map(p3.data() + 1, 3, 1);
    MatrixXd pL3 = MatrixXd::Map(p3.data() + 4, 3, 1);
    MatrixXd qk3_06 = ikfun_final(pR3, pL3, part_length, s, z0);

    double t1 = p2(0) - p1(0);
    double t2 = p3(0) - p1(0);
    double dt = 0.005;
    int n = t1 / dt;

    for (int i = 0; i < n; i++)
    {
        for (int m = 0; m < 7; m++)
        {
            pair<double, double> p = iconf_fun(qk1_06(m), qk2_06(m), qk3_06(m), qv_in(m), t1, t2, t_now - p1(0) + dt * (i - 1));
            qt(m) = p.first;
            qv_in(m) = p.second;
        }

        pair<double, double> qWrist = qRL_fun(t_wrist_madi, t_now + dt * (i - 1));
        pair<double, double> qElbow = qRL_fun(t_elbow_madi, t_now + dt * (i - 1));

        qt(4) += qElbow.first;
        qt(6) += qElbow.second;
        qt(7) = qWrist.first;
        qt(8) = qWrist.second;

        Motors_sendBuffer(qt, qv_in);
    }
}

void PathManager::GetArr(vector<double> &arr)
{
    cout << "Get Array...\n";
    struct can_frame frame;

    vector<double> Qi;
    vector<vector<double>> q_setting;

    getMotorPos();

    int n = 800;
    for (int k = 0; k < n; ++k)
    {
        // Make GetBack Array
        Qi = connect(c_MotorAngle, arr, k, n);
        q_setting.push_back(Qi);

        // Send to Buffer
        for (auto &entry : motors)
        {
            if (std::shared_ptr<TMotor> motor = std::dynamic_pointer_cast<TMotor>(entry.second))
            {
                float p_des = Qi[motor_mapping[entry.first]];
                TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, motor->Kp, motor->Kd, 0.0);
                entry.second->sendBuffer.push(frame);
            }
            else if (std::shared_ptr<MaxonMotor> motor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
            {
                float p_des = Qi[motor_mapping[entry.first]];
                MParser.getTargetPosition(*motor, &frame, p_des);
                entry.second->sendBuffer.push(frame);
            }
        }
    }

    c_MotorAngle = Qi;
}