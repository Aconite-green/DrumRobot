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
        c_MotorAngle[motor_mapping[entry.first]] = entry.second->currentPos;
        // 각 모터의 현재 위치 출력
        cout << "Motor " << entry.first << " current position: " << entry.second->currentPos << "\n";
    }
}

double determinant(double mat[3][3])
{ // 행렬의 determinant 계산 함수
    return mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) -
           mat[0][1] * (mat[1][0] * mat[2][2] - mat[2][0] * mat[1][2]) +
           mat[0][2] * (mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1]);
}

void inverseMatrix(double mat[3][3], double inv[3][3])
{ // 역행렬 계산 함수
    double det = determinant(mat);

    if (det == 0)
    {
        std::cerr << "역행렬이 존재하지 않습니다." << std::endl;
        return;
    }

    double invDet = 1.0 / det;

    inv[0][0] = (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) * invDet;
    inv[0][1] = (mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2]) * invDet;
    inv[0][2] = (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) * invDet;

    inv[1][0] = (mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2]) * invDet;
    inv[1][1] = (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) * invDet;
    inv[1][2] = (mat[1][0] * mat[0][2] - mat[0][0] * mat[1][2]) * invDet;

    inv[2][0] = (mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1]) * invDet;
    inv[2][1] = (mat[2][0] * mat[0][1] - mat[0][0] * mat[2][1]) * invDet;
    inv[2][2] = (mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1]) * invDet;
}

void PathManager::iconnect(vector<double> &P0, vector<double> &P1, vector<double> &P2, vector<double> &V0, double t1, double t2, double t)
{
    vector<double> V1;
    vector<double> p_out;
    vector<double> v_out;
    for (size_t i = 0; i < P0.size(); ++i)
    {
        if ((P1[i] - P0[i]) / (P2[i] - P1[i]) > 0)
            V1.push_back((P2[i] - P0[i]) / t2);
        else
            V1.push_back(0);

        double f = P0[i];
        double d = 0;
        double e = V0[i];

        double M[3][3] = {
            {20.0 * pow(t1, 2), 12.0 * t1, 6.0},
            {5.0 * pow(t1, 4), 4.0 * pow(t1, 3), 3.0 * pow(t1, 2)},
            {pow(t1, 5), pow(t1, 4), pow(t1, 3)}};
        double ANS[3] = {0, V1[i] - V0[i], P1[i] - P0[i] - V0[i] * t1};

        double invM[3][3];
        inverseMatrix(M, invM);
        // Multiply the inverse of T with ANS
        double tem[3];
        for (size_t j = 0; j < 3; ++j)
        {
            tem[j] = 0;
            for (size_t k = 0; k < 3; ++k)
            {
                tem[j] += invM[j][k] * ANS[k];
            }
        }

        double a = tem[0];
        double b = tem[1];
        double c = tem[2];

        p_out.push_back(a * pow(t, 5) + b * pow(t, 4) + c * pow(t, 3) + d * pow(t, 2) + e * t + f);
        v_out.push_back(5 * a * pow(t, 4) + 4 * b * pow(t, 3) + 3 * c * pow(t, 2) + 3 * d * t + e);
    }

    p.push_back(p_out);
    v.push_back(v_out);
}

vector<double> PathManager::fkfun()
{
    vector<double> P;
    vector<double> theta(7);
    for (auto &motorPair : motors)
    {
        auto &name = motorPair.first;
        auto &motor = motorPair.second;
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            theta[motor_mapping[name]] = tMotor->currentPos * tMotor->cwDir;
        }
    }
    double r1 = R[0], r2 = R[1], l1 = R[2], l2 = R[3];
    double r, l;
    r = r1 * sin(theta[3]) + r2 * sin(theta[3] + theta[4]);
    l = l1 * sin(theta[5]) + l2 * sin(theta[5] + theta[6]);

    P.push_back(0.5 * s * cos(theta[0]) + r * cos(theta[0] + theta[1]));
    P.push_back(0.5 * s * sin(theta[0]) + r * sin(theta[0] + theta[1]));
    P.push_back(z0 - r1 * cos(theta[3]) - r2 * cos(theta[3] + theta[4]));
    P.push_back(0.5 * s * cos(theta[0] + M_PI) + l * cos(theta[0] + theta[2]));
    P.push_back(0.5 * s * sin(theta[0] + M_PI) + l * sin(theta[0] + theta[2]));
    P.push_back(z0 - l1 * cos(theta[5]) - l2 * cos(theta[5] + theta[6]));

    return P;
}

vector<double> PathManager::IKfun(vector<double> &P1, vector<double> &P2)
{
    // 드럼위치의 중점 각도
    double direction = 0.0 * M_PI; //-M_PI / 3.0;

    // 몸통과 팔이 부딧히지 않을 각도 => 36deg
    double differ = M_PI / 5.0;

    vector<double> Qf(7);

    double X1 = P1[0], Y1 = P1[1], z1 = P1[2];
    double X2 = P2[0], Y2 = P2[1], z2 = P2[2];
    double r1 = R[0], r2 = R[1], r3 = R[2], r4 = R[3];

    vector<double> the3(1801);
    for (int i = 0; i < 1801; i++)
    { // 오른팔 들어올리는 각도 범위 : -90deg ~ 90deg
        the3[i] = -M_PI / 2 + (M_PI * i) / 1800;
    }

    double zeta = z0 - z2;

    double det_the0, det_the1, det_the2, det_the4, det_the5, det_the6;
    double the0_f, the0, the1, the2, the34, the4, the5, the6;
    double r, L, Lp, T;
    double sol;
    double alpha;
    bool first = true;

    for (long unsigned int i = 0; i < the3.size(); i++)
    {
        det_the4 = (z0 - z1 - r1 * cos(the3[i])) / r2;

        if (det_the4 < 1 && det_the4 > -1)
        {
            the34 = acos((z0 - z1 - r1 * cos(the3[i])) / r2);
            the4 = the34 - the3[i];
            if (the4 > 0 && the4 < M_PI * 0.75)
            { // 오른팔꿈치 각도 범위 : 0 ~ 135deg
                r = r1 * sin(the3[i]) + r2 * sin(the34);

                det_the1 = (X1 * X1 + Y1 * Y1 - r * r - s * s / 4) / (s * r);
                if (det_the1 < 1 && det_the1 > -1)
                {
                    the1 = acos(det_the1);
                    if (the1 > 0 && the1 < (M_PI - differ))
                    { // 오른팔 돌리는 각도 범위 : 0 ~ 150deg
                        alpha = asin(X1 / sqrt(X1 * X1 + Y1 * Y1));
                        det_the0 = (s / 4 + (X1 * X1 + Y1 * Y1 - r * r) / s) / sqrt(X1 * X1 + Y1 * Y1);
                        if (det_the0 < 1 && det_the0 > -1)
                        {
                            the0 = asin(det_the0) - alpha;

                            L = sqrt(pow(X2 - 0.5 * s * cos(the0 + M_PI), 2) +
                                     pow(Y2 - 0.5 * s * sin(the0 + M_PI), 2));
                            det_the2 = (X2 + 0.5 * s * cos(the0)) / L;

                            if (det_the2 < 1 && det_the2 > -1)
                            {
                                the2 = acos(det_the2) - the0;
                                if (the2 > differ && the2 < M_PI)
                                { // 왼팔 돌리는 각도 범위 : 30deg ~ 180deg
                                    Lp = sqrt(L * L + zeta * zeta);
                                    det_the6 = (Lp * Lp - r3 * r3 - r4 * r4) / (2 * r3 * r4);
                                    if (det_the6 < 1 && det_the6 > -1)
                                    {
                                        the6 = acos(det_the6);
                                        if (the6 > 0 && the6 < M_PI * 0.75)
                                        { // 왼팔꿈치 각도 범위 : 0 ~ 135deg
                                            T = (zeta * zeta + L * L + r3 * r3 - r4 * r4) / (r3 * 2);
                                            det_the5 = L * L + zeta * zeta - T * T;

                                            if (det_the5 > 0)
                                            {
                                                sol = T * L - zeta * sqrt(L * L + zeta * zeta - T * T);
                                                sol /= (L * L + zeta * zeta);
                                                the5 = asin(sol);
                                                if (the5 > -M_PI / 4 && the5 < M_PI / 2)
                                                { // 왼팔 들어올리는 각도 범위 : -45deg ~ 90deg

                                                    if (first || abs(the0 - direction) < abs(the0_f - direction))
                                                    {
                                                        the0_f = the0;
                                                        Qf[0] = the0;
                                                        Qf[1] = the1;
                                                        Qf[2] = the2;
                                                        Qf[3] = the3[i];
                                                        Qf[4] = the4;
                                                        Qf[5] = the5;
                                                        Qf[6] = the6;

                                                        first = false;
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

    if (first)
    {
        std::cout << "IKfun Not Solved!!\n";
        systemState.main = Main::Pause;
    }

    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            Qf[motor_mapping[entry.first]] *= tMotor->cwDir;
        }
    }

    return Qf;
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

vector<double> PathManager::pos_madi_fun(MatrixXd &A)
{
    double time = A(0, 0);

    // inst_right, inst_left 추출
    MatrixXd inst_right = A.block(1, 0, 9, 1);
    MatrixXd inst_left = A.block(10, 0, 9, 1);

    // inst_right_01, inst_left_01 계산
    MatrixXd inst_right_01 = inst_right / inst_right.sum();
    MatrixXd inst_left_01 = inst_left / inst_left.sum();

    // inst_right_state, inst_left_state 계산
    double inst_right_state = inst_right.sum();
    double inst_left_state = inst_left.sum();

    // inst_p 계산
    MatrixXd inst_p(18, 1);
    inst_p << inst_right_01,
              inst_left_01;

    // p 계산
    MatrixXd p = (MatrixXd(3, 9) << R, MatrixXd::Zero(3, 9), MatrixXd::Zero(3, 9), L).transpose() * inst_p;

    // output 행렬 초기화
    MatrixXd output(6, 1);

    // output 설정
    output << time,
              p,
              inst_right_state,
              inst_left_state;
}

vector<vector<double>> PathManager::sts2wrist_fun(MatrixXd &AA, double v_wrist)
{
}

vector<vector<double>> PathManager::sts2elbow_fun(MatrixXd &AA, double v_elbow)
{
}

MatrixXd PathManager::ikfun_final(MatrixXd &pR, MatrixXd &pL, MatrixXd &part_length, double s0, double z0)
{
}

double PathManager::con_fun()
{
}

pair<double, double> PathManager::iconf_fun(double qk1_06, double qk2_06, double qk3_06, double qv_in, double t1, double t2, double t)
{
}

pair<double, double> PathManager::q78_fun(vector<vector<double>> &t_madi, double)
{
}

pair<double, double> PathManager::q46_fun(vector<vector<double>> &t_madi, double)
{
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
    
    for (int i = 0; i < 3; ++i) {
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


    for (int i = 3; i < 6; ++i)
    {
        left_S.push_back(inst_xyz[i][0]);
        left_FT.push_back(inst_xyz[i][1]);
        left_MT.push_back(inst_xyz[i][2]);
        left_HT.push_back(inst_xyz[i][3]);
        left_HH.push_back(inst_xyz[i][4]);
        left_R.push_back(inst_xyz[i][5]);
        left_RC.push_back(inst_xyz[i][6]);
        left_LC.push_back(inst_xyz[i][7]);
    }

    // Combine the elements into right_inst and left_inst
    right_inst = {right_RC, right_R, right_S, right_HH, right_HH, right_FT, right_MT, right_LC, right_HT};
    left_inst = {left_RC, left_R, left_S, left_HH, left_HH, left_FT, left_MT, left_LC, left_HT};
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

    vector<double> p1, p2, p3;
    vector<vector<double>> t_wrist_madi, t_elbow_madi;

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
    MatrixXd pR(p1.begin() + 1, p1.begin() + 4);
    MatrixXd pL(p1.begin() + 4, p1.begin() + 7);
    MatrixXd qk1_06 = ikfun_final(pR, pL, part_length, s, z0);
    MatrixXd pR(p2.begin() + 1, p2.begin() + 4);
    MatrixXd pL(p2.begin() + 4, p2.begin() + 7);
    MatrixXd qk2_06 = ikfun_final(pR, pL, part_length, s, z0);
    MatrixXd pR(p3.begin() + 1, p3.begin() + 4);
    MatrixXd pL(p3.begin() + 4, p3.begin() + 7);
    MatrixXd qk3_06 = ikfun_final(pR, pL, part_length, s, z0);

    double t1 = p2[0] - p1[0];
    double t2 = p3[0] - p1[0];
    double dt = 0.005;
    int n = t1 / dt;

    for (int i = 0; i < n; i++)
    {
        for (int m = 0; m < 7; m++)
        {
            pair<double, double> p = iconf_fun(qk1_06[m], qk2_06[m], qk3_06[m], qv_in[m], t1, t2, t_now - p1[0] + dt * (i - 1));
            qt[m] = p.first;
            qv_in[m] = p.second;
        }

        pair<double, double> qWrist = q78_fun(t_wrist_madi, t_now + dt * (i - 1));
        pair<double, double> qElbow = q46_fun(t_elbow_madi, t_now + dt * (i - 1));

        qt[4] += qElbow.first;
        qt[6] += qElbow.second;
        qt[7] = qWrist.first;
        qt[8] = qWrist.second;

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