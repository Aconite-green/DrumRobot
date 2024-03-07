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

void PathManager::Motors_sendBuffer(vector<double> &Qi, vector<double> &Vi)
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




vector<vector<double>> PathManager::tms_fun(double t2_0, double t2_1, vector<double> &inst2_0, vector<double> &inst2_1)
{
}

void PathManager::itms0_fun(vector<double> &t, MatrixXd &inst2, MatrixXd &A30, MatrixXd &A31, MatrixXd &AA40, MatrixXd &AA41)
{
    MatrixXd T(18,1);

    for (int k = 0; k < 4; ++k)
    {
        vector<double> inst_0, inst_1;
        for (size_t i = 0; i < inst2.size(); ++i)
        {
            inst_0.push_back(inst2[i][k]);
            inst_1.push_back(inst2[i][k + 1]);
        }
        vector<vector<double>> inst3 = tms_fun(t2[k], t2[k + 1], inst_0, inst_1);

        int n = T[0].size();
        if (n == 1)
            T = inst3;
        else
        {
            for (size_t i = 0; i < T.size(); ++i)
            {
                
            }
            T = {T[0], inst3};
        }
        
    }

    int nn = T[0].size();

    for (int k = 1; k < nn; ++k)
    {
        double norm_prev = 0.0;
        for (int i = 1; i <= 9; ++i)
            norm_prev += std::abs(T[i][k - 1]);

        if (std::accumulate(T[1].begin() + 1, T[10].begin() + 1, 0.0) == 0)
        {
            double scale_factor = 0.5 * norm_prev / std::sqrt(norm_prev);
            for (int i = 1; i <= 9; ++i)
                T[i][k] = -0.5 * std::abs(T[i][k - 1]) / norm_prev;
        }

        norm_prev = 0.0;
        for (int i = 11; i <= 19; ++i)
            norm_prev += std::abs(T[i][k - 1]);

        if (std::accumulate(T[11].begin() + 1, T[19].begin() + 1, 0.0) == 0)
        {
            double scale_factor = 0.5 * norm_prev / std::sqrt(norm_prev);
            for (int i = 11; i <= 19; ++i)
                T[i][k] = -0.5 * std::abs(T[i][k - 1]) / norm_prev;
        }
    }

    int j = 0;
    std::vector<std::vector<double>> t4_inst4(18, std::vector<double>(1, 0.0)); // 타격/준비/대기/해당없음 4개의 상태로 표현된 악보등장

    for (int k = 0; k < nn; ++k)
    {
        if (T[0][k] <= t2[3])
        {
            ++j;
            t4_inst4[k] = T[k];
        }
    }

    std::vector<std::vector<double>> hit3_0, hit3_1, state4_0, state4_1;

    int kk = 0;
    for (int k = 0; k < j; ++k)
    {
        for (int i = 1; i <= 3; ++i)
        {
            if (t4_inst4[0][k] == t2[i])
            {
                if (kk == 0)
                {
                    hit3_0 = {t4_inst4[k]};
                    kk = 1;
                }
                else
                    hit3_0.push_back(t4_inst4[k]);
            }
        }
    }

    state4_0 = {{t4_inst4[0][0], t4_inst4[0][1], t4_inst4[0][2], t4_inst4[0][3]},
                {std::accumulate(t4_inst4[1].begin(), t4_inst4[1].end(), 0.0),
                 std::accumulate(t4_inst4[2].begin(), t4_inst4[2].end(), 0.0),
                 std::accumulate(t4_inst4[3].begin(), t4_inst4[3].end(), 0.0),
                 std::accumulate(t4_inst4[4].begin(), t4_inst4[4].end(), 0.0),
                 std::accumulate(t4_inst4[5].begin(), t4_inst4[5].end(), 0.0),
                 std::accumulate(t4_inst4[6].begin(), t4_inst4[6].end(), 0.0),
                 std::accumulate(t4_inst4[7].begin(), t4_inst4[7].end(), 0.0),
                 std::accumulate(t4_inst4[8].begin(), t4_inst4[8].end(), 0.0),
                 std::accumulate(t4_inst4[9].begin(), t4_inst4[9].end(), 0.0)},
                {std::accumulate(t4_inst4[10].begin(), t4_inst4[10].end(), 0.0),
                 std::accumulate(t4_inst4[11].begin(), t4_inst4[11].end(), 0.0),
                 std::accumulate(t4_inst4[12].begin(), t4_inst4[12].end(), 0.0),
                 std::accumulate(t4_inst4[13].begin(), t4_inst4[13].end(), 0.0),
                 std::accumulate(t4_inst4[14].begin(), t4_inst4[14].end(), 0.0),
                 std::accumulate(t4_inst4[15].begin(), t4_inst4[15].end(), 0.0),
                 std::accumulate(t4_inst4[16].begin(), t4_inst4[16].end(), 0.0),
                 std::accumulate(t4_inst4[17].begin(), t4_inst4[17].end(), 0.0),
                 std::accumulate(t4_inst4[18].begin(), t4_inst4[18].end(), 0.0)}};

    int kk = 0;
    for (int k = 0; k < j; ++k)
    {
        for (int i = 1; i <= 3; ++i)
        {
            if (t4_inst4[0][k] == t2[i])
            {
                if (kk == 0)
                {
                    hit3_1 = {t4_inst4[k]};
                    kk = 1;
                }
                else
                    hit3_1.push_back(t4_inst4[k]);
            }
        }
    }

    state4_1 = {{t4_inst4[0][0], t4_inst4[0][1], t4_inst4[0][2], t4_inst4[0][3]},
                {std::accumulate(t4_inst4[1].begin(), t4_inst4[1].end(), 0.0),
                 std::accumulate(t4_inst4[2].begin(), t4_inst4[2].end(), 0.0),
                 std::accumulate(t4_inst4[3].begin(), t4_inst4[3].end(), 0.0),
                 std::accumulate(t4_inst4[4].begin(), t4_inst4[4].end(), 0.0),
                 std::accumulate(t4_inst4[5].begin(), t4_inst4[5].end(), 0.0),
                 std::accumulate(t4_inst4[6].begin(), t4_inst4[6].end(), 0.0),
                 std::accumulate(t4_inst4[7].begin(), t4_inst4[7].end(), 0.0),
                 std::accumulate(t4_inst4[8].begin(), t4_inst4[8].end(), 0.0)},
                {std::accumulate(t4_inst4[9].begin(), t4_inst4[9].end(), 0.0),
                 std::accumulate(t4_inst4[10].begin(), t4_inst4[10].end(), 0.0),
                 std::accumulate(t4_inst4[11].begin(), t4_inst4[11].end(), 0.0),
                 std::accumulate(t4_inst4[12].begin(), t4_inst4[12].end(), 0.0),
                 std::accumulate(t4_inst4[13].begin(), t4_inst4[13].end(), 0.0),
                 std::accumulate(t4_inst4[14].begin(), t4_inst4[14].end(), 0.0),
                 std::accumulate(t4_inst4[15].begin(), t4_inst4[15].end(), 0.0),
                 std::accumulate(t4_inst4[16].begin(), t4_inst4[16].end(), 0.0),
                 std::accumulate(t4_inst4[17].begin(), t4_inst4[17].end(), 0.0)}};
}

void PathManager::itms_fun(vector<double> &t, MatrixXd &inst2, MatrixXd &B, MatrixXd &BB)
{
}

vector<double> PathManager::pos_madi_fun(MatrixXd &A)
{
}

vector<vector<double>> PathManager::sts2wrist_fun(MatrixXd &AA, double v_wrist)
{
}

vector<vector<double>> PathManager::sts2elbow_fun(MatrixXd &AA, double v_elbow)
{
}

vector<double> PathManager::ikfun_final(vector<double> &pR, vector<double> &pL, MatrixXd &part_length, double s0, double z0)
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
    vector<vector<double>> inst_xyz(6, vector<double>(8, 0));

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 8; ++j)
        {
            inputFile >> inst_xyz[i][j];
            if (i == 1 || i == 4)
                inst_xyz[i][j] = inst_xyz[i][j] * 1.0;
        }
    }

    // Extract the desired elements
    vector<double> right_B = {0, 0, 0};
    vector<double> right_S;
    vector<double> right_FT;
    vector<double> right_MT;
    vector<double> right_HT;
    vector<double> right_HH;
    vector<double> right_R;
    vector<double> right_RC;
    vector<double> right_LC;

    for (int i = 0; i < 3; ++i)
    {
        right_S.push_back(inst_xyz[i][0]);
        right_FT.push_back(inst_xyz[i][1]);
        right_MT.push_back(inst_xyz[i][2]);
        right_HT.push_back(inst_xyz[i][3]);
        right_HH.push_back(inst_xyz[i][4]);
        right_R.push_back(inst_xyz[i][5]);
        right_RC.push_back(inst_xyz[i][6]);
        right_LC.push_back(inst_xyz[i][7]);
    }

    vector<double> left_B = {0, 0, 0};
    vector<double> left_S;
    vector<double> left_FT;
    vector<double> left_MT;
    vector<double> left_HT;
    vector<double> left_HH;
    vector<double> left_R;
    vector<double> left_RC;
    vector<double> left_LC;

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
    MatrixXd B = MatrixXd::Zero(19, 3); // 크기가 19x3인 2차원 벡터
    MatrixXd BB = MatrixXd::Zero(3, 4); // 크기가 3x4인 2차원 벡터

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
    std::vector<double> pR(p1.begin() + 1, p1.begin() + 4);
    std::vector<double> pL(p1.begin() + 4, p1.begin() + 7);
    std::vector<double> qk1_06 = ikfun_final(pR, pL, part_length, s, z0);
    std::vector<double> pR(p2.begin() + 1, p2.begin() + 4);
    std::vector<double> pL(p2.begin() + 4, p2.begin() + 7);
    std::vector<double> qk2_06 = ikfun_final(pR, pL, part_length, s, z0);
    std::vector<double> pR(p3.begin() + 1, p3.begin() + 4);
    std::vector<double> pL(p3.begin() + 4, p3.begin() + 7);
    std::vector<double> qk3_06 = ikfun_final(pR, pL, part_length, s, z0);

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