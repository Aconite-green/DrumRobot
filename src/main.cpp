#include <thread>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <memory>
#include <map>
#include <atomic>

#include "../include/motors/Motor.hpp"
#include "../include/managers/PathManager.hpp"
#include "../include/managers/CanManager.hpp"
#include "../include/managers/TestManager.hpp"
#include "../include/tasks/DrumRobot.hpp"
#include "../include/tasks/SystemState.hpp"
#include "../include/managers/GuiManager.hpp"
#include "../include/USBIO_advantech/USBIO_advantech.hpp"
#include "../include/tasks/Functions.hpp"

using namespace std;

// 스레드 우선순위 설정 함수
bool setThreadPriority(std::thread &th, int priority, int policy = SCHED_FIFO)
{
    sched_param sch_params;
    sch_params.sched_priority = priority;
    if (pthread_setschedparam(th.native_handle(), policy, &sch_params))
    {
        std::cerr << "Failed to set Thread scheduling : " << std::strerror(errno) << std::endl;
        return false;
    }
    return true;
}


int main(int argc, char *argv[])
{
    // Create Share Resource
    State state;
    std::map<std::string, std::shared_ptr<GenericMotor>> motors;
    USBIO usbio;

    Functions fun(motors);
    CanManager canManager(motors, fun);
    PathManager pathManager(state, canManager, motors, fun);
    TestManager testManager(state, canManager, motors, usbio, fun);

    DrumRobot drumRobot(state, canManager, pathManager, testManager, motors, usbio, fun);
    GuiManager guiManager(state, canManager, motors);

    //shy-desktop -> 1반환
    //shy-MINIPC-VC66-C2 -> 2반환
    // int com_number = canManager.get_com_number_by_hostname();
    int com_number = fun.get_com_number_by_hostname();

    // 포트를 비활성화하고 다시 활성화
    // canManager.restCanPort(com_number);
    fun.restCanPort(com_number);

    // Create Threads
    std::thread stateThread(&DrumRobot::stateMachine, &drumRobot);
    std::thread sendThread(&DrumRobot::sendLoopForThread, &drumRobot);
    std::thread receiveThread(&DrumRobot::recvLoopForThread, &drumRobot);
    //std::thread guiThread(&GuiManager::guiThread, &guiManager);
    
    // Threads Priority Settings
    if (!setThreadPriority(sendThread, 3))
    {
        std::cerr << "Error setting priority for sendCanFrame" << std::endl;
        return -1;
    }
    if (!setThreadPriority(receiveThread, 2))
    {
        std::cerr << "Error setting priority for receiveCanFrame" << std::endl;
        return -1;
    }
    if (!setThreadPriority(stateThread, 1))
    {
        std::cerr << "Error setting priority for stateMachine" << std::endl;
        return -1;
    }
    //if (!setThreadPriority(guiThread, 4))
    //{
    //    std::cerr << "Error setting priority for stateMachine" << std::endl;
    //    return -1;
    //}


    // Wait Threads
    stateThread.join();
    sendThread.join();
    receiveThread.join();
    //guiThread.join();
}



bool PathManager::innu_readMeasure(ifstream& inputFile, bool &BPMFlag)
{
    string row;
    double timeSum = 0.0;

    for (int i = 1; i < innuMeasure.rows(); i++)
    {
        timeSum += innuMeasure(i, 1);
    }

    while(getline(inputFile, row))
    {
        istringstream iss(row);
        string item;
        vector<string> items;

        while (getline(iss, item, '\t'))
        {
            item = trimWhitespace(item);
            items.push_back(item);
        }

        if (!BPMFlag)
        {
            cout << "music";
            bpm = stod(items[0].substr(4));
            cout << " bpm = " << bpm << "\n";
            BPMFlag = 1;

            pre_inst_R << default_right;
            pre_inst_L << default_left;

            innuMeasure.resize(1, 9);
            innuMeasure = MatrixXd::Zero(1, 9);

            // 초기 위치 스네어
            innuMeasure(0, 2) = 1.0;
            innuMeasure(0, 3) = 1.0;
        }
        else
        {
            innuMeasure.conservativeResize(innuMeasure.rows() + 1, innuMeasure.cols());
            for (int i = 0; i < 8; i++)
            {
                innuMeasure(innuMeasure.rows() - 1, i) = stod(items[i]);
            }

            // total time 누적
            totalTime += innuMeasure(innuMeasure.rows() - 1, 1);
            innuMeasure(innuMeasure.rows() - 1, 8) = totalTime;

            // timeSum 누적
            timeSum += innuMeasure(innuMeasure.rows() - 1, 1);

            // timeSum이 threshold를 넘으면 true 반환
            if (timeSum >= threshold)
            {
                std::cout << innuMeasure;
                std::cout << "\n ////////////// time sum : " << timeSum << "\n";

                return true;
            }
        }
    }
    return false;
}

void PathManager::innu_parseMeasure(MatrixXd &measureMatrix)
{
    VectorXd Measure_time = measureMatrix.col(8);
    VectorXd Measure_R = measureMatrix.col(2);
    VectorXd Measure_L = measureMatrix.col(3);

    pair<VectorXd, VectorXd> R = innu_parseOneArm(Measure_time, Measure_R, innu_state.row(0));
    pair<VectorXd, VectorXd> L = innu_parseOneArm(Measure_time, Measure_L, innu_state.row(1));

    // 데이터 저장
    innu_inst_i << R.first.block(1,0,9,1), L.first.block(1,0,9,1);
    innu_inst_f << R.first.block(11,0,9,1), L.first.block(11,0,9,1);

    innu_t_i_R = R.first(0);
    innu_t_i_L = L.first(0);
    innu_t_f_R = R.first(10);
    innu_t_f_L = L.first(10);

    innu_t1 = measureMatrix(0, 8);
    innu_t2 = measureMatrix(1, 8);

    innu_state.block(0,0,1,3) = R.second.transpose();
    innu_state.block(1,0,1,3) = L.second.transpose();

    std::cout << "\n ////////////// R\n";
    std::cout << innu_inst_i.block(0,0,9,1).transpose() << " -> " << innu_inst_f.block(0,0,9,1).transpose();
    std::cout << "\n /// ti -> tf : " << innu_t_i_R << " -> " << innu_t_f_R;
    
    std::cout << "\n ////////////// L\n";
    std::cout << innu_inst_i.block(9,0,9,1).transpose() << " -> " << innu_inst_f.block(9,0,9,1).transpose();
    std::cout << "\n /// ti -> tf : " << innu_t_i_L << " -> " << innu_t_f_L;

    std::cout << "\n ////////////// t1 -> t2\n";
    std::cout << innu_t1 << " -> " << innu_t2;
    std::cout << "\n ////////////// state\n";
    std::cout << innu_state;
    std::cout << "\n ////////////// \n";

    // 읽은 줄 삭제
    MatrixXd tmp_matrix(measureMatrix.rows() - 1, measureMatrix.cols());
    tmp_matrix = measureMatrix.block(1, 0, tmp_matrix.rows(), tmp_matrix.cols());
    measureMatrix.resize(tmp_matrix.rows(), tmp_matrix.cols());
    measureMatrix = tmp_matrix;
}

pair<VectorXd, VectorXd> PathManager::innu_parseOneArm(VectorXd t, VectorXd inst, VectorXd stateVector)
{
    map<int, int> instrument_mapping = {
    {1, 2}, {2, 5}, {3, 6}, {4, 8}, {5, 3}, {6, 1}, {7, 0}, {8, 7}, {11, 2}, {51, 2}, {61, 2}, {71, 2}, {81, 2}, {91, 2}};
    // S      FT      MT      HT      HH      R       RC      LC       S        S        S        S        S        S

    VectorXd inst_i = VectorXd::Zero(9), inst_f = VectorXd::Zero(9);
    VectorXd outputVector = VectorXd::Zero(20);

    VectorXd nextStateVector;

    bool detectHit = false;
    double detectTime = 0, t_i, t_f;
    int detectInst = 0, instNum_i, instNum_f;
    int preState, nextState;
    double threshold = 1.2;
    
    // 타격 감지
    for (int i = 1; i < t.rows(); i++)
    {
        if (threshold < t(i) - t(0))
        {
            break;
        }

        if (inst(i) != 0)
        {
            detectHit = true;
            detectTime = t(i);
            detectInst = inst(i);

            break;
        }
    }

    // inst
    preState = stateVector(2);

    if (inst(0) == 0)
    {
        // 타격으로 끝나지 않음
        if (preState == 2 || preState == 3)
        {
            // 궤적 생성 중
            nextState = preState;

            instNum_i = stateVector(1);
            instNum_f = detectInst;

            t_i = stateVector(0);
            t_f = detectTime;
        }
        else
        {
            if (detectHit)
            {
                // 다음 타격 감지
                nextState = 2;

                instNum_i = stateVector(1);
                instNum_f = detectInst;

                t_i = t(0);
                t_f = detectTime;
            }
            else
            {
                // 다음 타격 감지 못함
                nextState = 0;

                instNum_i = stateVector(1);
                instNum_f = stateVector(1);

                t_i = t(0);
                t_f = t(1);
            }
        }
    }
    else
    {
        // 타격으로 끝남
        if (detectHit)
        {
            // 다음 타격 감지
            nextState = 3;

            instNum_i = inst(0);
            instNum_f = detectInst;

            t_i = t(0);
            t_f = detectTime;
        }
        else
        {
            // 다음 타격 감지 못함
            nextState = 1;

            instNum_i = inst(0);
            instNum_f = inst(0);
            
            t_i = t(0);
            t_f = t(1);
        }
    }

    inst_i(instrument_mapping[instNum_i]) = 1.0;
    inst_f(instrument_mapping[instNum_f]) = 1.0;
    outputVector << t_i, inst_i, t_f, inst_f;

    nextStateVector.resize(3);
    nextStateVector << t_i, instNum_i, nextState;

    return std::make_pair(outputVector, nextStateVector);
}