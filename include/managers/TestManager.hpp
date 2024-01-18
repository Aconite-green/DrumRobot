#include <stdio.h>
#include "../include/managers/CanManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/TaskUtility.hpp"
#include "../include/usbio/Global.hpp"
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

using namespace std;

class TestManager
{
public:
    TestManager(CanManager &canManagerRef, queue<can_frame> &sendBufferRef, queue<can_frame> &recieveBufferRef,
                std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    void motorInitialize(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    void run();
    void TestArr(double t, int cycles, int type, int LnR, double amp[]);

private:
    CanManager &canManager;
    queue<can_frame> &sendBuffer;
    queue<can_frame> &recieveBuffer;
    TMotorCommandParser TParser;
    MaxonCommandParser MParser;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    // 각 관절에 해당하는 열
    map<string, int> motor_mapping = {
        {"waist", 0}, {"R_arm1", 1}, {"L_arm1", 2}, {"R_arm2", 3}, {"R_arm3", 4}, {"L_arm2", 5}, {"L_arm3", 6}, {"R_wrist", 7}, {"L_wrist", 8}};
    // 각 열에 해당하는 관절방향
    map<int, int> motor_dir = { // 1 : CW , -1 : CCW
        {0, 1},
        {1, 1},
        {2, 1},
        {3, 1},
        {4, 1},
        {5, 1},
        {6, 1},
        {7, 1},
        {8, 1}};

    map<int, int> motor_Kp = { // Desired Kp Gain
        {0, 200},
        {1, 200},
        {2, 200},
        {3, 200},
        {4, 200},
        {5, 200},
        {6, 200},
        {7, 200},
        {8, 200}};

    void ApplyDir();
    void getMotorPos();
    void wristarr(vector<vector<double>> &T, int time, int LnR, double amp, int kp[]);
    void waistarr(vector<vector<double>> &T, int time, double amp, int kp[]);
    void arm1arr(vector<vector<double>> &T, int time, int LnR, double amp, int kp[]);
    void arm2arr(vector<vector<double>> &T, int time, int LnR, double amp, int kp[]);
    void arm3arr(vector<vector<double>> &T, int time, int LnR, double amp, int kp[]);
    void writeToSocket(const std::map<std::string, int> &sockets);
    void SendLoop();

    vector<double> c_MotorAngle = {0, -M_PI / 2, M_PI / 2, M_PI / 4, -M_PI / 2.4, -M_PI / 4, -M_PI / 2.4, 0, 0};
};