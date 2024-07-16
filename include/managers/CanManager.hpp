#ifndef CAN_SOCKET_UTILS_H
#define CAN_SOCKET_UTILS_H

#include <linux/can.h>
#include <math.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <bits/types.h>
#include <linux/can/raw.h>
#include <sys/time.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <map>
#include <queue>
#include <memory>
#include <gpiod.h>
#include <chrono>
#include <fstream>

#define GPIO_CHIP "/dev/gpiochip0"
#define GPIO_OUTPUT_LINES 8 // 사용할 GPIO 핀 개수
#include "Motor.hpp"
#include "CommandParser.hpp"

// serial to Arduino
//#include <sys/ioctl.h> // For TIOCINQ
//#include <fcntl.h>
//#include <unistd.h>
//#include <termios.h>
#include <cstring> // For memset
#include <errno.h> // For errno

#define SERIAL_PORT "/dev/ttyACM1"
#define BAUD_RATE B1000000

#define POS_LOOP 0
#define POS_SPD_LOOP 1

using namespace std;

class CanManager
{
public:
    static const int ERR_SOCKET_CREATE_FAILURE = -1;
    static const int ERR_SOCKET_CONFIGURE_FAILURE = -2;

    CanManager(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    ~CanManager();

    void initializeCAN();

    void setSocketsTimeout(int sec, int usec);
    void flushCanBuffer(int socket);
    void resetCanFilter(int socket);
    void checkCanPortsStatus();

    void setMotorsSocket();

    bool sendAndRecv(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);

    bool sendFromBuff(std::shared_ptr<GenericMotor> &motor);

    bool sendMotorFrame(std::shared_ptr<GenericMotor> motor);

    bool checkMaxon();

    bool checkAllMotors_Fixed();

    bool sendForCheck_Fixed(std::shared_ptr<GenericMotor> motor);

    bool recvToBuff(std::shared_ptr<GenericMotor> &motor, int readCount);

    bool txFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);

    bool rxFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);

    void readFramesFromAllSockets();

    bool distributeFramesToMotors(bool setlimit);

    void clearReadBuffers();

    void setSocketNonBlock();
    void setSocketBlock();
    std::map<std::string, int> sockets;      ///< 모터와 통신하는 소켓의 맵.
    std::map<std::string, bool> isConnected; ///< 모터의 연결 상태를 나타내는 맵.
    int maxonCnt = 0;
    // Functions for Thread Case

    void setCANFrame();
    bool safetyCheck_T(std::shared_ptr<GenericMotor> &motor, std::tuple<int, float, float, float, int8_t, int8_t> parsedData);
    bool safetyCheck_M(std::shared_ptr<GenericMotor> &motor, std::tuple<int, float, float, unsigned char> parsedData);

    vector<vector<float>> Input_pos;
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

    std::vector<std::string> ifnames;
    int errorCnt = 0;

    int serial_fd;
    int setup_serial_port();
    void send_char_to_serial(int fd, char data);
    std::string read_char_from_serial(int fd);

    /*save csv file*/
    std::chrono::high_resolution_clock::time_point start;  
    const std::string basePath = "../../READ/";  // 기본 경로
    // 변수를 CSV 파일에 한 줄씩 저장하는 함수
    void appendToCSV_CM(const std::string& filename, float fixed_position, float current_position);
    void appendToCSV_CAN(const std::string& filename, can_frame& c_frame);
    void appendToCSV_time(const std::string& filename);

    // tMotor 제어 모드 결정
    int tMotor_control_mode = POS_SPD_LOOP;

private:

    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    TMotorServoCommandParser tservocmd;

    std::map<int, std::vector<can_frame>> tempFrames;

    bool getCanPortStatus(const char *port);
    void activateCanPort(const char *port);
    void deactivateCanPort(const char *port);
    void deactivateAllCanPorts();
    void list_and_activate_available_can_ports();

    int createSocket(const std::string &ifname);
    int setSocketTimeout(int socket, int sec, int usec);
    void clearCanBuffer(int canSocket);
};

#endif // CAN_SOCKET_UTILS_H
