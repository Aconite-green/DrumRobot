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
#include "Motor.hpp"
#include "CommandParser.hpp"

using namespace std;

/**
 * @class CanManager
 * @brief 관리 CAN 통신 및 모터 제어를 위한 클래스.
 *
 * CAN 통신을 통해 모터와의 연결을 관리하고, 데이터 송수신 및 명령 실행을 담당합니다.
 */
class CanManager
{
public:
    static const int ERR_SOCKET_CREATE_FAILURE = -1; ///< 소켓 생성 실패 오류 코드.
    static const int ERR_SOCKET_CONFIGURE_FAILURE = -2; ///< 소켓 설정 실패 오류 코드.

    /**
     * @brief CanManager 생성자.
     * @param motorsRef 모터 객체의 참조를 매핑하는 맵.
     */
    CanManager(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);
    
    /**
     * @brief CanManager 소멸자.
     */
    ~CanManager();

    void initializeCAN(); ///< CAN 포트 초기화.
    void restartCanPorts(); ///< CAN 포트 재시작.
    void setSocketsTimeout(int sec, int usec); ///< 소켓 타임아웃 설정.
    void checkCanPortsStatus(); ///< CAN 포트 상태 체크.
    
    void setMotorsSocket(); ///< 모터 소켓 설정.
    bool checkConnection(std::shared_ptr<GenericMotor> motor); ///< 모터 연결 체크.
    bool checkAllMotors(); ///< 모든 모터 연결 체크.

    bool sendAndRecv(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame); ///< 데이터 송수신.
    bool sendFromBuff(std::shared_ptr<GenericMotor> &motor); ///< 버퍼에서 데이터 송신.
    bool recvToBuff(std::shared_ptr<GenericMotor> &motor, int readCount); ///< 버퍼로 데이터 수신.
    bool txFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame); ///< 데이터 프레임 송신.
    bool rxFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame); ///< 데이터 프레임 수신.
    
    void readFramesFromAllSockets(); ///< 모든 소켓에서 프레임 읽기.
    void distributeFramesToMotors(); ///< 모터로 프레임 분배.

    void clearReadBuffers(); ///< 읽기 버퍼 클리어.

    std::map<std::string, int> sockets; ///< 소켓 매핑.
    std::map<std::string, bool> isConnected; ///< 연결 상태 매핑.
    int maxonCnt=0; ///< Maxon 모터 카운트.

private:
    std::vector<std::string> ifnames; ///< 인터페이스 이름.
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors; ///< 모터 매핑.

    TMotorCommandParser tmotorcmd; ///< T 모터 명령 파서.
    MaxonCommandParser maxoncmd; ///< Maxon 명령 파서.

    std::map<int, std::vector<can_frame>> tempFrames; ///< 임시 프레임.

    bool getCanPortStatus(const char *port); ///< CAN 포트 상태 가져오기.
    void activateCanPort(const char *port); ///< CAN 포트 활성화.
    void list_and_activate_available_can_ports(); ///< 사용 가능한 CAN 포트 활성화.
    void deactivateCanPort(const char *port); ///< CAN 포트 비활성화.

    int createSocket(const std::string &ifname); ///< 소켓 생성.
    int setSocketTimeout(int socket, int sec, int usec); ///< 소켓 타임아웃 설정.

    void clearCanBuffer(int canSocket); ///< CAN 버퍼 클리어.
};

#endif // CAN_SOCKET_UTILS_H
