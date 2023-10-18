#include "CommandParser.hpp"

// Tmotor parser

void TMotorCommandParser::parseSendCommand(TMotor &motor, struct can_frame *frame, int canId, int dlc, float p_des, float v_des, float kp, float kd, float t_ff)
{

    // 기존 변수를 계산
    p_des = fminf(fmaxf(motor.pMin, p_des), motor.pMax);
    v_des = fminf(fmaxf(motor.vMin, v_des), motor.vMax);
    kp = fminf(fmaxf(motor.kpMin, kp), motor.kpMax);
    kd = fminf(fmaxf(motor.kdMin, kd), motor.kdMax);
    t_ff = fminf(fmaxf(motor.tMin, t_ff), motor.tMax); // tff를 tFf로 변경, 클래스에 따라 적절히 수정

    // 계산된 변수를 이용하여 unsigned int로 변환
    int p_int = float_to_uint(p_des, motor.pMin, motor.pMax, 16); // motor.P_MIN 대신 motor.pMin 사용
    int v_int = float_to_uint(v_des, motor.vMin, motor.vMax, 12); // motor.V_MIN 대신 motor.vMin 사용
    int kp_int = float_to_uint(kp, motor.kpMin, motor.kpMax, 12); // motor.Kp_MIN 대신 motor.kpMin 사용
    int kd_int = float_to_uint(kd, motor.kdMin, motor.kdMax, 12); // motor.Kd_MIN 대신 motor.kdMin 사용
    int t_int = float_to_uint(t_ff, motor.tMin, motor.tMax, 12);  // motor.T_MIN 대신 motor.tMin 사용

    // Set CAN frame id and data length code
    frame->can_id = canId & CAN_SFF_MASK; // Replace YOUR_CAN_ID with the appropriate id
    frame->can_dlc = dlc;                 // Data Length Code is set to maximum allowed length

    /// pack ints into the can buffer ///
    frame->data[0] = p_int >> 8;                           // Position 8 higher
    frame->data[1] = p_int & 0xFF;                         // Position 8 lower
    frame->data[2] = v_int >> 4;                           // Speed 8 higher
    frame->data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); // Speed 4 bit lower KP 4bit higher
    frame->data[4] = kp_int & 0xFF;                        // KP 8 bit lower
    frame->data[5] = kd_int >> 4;                          // Kd 8 bit higher
    frame->data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); // KP 4 bit lower torque 4 bit higher
    frame->data[7] = t_int & 0xff;                         // torque 4 bit lower
}

std::tuple<int, float, float, float> TMotorCommandParser::parseRecieveCommand(TMotor &motor, struct can_frame *frame)
{
    int id;
    float position, speed, torque;
    /// unpack ints from can buffer ///
    id = frame->data[0];
    int p_int = (frame->data[1] << 8) | frame->data[2];
    int v_int = (frame->data[3] << 4) | (frame->data[4] >> 4);
    int i_int = ((frame->data[4] & 0xF) << 8) | frame->data[5];

    /// convert ints to floats ///
    position = uint_to_float(p_int, motor.pMin, motor.pMax, 16);
    speed = uint_to_float(v_int, motor.vMin, motor.vMax, 12);
    torque = uint_to_float(i_int, motor.tMin, motor.tMax, 12);

    return std::make_tuple(id, position, speed, torque);
}
int TMotorCommandParser::float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    if (x < x_min)
        x = x_min;
    else if (x > x_max)
        x = x_max;
    return (int)((x - x_min) * ((float)((1 << bits) / span)));
};

float TMotorCommandParser::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

//////////////////////////////////////////////////////
// Maxon Parser definition
/////////////////////////////////////////////////////
void MaxonCommandParser::parseSendCommand(MaxonMotor &motor, struct can_frame *frame, int p_des)
{
    unsigned char posByte0 = p_des & 0xFF;         // 하위 8비트
    unsigned char posByte1 = (p_des >> 8) & 0xFF;  // 다음 8비트
    unsigned char posByte2 = (p_des >> 16) & 0xFF; // 다음 8비트
    unsigned char posByte3 = (p_des >> 24) & 0xFF; // 최상위 8비트

    // Set CAN frame id and data length code
    frame->can_id = motor.pdoIds[1]; // Replace YOUR_CAN_ID with the appropriate id
    frame->can_dlc = 4;              // Data Length Code is set to maximum allowed length

    /// pack ints into the can buffer ///
    frame->data[0] = posByte0;
    frame->data[1] = posByte1;
    frame->data[2] = posByte2;
    frame->data[3] = posByte3;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

std::tuple<int, int> MaxonCommandParser::parseRecieveCommand(struct can_frame *frame)
{
    int id = frame->can_id;

    int currentPosition = 0;
    currentPosition |= frame->data[2];
    currentPosition |= (frame->data[3] << 8);
    currentPosition |= (frame->data[4] << 16);
    currentPosition |= (frame->data[5] << 24);

    return std::make_tuple(id, currentPosition);
}

void MaxonCommandParser::makeSync(struct can_frame *frame)
{
    frame->can_id = 0x80; // Replace YOUR_CAN_ID with the appropriate id
    frame->can_dlc = 1;   // Data Length Code is set to maximum allowed length

    /// pack ints into the can buffer ///
    frame->data[0] = 0x00;
}
