#ifndef CAN_SERVICE_H
#define CAN_SERVICE_H

#include <linux/can.h>
#include "MotorInterface.hpp"

class CanService {
public:
    void enterControlMode(MotorInterface& motor, int can_id) {
        struct can_frame frame;
        motor.fillCanFrameForControlMode(&frame, can_id);
        // 여기서 실제 CAN 통신 코드를 작성할 수 있습니다.
    }

    void setToZero(MotorInterface& motor, int can_id) {
        struct can_frame frame;
        motor.fillCanFrameForZeroing(&frame, can_id);
        // 여기서 실제 CAN 통신 코드를 작성할 수 있습니다.
    }


    // ... 이외에도 공통적인 작업을 정의할 수 있습니다 ...
};

#endif // CAN_SERVICE_H
