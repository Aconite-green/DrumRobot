#include "SharedBuffer.hpp"
#include "MotorSignalSendTask.hpp"
#include "MotorResponseReadTask.hpp"
#include "MotorPathTask.hpp"
#include "MotorSignalSendTask.hpp"
#include "SharedBuffer.hpp"
#include "MotorResponseReadTask.hpp"
#include "SensorSignalReadTask.hpp"
#include "ActivateControlTask.hpp"
#include "DeactivateControlTask.hpp"
#include <atomic>
#include <thread>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <memory>
#include <map>

class ThreadLoopTask
{
public:
    ThreadLoopTask(ActivateControlTask &activateTask, 
                 DeactivateControlTask &deactivateTask,
                 MotorPathTask &pathTask,
                 MotorSignalSendTask &sendTask,
                 MotorResponseReadTask &readTask,
                 SharedBuffer<can_frame> &sendBuffer, 
                 SharedBuffer<can_frame> &receiveBuffer,
                 std::atomic<bool> &stop);

    void operator()();

private:
    ActivateControlTask &activateTask;
    DeactivateControlTask &deactivateTask;
    MotorPathTask &pathTask;
    MotorSignalSendTask &sendTask;
    MotorResponseReadTask &readTask;
    SharedBuffer<can_frame> &sendBuffer;
    SharedBuffer<can_frame> &receiveBuffer;
    std::atomic<bool> &stop;
};
