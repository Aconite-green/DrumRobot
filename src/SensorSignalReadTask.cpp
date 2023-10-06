
#include "../include/SensorSignalReadTask.hpp"


SensorSignalReadTask::SensorSignalReadTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, std::atomic<bool> &paused)
: tmotors(tmotors), paused(paused)
{}
void SensorSignalReadTask::operator()(SharedBuffer<int> &buffer)
{
    
}
