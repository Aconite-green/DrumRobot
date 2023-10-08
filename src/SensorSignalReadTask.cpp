
#include "../include/SensorSignalReadTask.hpp"


SensorSignalReadTask::SensorSignalReadTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, std::atomic<bool> &paused,std::atomic<bool> &stop)
: tmotors(tmotors), paused(paused), stop(stop)
{}
void SensorSignalReadTask::operator()(SharedBuffer<int> &buffer)
{
    
}
