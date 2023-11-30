#include "../include/RecieveLoopTask.hpp"

RecieveLoopTask::RecieveLoopTask(SystemState &systemStateRef, CanSocketUtils &canUtilsRef)
    : systemState(systemStateRef), canUtils(canUtilsRef)
{
}

void RecieveLoopTask::operator()()
{
    while (systemState.main != Main::Shutdown)
    {
        while(systemState.runMode ==RunMode::Running){
            //읽기 로직 수행
        }
    }
}
