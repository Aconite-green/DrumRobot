// InitializeTask.cpp
#include "../include/InitializeTask.hpp"
#include "../include/SharedBuffer.hpp"
#include "../include/CanService.hpp"
#include <string>

InitializeTask::InitializeTask(std::map<std::string, std::shared_ptr<TMotor>>& tmotors,const std::map<std::string, int>& sockets)
: tmotors(tmotors), sockets(sockets)
{
}

void InitializeTask::operator()()
{
    // 초기화 작업을 여기서 수행. tmotors와 sockets 변수를 사용할 수 있습니다.
}
