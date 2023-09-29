#include <stdio.h>
#include "../include/CanService.hpp"
#include "../include/CanSocketUtils.hpp"
#include "../include/CommandParser.hpp"
#include "../include/ErrorHandle.hpp"
#include "../include/Motor.hpp"
#include "../include/MotorInterface.hpp"

int main()
{

    if (geteuid() != 0)
    {
        fprintf(stderr, "이 프로그램은 관리자 권한으로 실행되어야 합니다.\n");
        exit(EXIT_FAILURE);
    }

    
}
