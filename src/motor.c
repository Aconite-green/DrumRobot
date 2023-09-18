// motor.c 파일
#include "../include/motor.h" // Include header file

MotorContainer container;

MotorContainer *get_motor_container()
{
    return &container;
}

// Include this for malloc and free

MotorLimits *get_motor_limits(const char *motortype)
{
    MotorLimits *limits = malloc(sizeof(MotorLimits));

    // Handle the case when malloc fails
    if (limits == NULL)
    {
        printf("Error: Memory allocation failed!\n");
        return NULL;
    }

    limits->P_MIN = -12.5;
    limits->P_MAX = 12.5;
    limits->Kp_MIN = 0;
    limits->Kp_MAX = 500;
    limits->Kd_MIN = 0;
    limits->Kd_MAX = 5;

    if (strcmp(motortype, "AK10_9") == 0)
    {
        limits->V_MIN = -50;
        limits->V_MAX = 50;
        limits->T_MIN = -65;
        limits->T_MAX = 65;
    }
    else if (strcmp(motortype, "AK70_10") == 0)
    {
        limits->V_MIN = -50;
        limits->V_MAX = 50;
        limits->T_MIN = -25;
        limits->T_MAX = 25;
    }
    else if (strcmp(motortype, "AK60_6") == 0)
    {
        limits->V_MIN = -45;
        limits->V_MAX = 45;
        limits->T_MIN = -15;
        limits->T_MAX = 15;
    }
    else if (strcmp(motortype, "AK80_6") == 0)
    {
        limits->V_MIN = -76;
        limits->V_MAX = 76;
        limits->T_MIN = -12;
        limits->T_MAX = 12;
    }
    else if (strcmp(motortype, "AK80_9") == 0)
    {
        limits->V_MIN = -50;
        limits->V_MAX = 50;
        limits->T_MIN = -18;
        limits->T_MAX = 18;
    }
    else if (strcmp(motortype, "AK80_80") == 0 || strcmp(motortype, "AK80_64") == 0)
    {
        limits->V_MIN = -8;
        limits->V_MAX = 8;
        limits->T_MIN = -144;
        limits->T_MAX = 144;
    }
    else if (strcmp(motortype, "AK80_8") == 0)
    {
        limits->V_MIN = -37.5;
        limits->V_MAX = 37.5;
        limits->T_MIN = -32;
        limits->T_MAX = 32;
    }
    else
    {
        printf("Error: Invalid motor type entered!\n");
        free(limits); // Free the memory when an error occurs
        return NULL;
    }

    return limits;
}

MotorLimits *get_motor_limits_by_id(MotorContainer *container, int id)
{
    for (int i = 0; i < container->num_motors; i++)
    {
        if (container->motors[i].id == id)
        {
            return &container->motors[i].limits;
        }
    }
    return NULL;
}
