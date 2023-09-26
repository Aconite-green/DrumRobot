#include "realtime_config.h"


void set_process_priority()
{
    // Set process to max priority for reliable real-time execution
    int result = setpriority(PRIO_PROCESS, 0, MAX_PRIORITY);
    if (result == -1)
    {
        perror("setpriority failed");
        exit(-1);
    }
}

void configure_scheduler()
{
    // Set up a real-time, absolute deadline scheduler
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHEDULING_POLICY);
    int result = sched_setscheduler(0, SCHEDULING_POLICY, &param);
    if (result == -1)
    {
        perror("sched_setscheduler failed");
        exit(-1);
    }
}

void configure_realtime()
{
    set_process_priority();
    configure_scheduler();
}
