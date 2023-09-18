#ifndef REALTIME_CONFIG_H
#define REALTIME_CONFIG_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/resource.h>
#include <sched.h>

#define MAX_PRIORITY -20
#define SCHEDULING_POLICY SCHED_RR

void set_process_priority();
void configure_realtime();

#endif // REALTIME_CONFIG_H
 
