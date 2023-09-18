#ifndef ERROR_HANDLING_H
#define ERROR_HANDLING_H

#include <stdio.h>
#include <stdlib.h>
#include <errno.h> 

// Error message and exit program
void error_exit(const char *message);

// Check the error from the socket communication
void check_socket_error(int result, const char *function_name);

#endif /* ERROR_HANDLING_H */
