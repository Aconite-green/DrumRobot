#include "../include/error_handling.h"

// Error message and exit program
void error_exit(const char *message) {
    perror(message);
    exit(EXIT_FAILURE);
}

// Check the error from the socket communication
void check_socket_error(int result, const char *function_name) {
    if (result < 0) {
        char error_message[256];
        sprintf(error_message, "Socket error in function %s", function_name);
        error_exit(error_message);
    }
}
