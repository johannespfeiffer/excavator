#ifndef EXCAVATOR_PLATFORM_H
#define EXCAVATOR_PLATFORM_H

typedef enum {
    PLATFORM_STATUS_OK = 0,
    PLATFORM_STATUS_ERROR = 1,
} platform_status_t;

platform_status_t platform_init(void);

#endif
