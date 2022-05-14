#pragma once

#include <iostream>
#include <cstring>
#ifdef ERROR
#undef ERROR
#endif
typedef enum {
    ERROR = 0,
    WARN = 1,
    INFO = 2,
    NOTICE = 3
} LogLevel;

static LogLevel s_logLevel = NOTICE;

#define FILEV(x) (strrchr((x), '/') ? strrchr((x), '/') + 1 : (x))
#define LOG(log_level)               \
    if (log_level <= s_logLevel) \
    std::cout << "[" << #log_level << "] " << "[" << FILEV(__FILE__) << "] [" << __LINE__ << "] [" << __FUNCTION__ << "] "
void setLogLevel(LogLevel verbose);