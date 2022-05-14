#include <iostream>
#include "Logger.h"

void setLogLevel(LogLevel verbose)
{
    s_logLevel = verbose;

    std::cout << "log level:" << s_logLevel << std::endl;
}
