
#ifndef _POWER_INFO_H
#define _POWER_INFO_H
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include "pico/stdlib.h"
#include "memory.h"

class power_info
{
    public:
    uint32_t tmark;
    float voltage;
};

#endif
