#pragma once
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include "pico/stdlib.h"
#include "memory.h"
//暂时放这里以后移动到气压计那边
class press_info{
    public:
    uint32_t tmark;
    float press;
    float temprature;

    press_info() {
        press=0;
        temprature=0;
        tmark=0;
    }

    press_info(const float v) {
        press=v;
        temprature=0;
        tmark=0;
    }

    press_info& 
    operator+=(const press_info &o)
    {
        press+= o.press;
        temprature+= o.temprature;     
        return *this;
    }
 
    press_info
    operator+(const press_info &rhs)
    {
        press_info sum;
        sum = *this;
        sum += rhs;
        return sum;
    }

    press_info
    operator/(const int n)
    {
        press_info sum;
        sum= *this;
        sum.press/=n;
        sum.temprature/=n;
        return sum;
    }

    press_info& operator=(const float v)
    {
        press=v;
        temprature=0;
        tmark=0;
        return *this;
    } 

    press_info& operator=(const press_info& o)
    {
        press=o.press;
        temprature=o.temprature;
        tmark=o.tmark;
        return *this;
    }     
};