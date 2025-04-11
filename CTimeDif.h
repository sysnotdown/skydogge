#pragma once
#include "pico/stdlib.h"

class CTimeDif
{
    public:
    CTimeDif() {
        sum=0;
        counter=0;
    }
    void Add(int64_t dif) {
        sum+=dif;
        counter++;
    }
    float getmean() {
        return float(sum)/counter;
    }

    protected:
    int64_t sum;
    int counter;
};