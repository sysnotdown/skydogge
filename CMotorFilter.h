#pragma once

class CMotorFilter
{
    public:
        CMotorFilter() {
            _old_val=0;
        }
        float Filter(float thrust);
    protected:
        float _old_val;
};

