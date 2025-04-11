#include "CMotorFilter.h"

//输入一个新的值，给出合理的过滤值。
float CMotorFilter::Filter(float thrust)
{
    if(thrust<0) thrust=0;
    if(thrust>1.0) thrust=1.0;

    if(thrust==0.0) {
        _old_val=0.0;
        return 0.0;
    }

    //采用不对称滤波是专为有刹车的四合一电调设计，希望减小刹车效应。
    //推高马达存在自然惯性，有延迟，减小马力在四合一电调则比较直接迅速。
    if(thrust < _old_val) {
        _old_val= (thrust+_old_val*31.0)/32.0;  //减力滤波大，限制强因为esc本身有刹车。
        return _old_val;
    }else{
        _old_val= (thrust+_old_val*7.0)/8.0;  //增力滤波小，因为有惯性。
        return _old_val;
    }

}