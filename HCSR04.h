#ifndef HCSR04_h
#define HCSR04_h
#include "Arduino.h"

 
const int MAX_RANGE = 300;
const int MIN_RANGE = 4;

class HCSR04
{
    public:
        HCSR04(byte trigger, byte recv);
        void measure();
        float get_distance();
        unsigned long get_duration();
    private:
        byte trigger_pin;
        byte recv_pin;
        unsigned long duration;
        float distance;
};
#endif 
