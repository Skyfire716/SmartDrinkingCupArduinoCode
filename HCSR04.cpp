#include "Arduino.h"
#include "HCSR04.h"

HCSR04::HCSR04(byte trigger, byte recv){
    trigger_pin = trigger;
    recv_pin = recv;
    pinMode(trigger_pin, OUTPUT);
    pinMode(recv_pin, INPUT);
    distance = 0;
    duration = 0;
}

void HCSR04::measure(){
    digitalWrite(trigger_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_pin, HIGH);
    delayMicroseconds(10);
    duration = pulseIn(recv_pin, HIGH);
    distance = duration * 0.034 / 2;
}

float HCSR04::get_distance(){
    return distance;
}

unsigned long HCSR04::get_duration(){
    return duration;
}
