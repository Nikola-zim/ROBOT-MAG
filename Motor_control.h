#include "Arduino.h"
#include <avr/interrupt.h>

class DC_Motor{
  public:
    DC_Motor(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM);  
    void rotate_two_ways (uint8_t power);
  private:
    uint8_t _L_EN;
    uint8_t _R_EN;
    uint8_t _L_PWM;
    uint8_t _R_PWM;      
    uint8_t power;
    uint32_t _myTimer0=0;
    uint32_t _myTimer1=0;
};
//
//class DCM_Monitor{
//  public:
//    DCM_Monitor(uint8_t M1_ENC1, uint8_t M1_ENC2);
//    void print_coordinates();
//    void start_DCM_monitoring();
//    volatile long get_pos();
//    volatile float get_rad_speed();
//    void EncTick();
//    
//  private:    
//    //ДОБАВИТЬ _ !!!
//    volatile long pos;
//    volatile long posAT2;
//    volatile float rad_speed;
//    volatile long timeAT;
//    volatile byte lastState;
//    volatile const int8_t increment[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
//    volatile byte state;
//    volatile int k;
//    volatile boolean _flag;
//    uint8_t _M1_ENC1;
//    uint8_t _M1_ENC2;
//  };
