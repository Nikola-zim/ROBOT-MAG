#include "Motor_control.h"


//Определим методы класса DC_Motor
//Конструктор класса
DC_Motor::DC_Motor(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM){
	  //установка управляющих пинов
	  pinMode(L_EN, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_PWM, OUTPUT);     
    pinMode(R_PWM, OUTPUT); 
    _L_EN = L_EN;
    _R_EN = R_EN;
    _L_PWM = L_PWM;
    _R_PWM = R_PWM;
    //Таймеры для движения в разные стороны
    uint32_t _myTimer0 = 0;
	  uint32_t _myTimer1 = 0;
}

void DC_Motor::rotate_two_ways(uint8_t power){
    if (millis()- _myTimer0 >= 5000 && millis()- _myTimer1 < 10000){
	    _myTimer0 = millis();
	    // движение против часовой стрелке
	    analogWrite(_L_PWM, 0);  
	    analogWrite(_R_PWM, 255); 
	    analogWrite(_L_EN, power);
	    analogWrite(_R_EN, power); 
    }
  	if (millis()-_myTimer1 >= 10000){
	    _myTimer1 = millis();
	    _myTimer0 = millis();
	    // движение по часовой стрелке
	    analogWrite(_L_PWM, 255);  
	    analogWrite(_R_PWM, 0); 
	    analogWrite(_L_EN, power);
	    analogWrite(_R_EN, power);   
    }	
}


//Определим методы класса DC_Monitor
//Конструктор класса
//DCM_Monitor::DCM_Monitor(uint8_t M1_ENC1, uint8_t M1_ENC2){
//    //ДОБАВИТЬ _ !!!
//    pos = 0;
//    posAT2 = 0;
//    rad_speed = 0;
//    timeAT = 0;
//    lastState = 0;
//    //increment = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
//    state;
//    k; 
//    _M1_ENC1 = M1_ENC1;
//    _M1_ENC2 = M1_ENC2;  
//    _flag = 0; 
//  }
//
//void DCM_Monitor::EncTick(){
//  state = digitalRead(_M1_ENC1) | (digitalRead(_M1_ENC2) << 1);
//  if (state != lastState) {
//  pos += increment[state | (lastState << 2)];
//  lastState = state;
//  _flag =true;
//  k++;
//    if(k>=20){
//    rad_speed = (float) 1640.52*(pos- posAT2)/(micros()-timeAT); //
//    posAT2 = pos;
//    timeAT = micros();
//    k=0;
//    }
//  }
//}
//
//void DCM_Monitor::start_DCM_monitoring(){
//    attachInterrupt(0, DCM_Monitor::EncTick, CHANGE);
//    attachInterrupt(1, DCM_Monitor::EncTick, CHANGE);  
//}
//
////Геттеры координаты и скорости
//volatile long DCM_Monitor::get_pos(){  
//  return pos;
//  }
//volatile float DCM_Monitor::get_rad_speed(){
//  return rad_speed;
//  }
