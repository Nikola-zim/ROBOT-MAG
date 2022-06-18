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

void DC_Motor::set_voltage(int power){
  power = constrain(power, -255, 255);
  if(power > 0){
      analogWrite(_L_PWM, power);  
      analogWrite(_R_PWM, 0); 
      analogWrite(_L_EN, power);
      analogWrite(_R_EN, power); 
    }
   if(power < 0){
      analogWrite(_L_PWM, 0);  
      analogWrite(_R_PWM, -power); 
      analogWrite(_L_EN, -power);
      analogWrite(_R_EN, -power); 
    } 
   if(power == 0){
      analogWrite(_L_PWM, 0);  
      analogWrite(_R_PWM, 0); 
      analogWrite(_L_EN, power);
      analogWrite(_R_EN, power); 
    } 
  }
