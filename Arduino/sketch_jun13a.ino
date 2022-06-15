#include <avr/interrupt.h>

#include "Motor_control.h"


//LEFT DCM (правый DC-мотор)
uint8_t L_PWM = 8;
uint8_t R_PWM = 9;
uint8_t L_EN = 6;
uint8_t R_EN = 7;
uint8_t M1_ENC1 = 2; //0 interrupt
uint8_t M1_ENC2 = 3; //1 interrupt
//Создадим класс для контроля левого двигателя
DC_Motor Left_motor(L_EN, R_EN, L_PWM, R_PWM);

//RIGHT DCM 
const uint8_t L_PWM2 = 4;
const uint8_t R_PWM2 = 5;
const uint8_t L_EN2 = 12;
const uint8_t R_EN2 = 11;
uint8_t DCM2_ENC1 = 20; //2 interrupt
uint8_t DCM2_ENC2 = 21; //3 interrupt
//Создадим класс для контроля правого двигателя
DC_Motor Right_motor(L_EN2, R_EN2, L_PWM2, R_PWM2);

uint8_t power; //Мощьность для rotate_two_ways

//Для энкодера Left DCM (левый DC-мотор)
volatile long position_Left_DCM = 0;
volatile long previous_position_LDCM = 0;
volatile float rad_speed_Left_DCM = 0;
volatile long timeAT = 0;
volatile byte lastState = 0;
volatile const int8_t increment[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
volatile byte state;
volatile int k;
volatile boolean flag_Left_DCM = 0;
float pos_rad_Left_DCM;//для скорости

//Для энкодера RIGHT
volatile long position_Right_DCM = 0;
volatile long previous_position_RDCM = 0;
volatile float rad_speed_Right_DCM = 0;
volatile long timeATR = 0;
volatile byte lastStateR = 0;
volatile const int8_t incrementR[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
volatile byte stateR;
volatile int kR;
volatile boolean flag_Right_DCM = 0;
float pos_rad_Right_DCM;//для скорости

//Для корректировки энкодера
uint32_t myTimer0 = 0;

void setup() {        
    power = 255; //Мощьность для rotate_two_ways          
    //Left энкодер
    attachInterrupt(0, EncTick_Left_DCM, CHANGE);
    attachInterrupt(1, EncTick_Left_DCM, CHANGE);
    //Right энкодер
    attachInterrupt(2, EncTick_Right_DCM, CHANGE);
    attachInterrupt(3, EncTick_Right_DCM, CHANGE);
    Serial.begin(9600);
    Serial.println("Right_DCM, Left_DCM");
}

void loop() {
  Left_motor.rotate_two_ways (power);
  Right_motor.rotate_two_ways (power);
  Serial.print(rad_speed_Right_DCM);
  Serial.print(' ');
  Serial.println(rad_speed_Left_DCM);
  
  //Корректировка энкодера для околонулевой скорости
  if(millis() - myTimer0 > 10){
    if (!flag_Left_DCM){
      rad_speed_Left_DCM = 0; 
      }
    if (!flag_Right_DCM){
      rad_speed_Right_DCM = 0; 
      }  
    flag_Left_DCM = false;
    flag_Right_DCM = false;
    myTimer0 = millis();
    }
} 

//Speed&rad Left DCM
void EncTick_Left_DCM(){
  state = digitalRead(M1_ENC1) | (digitalRead(M1_ENC2) << 1);
  if (state != lastState) {
  position_Left_DCM += increment[state | (lastState << 2)];
  lastState = state;
  flag_Left_DCM = true;
  k++;
    if(k>=25){
    rad_speed_Left_DCM = (float) 1640.52*(position_Left_DCM- previous_position_LDCM)/(micros()-timeAT); 
    previous_position_LDCM = position_Left_DCM;
    timeAT = micros();
    k=0;
    }
  }
}

//Speed&rad Right DCM
void EncTick_Right_DCM(){
  stateR = digitalRead(DCM2_ENC1) | (digitalRead(DCM2_ENC2) << 1);
  if (stateR != lastStateR) {
  position_Right_DCM += increment[stateR | (lastStateR << 2)];
  lastStateR = stateR;
  flag_Right_DCM = true;
  kR++;
    if(kR >= 25){
    rad_speed_Right_DCM = (float) 1640.52*(position_Right_DCM - previous_position_RDCM)/(micros()-timeATR); 
    previous_position_RDCM = position_Right_DCM;
    timeATR = micros();
    kR = 0;
    }
  }
}

//
