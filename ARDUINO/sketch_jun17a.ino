#include <avr/interrupt.h>

#include "Motor_control.h"
#include "GyverPID.h"
#include "Parser.h"       // библиотека парсера
#include "AsyncStream.h"  // асинхронное чтение сериал
#include "PIDtuner2.h"
#include "GyverFilters.h"
AsyncStream<50> serial(&Serial, ';');   // указываем обработчик и стоп символ


//LEFT DCM (правый DC-мотор)
uint8_t L_PWM = 8;
uint8_t R_PWM = 9;
uint8_t L_EN = 6;
uint8_t R_EN = 7;
uint8_t M1_ENC1 = 2; //0 interrupt
uint8_t M1_ENC2 = 3; //1 interrupt
int power_Left_speed_PID = 0;
//Создадим класс для контроля левого двигателя
DC_Motor Left_motor(L_EN, R_EN, L_PWM, R_PWM);

//RIGHT DCM 
const uint8_t L_PWM2 = 4;
const uint8_t R_PWM2 = 5;
const uint8_t L_EN2 = 12;
const uint8_t R_EN2 = 11;
uint8_t DCM2_ENC1 = 20; //2 interrupt
uint8_t DCM2_ENC2 = 21; //3 interrupt
int power_Right_speed_PID = 0;
float power_adjustment; // Корректировка разности характеристик моторов
//Создадим класс для контроля правого двигателя
DC_Motor Right_motor(L_EN2, R_EN2, L_PWM2, R_PWM2);

int power; //Мощьность для rotate_two_ways


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

//Экземпляры регуляторов
GyverPID regulator_Left(8, 5, 0.2, 10);  // коэф. П, коэф. И, коэф. Д, период дискретизации dt (мс)
GyverPID regulator_Right(8, 5, 0.2, 10);
GyverPID regulator_speed_Left(5, 1, 1, 20);  // коэф. П, коэф. И, коэф. Д, период дискретизации dt (мс)
GyverPID regulator_speed_Right(5, 1, 1, 20);
// Желаемая позиция
long desire_position_Left_DCM;
long desire_position_Right_DCM;
// Желаемая скорость
float desire_speed_Left_DCM;
float desire_speed_Right_DCM;
// Выбор метода управления. 0 - по координатам, 1 - по скорости
uint8_t control_method; 

// Экземпляр тюнера
PIDtuner2 tuner;

//
GKalman Speed_filter_Left(40,40,0.3);
GKalman Speed_filter_Right(40,40,0.3);

// Тесты
int parsing_result;
uint32_t myTimer1 = 0;
int power_test = 0;


void setup() {        
    power = 0; //Мощьность для rotate_two_ways          
    //Left энкодер
    attachInterrupt(0, EncTick_Left_DCM, CHANGE);
    attachInterrupt(1, EncTick_Left_DCM, CHANGE);
    //Right энкодер
    attachInterrupt(2, EncTick_Right_DCM, CHANGE);
    attachInterrupt(3, EncTick_Right_DCM, CHANGE);
    Serial.begin(115200);
    Serial.println("Left_DCM, Right_DCM");
    // Установки для регулятора по координатам
    regulator_Left.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
    regulator_Left.setLimits(-255, 255); // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
    regulator_Right.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
    regulator_Right.setLimits(-255, 255); // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
}

void loop() {  
// Графики координат
  Serial.print(position_Left_DCM);
  Serial.print(' ');
  Serial.print(position_Right_DCM);
  Serial.print(' ');
// Графики угловых скоростей
  rad_speed_Left_DCM = Speed_filter_Left.filtered((int)rad_speed_Left_DCM);
  Serial.print(rad_speed_Left_DCM);
  Serial.print(' ');
  rad_speed_Right_DCM = Speed_filter_Right.filtered((int)rad_speed_Right_DCM);
  Serial.println(rad_speed_Right_DCM);
  // Работа ПИД регулятора
  switch (control_method){
    case 0: 
      // Управление по координате
      regulator_Left.input = position_Left_DCM;
      regulator_Right.input = position_Right_DCM;
      Left_motor.set_voltage (regulator_Left.getResultTimer());
      Right_motor.set_voltage (regulator_Right.getResultTimer());
      break;
    case 1: 
      // Управление джойстиком (передача напряжения)
      Left_motor.set_voltage(power_Left_speed_PID );
      Right_motor.set_voltage(power_Right_speed_PID * 1.3);
      break; 
    case 9: 
    // Здесь была  калибровка
    break;
    }
  // Tests
  parsing();// Работа парсера (приём и обработка данных)
//  if(millis() - myTimer1 > 1500){
//    Serial.println(parsing_result);
//    myTimer1 = millis();
//    }
  
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
    //rad_speed_Left_DCM = (float) 1640.52*(position_Left_DCM- previous_position_LDCM)/(micros()-timeAT); 
    rad_speed_Left_DCM = (float) 40000*(position_Left_DCM- previous_position_LDCM)/(micros()-timeAT);
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
    //rad_speed_Right_DCM = (float) 1640.52*(position_Right_DCM - previous_position_RDCM)/(micros()-timeATR); 
    rad_speed_Right_DCM = (float) 40000*(position_Right_DCM - previous_position_RDCM)/(micros()-timeATR); 
    previous_position_RDCM = position_Right_DCM;
    timeATR = micros();
    kR = 0;
    }
  }
}

// функция для отправки пакета на ПК
void sendPacket(int key, int* data, int amount) {
  Serial.print(key);
  Serial.print(',');
  for (int i = 0; i < amount; i++) {
    Serial.print(data[i]);
    if (i != amount - 1) Serial.print(',');
  }
  Serial.print('\n');
}

// функция парсинга, опрашивать в лупе
void parsing() {
  if (serial.available()) {
    Parser data(serial.buf, ',');  // отдаём парсеру
    int ints[10];           // массив для численных данных
    data.parseInts(ints);   // парсим в него
    control_method = ints[1];
    regulator_Left.setpoint = ints[2];
    regulator_Right.setpoint = ints[3];
    power_Left_speed_PID = ints[4];
    power_Right_speed_PID = ints[5];
  }
}
