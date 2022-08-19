#include <M5Core2.h>
#include <ESP32Servo.h>
#include <ESP32Time.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>
#include <analogWrite.h>
#include <EEPROM.h>
#include <math.h>
#include "logo_big.h"
#include "logo_small.h"
#define MEAN_SIZE 200

extern uint8_t logo[];
File myFile;
Servo myServo;
size_t bytesRecieved;
byte Telemetry[212];
String str, file_name, date_str[6] = {"Hours", "Minutes", "Seconds", "Year", "Month", "Day"}, K_str[3] = {"Kp", "Ki", "Kd"};
bool eeprom_ok = false, sd_ok = false;
int i = 0, j = 0, addr = 0;
int select_time = 0, select_K, menu = 0;

float Motor_hours = 0;
float Temp_array[MEAN_SIZE];
float Temps = 0, Tempi = 30;
float PID_p = 0, PID_i = 0, PID_d = 0, PID_value = 0, PID_error = 0, PREV_error = 0;
float K_p = 1.1, K_i = 0.5, K_d = 0.175;
float Time_now = 0, Time_prev = 0, Time = 0;
 
RTC_TimeTypeDef RTCTime;
RTC_DateTypeDef RTCDate;

Gesture swipeLeft("swipe left", 160, DIR_LEFT, 30, true);
 
int Limits(int Value, int SupLimit, int InfLimit){
  if(Value > SupLimit){return InfLimit;}
  if(Value < InfLimit){return SupLimit;}
  return Value;
}

void Swiped(Event& e){
  if(++menu > 4){menu = 0;}
  M5.Lcd.fillScreen(WHITE);
}

void Scroll(Event& e) {
  if(menu == 3){
    if(++select_K >= 3){select_K = 0;}
  }
  if(menu == 2){
    if(++select_time >= 6){select_time = 0;}
  }
  M5.Lcd.fillScreen(WHITE);
} 
 
void DateEvent(Event& e) {
  M5.Rtc.GetDate(&RTCDate);
  M5.Rtc.GetTime(&RTCTime);    
  int aux_i[6] = {RTCTime.Hours, RTCTime.Minutes, RTCTime.Seconds, RTCDate.Year, RTCDate.Month, RTCDate.Date};
  float aux_f[3] = {K_p, K_i, K_d};
  
  if(menu == 3){
    M5.Lcd.fillScreen(WHITE);    
    if(M5.BtnC.pressedFor(700)){
      aux_f[0] = K_p + 0.1;
      aux_f[1] = K_i + 0.1;
      aux_f[2] = K_d + 0.1;
    }else if(M5.BtnB.pressedFor(700)){
      aux_f[0] = K_p - 0.1;
      aux_f[1] = K_i - 0.1;
      aux_f[2] = K_d - 0.1;
    }else if(M5.BtnC.wasPressed() && !M5.BtnC.isPressed()){
      aux_f[0] = K_p + 0.01;
      aux_f[1] = K_i + 0.01;
      aux_f[2] = K_d + 0.01;
    }else if(M5.BtnB.wasPressed() && !M5.BtnB.isPressed()){
      aux_f[0] = K_p - 0.01;
      aux_f[1] = K_i - 0.01;
      aux_f[2] = K_d - 0.01;
    }
    
    switch(select_K){
      case 0:
        K_p = aux_f[0];
        break;
      case 1:
        K_i = aux_f[1];
        break;
      case 2:
        K_d = aux_f[2];
        break;    
      default:
        break;
    }    
  }

  if(menu == 2){
    M5.Lcd.fillScreen(WHITE);
    if(M5.BtnC.wasPressed()){
      aux_i[3] = Limits(aux_i[3] + 1, 1000000, 0);
      aux_i[4] = Limits(aux_i[4] + 1, 12, 1);
      aux_i[5] = Limits(aux_i[5] + 1, 31, 1);
      aux_i[0] = Limits(aux_i[0] + 1, 23, 0);
      aux_i[1] = Limits(aux_i[1] + 1, 59, 0);
      aux_i[2] = Limits(aux_i[2] + 1, 59, 0);  
    }else if(M5.BtnB.wasPressed()){
      aux_i[3] = Limits(aux_i[3] - 1, 1000000, 0);
      aux_i[4] = Limits(aux_i[4] - 1, 12, 1);
      aux_i[5] = Limits(aux_i[5] - 1, 31, 1);
      aux_i[0] = Limits(aux_i[0] - 1, 23, 0);
      aux_i[1] = Limits(aux_i[1] - 1, 59, 0);
      aux_i[2] = Limits(aux_i[2] - 1, 59, 0);    
    }
  
    switch(select_time){
      case 3:
        RTCDate.Year = aux_i[3];
        break;
      case 4:
        RTCDate.Month = aux_i[4];
        break;
      case 5:
        RTCDate.Date = aux_i[5];
        break;
      case 0:
        RTCTime.Hours = aux_i[0];
        break;
      case 1:
        RTCTime.Minutes = aux_i[1];
        break;
      case 2:
        RTCTime.Seconds = aux_i[2];
        break;    
      default:
        break;
    }

    M5.Rtc.SetDate(&RTCDate);
    M5.Rtc.SetTime(&RTCTime);
  }
}

void setup(){
  M5.begin();
  M5.Rtc.begin();
  M5.Lcd.pushImage(0,0,320,240, (uint16_t *) logo_big);
  delay(4000);
  M5.Lcd.setTextColor(BLACK, WHITE);
  M5.Lcd.fillScreen(WHITE);
    
  // Serial Config
  Serial2.begin(115200 , SERIAL_8N1, 32 , 33 );
  Serial2.setTimeout(300);
  
  // Servo Config    
  myServo.setPeriodHertz(50);
  myServo.attach(26);
  
  // ADC Config  
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  // Memory Config
  if (!SD.begin()){  
    M5.Lcd.drawString("SDcard failed to mount.", 0, 225, 2);
    sd_ok = false;
  }else{
    M5.Lcd.drawString("SDcard successfully mounted.", 0, 225, 2);
    sd_ok = true;
    do{
      file_name = "/file" + String(i) + ".txt";
      i++;
    }while(SD.exists(file_name));
  }

  if(!EEPROM.begin(64)){
    M5.Lcd.drawString("Failed to initialise EEPROM.", 0, 200, 2);
    eeprom_ok = false;
  }else{
    M5.Lcd.drawString("Successfully initialise EEPROM.", 0, 200, 2);
    eeprom_ok = true;
    Motor_hours = (float) EEPROM.readFloat(addr);
  }
  
  // Button Config
  M5.BtnA.addHandler(Scroll, E_TOUCH);
  M5.BtnB.addHandler(DateEvent, E_TOUCH);
  M5.BtnC.addHandler(DateEvent, E_TOUCH);
  swipeLeft.addHandler(Swiped, E_GESTURE); 
  Temp_array[MEAN_SIZE - 1] = 0;
}

void menu_0(){
  str = "Read temp: ";
  str += String(Temps);
  M5.Lcd.drawString(str, 0, 0, 4);

  str = "Ideal temp: ";
  str += String(Tempi);  
  M5.Lcd.drawString(str, 0, 40, 4);
}

void menu_1(){
  if(bytesRecieved == 212){print_telemetry(0);} 
}

void menu_2(){
  M5.Rtc.GetDate(&RTCDate);
  M5.Rtc.GetTime(&RTCTime);
  
  str = "Hour: ";
  if(0 <= RTCTime.Hours && RTCTime.Hours < 10){str += "0";}
  str += String(RTCTime.Hours);
  str += "-";
  if(0 <= RTCTime.Minutes && RTCTime.Minutes < 10){str += "0";}  
  str += String(RTCTime.Minutes);
  str += "-";
  if(0 <= RTCTime.Seconds && RTCTime.Seconds < 10){str += "0";}  
  str += String(RTCTime.Seconds);
  M5.Lcd.drawString(str, 0, 40, 4); 
  str = "Date: ";
  str += String(RTCDate.Year);      
  str += "-";
  if(0 <= RTCDate.Month && RTCDate.Month < 10){str += "0";}  
  str += String(RTCDate.Month);   
  str += "-";
  if(0 <= RTCDate.Date && RTCDate.Date < 10){str += "0";}  
  str += String(RTCDate.Date); 
  M5.Lcd.drawString(str, 0, 0, 4);
  M5.Lcd.drawString(date_str[select_time], 40, 80, 4); //date_str[select_time] 
}

void menu_3(){
  str = "K_p: ";
  str += String(K_p); 
  M5.Lcd.drawString(str, 0, 0, 4);   
  str = "K_i: ";
  str += String(K_i); 
  M5.Lcd.drawString(str, 0, 40, 4); 
  str = "K_d: ";
  str += String(K_d); 
  M5.Lcd.drawString(str, 0, 80, 4);  
  M5.Lcd.drawString(K_str[select_K], 40, 120, 4);   
}

void loop() {
  M5.update();
  M5.Lcd.pushImage(220,210,100,30, (uint16_t *) logo_small);  
  Temps = mean_temp(((1.1*analogRead(35)/4095*3.5481)-0.5)*100);

  if(Serial2.available() > 0){               
    bytesRecieved = Serial2.readBytes(Telemetry,212);
    if(bytesRecieved == 212){
      writeSD();
    }
  }

  str = "Menu ";
  str += String(menu); 
  M5.Lcd.drawString(str, 220, 0, 4);
  
  switch(menu){
    case 0:
      menu_0();
      break;
    case 1:
      menu_1();
      break;
    case 2:
      menu_2();
      break;
    case 3:
      menu_3();
      break;                      
    default:
      break;  
  }
  
  timed();
}
 
void writeSD(){
  if (sd_ok == true) {
    myFile = SD.open(file_name, FILE_APPEND);
    M5.Rtc.GetDate(&RTCDate);
    M5.Rtc.GetTime(&RTCTime);
    print_telemetry(1);
    myFile.printf(" Hour: %2d-%2d-%2d Date: %4d-%2d-%2d\n",RTCTime.Hours,RTCTime.Minutes,RTCTime.Seconds,RTCDate.Year,RTCDate.Month,RTCDate.Date);
    myFile.close(); 
  }
}

void timed(){
  Time_prev = Time_now;
  Time = millis();
 
  if((Time - Time_prev) > 200){
    Serial2.write('A');
    
    PID_error = Tempi - Temps; 
    PID_p = K_p * PID_error;
    PID_i = PID_i + (K_i * PID_error);
    Time_now = millis(); 
    PID_d = K_d*((PID_error - PREV_error)/((Time_now - Time_prev)/1000));
    PID_value = PID_p + PID_i + PID_d;
    PREV_error = PID_error;
 
    if(PID_value > 105){
      PID_value = 105;
      if(PID_i > 110){PID_i = 110;}
    }else if(PID_value < 15){
      PID_value = 15;
      if(PID_i < 0){PID_i = 0;}
    }
    
    myServo.write(map(PID_value, 15, 105, 61, 151));
  }
}

void print_telemetry(int aux){
  str = "Seconds = ";
  str += String(Telemetry[0]*256 + Telemetry[1]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 50, 2);
  }else if(aux == 1){
    myFile.print(str);    
  }

  str = "Barometer = ";
  str += String(Telemetry[16]*256 + Telemetry[17]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 75, 2);
  }else if(aux == 1){
    myFile.print(str);    
  }

  str = "MAP = ";
  str += String(Telemetry[18]*256 + Telemetry[19]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 100, 2);
  }else if(aux == 1){
    myFile.print(str);    
  }  

  str = "MAT = ";
  str += String(Telemetry[20]*256 + Telemetry[21]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 125, 2);
  }else if(aux == 1){
    myFile.print(str);    
  }    

  str = "Coolant = ";
  str += String(Telemetry[22]*256 + Telemetry[23]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 150, 2);
  }else if(aux == 1){
    myFile.print(str);    
  }   

  str = "TPS = ";
  str += String(Telemetry[24]*256 + Telemetry[25]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 25, 2);
  }else if(aux == 1){
    myFile.print(str);    
  }

  str = "Voltage = ";
  str += String(Telemetry[26]*256 + Telemetry[27]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 0, 2);
  }else if(aux == 1){
    myFile.print(str);    
  }  
}

float mean_temp(float Temps){
  float Temp_mean = 0;
  Temp_array[j] = Temps;
  if(++j == MEAN_SIZE){j=0;}
  if(Temp_array[MEAN_SIZE-1]==0){return Temp_array[j-1];}
  for(i = 0; i < MEAN_SIZE; i++){Temp_mean += (Temp_array[i])/((float)(MEAN_SIZE));}
  
  return Temp_mean;
}
