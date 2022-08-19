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
#define SERIAL_NUMBER "XXXXXXXX"

File myFile;
Servo myServo;
size_t bytesRecieved;
byte Telemetry[212];
String str, file_name, date_str[6] = {"Hours", "Minutes", "Seconds", "Year", "Month", "Day"}, K_str[3] = {"Kp", "Ki", "Kd"}, menu_str[] = {"Temps","Serial","  RTC","   PID","      4"};
bool eeprom_ok = false, sd_ok = false, temp_ok = false;
int i = 0, j = 0, addr = 0;
int select_time = 0, select_K, menu = 0, error_pos = 2;

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
  M5.Lcd.fillRect(0, 0, 320, 180, WHITE);
}

void Scroll(Event& e) {
  if(menu == 3){
    if(++select_K >= 3){select_K = 0;}
  }
  if(menu == 2){
    if(++select_time >= 6){select_time = 0;}
  }
  M5.Lcd.fillRect(0, 0, 320, 180, WHITE);
} 
 
void DateEvent(Event& e) {
  M5.Rtc.GetDate(&RTCDate);
  M5.Rtc.GetTime(&RTCTime);    
  int aux_i[6] = {RTCTime.Hours, RTCTime.Minutes, RTCTime.Seconds, RTCDate.Year, RTCDate.Month, RTCDate.Date};
  float aux_f[3] = {K_p, K_i, K_d};
  
  if(menu == 3){
    M5.Lcd.fillRect(0, 0, 320, 180, WHITE);    
 
    if(String(e.button->getName()) == "BtnC" && e.type == E_DBLTAP){
      aux_f[0] = K_p + 0.1;
      aux_f[1] = K_i + 0.1;
      aux_f[2] = K_d + 0.1;
    }else if(String(e.button->getName()) == "BtnB" && e.type == E_DBLTAP){
      aux_f[0] = K_p - 0.1;
      aux_f[1] = K_i - 0.1;
      aux_f[2] = K_d - 0.1;
    }else if(String(e.button->getName()) == "BtnC" && e.type == E_TAP){
      aux_f[0] = K_p + 0.01;
      aux_f[1] = K_i + 0.01;
      aux_f[2] = K_d + 0.01;
    }else if(String(e.button->getName()) == "BtnB" && e.type == E_TAP){
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
    M5.Lcd.fillRect(0, 0, 320, 180, WHITE);
    if(String(e.button->getName()) == "BtnC"){
      aux_i[3] = Limits(aux_i[3] + 1, 1000000, 0);
      aux_i[4] = Limits(aux_i[4] + 1, 12, 1);
      aux_i[5] = Limits(aux_i[5] + 1, 31, 1);
      aux_i[0] = Limits(aux_i[0] + 1, 23, 0);
      aux_i[1] = Limits(aux_i[1] + 1, 59, 0);
      aux_i[2] = Limits(aux_i[2] + 1, 59, 0);  
    }else if(String(e.button->getName()) == "BtnB"){
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

  if(menu == 4){
    str = "BtnB: " + String(M5.BtnB.wasPressed()) + " | BtnC: " + String(M5.BtnC.wasPressed());
    M5.Lcd.drawString(str, 40, 40, 4);
    str = "Event: " + String(e.type) + " | Name: " + String(e.button->getName());
    M5.Lcd.drawString(str, 40, 80, 4);    
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
    warnings("SDcard failed to mount.");
    sd_ok = false;
  }else{
    warnings("SDcard successfully mounted.");
    sd_ok = true;
    do{
      file_name = "/file" + String(i) + ".txt";
      i++;
    }while(SD.exists(file_name));
  }

  if(!EEPROM.begin(64)){
    warnings("Failed to initialise EEPROM.");
    eeprom_ok = false;
  }else{
    warnings("Successfully initialise EEPROM.");
    eeprom_ok = true;
    Motor_hours = (float) EEPROM.readFloat(addr);
  }
  
  // Button & Gesture Config
  M5.BtnA.addHandler(Scroll, E_TOUCH);
  M5.BtnB.addHandler(DateEvent, E_TAP | E_DBLTAP);
  M5.BtnC.addHandler(DateEvent, E_TAP | E_DBLTAP);
  swipeLeft.addHandler(Swiped, E_GESTURE);
  for(i = 0; i < MEAN_SIZE; i++){Temp_array[i] = 0;}
}

void menu_0(){
  str = "Read temp: " + String(Temps);
  M5.Lcd.drawString(str, 0, 0, 4);

  str = "Ideal temp: " + String(Tempi);
  M5.Lcd.drawString(str, 0, 40, 4);
}

void menu_1(){

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
  str = "Date: " + String(RTCDate.Year) + "-";
  if(0 <= RTCDate.Month && RTCDate.Month < 10){str += "0";}  
  str += String(RTCDate.Month);   
  str += "-";
  if(0 <= RTCDate.Date && RTCDate.Date < 10){str += "0";}  
  str += String(RTCDate.Date); 
  M5.Lcd.drawString(str, 0, 0, 4);
  M5.Lcd.drawString(date_str[select_time], 40, 80, 4); //date_str[select_time] 
}

void menu_3(){
  str = "K_p: " + String(K_p);
  M5.Lcd.drawString(str, 0, 0, 4);   
  str = "K_i: " + String(K_i);
  M5.Lcd.drawString(str, 0, 40, 4); 
  str = "K_d: " + String(K_d);
  M5.Lcd.drawString(str, 0, 80, 4);  
  M5.Lcd.drawString(K_str[select_K], 40, 120, 4);   
}

void loop() {
  M5.update();
  M5.Lcd.pushImage(220,210,100,30, (uint16_t *) logo_small);  
  Temps = mean_temp(((1.1*analogRead(35)/4095*3.5481)-0.5)*100);

  float battery_voltage = M5.Axp.GetBatVoltage();
  if (battery_voltage < 3.6){warnings("Low battery.             ");}
  str = "#" + String(SERIAL_NUMBER);
  M5.Lcd.drawString(str, 260, 190, 2);
  
  if((Temps < -20 || Temps > 190) && temp_ok){
    warnings("Failed to read temperature.        ");
    temp_ok = false;
  }else if ((Temps > -20 && Temps < 190)){temp_ok = true;}  
  
  if(Serial2.available() > 0){               
    bytesRecieved = Serial2.readBytes(Telemetry,212);
    if(bytesRecieved == 212){
      writeSD();
      if(menu == 1){print_telemetry(0);}
    }
  }
  
  M5.Lcd.drawString("Menu", 235, 0, 4);
  M5.Lcd.drawString(menu_str[menu], 235, 30, 4);  
  
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
    myFile.printf("Hour: %2d-%2d-%2d Date: %4d-%2d-%2d\n",RTCTime.Hours,RTCTime.Minutes,RTCTime.Seconds,RTCDate.Year,RTCDate.Month,RTCDate.Date);
    print_telemetry(1);
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
  str = "Seconds = " + String(Telemetry[0]*256 + Telemetry[1]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 44, 2);
  }else if(aux == 1){
    myFile.println(str);    
  }

  str = "Barometer = " + String(Telemetry[16]*256 + Telemetry[17]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 66, 2);
  }else if(aux == 1){
    myFile.println(str);    
  }

  str = "MAP = " + String(Telemetry[18]*256 + Telemetry[19]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 88, 2);
  }else if(aux == 1){
    myFile.println(str);    
  }  

  str = "MAT = " + String(Telemetry[20]*256 + Telemetry[21]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 110, 2);
  }else if(aux == 1){
    myFile.println(str);    
  }    

  str = "Coolant = " + String(Telemetry[22]*256 + Telemetry[23]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 132, 2);
  }else if(aux == 1){
    myFile.println(str);    
  }   

  str = "TPS = " + String(Telemetry[24]*256 + Telemetry[25]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 22, 2);
  }else if(aux == 1){
    myFile.println(str);    
  }

  str = "Voltage = " + String(Telemetry[26]*256 + Telemetry[27]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 0, 2);
  }else if(aux == 1){
    myFile.println(str);    
  }  
}

float mean_temp(float Temps){
  float Temp_mean = 0;
  Temp_array[j] = Temps;
  if(++j == MEAN_SIZE){j=0;}
  for(i = 0; i < MEAN_SIZE; i++){Temp_mean += (Temp_array[i])/((float)(MEAN_SIZE));}
  return Temp_mean;
}

void warnings(String aux){
  M5.Lcd.drawString(aux, 0, 225 - error_pos * 20, 2);
  if(--error_pos == -1) error_pos = 2; 
}
