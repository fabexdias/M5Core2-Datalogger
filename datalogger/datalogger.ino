#include <M5Core2.h>
#include <ESP32Servo.h>
#include <ESP32Time.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>
#include <analogWrite.h>
#include <math.h>

File myFile;
Servo myServo;
size_t bytesRecieved;
byte Telemetry[212];
String str, file_name;
bool sd_ok = false;
int i = 0, selecting = 0;

float Temps = 0, Tempi = 30;
float PID_p = 0, PID_i = 0, PID_d = 0, PID_value = 0, PID_error = 0, PREV_error = 0;
float K_p = 1.1, K_i = 0.5, K_d = 0.175;
float Time_now = 0, Time_prev = 0, Time = 0;

ButtonColors on_clrs  = {BLACK, BLACK, WHITE};
ButtonColors off_clrs = {BLACK, WHITE, WHITE};
Button A(220, 30, 80, 80, false ,"UP", off_clrs, on_clrs, MC_DATUM);
Button B(220, 150, 80, 80, false ,"DOWN", off_clrs, on_clrs, MC_DATUM);
 
RTC_TimeTypeDef RTCTime;
RTC_DateTypeDef RTCDate;
 
void UpAndDown(Event& e) {
  Button& b = *e.button;
  if(b.y == 30){Tempi++;}
  else if(b.y == 150){Tempi--;}
  M5.Lcd.fillScreen(BLACK);
  M5.Buttons.draw();
}

void Scroll(Event& e) {
  if(++selecting >= 6){selecting = 0;}
  M5.Lcd.fillScreen(BLACK);
  M5.Buttons.draw();  
} 
 
int Limits(int Value, int SupLimit, int InfLimit){
  if(Value > SupLimit){return InfLimit;}
  if(Value < InfLimit){return SupLimit;}
  return Value;
}
 
void DownDate(Event& e) {
  M5.Rtc.GetDate(&RTCDate);
  M5.Rtc.GetTime(&RTCTime);    
  int auxYear = RTCDate.Year;
  int auxMonth = RTCDate.Month;
  int auxHours = RTCTime.Hours;              
  int auxDate = RTCDate.Date;
  int auxMinutes = RTCTime.Minutes;
  int auxSeconds = RTCTime.Seconds;
  M5.Lcd.fillScreen(BLACK);
  M5.Buttons.draw();

  switch(selecting){
    case 0:
      auxYear = Limits(auxYear - 1, 1000000, 0);
      RTCDate.Year = auxYear;
      break;
    case 1:
      auxMonth = Limits(auxMonth - 1, 12, 1);
      RTCDate.Month = auxMonth;
      break;
    case 2:
      auxDate = Limits(auxDate - 1, 31, 1);
      RTCDate.Date = auxDate;
      break;//RTCDate.Year,RTCDate.Month,RTCDate.Date,RTCTime.Hours,RTCTime.Minutes,RTCTime.Seconds
    case 3:
      auxHours = Limits(auxHours - 1, 23, 0);
      RTCTime.Hours = auxHours;
      break;
    case 4:
      auxMinutes = Limits(auxMinutes - 1, 59, 0);
      RTCTime.Minutes = auxMinutes;
      break;
    case 5:
      auxSeconds = Limits(auxSeconds - 1, 59, 0);
      RTCTime.Seconds = auxSeconds;
      break;    
    default:
      break;
  }
  M5.Rtc.SetDate(&RTCDate);
  M5.Rtc.SetTime(&RTCTime);
}
 
void UpDate(Event& e) {
  M5.Rtc.GetDate(&RTCDate);
  M5.Rtc.GetTime(&RTCTime);  
  int auxYear = RTCDate.Year;
  int auxMonth = RTCDate.Month;
  int auxHours = RTCTime.Hours;              
  int auxDate = RTCDate.Date;
  int auxMinutes = RTCTime.Minutes;
  int auxSeconds = RTCTime.Seconds;
  M5.Lcd.fillScreen(BLACK);
  M5.Buttons.draw();    
 
  switch(selecting){
    case 3:
      auxYear = Limits(auxYear + 1, 1000000, 0);
      RTCDate.Year = auxYear;
      break;
    case 4:
      auxMonth = Limits(auxMonth + 1, 12, 1);
      RTCDate.Month = auxMonth;
      break;
    case 5:
      auxDate = Limits(auxDate + 1, 31, 1);
      RTCDate.Date = auxDate;
      break;
    case 0:
      auxHours = Limits(auxHours + 1, 23, 0);
      RTCTime.Hours = auxHours;
      break;
    case 1:
      auxMinutes = Limits(auxMinutes + 1, 59, 0);
      RTCTime.Minutes = auxMinutes;
      break;
    case 2:
      auxSeconds = Limits(auxSeconds + 1, 59, 0);
      RTCTime.Seconds = auxSeconds;
      break; 
    default:
      break;   
  }
  M5.Rtc.SetDate(&RTCDate);
  M5.Rtc.SetTime(&RTCTime);
} 
 
void setup(){
  M5.begin();
  // Serial Config
  Serial2.begin(115200 , SERIAL_8N1, 32 , 33 );
  Serial2.setTimeout(300);
  
  // Servo Config    
  myServo.setPeriodHertz(50);
  myServo.attach(26);
  
  // ADC Config  
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  // SDcard Config
  if (!SD.begin()){  
    str = "SDcard failed to mount.";
    M5.Lcd.drawString(str, 20, 225, 2);
    sd_ok = false;
  }else{
    str = "SDcard successfully mounted.";
    M5.Lcd.drawString(str, 20, 225, 2);
    sd_ok = true;
    do{
      file_name = "/file" + String(i) + ".txt";
      i++;
    }while(SD.exists(file_name));
  }

  // RTC Config
  RTCTime.Hours = 12;  
  RTCTime.Minutes = 13;
  RTCTime.Seconds = 20;
  RTCDate.Year = 2022;
  RTCDate.Month = 8;
  RTCDate.Date = 11;
  M5.Rtc.SetTime(&RTCTime);  
  M5.Rtc.SetDate(&RTCDate);

  // Button Config
  M5.BtnA.addHandler(Scroll, E_TOUCH);
  M5.BtnB.addHandler(DownDate, E_TOUCH);
  M5.BtnC.addHandler(UpDate, E_TOUCH);  
  A.addHandler(UpAndDown, E_TOUCH);
  B.addHandler(UpAndDown, E_TOUCH);  
  M5.Buttons.draw();
}
 
void loop() {
  M5.update();

  top_info();
  
  if(Serial2.available() > 0){               
    bytesRecieved = Serial2.readBytes(Telemetry,212);
    if(bytesRecieved == 212){
      print_telemetry(0); 
      writeSD();
    }
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

void top_info(){
  Temps = ((1.1*analogRead(35)/4095*3.5481)-0.5)*100; // Using MCP9700 as temperature sensor
  str = "Read temp: ";
  str += String((int) round(Temps));
  M5.Lcd.drawString(str, 20, 0, 4);
  str = "Ideal temp: ";
  str += String(Tempi);  
  M5.Lcd.drawString(str, 300, 0, 4);

  M5.Rtc.GetDate(&RTCDate);
  M5.Rtc.GetTime(&RTCTime);
  str = "Hour: ";
  str += String(RTCTime.Hours);
  str += "-";
  str += String(RTCTime.Minutes);
  str += "-";
  str += String(RTCTime.Seconds);
  str += " Date: ";
  str += String(RTCDate.Year);      
  str += "-";
  str += String(RTCDate.Month);   
  str += "-";
  str += String(RTCDate.Date); 
  M5.Lcd.drawString(str, 20, 25, 4); 
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
    M5.Lcd.drawString(str, 20, 175, 2);
  }else if(aux == 1){
    myFile.print(str);    
  }

  str = "Voltage = ";
  str += String(Telemetry[26]*256 + Telemetry[27]);
  if(aux == 0){
    M5.Lcd.drawString(str, 20, 200, 2);
  }else if(aux == 1){
    myFile.print(str);    
  }  
}
