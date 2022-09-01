#include <M5Core2.h>
#include <ESP32Servo.h>
#include <ESP32Time.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>
#include <Preferences.h>
#include <analogWrite.h>
#include <EEPROM.h>
#include <math.h>

// Imagens passadas para byte streams
#include "logo_big.h"
#include "logo_small.h"
#include "logo_smaller.h"

#define PASSWORD "senha"
#define MEAN_SIZE 601 // Tamanho do vetor de aquisições
#define SERIAL_NUMBER "XXXXX" // Numero de série
#define MENU_NUMBER 1 // Numero de menus
#define SAMPLE_TIME 1000 // Tempo de amostragem para o PID e Intervalo entre serial 'A'
#define ERROR_TIMEOUT 60000 // Tempo de timeout para erros
#define SAMPLE_TIMEOUT 3000 // Tempo de timeout para receção dos dados
#define SERIAL_TIMEOUT 60000 // Tempo de timeout para configuração em monitor de série
#define DEVICE_TIMEOUT 120000 // Tempo de timeout caso não haja alimentação
#define MOTOR_TIMEOUT 30000 // Tempo de funcionamento do motor a partir do qual se conta o tempo de vida

// Auxiliares para o vetor data_logging
#define SECONDS 0
#define RPM 1
#define BARO 2
#define MAP 3
#define MAT 4
#define CHT 5
#define TPS 6
#define VOLTAGE 7
#define WARMCOR 8
#define BAROCOR 9

typedef struct {
  float_t CHT_MAX;
  float_t CHT_MIN;
  float_t FUEL_PRESSURE_MAX;
  float_t FUEL_PRESSURE_MIN;
  float_t BATTERY_MAX;
  float_t BATTERY_MIN;
  float_t MAT_MAX;
  float_t MAT_MIN;
  float_t IDEAL_TEMP;
  float_t RPM_HOURS;  
  float_t K_P[2];
  float_t K_I[2];
  float_t K_D[2];
} config_t; // estrutura de configuração

config_t configy = {0,0,0,0,0,0,0,0,0,0,{0,0},{0,0},{0,0}};
Preferences prefs;
File myFile;
Servo myServo;
size_t bytesRecieved;

static const uint8_t charging[336] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0x00, 0x00, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0x00, 0x00, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0x00, 0x00, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0x00, 0x00, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0x00, 0x00, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0x60, 0xff, 0x60, 0xff, 0x60, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0x60, 0xff, 0x60, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0x60, 0xff, 0x60, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0x60, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};


static const uint8_t bat_3[784] =
{   0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x25, 0x89, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

static const uint8_t bat_2[784] =
{   0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff

};

static const uint8_t bat_1[784] =
{   0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0x00, 0x00, 0x00, 0x00, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0x00, 0x00, 0x00, 0x00, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0xe8, 0xe4, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

typedef struct {
  int tipo; // error_handler
  int time_stamp;
  String texto;
} warnings;

warnings w_struct[8];

byte Telemetry[212];
String str, file_name[2], warning_str = "";
bool eeprom_ok = false, sd_ok = false, motor_ok = false, serial_ok = true, menu0_ok = true, password = false, data_ok[6] = {true, true, true, true, true, true};
int i = 0, j = 0, addr = 0;
int menu = 0;

int64_t Motor_hours = 0, Motor_start = 0;
float Temp_array[MEAN_SIZE];
float Temps = 0, Tempi = 30;
float PID_p = 0, PID_i = 0, PID_d = 0, PID_value = 0, PID_error = 0, PREV_error = 0;
float K_p = 1.1, K_i = 0.5, K_d = 0.175;
float Time_now = 0, Time_prev = 0, Time_serial = 0, Time_motor = 0, Time_bat = 0, Time_rx = 0, Time_error = 0, battery = 0;
float data_logging[10];

RTC_TimeTypeDef RTCTime;
RTC_DateTypeDef RTCDate;

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------PARTE GRÁFICA----------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------*/

// Gestos para passar de menu em menu
Gesture swipeLeft("swipe left", 160, DIR_LEFT, 30, true);
Gesture swipeRight("swipe right", 160, DIR_RIGHT, 30, true);

// Função de interrupção para gestos (passa de menu em menu)
void Swiped(Event& e){
  if(e.gesture != NULL){
    if(String(e.gesture->getName()) == "swipe left"){if(++menu > MENU_NUMBER){menu = 0;}}
    else if(String(e.gesture->getName()) == "swipe right"){if(--menu < 0){menu = MENU_NUMBER;}}
    M5.Lcd.fillRect(0, 27, 320, 186, WHITE); // Este comando é usado sempre que é premido um botão e serve para pintar a parte do ecrã referente aos diferentes menus.
                                             // Isto é feito para que não haja strings sobrepostas nem que haja erros (por exemplo, um valor que passe de 105 para 2 ficaria 205)        
  }else if(e.button != NULL){
    if(String(e.button->getName()) == "BtnA"){if(--menu < 0){menu = MENU_NUMBER;}}
    else if(String(e.button->getName()) == "BtnC"){if(++menu > MENU_NUMBER){menu = 0;}}
    else if(String(e.button->getName()) == "BtnB"){menu0_ok = true; menu = 0;}
    M5.Lcd.fillRect(0, 27, 320, 186, WHITE);
  }
}

// Função relativa ao menu 0
void menu_0(){
  if(menu0_ok){
    menu0_ok = false;
    M5.Lcd.fillCircle(300, 37, 7, BLACK);
    M5.Lcd.fillCircle(300, 60, 7, BLACK);
    M5.Lcd.fillCircle(300, 82, 7, BLACK);
    M5.Lcd.fillCircle(300, 104, 7, BLACK);
    M5.Lcd.fillCircle(300, 127, 7, BLACK);
    M5.Lcd.fillCircle(300, 149, 7, BLACK);
    M5.Lcd.fillCircle(300, 171, 7, BLACK);    
  }
  if(Motor_start != 0){str = SecondsToString(Motor_hours + (millis() - Motor_start)/1000); M5.Lcd.fillCircle(300, 37, 5, GREEN);}else{str = SecondsToString(Motor_hours); M5.Lcd.fillCircle(300, 37, 5, RED);}
  
  M5.Lcd.drawString("Work Time: " + str, 1, 31, 1);
  M5.Lcd.drawString("ECU Time :", 1, 53, 1); if(bytesRecieved == 212){M5.Lcd.fillCircle(300, 60, 5, GREEN);}else if(millis() - Time_rx > SAMPLE_TIMEOUT){M5.Lcd.fillCircle(300, 60, 5, LIGHTGREY);}
  M5.Lcd.drawString("Throttle :", 1, 75, 1); if(bytesRecieved == 212){M5.Lcd.fillCircle(300, 82, 5, GREEN);}else if(millis() - Time_rx > SAMPLE_TIMEOUT){ M5.Lcd.fillCircle(300, 82, 5, LIGHTGREY);}
  M5.Lcd.drawString("Voltage  :", 1, 97, 1); if(bytesRecieved == 212){if(data_ok[2] == true){M5.Lcd.fillCircle(300, 104, 5, GREEN);}else{M5.Lcd.fillCircle(300, 104, 5, RED);}}else if(millis() - Time_rx > SAMPLE_TIMEOUT){ M5.Lcd.fillCircle(300, 104, 5, LIGHTGREY);}
  M5.Lcd.drawFastHLine(0, 115, 320, NAVY); 
  M5.Lcd.drawString("RPM:", 1, 120, 1); if(bytesRecieved == 212){M5.Lcd.fillCircle(300, 127, 5, GREEN);}else if(millis() - Time_rx > SAMPLE_TIMEOUT){ M5.Lcd.fillCircle(300, 127, 5, LIGHTGREY);}
  M5.Lcd.drawString("IAT:", 1, 142, 1); if(bytesRecieved == 212){if(data_ok[4] == true){M5.Lcd.fillCircle(300, 149, 5, GREEN);}else{M5.Lcd.fillCircle(300, 149, 5, RED);}}else if(millis() - Time_rx > SAMPLE_TIMEOUT){ M5.Lcd.fillCircle(300, 149, 5, LIGHTGREY);}
  M5.Lcd.drawString("CHT:", 1, 164, 1); if(bytesRecieved == 212){if(data_ok[5] == true){M5.Lcd.fillCircle(300, 171, 5, GREEN);}else{M5.Lcd.fillCircle(300, 171, 5, RED);}}else if(millis() - Time_rx > SAMPLE_TIMEOUT){ M5.Lcd.fillCircle(300, 171, 5, LIGHTGREY);}
  M5.Lcd.drawFastHLine(0, 182, 320, NAVY);

  M5.Rtc.GetDate(&RTCDate);
  str = String(RTCDate.Year) + "-";
  if(RTCDate.Month < 10){str += "0";}  
  str += String(RTCDate.Month) + "-";
  if(RTCDate.Date < 10){str += "0";}  
  str += String(RTCDate.Date) + " ";  
  M5.Rtc.GetTime(&RTCTime);  
  if(RTCTime.Hours < 10){str += "0";}    
  str += String(RTCTime.Hours) + ":";
  if(RTCTime.Minutes < 10){str += "0";}  
  str += String(RTCTime.Minutes) + ":";
  if(RTCTime.Seconds < 10){str += "0";}  
  str += String(RTCTime.Seconds);
  M5.Lcd.setTextDatum(TC_DATUM); M5.Lcd.drawString(str, 160, 190, 1); M5.Lcd.setTextDatum(TL_DATUM);
}

// Função relativa ao menu 1
void menu_1(){ // nestas funções pouco se trata para além da interface gráfica
  menu0_ok = true;
  M5.Lcd.drawString(w_struct[0].texto, 1, 31, 1);
  M5.Lcd.drawString(w_struct[1].texto, 1, 52, 1);
  M5.Lcd.drawString(w_struct[2].texto, 1, 73, 1);
  M5.Lcd.drawString(w_struct[3].texto, 1, 94, 1);
  M5.Lcd.drawString(w_struct[4].texto, 1, 115, 1);
  M5.Lcd.drawString(w_struct[5].texto, 1, 136, 1);
  M5.Lcd.drawString(w_struct[6].texto, 1, 157, 1);
  M5.Lcd.drawString(w_struct[7].texto, 1, 178, 1);

  M5.Lcd.drawFastHLine(0, 196, 320, NAVY); 
  M5.Lcd.setTextSize(1); M5.Lcd.setTextDatum(TC_DATUM); M5.Lcd.drawString("LAST 8 ERROR LOG", 160, 197, 2); M5.Lcd.setTextDatum(TL_DATUM); M5.Lcd.setTextSize(2);
}

void header(){
  if(battery == 0){
    M5.Lcd.setTextSize(1); M5.Lcd.drawString(("#" + String(SERIAL_NUMBER)), 220, 4, 2);  M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(NAVY, WHITE); M5.Lcd.setTextDatum(TC_DATUM); M5.Lcd.drawString("EMU V1", 160, 2, 1); M5.Lcd.setTextColor(BLACK, WHITE); M5.Lcd.setTextDatum(TL_DATUM);
    M5.Lcd.pushImage(3,1,80,20, (uint16_t *) logo_smaller); // Inserção do logo pequeno e número de série no canto inferior do ecrã
    M5.Lcd.drawFastHLine(0, 25, 320, NAVY); 
    M5.Lcd.drawFastHLine(0, 26, 320, NAVY);
    M5.Lcd.drawFastHLine(0, 213, 320, NAVY); 
    M5.Lcd.drawFastHLine(0, 214, 320, NAVY);
    //M5.Lcd.setTextSize(1); M5.Lcd.drawString("left center right", 200, 216, 2);
    M5.Lcd.setTextSize(2);
    M5.Lcd.fillCircle(158, 227, 9, NAVY);
    M5.Lcd.fillTriangle(40, 228, 60, 220, 60, 236, NAVY);
    M5.Lcd.fillTriangle(274, 228, 254, 220, 254, 236, NAVY);
  }
  battery = M5.Axp.GetBatteryLevel();
  if (battery >= 75){
    M5.Lcd.pushImage(289, 5, 28, 14, (uint16_t *) bat_3);
  }
  else if (battery >= 40){
    M5.Lcd.pushImage(289, 5, 28, 14, (uint16_t *) bat_2);
  }
  else if (battery < 40){
    M5.Lcd.pushImage(289, 5, 28, 14, (uint16_t *) bat_1);
  }
  if(!(!M5.Axp.isVBUS() && (!M5.Axp.isCharging() || !M5.Axp.isACIN()))){M5.Lcd.pushImage(275,5,12,14, (uint16_t *) charging);}else{M5.Lcd.fillRect(275, 5, 12, 14, WHITE);}  
}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------*/

// Função de setup corre uma vez, quando o código é inicializado
void setup(){
  M5.begin();
  M5.Rtc.begin();

  // Random inicialization
  M5.Lcd.pushImage(0,0,320,240, (uint16_t *) logo_big);
  M5.Lcd.setTextDatum(TC_DATUM);
  M5.Lcd.setTextColor(BLACK, WHITE);
  M5.Lcd.setTextSize(1);
  for(i = 0; i < MEAN_SIZE; i++){Temp_array[i] = 0;}
  for(i = 0; i <= BAROCOR; i++){data_logging[i] = 0;}
  for(i = 0; i < 8; i++){w_struct[i].tipo = 0; w_struct[i].texto = "";}  
  delay(1000);
  
  // Serial Config
  Serial2.begin(115200, SERIAL_8N1, 32 , 33 );
  Serial2.setTimeout(300);
  Serial.begin(115200);
  
  // Servo Config    
  myServo.setPeriodHertz(50);
  myServo.attach(26);
  
  // ADC Config  
  M5.Axp.SetBusPowerMode(1); // Select source for BUS_5V -> 0 : use internal boost | 1 : powered externally
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(36, INPUT);
  
  // Memory Config
  M5.Rtc.GetDate(&RTCDate);
  str = "-" + String(RTCDate.Year) + "-";
  if(0 <= RTCDate.Month && RTCDate.Month < 10){str += "0";}  
  str += String(RTCDate.Month) + "-";
  if(0 <= RTCDate.Date && RTCDate.Date < 10){str += "0";}  
  str += String(RTCDate.Date);
  
  if (!SD.begin()){  
    M5.Lcd.drawString("SDcard failed to mount.", 160, 210, 2); //warnings("SDcard failed to mount.");
    sd_ok = false;
  }else{
    M5.Lcd.drawString("SDcard successfully mounted.", 160, 210, 2); //warnings("SDcard successfully mounted.");
    sd_ok = true; i = 0;
    do{
      file_name[0] = "/file" + String(i) + str + ".csv";
      i++;
    }while(SD.exists(file_name[0])); // este ciclo DO WHILE tem o objetivo de criar um novo ficheiro com continuação
    file_name[1] = "/file" + String(i-1) + str + "-backup.csv";

    myFile = SD.open(file_name[0], FILE_APPEND);
    M5.Rtc.GetDate(&RTCDate);
    M5.Rtc.GetTime(&RTCTime);
    myFile.println("ECUSeconds,RPM,Barometer,MAP,MAT,CHT,TPS,Voltage,WarmCor,BaroCorrection,Hours,Minutes,Seconds,Faults,Start at: " + String(RTCTime.Hours) + ":" + String(RTCTime.Minutes) + ":" + String(RTCTime.Seconds) + " " + String(RTCDate.Year) + "-" + String(RTCDate.Month) + "-" + String(RTCDate.Date));
    myFile.close();  
    myFile = SD.open(file_name[1], FILE_APPEND);
    M5.Rtc.GetDate(&RTCDate);
    M5.Rtc.GetTime(&RTCTime);
    myFile.println("ECUSeconds,RPM,Barometer,MAP,MAT,CHT,TPS,Voltage,WarmCor,BaroCorrection,Hours,Minutes,Seconds,Faults,Start at: " + String(RTCTime.Hours) + ":" + String(RTCTime.Minutes) + ":" + String(RTCTime.Seconds) + " " + String(RTCDate.Year) + "-" + String(RTCDate.Month) + "-" + String(RTCDate.Date));
    myFile.close();  
  }
  
  delay(1000);
  
  if(!EEPROM.begin(64)){ // Guarda-se na EEPROM os valores mais importantes e começa-se por inicializar devidamente as variaveis importantes
    M5.Lcd.drawString("Failed to initialise EEPROM.", 160, 210, 2); // warnings("Failed to initialise EEPROM.");
    eeprom_ok = false;
  }else{
    M5.Lcd.drawString("Successfully initialise EEPROM.", 160, 210, 2); //warnings("Successfully initialise EEPROM.");
    eeprom_ok = true;
    Motor_hours = (int64_t) EEPROM.readLong64(addr);
  }
  delay(1000);
  
  if(prefs.begin("config")){ // Biblioteca de preferência, todos os que é necessário guardar em cada dispositivo
    size_t configLen = prefs.getBytesLength("config");
    prefs.getBytes("config",(void*) &configy, configLen); // prefs.putBytes("config",(void*) &configy,(size_t) sizeof(configy));  
    M5.Lcd.drawString("Successfully initialise the config.", 160, 210, 2); //warnings("Successfully initialise EEPROM.");   
  }else{
    M5.Lcd.drawString("Failed to initialise config.", 160, 210, 2); // warnings("Failed to initialise EEPROM.");     
  }
  
  // Buttons & Gestures Config
  M5.BtnA.addHandler(Swiped, E_TOUCH);
  M5.BtnB.addHandler(Swiped, E_TOUCH);
  M5.BtnC.addHandler(Swiped, E_TOUCH);
  swipeLeft.addHandler(Swiped, E_GESTURE);
  swipeRight.addHandler(Swiped, E_GESTURE);  
  delay(2000);

  // Text Config
  M5.Lcd.setTextSize(2);
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextWrap(true, true);
  M5.Lcd.setTextDatum(TL_DATUM);
}

// Esta função fica em loop infinito durante a execução do programa
void loop() {
  M5.update();
  header();
  timed();
  serial_stuff();
  if(bytesRecieved == 212){error_handler();}

  switch(menu){
    case 0:
      menu_0();
      break;
    case 1:
      menu_1();
      break;                     
    default:
      break;  
  }

  if(!M5.Axp.isVBUS() && (!M5.Axp.isCharging() || !M5.Axp.isACIN()) && battery < 100){ // timeout se não é alimentado por 5V nem por USB-C e a bateria é menor que 100%
      if(Time_bat == 0){Time_bat = millis();}
      if((millis() - Time_bat) > DEVICE_TIMEOUT){M5.shutdown();}
  }else{
    Time_bat = 0;
  } 
}

void error_handler(){
  warning_str = "";
  M5.Rtc.GetDate(&RTCDate);
  str = String(RTCDate.Year) + "-";
  if(RTCDate.Month < 10){str += "0";}  
  str += String(RTCDate.Month) + "-";
  if(RTCDate.Date < 10){str += "0";}  
  str += String(RTCDate.Date) + " ";  
  M5.Rtc.GetTime(&RTCTime);  
  if(RTCTime.Hours < 10){str += "0";}    
  str += String(RTCTime.Hours) + ":";
  if(RTCTime.Minutes < 10){str += "0";}  
  str += String(RTCTime.Minutes) + ":";
  if(RTCTime.Seconds < 10){str += "0";}  
  str += String(RTCTime.Seconds);
  
  /*if((Temps < configy.CHT_MIN || Temps > configy.CHT_MAX) && temp_ok){
    //warnings("Failed to read temperature.        ");
    warning_str += "Tmp ";
    temp_ok = false;
  }else if ((Temps > configy.CHT_MIN && Temps < configy.CHT_MAX)){temp_ok = true;}*/
  if (data_logging[VOLTAGE] < configy.BATTERY_MIN || data_logging[VOLTAGE] > configy.BATTERY_MAX){ // type = 1;
    data_ok[2] = false;
    warning_str += "Battery ";
    insert_error(1, millis(), "BAT  @ " + str);
  }else{data_ok[2] = true;}  
  if(data_logging[CHT] > configy.CHT_MAX){ // type = 2;
    data_ok[5] = false;
    warning_str += "CHT_OVER ";
    insert_error(2, millis(), "CHT+ @ " + str);
  }else if(data_logging[CHT] < configy.CHT_MIN && data_logging[RPM] > 2500){ // type = 3;
    data_ok[5] = false;
    warning_str += "CHT_UNDER ";
    insert_error(3, millis(), "CHT- @ " + str);    
  }else{data_ok[5] = true;}
  if(data_logging[MAT] < configy.MAT_MIN || data_logging[MAT] > configy.MAT_MAX){ // type = 4;
    data_ok[4] = false;
    warning_str += "MAT ";
    insert_error(4, millis(), "IAT  @ " + str);        
  }else{data_ok[4] = true;}
}

void insert_error(int tipo, int time_stamp, String texto){
  bool printing = true;

  for(i = 0; i < 8; i++){
    if(w_struct[i].tipo == tipo){
      printing = false;
      if((time_stamp - w_struct[i].time_stamp) > ERROR_TIMEOUT){
        printing = true;
      }
    }
  }

  if(printing == true){
    for(i = 7; i > 0; i--){
      w_struct[i].tipo = w_struct[i-1].tipo;
      w_struct[i].time_stamp = w_struct[i-1].time_stamp;
      w_struct[i].texto = w_struct[i-1].texto;
    }
    w_struct[0].tipo = tipo;
    w_struct[0].time_stamp = time_stamp;
    w_struct[0].texto = texto;
  }
}

void serial_stuff(){
  bytesRecieved = 0;
  if(Serial2.available() > 0){               
    bytesRecieved = Serial2.readBytes(Telemetry,212);
    if(bytesRecieved == 212){
      Time_rx = millis();
      writeSD();
      warning_str = "";
      if(menu == 0){print_telemetry(0);}
      if((data_logging[RPM] > configy.RPM_HOURS) && !motor_ok){
        motor_ok = true;
        Time_motor = millis();
        Motor_start = 0;
      }else if(motor_ok && (millis() - Time_motor > MOTOR_TIMEOUT) && Motor_start == 0){
        Motor_start = millis();
      }else if(motor_ok && data_logging[RPM] <= configy.RPM_HOURS  && Motor_start != 0){
        motor_ok = false;
        if((millis() - Motor_start) > MOTOR_TIMEOUT){
          Motor_hours += (millis() - Motor_start)/1000;
          EEPROM.writeLong64(addr, (int64_t) Motor_hours);
          EEPROM.commit();
          Motor_start = 0;               
        }
      }      
    }
  }
  
  if((millis() - Time_serial > SERIAL_TIMEOUT) && password == true){
    password = false;
    Serial.println("------------------------------------------------------------");    
    Serial.println("Session timed out, login again please.");
    Serial.println("------------------------------------------------------------");
  }
  
  if(Serial.available()>0){
    serial_commands();
  }
}

// A função writeSD é executada sempre que se recebe 212 bytes via Serial e guarda apenas as informações de interesse
void writeSD(){
  if (sd_ok == true) {
    // Ficheiro original
    myFile = SD.open(file_name[0], FILE_APPEND);
    M5.Rtc.GetTime(&RTCTime);
    print_telemetry(1);
    myFile.printf("%2d,%2d,%2d,",RTCTime.Hours,RTCTime.Minutes,RTCTime.Seconds);
    myFile.println(warning_str);
    myFile.close();
    // Backup
    myFile = SD.open(file_name[1], FILE_APPEND);
    print_telemetry(1);
    myFile.printf("%2d,%2d,%2d,",RTCTime.Hours,RTCTime.Minutes,RTCTime.Seconds);
    myFile.println(warning_str);
    myFile.close();    
  }
}

// A função timed é sempre executada, porém só faz efetivamente algo dee 300 em 300 milisegundos
void timed(){
  float aux;
  int k = 0;
  Time_prev = Time_now;

  if((millis() - Time_prev) > SAMPLE_TIME){
    Serial2.write('A'); // Isto serve para requesitar dados à centralina 

    Temps = median_temp(); // Leitura da temperatura com filtro de mediana
    
    PID_error = Tempi - Temps; // Isto é o controlador PID para o servo
    PID_p = PID_error;
    PID_i = PID_i + PID_error;
    if(PID_i > (151/K_i)){PID_i = (151/K_i);} // Anti-windup
    else if(PID_i < (61/K_i)){PID_i = (61/K_i);} 
    Time_now = millis(); 
    PID_d = (PID_error - PREV_error)/((Time_now - Time_prev)/1000);
    PID_value = K_p * PID_p + K_i * PID_i + K_d * PID_d;
    PREV_error = PID_error;
 
    if(PID_value > 151){PID_value = 151;} // 151 e 61 são equivalentes a 0º e 90º, respectivamente
    else if(PID_value < 61){PID_value = 61;}
    
    myServo.write(PID_value);
  }
}

// Esta função é usada para printar a telemetria quer no SDcard quer no display, usado aux = 1 e aux = 0, respectivamente
/*M5.Lcd.drawString("ECU Time :", 1, 53, 1);
  M5.Lcd.drawString("Throttle :", 1, 75, 1);
  M5.Lcd.drawString("Voltage  :", 1, 97, 1);
  M5.Lcd.drawString("RPM:", 1, 120, 1);
  M5.Lcd.drawString("IAT:", 1, 142, 1);
  M5.Lcd.drawString("CHT:", 1, 164, 1);*/
void print_telemetry(int aux){
  //if((((float) Telemetry[128]*256 + Telemetry[129])*0.002738095-0.355952381) < 0){str = "0.00";}else{str = String(((float) Telemetry[128]*256 + Telemetry[129])*0.002738095-0.355952381);} // ADC6 - pressão, reta de valores com os pontos (0 bar, 130) e (1.725 bar, 760)
  data_logging[SECONDS] = (float) (Telemetry[0]*256 + Telemetry[1]);
  if(aux == 0){
    M5.Lcd.drawString("ECU Time : " + SecondsToString((int) data_logging[SECONDS]), 1, 53, 1);
  }else if(aux == 1){
    myFile.printf("%.2f,", data_logging[SECONDS]);    
  }
  
  data_logging[RPM] = (float) (Telemetry[6]*256 + Telemetry[7]);
  if(aux == 0){ 
    M5.Lcd.drawString("RPM: " + String(data_logging[RPM],0) + " RPM    ", 1, 120, 1);
  }else if(aux == 1){
    myFile.printf("%.2f,", data_logging[RPM]);    
  }

  data_logging[BARO] = (float)((float)(Telemetry[16]*256 + Telemetry[17])/10);
  if(aux == 0){
    //M5.Lcd.drawString("Barometer: " + String(data_logging[BARO],1) + " kPa      ", 1, 66, 1);
  }else if(aux == 1){
    myFile.printf("%.2f,", data_logging[BARO]);    
  }

  data_logging[MAP] = (float)((float)(Telemetry[18]*256 + Telemetry[19])/10);
  if(aux == 0){
    //M5.Lcd.drawString("MAP: " + String(data_logging[MAP],1) + " kPa   ", 1, 88, 1);
  }else if(aux == 1){
    myFile.printf("%.2f,", data_logging[MAP]);    
  }  

  data_logging[MAT] = (float)(((float)(Telemetry[20]*256 + Telemetry[21])/10 - 32)*5/9);
  if(aux == 0){
    M5.Lcd.drawString("IAT: " + String(data_logging[MAT],1) + " C   ", 1, 142, 1);
  }else if(aux == 1){
    myFile.printf("%.2f,", data_logging[MAT]);    
  }    

  data_logging[CHT] = (float)(((float)(Telemetry[22]*256 + Telemetry[23])/10 - 32)*5/9);
  if(aux == 0){
    M5.Lcd.drawString("CHT: " + String(data_logging[CHT],1) + " C   ", 1, 164, 1);
  }else if(aux == 1){
    myFile.printf("%.2f,", data_logging[CHT]);    
  }   

  data_logging[TPS] = (float)((float)(Telemetry[24]*256 + Telemetry[25])/10);
  if(aux == 0){
    M5.Lcd.drawString("Throttle : " + String(data_logging[TPS],1) + " %  ", 1, 75, 1);
  }else if(aux == 1){
    myFile.printf("%.2f,", data_logging[TPS]);    
  }

  data_logging[VOLTAGE] = (float)((float)(Telemetry[26]*256 + Telemetry[27])/10);
  if(aux == 0){
    M5.Lcd.drawString("Voltage  : " + String(data_logging[VOLTAGE],1) + " V   ", 1, 97, 1);
  }else if(aux == 1){
    myFile.printf("%.2f,", data_logging[VOLTAGE]);    
  } 
  data_logging[WARMCOR] = (float)((float)(Telemetry[40]*256 + Telemetry[41])/10);
  if(aux == 0){
    //M5.Lcd.drawString("Warmup: " + String(data_logging[WARMCOR],1) + " %        ", 160, 110, 1);
  }else if(aux == 1){
    myFile.printf("%.2f,", data_logging[WARMCOR]);    
  }
  data_logging[BAROCOR] = (float)((float)(Telemetry[46]*256 + Telemetry[47])/10);
  if(aux == 0){
    //M5.Lcd.drawString("BCorr: " + String(data_logging[BAROCOR],1) + " %        ", 160, 88, 1);
  }else if(aux == 1){
    myFile.printf("%.2f,", data_logging[BAROCOR]);    
  }
}

float median_temp(){
  float a;
  for(i = 0; i < MEAN_SIZE; i++){Temp_array[i] = ((1.1*analogRead(36)/4095*3.5481)-0.5)*100;} // Alterar consuante o sensor usado
  for(i = 0; i < MEAN_SIZE; i++)
    for (int k = i + 1; k < MEAN_SIZE; k++)
      if (Temp_array[i] > Temp_array[k]){
        a = Temp_array[i];
        Temp_array[i] = Temp_array[k];
        Temp_array[k] = a;
      }
  return Temp_array[300];  
}

String SecondsToString(int tme)
{
  int hr = tme/3600;                                                        //Number of seconds in an hour
  int mins = (tme-hr*3600)/60;                                              //Remove the number of hours and calculate the minutes.
  int sec = tme-hr*3600-mins*60;                                            //Remove the number of hours and minutes, leaving only seconds.
  if(hr < 10){str = "0" + String(hr) + ":";}else{str = String(hr) + ":";}
  if(mins < 10){str += "0" + String(mins) + ":";}else{str += String(mins) + ":";}
  if(sec < 10){str += "0" + String(sec);}else{str += String(sec);}
  return str;             //Converts to HH:MM:SS string. This can be returned to the calling function.
}

void serial_commands(){
  String command;
  command = Serial.readStringUntil('\n');
  
  if(serial_ok){
    Serial.println("---------------------------------------------------------------------------------------------");
    Serial.println("Welcome! Please login.");
    serial_ok = false;
  }
  
  if(command == PASSWORD){
    password = true;
    Time_serial = millis();
    Serial.println("Login succefully, please use a command.");
    Serial.println("---------------------------------------------------------------------------------------------");
  }else if(password == false){
    Serial.println("Wrong password.");
    Serial.println("---------------------------------------------------------------------------------------------");
  }
  
  if(password == true){
    Time_serial = millis();
    if(command == "help"){
      Serial.println("Valid command:");
      Serial.println("reset hour - Reset motor clock to 0");
      Serial.println("set time [hours] [minutes] [seconds] - Set RTC hours (Ex: set time 09 31 03)"); //por unidades em tudo,condicionais e para tirar a senha
      Serial.println("set date [year] [month] [day] - Set RTC date (Ex: set date 2022 08 24)");
      Serial.println("set cht [min,max] [value] - Set cylinder head temperature min or max by a given value");   
      Serial.println("set mat [min,max] [value] - Set manifold air temperature min or max by a given value");    
      Serial.println("set battery [min,max] [value] - Set battery min or max by a given value");   
      Serial.println("set rpm min [value] - Set RPM min value for which the motor clock start counting"); 
      Serial.println("set idealtemperature [value] - Set the ideal temperature for which the motor should operate"); 
      Serial.println("set fuelpressure [min,max] [value] - Set fuel pressure min or max by a given value");   
      Serial.println("set k_p [servo1/servo2] [value] - Set Kp constant for servo 1 or 2 by a given value"); 
      Serial.println("set k_i [servo1/servo2] [value] - Set Ki constant for servo 1 or 2 by a given value");   
      Serial.println("set k_d [servo1/servo2] [value] - Set Kd constant for servo 1 or 2 by a given value"); 
      Serial.println("see parameters - Expose the saved parameters");
      Serial.println("save config - Saves configuration values in memory (use only at the end of the configuration)");
      Serial.println("---------------------------------------------------------------------------------------------");      
    }
    else if(command == "reset hour"){
      Motor_hours = 0;
      EEPROM.writeLong64(addr, (int64_t) Motor_hours);
      EEPROM.commit();      
      Serial.println("Flight Hours = " + String((long)Motor_hours));
      }
      
    else if(command.substring(0,8) == "set time"){
      RTCTime.Hours = (command.substring(9,11)).toInt();
      RTCTime.Minutes = (command.substring(12,14)).toInt();
      RTCTime.Seconds = (command.substring(15)).toInt();
      M5.Rtc.SetTime(&RTCTime);
      Serial.println("Time: " + String(RTCTime.Hours)+":"+String(RTCTime.Minutes)+":"+String(RTCTime.Seconds));
      }
      
    else if(command.substring(0,8) == "set date"){
      RTCDate.Year = (command.substring(9,13)).toInt();
      RTCDate.Month = (command.substring(14,16)).toInt();
      RTCDate.Date = (command.substring(16)).toInt();
      M5.Rtc.SetDate(&RTCDate);
      Serial.println("Date: " + String(RTCDate.Year) + "/" + String(RTCDate.Month) + "/" + String(RTCDate.Date));
      }
      
    else if(command.substring(0,11) == "set cht min"){
      configy.CHT_MIN = (command.substring(12)).toFloat();
      Serial.println("CHT min = " +String(configy.CHT_MIN)+ " C");
      }
      
    else if(command.substring(0,11) == "set cht max"){
      configy.CHT_MAX = (command.substring(12)).toFloat();
      Serial.println("CHT max = " + String(configy.CHT_MAX)+ " C");
      }
      
    else if(command.substring(0,11) == "set mat min"){
      configy.MAT_MIN = (command.substring(12)).toFloat();
      Serial.println("MAT min = " + String(configy.MAT_MIN)+ " C");
      }
      
    else if(command.substring(0,11) == "set mat max"){
      configy.MAT_MAX = (command.substring(12)).toFloat();
      Serial.println("MAT max = " + String(configy.MAT_MAX)+ " C");
      }

    else if(command.substring(0,15) == "set battery min"){
      configy.BATTERY_MIN = (command.substring(16)).toFloat();
      Serial.println("Battery min = " + String(configy.BATTERY_MIN) + " V");
      }
      
    else if(command.substring(0,15) == "set battery max"){
      configy.BATTERY_MAX = (command.substring(16)).toFloat();
      Serial.println("Battery max = " + String(configy.BATTERY_MAX) + " V");
      }

    else if(command.substring(0,11) == "set rpm min"){
      configy.RPM_HOURS = (command.substring(12)).toFloat();
      Serial.println("RPM min = " + String(configy.RPM_HOURS) + " RPM");
      }
      
    else if(command.substring(0,20) == "set idealtemperature"){
      configy.IDEAL_TEMP = (command.substring(21)).toFloat();
      Serial.println("Idealtemperature  = " + String(configy.IDEAL_TEMP) + " C");
      }

    else if(command.substring(0,20) == "set fuelpressure min"){
      configy.FUEL_PRESSURE_MIN = (command.substring(21)).toFloat();
      Serial.println("Fuelpressure min = " + String(configy.FUEL_PRESSURE_MIN) + " bar");
      }
      
    else if(command.substring(0,20) == "set fuelpressure max"){
      configy.FUEL_PRESSURE_MAX = (command.substring(21)).toFloat();
      Serial.println("Fuelpressure max = " + String(configy.FUEL_PRESSURE_MAX) + " bar");
      }

    else if(command.substring(0,14) == "set k_p servo1"){
      configy.K_P[0] = (command.substring(15)).toFloat();
      Serial.println("k_p (servo 1) = " + String(configy.K_P[0]));
      }

    else if(command.substring(0,14) == "set k_i servo1"){
      configy.K_I[0] = (command.substring(15)).toFloat();
      Serial.println("k_i (servo 1) = " + String(configy.K_I[0]));
      }

    else if(command.substring(0,14) == "set k_d servo1"){
      configy.K_D[0] = (command.substring(15)).toFloat();
      Serial.println("k_d (servo 1) = " + String(configy.K_D[0]));
      }
      
    else if(command.substring(0,14) == "set k_p servo2"){
      configy.K_P[1] = (command.substring(15)).toFloat();
      Serial.println("k_p (servo 2) = " + String(configy.K_P[1]));
      }
      
    else if(command.substring(0,14) == "set k_i servo2"){
      configy.K_I[1] = (command.substring(15)).toFloat();
      Serial.println("k_i (servo 2) = " + String(configy.K_I[1]));
      }
      
    else if(command.substring(0,14) == "set k_d servo2"){
      configy.K_D[1] = (command.substring(15)).toFloat();
      Serial.println("k_d (servo 2) = " + String(configy.K_D[1]));
      }
    else if(command.substring(0,14) == "see parameters"){
      Serial.println("---------------------------------------------------------------------------------------------");
      Serial.println("CHT min = " +String(configy.CHT_MIN)+ " C");
      Serial.println("CHT max = " + String(configy.CHT_MAX)+ " C");
      Serial.println("MAT min = " + String(configy.MAT_MIN)+ " C"); 
      Serial.println("MAT max = " + String(configy.MAT_MAX)+ " C");
      Serial.println("Battery min = " + String(configy.BATTERY_MIN) + " V");
      Serial.println("Battery max = " + String(configy.BATTERY_MAX) + " V");
      Serial.println("RPM min (motor clock) = " + String(configy.RPM_HOURS) + " RPM");
      Serial.println("Idealtemperature = " + String(configy.IDEAL_TEMP) + " C");
      Serial.println("Fuelpressure min = " + String(configy.FUEL_PRESSURE_MIN) + " bar");
      Serial.println("Fuelpressure max = " + String(configy.FUEL_PRESSURE_MAX) + " bar");
      Serial.println("k_p (servo 1) = " + String(configy.K_P[0]));
      Serial.println("k_i (servo 1) = " + String(configy.K_I[0]));
      Serial.println("k_d (servo 1) = " + String(configy.K_D[0]));
      Serial.println("k_p (servo 2) = " + String(configy.K_P[1]));
      Serial.println("k_i (servo 2) = " + String(configy.K_I[1]));
      Serial.println("k_d (servo 2) = " + String(configy.K_D[1]));      
    }
    else if(command.substring(0,11) == "save config"){
      prefs.putBytes("config",(void*) &configy,(size_t) sizeof(configy));
      Serial.println("---------------------------------------------------------------------------------------------");
      Serial.println("CHT min = " +String(configy.CHT_MIN)+ " C");
      Serial.println("CHT max = " + String(configy.CHT_MAX)+ " C");
      Serial.println("MAT min = " + String(configy.MAT_MIN)+ " C"); 
      Serial.println("MAT max = " + String(configy.MAT_MAX)+ " C");
      Serial.println("Battery min = " + String(configy.BATTERY_MIN) + " V");
      Serial.println("Battery max = " + String(configy.BATTERY_MAX) + " V");
      Serial.println("RPM min (motor clock) = " + String(configy.RPM_HOURS) + " RPM");
      Serial.println("Idealtemperature = " + String(configy.IDEAL_TEMP) + " C");
      Serial.println("Fuelpressure min = " + String(configy.FUEL_PRESSURE_MIN) + " bar");
      Serial.println("Fuelpressure max = " + String(configy.FUEL_PRESSURE_MAX) + " bar");
      Serial.println("k_p (servo 1) = " + String(configy.K_P[0]));
      Serial.println("k_i (servo 1) = " + String(configy.K_I[0]));
      Serial.println("k_d (servo 1) = " + String(configy.K_D[0]));
      Serial.println("k_p (servo 2) = " + String(configy.K_P[1]));
      Serial.println("k_i (servo 2) = " + String(configy.K_I[1]));
      Serial.println("k_d (servo 2) = " + String(configy.K_D[1]));  
      Serial.println("Configuration saved.");
      }
    else if (command == PASSWORD){}
    else {
      Serial.println("Wrong command.");
      }         
  }
}


// Função que trata de filtrar os dados da temperatura com filtro de média
/*float mean_temp(float Temps){
  float Temp_mean = 0;
  Temp_array[j] = Temps;
  if(++j == MEAN_SIZE){j=0;}
  for(i = 0; i < MEAN_SIZE; i++){Temp_mean += (Temp_array[i])/((float)(MEAN_SIZE));}
  return Temp_mean;
}*/

// Função para alterar parametros
/*void ParamEvent(Event& e) {
  M5.Rtc.GetDate(&RTCDate);
  M5.Rtc.GetTime(&RTCTime);    
  int aux_i[6] = {RTCTime.Hours, RTCTime.Minutes, RTCTime.Seconds, RTCDate.Year, RTCDate.Month, RTCDate.Date};
  float aux_f[3] = {K_p, K_i, K_d};

  switch(menu){
    case 0:
      M5.Lcd.fillRect(0, 0, 320, 180, WHITE);    
      if(String(e.button->getName()) == "BtnC"){ // Distinguindo que botão foi premido
        Tempi++;
      }else if(String(e.button->getName()) == "BtnB"){
        Tempi--;
      }      
      break;
    case 3:
      M5.Lcd.fillRect(0, 0, 320, 180, WHITE);
      if(String(e.button->getName()) == "BtnC" && e.type == E_DBLTAP){ // Dois cliques aumentam/diminuem 0.1
        aux_f[0] = K_p + 0.1;
        aux_f[1] = K_i + 0.1;
        aux_f[2] = K_d + 0.1;
      }else if(String(e.button->getName()) == "BtnB" && e.type == E_DBLTAP){
        aux_f[0] = K_p - 0.1;
        aux_f[1] = K_i - 0.1;
        aux_f[2] = K_d - 0.1;
      }else if(String(e.button->getName()) == "BtnC" && e.type == E_TAP){ // Um clique aumentam/diminuem 0.01
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
      break;
    case 2:
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
      break;
    default:
      break;
  }
}*/

// Função relativa ao menu 2
/*void menu_2(){
  M5.Rtc.GetDate(&RTCDate);
  M5.Rtc.GetTime(&RTCTime);
  
  str = "Hour: ";
  if(0 <= RTCTime.Hours && RTCTime.Hours < 10){str += "0";}
  str += String(RTCTime.Hours);
  str += ":";
  if(0 <= RTCTime.Minutes && RTCTime.Minutes < 10){str += "0";}  
  str += String(RTCTime.Minutes);
  str += ":";
  if(0 <= RTCTime.Seconds && RTCTime.Seconds < 10){str += "0";}  
  str += String(RTCTime.Seconds);
  M5.Lcd.drawString(str, 0, 40, 2); 
  str = "Date: " + String(RTCDate.Year) + "-";
  if(0 <= RTCDate.Month && RTCDate.Month < 10){str += "0";}  
  str += String(RTCDate.Month);   
  str += "-";
  if(0 <= RTCDate.Date && RTCDate.Date < 10){str += "0";}  
  str += String(RTCDate.Date); 
  M5.Lcd.drawString(str, 0, 0, 2);
  M5.Lcd.drawString((date_str[select_time] + " selected"), 40, 120, 2);
}*/

// Função relativa ao menu 3
/*void menu_3(){
  M5.Lcd.drawString(("Kp: " + String(K_p)), 0, 0, 2);   
  M5.Lcd.drawString(("Ki: " + String(K_i)), 0, 40, 2); 
  M5.Lcd.drawString(("Kd: " + String(K_d)), 0, 80, 2);  
  M5.Lcd.drawString((K_str[select_K] + " selected"), 40, 120, 2);   
}*/

// Função auxiliar para a difinição de horas (passa de 23h para 0h, por exemplo)
/*int Limits(int Value, int SupLimit, int InfLimit){
  if(Value > SupLimit){return InfLimit;}
  if(Value < InfLimit){return SupLimit;}
  return Value;
}*/

// Função que faz passar de parametro em parametro, consuante o menu
/*void Scroll(Event& e){
  switch(menu){
    case 3:
      M5.Lcd.fillRect(0, 0, 320, 180, WHITE);
      if(++select_K >= 3){select_K = 0;}   
      break;
    case 2:
      M5.Lcd.fillRect(0, 0, 320, 180, WHITE);
      if(++select_time >= 6){select_time = 0;}
      break;
    default:
      break;
  }
}*/
/*
void warnings(String aux){
  M5.Lcd.setTextSize(1);
  M5.Lcd.drawString(aux, 0, 225 - select_pos * 20, 2);
  if(--select_pos == -1) select_pos = 2; 
  M5.Lcd.setTextSize(2);
}*/
