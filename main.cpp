/*
#################################
########### Librerías ###########
#################################
*/
#include <Arduino.h>
#include <stdint.h>
#include <driver/ledc.h>
#include <math.h>
#include "config.h"      // Debe definir el objeto global 'io' de Adafruit IO
#include <AdafruitIO.h>

/*
#####################################
########### Definir Pines ###########
#####################################
*/
#define Sr    5    // Servo motor (LEDC)
#define BTN1  23   // Botón

#define LEDR  4    // Rojo (PWM)
#define LEDV  2    // Verde (PWM)
#define LEDA  15   // Azul  (PWM)

// Sensor de temperatura (ADC) - LM35
#define LM35PIN 34
#define ADCPIN  35
#define TEMP_PIN LM35PIN     // Cambia a ADCPIN si usas GPIO35

/*
###########################################
########### PWM (LEDC) parámetros #########
###########################################
*/
#define CH_LEDR  0
#define CH_LEDV  1
#define CH_LEDA  2
#define CH_SERVO 3

const uint32_t PWM_FRECUENCIA_LED = 5000; // 5 kHz para LEDs
const uint8_t  PWM_RESOLUCION     = 12;   // 12 bits
const uint16_t PWM_MAX            = 4095; // 0..4095

// Servo
#define FREQ_SERVO 50     // 50 Hz (20 ms)
#define RES_SERVO  12     // 12 bits

// Duty para servo aprox. (50 Hz, 12 bits)
#define SERVO_IZQ  102    // ~0.5 ms  (extremo izquierdo)
#define SERVO_CEN  307    // ~1.5 ms  (centro)
#define SERVO_DER  512    // ~2.5 ms  (extremo derecho)

/*
#################################################
############ Variables IO módulo Wifi ###########
#################################################
*/
#define IO_LOOP_DELAY 5000UL
unsigned long lastUpdate = 0;
AdafruitIO_Feed *canalTemperatura = io.feed("temperatura");

/*
#####################################
########### Prototipos  #############
#####################################
*/
void initButton();
void IRAM_ATTR BTN1_ISR();

float leerTemperaturaC_estable();
void  semaforoYServo(float tC);

/*
#####################################
############ Variables  #############
#####################################
*/
// Botón
volatile bool btn1ON = false;
volatile uint32_t lastISRbtn1 = 0;
const uint32_t antiBounce = 200; // ms

// ===== Calibración y filtros =====
const float ADC_VREF      = 5.5f;    // Vref efectivo con atenuación (ajústalo a tu hardware)
const float TEMP_Kv       = 100.0f;  // 10 mV/°C => V*100 = °C
const float TEMP_OFFSET_C = 12.0f;   // offset opcional (ajústalo si quieres)

// Multimuestreo + EMA
const int   N_SAMPLES = 16;
const int   TRIM_K    = 2;
const float ALPHA     = 0.15f;        // EMA
static bool  emaInit  = false;
static float emaTemp  = 0.0f;

// Umbrales y histeresis (°C)
const float T_LOW  = 25.0f;   // límite entre BAJA y MEDIA
const float T_HIGH = 27.0f;   // límite entre MEDIA y ALTA
const float HYST   = 0.3f;    // banda muerta

enum Zona { Z_BAJA, Z_MEDIA, Z_ALTA };
static Zona zonaActual = Z_MEDIA;

// Última lectura para logs
float tC_last = 0.0f;

/*
##########################################
########### Programación base ############
##########################################
*/
void setup() {
  Serial.begin(115200);
  initButton();

  // LEDs por PWM
  ledcSetup(CH_LEDR, PWM_FRECUENCIA_LED, PWM_RESOLUCION);
  ledcSetup(CH_LEDV, PWM_FRECUENCIA_LED, PWM_RESOLUCION);
  ledcSetup(CH_LEDA, PWM_FRECUENCIA_LED, PWM_RESOLUCION);
  ledcAttachPin(LEDR, CH_LEDR);  // ROJO -> GPIO4
  ledcAttachPin(LEDV, CH_LEDV);  // VERDE -> GPIO2
  ledcAttachPin(LEDA, CH_LEDA);  // AZUL  -> GPIO15
  ledcWrite(CH_LEDR, 0);
  ledcWrite(CH_LEDV, 0);
  ledcWrite(CH_LEDA, 0);

  // ADC
  analogReadResolution(12);                 // 0..4095
  analogSetPinAttenuation(TEMP_PIN, ADC_11db);

  // Servo
  ledcSetup(CH_SERVO, FREQ_SERVO, RES_SERVO);
  ledcAttachPin(Sr, CH_SERVO);
  ledcWrite(CH_SERVO, SERVO_CEN);

  // Adafruit IO
  while (!Serial) { delay(10); }
  Serial.print("Connecting to Adafruit IO");
  io.connect();
  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println(io.statusText());
}

void loop() {
  io.run();

  // Leer temp (multimuestreo + EMA), limitar a 0..99.9
  tC_last = leerTemperaturaC_estable();

  // Acciones del botón
  if (btn1ON) {
    btn1ON = false;

    Serial.print("Temperatura (est): ");
    Serial.print(tC_last, 1);
    Serial.println(" °C");

    semaforoYServo(tC_last);

    canalTemperatura->save(tC_last);
  }

  // Telemetría periódica
  if (millis() - lastUpdate > IO_LOOP_DELAY) {
    canalTemperatura->save(tC_last);
    lastUpdate = millis();
  }
}

/*
###############################################
########### Programación Funciones ############
###############################################
*/
void initButton() {
  pinMode(BTN1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN1), BTN1_ISR, FALLING);
}

void IRAM_ATTR BTN1_ISR() {
  uint32_t t = millis();
  if (t - lastISRbtn1 > antiBounce) {
    btn1ON = true;
    lastISRbtn1 = t;
  }
}

/* ========= Lectura estable del LM35 ========= */
float leerTemperaturaC_estable() {
  (void)analogRead(TEMP_PIN);  // dummy read para estabilizar

  int v[N_SAMPLES];
  for (int i = 0; i < N_SAMPLES; ++i) {
    v[i] = analogRead(TEMP_PIN);
    delayMicroseconds(500);
  }

  int min1 = 4096, min2 = 4096, max1 = -1, max2 = -1;
  long sum = 0;
  for (int i = 0; i < N_SAMPLES; ++i) {
    int x = v[i];
    sum += x;
    if (x <= min1) { min2 = min1; min1 = x; }
    else if (x < min2) { min2 = x; }
    if (x >= max1) { max2 = max1; max1 = x; }
    else if (x > max2) { max2 = x; }
  }

  long trimmedSum = sum - min1 - min2 - max1 - max2;
  int trimmedN = N_SAMPLES - 2*TRIM_K;
  if (trimmedN < 1) {
    trimmedSum = sum;
    trimmedN   = N_SAMPLES;
  }

  float adcMean = (float)trimmedSum / (float)trimmedN;

  // ADC -> Volt -> °C (LM35: 10 mV/°C) + offset
  float volt  = adcMean * (ADC_VREF / 4095.0f);
  float tempC = volt * TEMP_Kv + TEMP_OFFSET_C;

  // EMA (filtro paso-bajo)
  if (!emaInit) { emaTemp = tempC; emaInit = true; }
  else          { emaTemp = ALPHA * tempC + (1.0f - ALPHA) * emaTemp; }

  if (emaTemp < 0)     emaTemp = 0;
  if (emaTemp > 99.9f) emaTemp = 99.9f;

  return emaTemp;
}

/* ====== Semáforo + servo con histeresis ====== */
void semaforoYServo(float tC) {
  Zona nueva = zonaActual;

  switch (zonaActual) {
    case Z_BAJA:
      // Sube a MEDIA cuando pase T_LOW + HYST
      if (tC >= T_LOW + HYST) nueva = Z_MEDIA;
      break;

    case Z_MEDIA:
      // Baja a BAJA cuando caiga por debajo de T_LOW - HYST
      if (tC < T_LOW - HYST) nueva = Z_BAJA;
      // Sube a ALTA cuando pase T_HIGH + HYST
      else if (tC >= T_HIGH + HYST) nueva = Z_ALTA;
      break;

    case Z_ALTA:
      // Baja a MEDIA cuando caiga por debajo de T_HIGH - HYST
      if (tC < T_HIGH - HYST) nueva = Z_MEDIA;
      break;
  }

  zonaActual = nueva;

  // LEDs: LEDR=rojo (GPIO4), LEDV=verde (GPIO2), LEDA=azul (GPIO15)
  uint16_t dutyR = 0, dutyV = 0, dutyA = 0;
  uint16_t servoDuty = SERVO_CEN;

  if (zonaActual == Z_BAJA) {         // t < 25
    dutyA = PWM_MAX;                  // AZUL encendido
    servoDuty = SERVO_DER;            // opcional: posición "frío"
  } else if (zonaActual == Z_MEDIA) { // 25 ≤ t < 27
    dutyV = PWM_MAX;                  // VERDE encendido
    servoDuty = SERVO_CEN;
  } else {                            // t ≥ 27
    dutyR = PWM_MAX;                  // ROJO encendido
    servoDuty = SERVO_IZQ;            // opcional: "caliente"
  }

  ledcWrite(CH_LEDR, dutyR);
  ledcWrite(CH_LEDV, dutyV);
  ledcWrite(CH_LEDA, dutyA);
  ledcWrite(CH_SERVO, servoDuty);
}
