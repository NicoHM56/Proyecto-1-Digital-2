//******************************************/
// Universidad del Valle de Guatemala
// BE3029 - Electronica Digital 2
// Pablo Mazariegos
// 12/08/2025
// Proyecto 1
// MCU: ESP32 dev kit 1.0
//******************************************/


/*
#################################
########### Librerías ###########
#################################
*/

#include <Arduino.h>
#include <stdint.h>        
#include <driver/ledc.h>   // Señales PWM
#include "config.h" // Adafruit io Arduino
#include <AdafruitIO.h>

/*
#####################################
########### Definir Pines ###########
#####################################
*/
#define Sr    5  // Servo motor
#define BTN1  14  // Botón temperatura


#define LED1  15  // Rojo
#define LED2  2  // Verde (ojo: GPIO12 es pin de arranque; evitarlo alto en boot)
#define LED3  4  // Azul

#define St    34  // Sensor de temperatura (ADC)

// Servo

#define pwmChannel 4   
#define freqPWM 50     
#define resPWM 12   

// Display

#define D1 18
#define D2 19
#define D3 21

// Patas del Display

#define A   13
#define B   12 
#define C   14 
#define D   27 
#define E   26 
#define F   25 
#define G   33
#define dp  32

// ADC
#define ADCPIN 35

/*
###########################################
########### PWM (LEDC) parámetros #########
###########################################
*/
#define CH_LED1  0
#define CH_LED2  1
#define CH_LED3  2

const uint32_t PWM_FRECUENCIA   = 5000;   // 5 kHz 
const uint8_t  PWM_RESOLUCION   = 12;     // 12 bits (0..4095)
const uint16_t PWM_MAX          = 4095;   // Resolución Máxima 4096-1

/*
######################################
############ Variables ADC ###########
######################################
*/

int adcRaw;
float adcFiltrado;

float adcRawEMA = 0; // Y(0)
float adcFiltrado = adcRaw; // S(0) = Y(0)
float alpha = 0.07; // Factor de suavizado (0-1)

/*
#################################################
############ Variables IO modulo Wifi ###########
#################################################

Adafruit IO */

#define IO_LOOP_DELAY 5000
unsigned long lastUpdate = 0;

// set up the 'counter' feed
AdafruitIO_Feed *canalTemperatura = io.feed("temperatura");



/*
###########################################
########### Prototipos/funciones ##########
###########################################
*/

float temperatura(int pin);
void  semaforo(float tC);

// Servo motor
void initPWM(void);

//ADC
void getADCEMA(void);
void getADCRAW(void);

/*
##########################################
########### Programación base ############
##########################################
*/
void setup() {
  Serial.begin(115200);

  // Botones
  pinMode(BTN1, INPUT_PULLUP);

  // Configurar canales PWM y asociar pines
  ledcSetup(CH_LED1, PWM_FRECUENCIA, PWM_RESOLUCION);
  ledcSetup(CH_LED2, PWM_FRECUENCIA, PWM_RESOLUCION);
  ledcSetup(CH_LED3, PWM_FRECUENCIA, PWM_RESOLUCION);

  ledcAttachPin(LED1, CH_LED1);
  ledcAttachPin(LED2, CH_LED2);
  ledcAttachPin(LED3, CH_LED3);

  // Entrada analógica (ESP32)
  analogReadResolution(12);               // 0–4095
  analogSetPinAttenuation(St, ADC_11db);  // ~0–3.6 V

  // Apagar LEDs al inicio
  ledcWrite(CH_LED1, 0);
  ledcWrite(CH_LED2, 0);
  ledcWrite(CH_LED3, 0);

  // Servo motor
  initPWM();

  //adafruit IO

  // wait for serial monitor to open
  while(! Serial);

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // set up a message handler for the count feed.
  // the handleMessage function (defined below)
  // will be called whenever a message is
  // received from adafruit io.
  //counter->onMessage(handleMessage);

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
  //counter->get();






  
}

void loop() {

  //ADC
  getADCRAW();
  getADCEMA();
  Serial.print('ADCRAW: ');
  Serial.println(adcRaw);
  Serial.print(', ADC_FIltrado:');
  Serial.println(adcFiltrado);

  delay(10);
  

  //Temperatura
  float tC = temperatura(St);

  // Mostrar temperatura al presionar BTN1 
  if (digitalRead(BTN1) == LOW) {
    Serial.print("Temperatura: ");
    Serial.print(tC, 2);
    Serial.println(" °C");
    delay(250); // anti-rebote 

    // Semáforo por temperatura (PWM)
    semaforo(tC);

  }

  // IO
   // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

  if (millis() > (lastUpdate + IO_LOOP_DELAY)) {
    // save count to the 'counter' feed on Adafruit IO
    Serial.print("sending -> ");
    Serial.println(tC);
    canalTemperatura->save(tC);

    // after publishing, store the current time
    lastUpdate = millis();
  }


  
}

/*
###############################################
########### Programación Funciones ############
###############################################
*/

// Conversor de temperatura 
float temperatura(int pin) {
  int lectura = analogRead(pin); // Leer el sensor de temperatura 
  float tempC = map(lectura, 0, 4095, 0, 125); // Convertir a temperatura
  return tempC;
}

// Semáforo
void semaforo(float p) {
  // Por defecto todas las LEDS apagadas
  uint16_t dutyR = 0, dutyV = 0, dutyA = 0;

  if (p < 45.0f) {
    dutyR = PWM_MAX;      // Rojo endencido
    // Servo -90
    ledcWrite(pwmChannel, 205);
    

  } else if (p <= 76.0f) {
    dutyV = PWM_MAX;      // Verde encendido
    // Servo 0
    ledcWrite(pwmChannel, 307);
    
  

  } else {
    dutyA = PWM_MAX;      // Azul encendido
    // Servo 90
    ledcWrite(pwmChannel, 410);
  
  }

  // Enviar PWMs
  ledcWrite(CH_LED1, dutyR);
  ledcWrite(CH_LED2, dutyV);
  ledcWrite(CH_LED3, dutyA);
}

void initPWM(void) {
  ledcSetup(pwmChannel, freqPWM, resPWM);
  ledcAttachPin(Sr, pwmChannel);
  ledcWrite(pwmChannel, 4);
}

void getADCRAW(void){
  adcRaw=analogRead(ADCPIN);
}

void getADCEMA(void){
  
  adcRaw = analogRead(ADCPIN);
  adcFiltrado = (alpha * adcRawEMA) + ((1.0 -alpha ) * adcFiltrado);
}



//########################################################################


   









