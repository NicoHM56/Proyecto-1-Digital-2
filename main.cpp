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

/*
#####################################
########### Definir Pines ###########
#####################################
*/
#define Sr    18  // Servo motor
#define BTN1  14  // Botón temperatura
#define BTN2   5  // Segundo botón
#define BTN3  27  // Botón contador

#define LED1  23  // Rojo
#define LED2  12  // Verde (ojo: GPIO12 es pin de arranque; evitarlo alto en boot)
#define LED3  15  // Azul

#define St    32  // Sensor de temperatura (ADC)

// Servo

#define pwmChannel 4   
#define freqPWM 50     
#define resPWM 12   

// Display

#define D1 16
#define D2 17 
#define D3 1

// Patas del Display

#define A   19 

#define B   21 
#define C   22 
#define D   33 
#define E   25 
#define F   26 
#define G   4 
#define dp 13 

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
###########################################
########### Prototipos/funciones ##########
###########################################
*/

float temperatura(int pin);
void  semaforo(float tC);

// Servo motor
void initPWM(void);

/*
##########################################
########### Programación base ############
##########################################
*/
void setup() {
  Serial.begin(115200);

  // Botones
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);

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

  
  
}

void loop() {
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





//########################################################################


   









