#include <RFduinoBLE.h>
#include "PID.h"
#include "Protocole.h"

#define MODEL "OpengDIY71"

//Valeurs recommandées pour montage KIT
#define kP 0.0005
#define kI 0.00000  
#define kD 0.00000 

// SERIAL_NUMBER MUST BE UNIQUE. THE FORMAT IS 6 CHARACTERS from 0-9 to A-Z (FIRST 000001, LAST ZZZZZZ)
// TO RETRIEVE YOUR UNIQUE SERIAL_NUMBER : SEND AN EMAIL TO DEV@OPENRADIATION.NET 
PLEASE MODIFY THIS VALUE : #define SERIAL_NUMBER "000000"

#define VERSION "1.0" //version software
#define SENSOR_TYPE "Geiger-Muller tube"
#define TUBE_TYPE "SBM-20"
#define NOMINAL_TENSION 380

union _intToChar {
  int i;
  char c[2];
} intToChar;

union _floatToChar {
  float f;
  char c[4];
} floatToChar;

int silentMode;
int stealthMode;

// GPIOTE :
#define GPIOTE_COUNT 0
#define GPIOTE_FOLLOW 1
#define GPIOTE_LED 2
#define GPIOTE_BUZZER 3

// Pins
#define PIN_FLASH 0
#define PIN_ALIM 1
#define PIN_PWM 2
#define PIN_MESURE_HT 3
#define PIN_COMPTEUR 4
#define PIN_LEDS 5 
#define PIN_BUZZER 6 


// Asservissement HT (Haute tension)
#define VOLTAGE_DIVIDER_INV 315


#define TOLERANCE 4.0
#define low_tension 200 // Valeur minimale de la haute tension stable
PID pid_HT(kP,kI,kD);
long pidlastTime;

float actual_tension = 0.0;
float set_tension = 0;

byte received_tension = 0; // to be remove after test
byte internal_state = 4; // to be remove after test
byte lastcall_byte = 0xDD; // to be remove after test
byte compteur = 2; // to be remove after test
uint8_t longdata = 0; // to be remove after test
uint8_t data1 = 0; // to be remove after test
uint8_t data2 = 0; // to be remove after test
uint8_t data3 = 0; // to be remove after test
uint8_t data4 = 0; // to be remove after test

// PWM
#define PERIOD 2000
#define PWM_RESOLUTION 255
#define max_duty_cycle 0.7  //Cette valeur est le pourcentage maximal du rapport cyclique que le PWM peut atteindre (produit 890 V)
#define min_duty_cycle 0.12 //Cette valeur est le pourcentage minimal du rapport cyclique que le PWM doit atteindre (debut excitations 10-40 V - zone instable)


float pwm_duty_cycle = 0;
int pwm_count = 0;
//float max_PWM=max_duty_cycle*PWM_RESOLUTION;
float max_PWM=204.0;
//float min_PWM=min_duty_cycle*PWM_RESOLUTION;
float min_PWM=30.6;

// Bluetooth et comptage
int count = 0;
long precTime;
long precTime2;
int isCo = 0;

// Alimentation
#define ALIM_VOLT_DIV_INV 1.3
#define BAT_LOW 3.5
int isBatLowOn = 0;

// Indicateurs à leds
#define LED_ETAT 0
#define LED_BLUETOOTH 1
#define NB_PIXELS 2
int ppi_buzzer ;
int ppi_led ;

const int nb_leds = NB_PIXELS*3;
uint8_t leds[nb_leds];

void TIMER1_INTERUPT(void) {
  if (NRF_TIMER1->EVENTS_COMPARE[0] != 0) {
    if (pwm_count < pwm_duty_cycle) {
      NRF_GPIO->OUTSET = (1UL << PIN_PWM);
    } else {
      NRF_GPIO->OUTCLR = (1UL << PIN_PWM);
    }
    pwm_count++;
    if (pwm_count == PWM_RESOLUTION) {
      pwm_count = 0;
    }
    NRF_TIMER1->EVENTS_COMPARE[0] = 0;
  }
}

// Timer PWM 
// passer en paramètre NRF_TIMER1
// NRF_TIMER0 est utilise par le BT
// NRF_TIMER2 est utilise pour le comptage
void configTimer(NRF_TIMER_Type* nrf_timer, IRQn_Type irqn, callback_t callback) {
  nrf_timer->TASKS_STOP = 1; // Arrete le timer
  nrf_timer->MODE = TIMER_MODE_MODE_Timer;
  nrf_timer->BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);
  nrf_timer->PRESCALER = 4; // résolution de 1 usec
  nrf_timer->TASKS_CLEAR = 1;
  nrf_timer->CC[0] = PERIOD / PWM_RESOLUTION;
  nrf_timer->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
  nrf_timer->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
  attachInterrupt(irqn, callback);
  nrf_timer->TASKS_START = 1; // Redemarre le timer
}

// Timer counter
void configCounter() {
  // Config GPIO Event (gpiote_channel, pin, polaritée)
  nrf_gpiote_event_config(GPIOTE_COUNT, PIN_COMPTEUR, NRF_GPIOTE_POLARITY_LOTOHI);
 
  // Config counter
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Counter;
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
  NRF_TIMER2->TASKS_START = 1;
 
  // Config PPI
  int ppi_counter = find_free_PPI_channel(255);
  NRF_PPI->CH[ppi_counter].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[GPIOTE_COUNT]; // Input
  NRF_PPI->CH[ppi_counter].TEP = (uint32_t)&NRF_TIMER2->TASKS_COUNT; // Output
 
  NRF_PPI->CHEN |= (1 << ppi_counter); // Active le canal PPI
}

void configFollower(){ 
  
  // GPIO Event (gpiote_channel, pin, polaritée)
  nrf_gpiote_event_config(GPIOTE_FOLLOW, PIN_COMPTEUR, NRF_GPIOTE_POLARITY_TOGGLE);
 
  // GPIO Task (gpiote_channel, pin, polaritée, etat_initial)
  nrf_gpiote_task_config(GPIOTE_LED, PIN_FLASH, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
  nrf_gpiote_task_config(GPIOTE_BUZZER, PIN_BUZZER, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
 
  // Valeures possibles pour la polaritée :
  // NRF_GPIOTE_POLARITY_LOTOHI
  // NRF_GPIOTE_POLARITY_HITOLO
  // NRF_GPIOTE_POLARITY_TOGGLE
 
  // Config PPI
  ppi_led = find_free_PPI_channel(255);
  NRF_PPI->CH[ppi_led].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[GPIOTE_FOLLOW]; // Input
  NRF_PPI->CH[ppi_led].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[GPIOTE_LED]; // Output
  NRF_PPI->CHEN |= (1 << ppi_led); // Active le canal PPI
  
  ppi_buzzer = find_free_PPI_channel(255);
  NRF_PPI->CH[ppi_buzzer].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[GPIOTE_FOLLOW]; // Input
  NRF_PPI->CH[ppi_buzzer].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[GPIOTE_BUZZER]; // Output

  NRF_PPI->CHEN |= (1 << ppi_buzzer); // Active le canal PPI
}

void disableFollowerBuzzer() {
  rfduino_ppi_channel_unassign(ppi_buzzer);
  nrf_gpiote_unconfig(GPIOTE_BUZZER);
  pinMode(PIN_BUZZER, OUTPUT);
  NRF_GPIO->OUTCLR = (1UL << PIN_BUZZER);
}

void disableFollowerLED() {
  rfduino_ppi_channel_unassign(ppi_led);
  nrf_gpiote_unconfig(GPIOTE_LED);
  pinMode(PIN_FLASH, OUTPUT);
  NRF_GPIO->OUTCLR = (1UL << PIN_FLASH);
}

void enableFollowerBuzzer(){
  nrf_gpiote_task_config(GPIOTE_BUZZER, PIN_BUZZER, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
  rfduino_ppi_channel_assign(ppi_buzzer, &NRF_GPIOTE->EVENTS_IN[GPIOTE_FOLLOW], &NRF_GPIOTE->TASKS_OUT[GPIOTE_BUZZER]);
}

void enableFollowerLED(){
  nrf_gpiote_task_config(GPIOTE_LED, PIN_FLASH, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
  rfduino_ppi_channel_assign(ppi_led, &NRF_GPIOTE->EVENTS_IN[GPIOTE_FOLLOW], &NRF_GPIOTE->TASKS_OUT[GPIOTE_LED]);
}

// Gestion des leds
// http://forum.rfduino.com/index.php?topic=30.30
void setRGB(int led, uint8_t r, uint8_t g, uint8_t b) {
  #ifdef LED_MODE_GRB
    leds[led*3] = g*0.3;
    leds[led*3+1] = r*0.3;
  #else
    leds[led*3] = r*0.3;
    leds[led*3+1] = g*0.3;
  #endif
  leds[led*3+2] = b*0.3;
} 


void showLeds() {
  noInterrupts();
  for (int wsOut = 0; wsOut < nb_leds; wsOut++) {
    for (int x=7; x>=0; x--) {
      NRF_GPIO->OUTSET = (1UL << PIN_LEDS);
      if (leds[wsOut] & (0x01 << x)) {
        __ASM ( \
              " NOP\n\t" \
              " NOP\n\t" \
              " NOP\n\t" \
              " NOP\n\t" \
              " NOP\n\t" \
              );
        NRF_GPIO->OUTCLR = (1UL << PIN_LEDS);
      } else {
        NRF_GPIO->OUTCLR = (1UL << PIN_LEDS);
        __ASM ( \
              " NOP\n\t" \
              " NOP\n\t" \
              " NOP\n\t" \
              );      
      }
    }
  }
  delayMicroseconds(50); // latch and reset WS2812.
  interrupts();  
}
 
void setup() {
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_MESURE_HT, INPUT);
  pinMode(PIN_COMPTEUR, INPUT);
  pinMode(PIN_LEDS, OUTPUT);
  
  setRGB(LED_BLUETOOTH, 0, 0, 0);
  setRGB(LED_ETAT, 0, 255, 0);
  showLeds();
  
  analogReference(VBG); // Référence de 1.2V interne
 
  configCounter();
  configFollower();

 
  RFduinoBLE.deviceName = MODEL; // Le nom et la description doivent faire
  RFduinoBLE.advertisementData = SERIAL_NUMBER; // moins de 18 octets en tout.
 
  RFduinoBLE.begin();  
 
  precTime = millis();
  pidlastTime = micros();
  configTimer(NRF_TIMER1, TIMER1_IRQn, TIMER1_INTERUPT);
}

void loop() {
  // On vérifie que la radio n'utilise pas les ressources
  while (!RFduinoBLE.radioActive);
  while (RFduinoBLE.radioActive);
  
  if (millis() - precTime > 1000) { // Toutes les secondes, on envoi le comptage et la tension au smartphone
   int alim_tension = ((analogRead(PIN_ALIM) * 360.0 * ALIM_VOLT_DIV_INV) / 1023.0);
   
   float temp = RFduino_temperature(CELSIUS);


   if ((alim_tension<BAT_LOW*100) && (!isBatLowOn)) {
     setRGB(LED_ETAT, 32, 32, 0);
     showLeds();
     isBatLowOn=1;
   }

   NRF_TIMER2->TASKS_CAPTURE[0] = 1; // capture le comptage dans CC[0]
   count = (uint8_t)NRF_TIMER2->CC[0];
  
   sendUint8Packet(OUT_PACKET_COUNT, count);
 

   sendFloat32Packet(OUT_PACKET_TEMPERATURE, temp);
   sendUint8Packet(OUT_PACKET_DEBUG_BYTE1, compteur);compteur++;
   sendFloat32Packet(OUT_PACKET_ACTUAL_TENSION, actual_tension);
   sendFloat32Packet(OUT_PACKET_PWM_DUTY_CYCLE, pwm_duty_cycle);
   sendBuffer();

   NRF_TIMER2->TASKS_CLEAR = 1; // count = 0;
   count = 0;
   
   precTime = millis();
   }
 
 actual_tension = analogRead(PIN_MESURE_HT);    
 actual_tension = ((actual_tension * 3.6) / 1023.0) * VOLTAGE_DIVIDER_INV;
 
 if (isCo==0) {
   set_tension = 0.0;
   pwm_duty_cycle = 0.0;
 }
 
 if (set_tension > low_tension) { 
    if (micros()-pidlastTime > 10000) {
       float dt = (float)(micros()-pidlastTime)*0.000001;
       pidlastTime=micros();
       float d = pid_HT.compute(set_tension, actual_tension, dt);
       pwm_duty_cycle = max(min_PWM, min(pwm_duty_cycle+d, max_PWM));
     }
   } else {
     pwm_duty_cycle = 0.0;
   }
 }


void RFduinoBLE_onConnect() {
  isCo = 1; // Un smartphone s'est connecté
  sendLenString(OUT_PACKET_VERSION,VERSION,strlen(VERSION));
  sendLenString(OUT_PACKET_SENSOR_TYPE,SENSOR_TYPE,strlen(SENSOR_TYPE));
  sendLenString(OUT_PACKET_TUBE_TYPE,TUBE_TYPE,strlen(TUBE_TYPE));
  sendBuffer();
  
  setRGB(LED_BLUETOOTH, 0, 0, 255); // Etat bluetooth
  showLeds();
}
 
void RFduinoBLE_onDisconnect() {
  RFduinoBLE_update_conn_interval(900, 1000);
  isCo = 0; // Un smartphone s'est déconnecté
  setRGB(LED_BLUETOOTH, 0, 0, 0); // Etat bluetooth
  showLeds();
}


// called by rfduino lib when data is send by smartphone, with the data pointer and the used length
// read only the first data, that is the data type and the first values
void RFduinoBLE_onReceive(char *data, int len) {
  if (len<1) return;
  byte datatype = data[0];
  
  switch (datatype) {
      case IN_PACKET_STEALTH:
      stealthMode = data[1];
      if (stealthMode) {
        disableFollowerLED();
      } else {
        enableFollowerLED();
      }
      break;
      case IN_PACKET_SILENT:
      silentMode = data[1];
      if (silentMode) {
        disableFollowerBuzzer();
      } else {
        enableFollowerBuzzer();
      }
      break;
      case IN_PACKET_SET_TENSION:
      set_tension = decodeFloat(data, 1);
      if (set_tension < low_tension) {
        set_tension = 0.0;
      }
      else {
        if (pwm_duty_cycle < min_PWM) {  
          pwm_duty_cycle = min_PWM ;  
        }
      }
      break;

      //allow the smartphone to ask for all the information of the device
      case IN_PACKET_SEND_INFO:
          sendLenString(OUT_PACKET_VERSION,VERSION,strlen(VERSION));
          sendLenString(OUT_PACKET_SENSOR_TYPE,SENSOR_TYPE,strlen(SENSOR_TYPE));
          sendLenString(OUT_PACKET_TUBE_TYPE,TUBE_TYPE,strlen(TUBE_TYPE));
          
          sendFloat32Packet(OUT_PACKET_ACTUAL_TENSION,actual_tension);
          sendBuffer();
      break;
      
      default:
      break;
      
  }

}
