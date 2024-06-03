#include <TeensyTimerTool.h>
#include <SPI.h>

// Can Initialization
#define CAN_2515
#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
const int SPI_CS_PIN = BCM8;
const int CAN_INT_PIN = BCM25;
#else
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#endif
#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin
#endif
#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif

IntervalTimer timer_for_fans;     // Create an IntervalTimer object
volatile bool timerExpired = false;

// Fan 1 and 2 Intiialization
using namespace TeensyTimerTool;
#define FAN1 19
#define FAN2 18
#define PUMP1 19
#define PUMP2 18
PeriodicTimer fanTimer(TCK);
PeriodicTimer pumpTimer(GPT1);

const uint32_t frequency = 1000; // PWM frequency in Hz
const float dutyCycle = 0.2;     // Duty cycle of low time  as a fraction (0.5 for 50%) 

int LED = 3;

// CAN Messages
unsigned char stmp[8] = {0x3D, 0x30, 0x64, 0, 0, 0, 0, 0};

void setup() {
  // pinMode(FAN1, OUTPUT);
  // pinMode(FAN2, OUTPUT);
  pinMode(PUMP1, OUTPUT);
  pinMode(PUMP2, OUTPUT);
  startFanPWM();
  startPumpPWM();

  pinMode (LED, OUTPUT) ;

  SERIAL_PORT_MONITOR.begin(115200);
  while(!Serial){};
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {            // init can bus : baudrate = 500k
      SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
      delay(100);
  }
  SERIAL_PORT_MONITOR.println("CAN init ok!");
}

void loop() {
    delay(1000);                      // send data per 1000ms
    unsigned char len = 8;
    unsigned char buf[8];
    if (CAN_MSGAVAIL == CAN.checkReceive()) {        // check if data coming
      // SERIAL_PORT_MONITOR.println("recieved...");
      digitalWrite(LED,HIGH);

      CAN.readMsgBuf(&len, buf);   // read data, len: data length, buf: data buf
      unsigned long canId = CAN.getCanId();
      SERIAL_PORT_MONITOR.println("-----------------------------");
      SERIAL_PORT_MONITOR.print("Get data from ID: 0x");
      SERIAL_PORT_MONITOR.println(canId, HEX);
      for (int i = 0; i < len; i++) { // print the data
          SERIAL_PORT_MONITOR.print(buf[i], HEX);
          SERIAL_PORT_MONITOR.print("\t");
      }
      SERIAL_PORT_MONITOR.println();
      digitalWrite(LED,LOW);
      
      int speed = buf[3];

      if(speed > 2) //something related to seeing the throttle)
      {
        timerExpired == false; 

        pwm_stop_fan();
        timer_for_fans.end();
      }
      else {
        // add a timer to look for 30s seconds while not seeinga any messages from throttle
        timer_for_fans.begin(timerCallback, 30000000); 
        if(timerExpired == true){
          // turn on fan 
          startFanPWM();
          // else
        }

      }
    }

    SERIAL_PORT_MONITOR.println("sending...");
    CAN.sendMsgBuf(0x201, 0, 3, stmp);
}

void timerCallback() {
  timerExpired = true;  // Set the flag to indicate the timer has expired
}

// Fan Timers
void startFanPWM() {
  uint32_t period = 1e6 / frequency;              // Total period in microseconds
  uint32_t highTime = period * dutyCycle;         // High time in microseconds

  fanTimer.begin(pwm_high_fan, highTime);               // Start with the high phase
}


void pwm_high_fan() {
  digitalWriteFast(FAN1, HIGH);
  digitalWriteFast(FAN2, HIGH);
  uint32_t period = 1e6 / frequency;              // Total period in microseconds
  uint32_t lowTime = period * (1.0 - dutyCycle);  // Low time in microseconds

  fanTimer.begin(pwm_low_fan, lowTime);                 // Schedule the low phase
}

void pwm_low_fan() {
  digitalWriteFast(FAN1, LOW);
  digitalWriteFast(FAN2, LOW);
  uint32_t period = 1e6 / frequency;              // Total period in microseconds
  uint32_t highTime = period * dutyCycle;         // High time in microseconds

  fanTimer.begin(pwm_high_fan, highTime);               // Schedule the high phase
}

void pwm_stop_fan() {
  digitalWriteFast(FAN1, LOW);
  digitalWriteFast(FAN2, LOW);
  fanTimer.end();                                   // Stop the timer
}

// Pump Timers
void startPumpPWM() {
  uint32_t period = 1e6 / frequency;              // Total period in microseconds
  uint32_t highTime = period * dutyCycle;         // High time in microseconds

  pumpTimer.begin(pwm_high_pump, highTime);               // Start with the high phase
}

void pwm_high_pump() {
  digitalWriteFast(PUMP1, HIGH);
  digitalWriteFast(PUMP2, HIGH);
  uint32_t period = 1e6 / frequency;              // Total period in microseconds
  uint32_t lowTime = period * (1.0 - dutyCycle);  // Low time in microseconds

  fanTimer.begin(pwm_low_pump, lowTime);                 // Schedule the low phase
}

void pwm_low_pump() {
  digitalWriteFast(PUMP1, LOW);
  digitalWriteFast(PUMP2, LOW);
  uint32_t period = 1e6 / frequency;              // Total period in microseconds
  uint32_t highTime = period * dutyCycle;         // High time in microseconds

  fanTimer.begin(pwm_high_pump, highTime);                 // Schedule the low phase
}

void pwm_stop_pump() {
  digitalWriteFast(PUMP1, LOW);
  digitalWriteFast(PUMP2, LOW);
  pumpTimer.end();                                   // Stop the timer
}