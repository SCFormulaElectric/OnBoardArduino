#include <avr/io.h>
#include <avr/interrupt.h>

#define FAN1 6
#define TEMPERATURESENSOR1 A0

volatile double tempSensor1Value = 0.0;

void setup() {
  Serial.begin(9600);
  pinMode(TEMPERATURESENSOR1, INPUT);
  pinMode(FAN1, OUTPUT);
  initTimer0();
  sei();
}


double evaluateTemperatureExpression(double x) {
    double exponent = -1.11 * pow(10, -4) * x;
    double result = 76.9 * exp(exponent);
    return result;
}

void loop() {
  // put your main code here, to run repeatedly:
  int voltageIn = analogRead(TEMPERATURESENSOR1);
  double convertedVoltage = (double) (voltageIn * (5.0 / 1023.0));
  double division = (500000 / convertedVoltage) - 100000;
  tempSensor1Value = evaluateTemperatureExpression(division);
}

void initTimer0() {
  TIMSK0 |= (1 << OCIE0A);
  TCCR0A |= (1 << WGM01) | (1 << WGM00);
  TCCR0A |= (0b10 << COM0A0); // Turn D11 on at 0x00 and off at OCR2A
	OCR0A = 205;                // Initial pulse duty cycle of 50%
  TCCR0B |= (0b010 << CS20);  // Prescaler = 1024 for 16ms period
}
