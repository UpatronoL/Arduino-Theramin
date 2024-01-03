#include <SPI.h>
#include <TimerOne.h>

#define RATE 20000

//const int ADCSSN  = 10;
const int DACSSN  =  9;
const int LDACN   =  8;
const int TRIG = 6;
const int ECHO = 7;

void setup() {
//  pinMode(ADCSSN, OUTPUT);  digitalWrite(ADCSSN, HIGH);  
  pinMode(DACSSN, OUTPUT);  digitalWrite(DACSSN, HIGH);  
  pinMode(LDACN,  OUTPUT);  digitalWrite(LDACN,  HIGH); 
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(A0, INPUT);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2); 
  SPI.setDataMode(SPI_MODE0);           
  SPI.setBitOrder(MSBFIRST);
  Timer1.initialize(1000000UL/RATE);
  Timer1.attachInterrupt(timer);
//  Serial.begin(115200);
}

void setDAC(unsigned int a, unsigned int b) {
  PORTB &= ~(1 << (DACSSN & 7)); 
  SPI.transfer(0b00010000 | ((a >> 8) & 0b00001111));
  SPI.transfer(a);
  PORTB |=  (1 << (DACSSN & 7));
#if 0
  PORTB &= ~(1 << (DACSSN & 7));
  SPI.transfer(0b10010000 | ((b >> 8) & 0b00001111));
  SPI.transfer(b);
  PORTB |=  (1 << (DACSSN & 7)); 
#endif
  PORTB &= ~(1 << (LDACN  & 7)); 
  PORTB |=  (1 << (LDACN  & 7)); 
}

#include "sine.h"

const unsigned int freq = 1000, rate = RATE;
volatile unsigned int phase1 = 0, delta1 = (uint32_t)freq * 4096L / rate;
volatile unsigned int phase2 = 0, delta2 = (uint32_t)freq * 4096L / rate;
int numer = 0;
unsigned int denom = 1;
volatile int volControl = 4;  // SET THIS FROM 0 (LOUD) TO 9 (QUIET)
volatile int val;

void timer(void) {
  //if (val<270) {
    int s1 = sine(phase1) - 2048;
  //  int s2 = sine(phase2) - 2048;
    s1 >>= volControl;
  //  s2 >>= volControl;
    s1 += 2048;
  //  s2 += 2048;
    setDAC(s1, 0/*s2*/);
    phase1 += delta1;
  //  phase2 += delta2;
  //}
}

double dmap(double inVal, double inMin, double inMax, double outMin, double outMax){
  double co = (outMax-outMin)/(inMax-inMin);
  return inVal*co+outMin;
}

void setFreq1(unsigned int freq) { delta1 = (uint32_t)freq * 4096L / rate; }
void setFreq2(unsigned int freq) { delta2 = (uint32_t)freq * 4096L / rate; }

void setFreqs(unsigned int a, unsigned int b) {
  setFreq1(a);
  setFreq2(b);
}

void setNotes(int a, int b) {
  setFreqs(a, b);
}

void sendWave(int a)
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(a);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(a);
  digitalWrite(TRIG, LOW);
}

void loop() {
  sendWave(5);
  int pitchControl = pulseIn(ECHO, HIGH) * 0.34 / 2;
  if (pitchControl>400) pitchControl=400;
  int i = map(pitchControl, 0, 400, 0, 1500);
  setNotes(i, i);
  val = analogRead(A0);
//  Serial.println(val);
}
