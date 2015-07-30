#include <avr/io.h>
#include <Serial.h>
#include <FastSerial.h>

#define PIN_TX 2
#define PIN_RX 3

void setup(){
  pinMode(PIN_TX, OUTPUT);
  pinMode(PIN_RX, INPUT);
  Serial.begin(9600);
}

void loop(){
  unsigned char value = 0;
  char i = 0;
//  value = digitalRead(PIN_RX);
//  delay(10000);
//  Serial.print(value);
  
  while(digitalRead(PIN_RX));
//  while(i < 10){
//  value[i] = digitalRead(PIN_RX);
////  Serial.print(value);
//  delayMicroseconds(50);
//  i++;
//  }
//  i= 0;
//  while(i<10){
//    Serial.print(value[i]);
//    i++;
//  }
  delayMicroseconds(52);
  while(i < 8){
    boolean bitr =  digitalRead(PIN_RX);
    value += bitr << i;
    i++;
    delayMicroseconds(50);
  }
  if(digitalRead(PIN_RX) == 1){
     Serial.print(value);
//     Serial.write("%i", value); 
  }
}
