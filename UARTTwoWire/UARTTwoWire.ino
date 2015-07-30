#include <avr/io.h>
#include <Serial.h>

#define PIN_TX 2
#define PIN_RX 3

//unsigned char data[8] = {1, 1, 0, 1, 0, 0, 1, 0};
unsigned char data[] = "hello";
//unsigned char index = 0;
//unsigned char data = 'M';

void setup(){
  pinMode(PIN_TX, OUTPUT);
  pinMode(PIN_RX, INPUT);
  Serial.begin(9600);
}

void loop(void){
  int i;
  int j = 0;
//  Serial.print(sizeof(data));
  digitalWrite(PIN_TX, HIGH);
  delay(100);
//  digitalWrite(PIN_TX, LOW);
//  delay(1000);
  while( j < sizeof(data)){
    digitalWrite(PIN_TX, LOW);
    delayMicroseconds(52);
    
    for(i = 0; i <8; i++){
      if(((data[j] >> i)& 0x01) == 0x01){
        digitalWrite(PIN_TX, HIGH);
      } else {
        digitalWrite(PIN_TX, LOW);
      }
//      Serial.println(data>>i);      
    delayMicroseconds(46);
    }
    
    digitalWrite(PIN_TX, HIGH);
    delayMicroseconds(52);
    j++;
  }
}
