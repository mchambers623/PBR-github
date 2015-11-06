#include <avr/io.h>
#include <avr/interrupt.h>
int IncrementPin = 2;   
int PotDirPin = 3;              

void setup()
{
  ADMUX |= (0 << REFS1) | (1 << REFS0); //Reference of AVcc
  ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | ( 1 << ADPS0); //Enable the ADC and set the prescaler to 128, ADC frequency of 125kHz
  ADCSRA |= (1 << ADATE);
  ADCSRB |= (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);
  ADMUX |= (0b00000111);               //Select ADC channel 6
  ADCSRA |= (1 << ADSC);
  pinMode(IncrementPin, OUTPUT);      // sets the digital pin as output
  pinMode(PotDirPin, OUTPUT);      // sets the digital pin as output
  digitalWrite(IncrementPin, HIGH);
  Serial.begin(250000);
}

void loop()
{
  if(ADC>=800){
    digitalWrite(PotDirPin, HIGH);
    digitalWrite(IncrementPin, LOW);   // sets the LED on
    digitalWrite(IncrementPin, HIGH);    // sets the LED off
  }
  if(ADC<790){
    digitalWrite(PotDirPin, LOW);
    digitalWrite(IncrementPin, LOW);   // sets the LED on
    digitalWrite(IncrementPin, HIGH);    // sets the LED off
  }
  delay(1000);                  // waits for a second
  Serial.println(ADC);
}
