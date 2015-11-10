#include <avr/io.h>
#include <avr/interrupt.h>
int DutyCycle = 1;
int IncrementPin = 2;   
int PotDirPin = 3;  
int IsolationFET = 5;
void setup() {
    
    
    /* Set up PWM signal */
    DDRD |= (1<<DDD6);                   //Set PD6 as output
    OCR0A = DutyCycle;                   //Set duty cycle (DC = x/228)
    TCCR0A |= (1<<COM0A1);               //Set to non inverting
    TCCR0A |= (1<<WGM01)|(1<<WGM00);     //Set to fast PWM
    TCCR0B |= (1<<CS01);                 //Set prescaler to 8 and start PWM
    
    
//    /* Set up ADC */
//    ADMUX |= (0 << REFS1) | (1 << REFS0); //Reference of AVcc
//    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | ( 1 << ADPS0); //Enable the ADC and set the prescaler to 128, ADC frequency of 125kHz
//    ADCSRA |= (1 << ADATE);
//    ADCSRB |= (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);
//    ADMUX |= (0b00000111);               //Select ADC channel 7
//    ADCSRA |= (1 << ADSC);

    pinMode(IsolationFET, OUTPUT);      // sets the digital pin as output
    digitalWrite(IsolationFET, HIGH);
    Serial.begin(19200);
    Serial.println("Hi");
}

void loop() {
  int i=0;
  for(i=0; i<=20; i++){
      int BoostInput = analogRead(7);
      if (BoostInput<760&&DutyCycle<200){
        DutyCycle++;
      }
      
      if (BoostInput>780&&DutyCycle>0){
        DutyCycle--;
      }

      delay(50);
      Serial.print("Boost: ");
      Serial.println(BoostInput);
      OCR0A = DutyCycle;                         //Set duty cycle (DC = x/228)
      
  }
 int PotInput = analogRead(6);
      if(PotInput>=560){
        digitalWrite(PotDirPin, LOW);
        digitalWrite(IncrementPin, HIGH);   // sets the LED on
        delay(1);
        digitalWrite(IncrementPin, LOW);    // sets the LED off
        Serial.println("GO DOWN");
      }
      if(PotInput<550){
        digitalWrite(PotDirPin, HIGH);
        digitalWrite(IncrementPin, HIGH);   // sets the LED on
        delay(.01);
        digitalWrite(IncrementPin, LOW);    // sets the LED off
        Serial.println("GO UP");
      }

      Serial.print("Pot: ");
      Serial.println(PotInput);
     
    } 


