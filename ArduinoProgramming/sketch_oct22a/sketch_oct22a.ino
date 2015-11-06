#include <avr/io.h>
#include <avr/interrupt.h>
int DutyCycle;
int main(void){
    
    DutyCycle = 1;
    
    /* Set up PWM signal */
    DDRD |= (1<<DDD6);                   //Set PD6 as output
    OCR0A = DutyCycle;                   //Set duty cycle (DC = x/228)
    TCCR0A |= (1<<COM0A1);               //Set to non inverting
    TCCR0A |= (1<<WGM01)|(1<<WGM00);     //Set to fast PWM
    TCCR0B |= (1<<CS01);                 //Set prescaler to 8 and start PWM
    
    
    /* Set up ADC */
    ADMUX |= (0 << REFS1) | (1 << REFS0); //Reference of AVcc
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | ( 1 << ADPS0); //Enable the ADC and set the prescaler to 128, ADC frequency of 125kHz
    ADCSRA |= (1 << ADATE);
    ADCSRB |= (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);
    ADMUX |= (0b00000111);               //Select ADC channel 7
    ADCSRA |= (1 << ADSC);
    
    
    
 
    while(1){
      delay(100);
      if (ADC<760&&DutyCycle<200){
        DutyCycle++;
      }
      
      if (ADC>780&&DutyCycle>0){
        DutyCycle--;
      }
      
      OCR0A = DutyCycle;                         //Set duty cycle (DC = x/228)
    } 
}


