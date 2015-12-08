/* Include Files */
#include <avr/io.h>
#include <avr/interrupt.h>


/* Digital Pins */ 
int MainFET = 3;
int FloatFET = 2; 
int InitialLED = 7;
int MainLED = 8;
int FloatLED = 9;
int PWMOutput = 5;

/* Analog Pins */
int CurrentCheckInput = A7;
int BatteryInput = A6;

/* ADC Values */
int InitialCurrent = 760;
int MainCurrent = 375;
int FloatCurrent = 36;
int Current;
int BoostLimit = 880;
int DutyCycle = 1;
int Counter = 0;
int Stage = 0;
int FloatVoltage = 750;

void setup() {

    /* Set PWM frequency to 62kHz */
    TCCR0B = (TCCR0B & 0b11111000) | 0b00000001;

    
    /* Initialize Timer 1 used for interupt*/
    cli();                               // disable global interrupts
    TCCR1A = 0;                          // set entire TCCR1A register to 0
    TCCR1B = 0;                          // same for TCCR1B

    
    /* Set up timer to interrupt every second */
    OCR1A = 15624;                       // set compare match register to desired timer count
    TCCR1B |= (1 << WGM12);              // turn on CTC mode
    TCCR1B |= (1 << CS10);               // Set CS10 bits for 1024 prescaler
    TCCR1B |= (1 << CS12);               // Set CS12 bits for 1024 prescaler
    TIMSK1 |= (1 << OCIE1A);             // enable timer compare interrupt
    sei();                               // enable global interrupts
    
    
    /* Set up output pin modes */
    pinMode(MainFET, OUTPUT);            // sets the Main Stage FET pin as output
    pinMode(FloatFET, OUTPUT);           // sets the Float Stage FET pin as output
    pinMode(InitialLED, OUTPUT);         // sets the Intitial LED pin as output
    pinMode(MainLED, OUTPUT);            // sets the Main LED pin as output
    pinMode(FloatLED, OUTPUT);           // sets the Float LED pin as output
    pinMode(PWMOutput, OUTPUT);          // sets PWM pin as output


    /* Set up input pin modes */
    pinMode(BatteryInput, INPUT);        // sets battery pin as an input
    pinMode(CurrentCheckInput, INPUT);   // sets current checkpoint as input


    /* Configure output pin initial conditions */
    digitalWrite(MainFET, HIGH);         // Main Stage Resistor is not part of circuit
    digitalWrite(FloatFET, HIGH);        // Float stage resistor is not part of circuit
    digitalWrite(InitialLED, LOW);       // Battery is not connected to boost converter
    digitalWrite(MainLED, HIGH);         // Main Stage Resistor is not part of circuit
    digitalWrite(FloatLED, HIGH);        // Float stage resistor is not part of circuit


    /* Set up serial */
    Serial.begin(19200);
    Serial.println("Configured");
}

void loop(){
  CheckBatteryVoltage();
  while(1){
      ChargeBattery();
  }
}

void CheckBatteryVoltage(void){

    /* Take reading of battery through a 1:3 voltage divider */
    int BatteryVoltage = analogRead(BatteryInput);

    /* If battery voltage is below */
    if(BatteryVoltage < 695){
      
        /* Set output LED's */
        digitalWrite(InitialLED, HIGH);
        digitalWrite(MainLED, LOW);
        digitalWrite(FloatLED, LOW);

        /* Set current setting FET's */
        digitalWrite(MainFET, HIGH);     // Main Stage Resistor is not part of circuit
        digitalWrite(FloatFET, HIGH);    // Float stage resistor is not part of circuit

        /* Set Charging Current */
        Current = InitialCurrent;

        /* Set stage of charging */
        Stage = 0;
    }
    
    if(BatteryVoltage >= 695 && BatteryVoltage < 720){
        /* Set output LED's */
        digitalWrite(InitialLED, LOW);
        digitalWrite(MainLED, HIGH);
        digitalWrite(FloatLED, LOW);

        /* Set current setting FET's */
        digitalWrite(MainFET, LOW);       // Main Stage Resistor is part of circuit
        digitalWrite(FloatFET, HIGH);     // Float stage resistor is not part of circuit

        /* Set Charging Current */
        Current = MainCurrent;

        /* Set stage of charging */
        Stage = 1;
    }

    if(BatteryVoltage >= 720){
        /* Set output LED's */
        digitalWrite(InitialLED, LOW);
        digitalWrite(MainLED, LOW);
        digitalWrite(FloatLED, HIGH);
        
        /* Set current setting FET's */
        digitalWrite(MainFET, LOW);       // Main Stage Resistor is part of circuit
        digitalWrite(FloatFET, LOW);      // Float stage resistor is part of circuit

        /* Set Charging Current */
        Current = FloatCurrent;

        /* Set stage of charging */
        Stage = 2;
    }
}


void ChargeBattery(void){

    if(Stage <= 1){
      int CurrentCheck = analogRead(CurrentCheckInput);
      if (CurrentCheck < Current-5 && DutyCycle < 200){
        DutyCycle++;
      }
      
      if (CurrentCheck > Current+5 && DutyCycle > 0){
        DutyCycle--;
      }
      analogWrite(PWMOutput, DutyCycle);
      delay(100);
    }

    if(Stage > 1){
      int VoltageCheck = analogRead(BatteryInput);
      if (VoltageCheck < FloatVoltage+20 && DutyCycle < 200){
        DutyCycle++;
      }
      
      if (VoltageCheck > FloatVoltage+20 && DutyCycle > 0){
        DutyCycle--;
      }
      analogWrite(PWMOutput, DutyCycle);
      delay(100);
    }
//    Serial.print("Duty Cycle:");
//    Serial.println(DutyCycle);
}

ISR(TIMER1_COMPA_vect){
    
    Counter++;
    Serial.println(Counter);
   
    if (Counter >= 25 && Stage == 0){
        Counter = 0;
        /* Set output LED's */
        digitalWrite(InitialLED, LOW);
        digitalWrite(MainLED, HIGH);
        digitalWrite(FloatLED, LOW);

        /* Set current setting FET's */
        digitalWrite(MainFET, LOW);       // Main Stage Resistor is not part of circuit
        digitalWrite(FloatFET, HIGH);     // Float stage resistor is not part of circuit

        /* Set Charging Current */
        Current = MainCurrent;

        Stage = 1;
        Serial.println("Main");
        Serial.println(Current);
    }

    if (Counter >=25 && Stage == 1){
      /* Set output LED's */
        digitalWrite(InitialLED, LOW);
        digitalWrite(MainLED, LOW);
        digitalWrite(FloatLED, HIGH);
        
        /* Set current setting FET's */
        digitalWrite(MainFET, LOW);   // Main Stage Resistor is not part of circuit
        digitalWrite(FloatFET, LOW);  // Float stage resistor is not part of circuit


        Stage = 2;

        Serial.println("Float");
     }
}
