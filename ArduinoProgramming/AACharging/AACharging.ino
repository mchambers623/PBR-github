/* Include Files */
#include <avr/io.h>
#include <avr/interrupt.h>


/* Digital Pins */ 
int AAMainFET = 4;
int AAFloatFET = 6; 
int IncrementPin = 10;
int PotDirPin = 11;
int AALED = 12;

/* Analog Pins */
int AACurrentCheckInput = A0;
int AABatteryInputPositive = A1;
int AABatteryInputNegative = A2;
int LM317Input = A3;


/* ADC Values */
int AAInitialCurrent = 750;
int AAMainCurrent = 360;
int AAFloatCurrent = 18;
int AACurrent;
int AACounter = 0;
int AAStage = 0;
int AAFloatVoltage = 660;


void setup() {

    
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
    pinMode(AAMainFET, OUTPUT);            // sets the Main Stage FET pin as output
    pinMode(AAFloatFET, OUTPUT);           // sets the Float Stage FET pin as output
    pinMode(AALED, OUTPUT);



    /* Set up input pin modes */
    pinMode(AABatteryInputPositive, INPUT);        // sets battery pin as an input
    pinMode(AABatteryInputNegative, INPUT);
    pinMode(AACurrentCheckInput, INPUT);   // sets current checkpoint as input
    pinMode(IncrementPin, OUTPUT);      // sets the digital pin as output
    pinMode(PotDirPin, OUTPUT);      // sets the digital pin as output
    pinMode(LM317Input, INPUT);



    /* Configure output pin initial conditions */
    digitalWrite(AAMainFET, HIGH);         // Main Stage Resistor is not part of circuit
    digitalWrite(AAFloatFET, HIGH);        // Float stage resistor is not part of circuit
    digitalWrite(IncrementPin, HIGH);
    digitalWrite(AALED, LOW);
  

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


    int LM317Voltage = analogRead(LM317Input);
    while(LM317Voltage < 200){
      LM317Voltage = analogRead(LM317Input);
    }
    digitalWrite(AALED, HIGH);
    /* Take reading of battery */
    int AABatteryVoltage = (analogRead(AABatteryInputPositive)-analogRead(AABatteryInputNegative));

    /* If battery voltage is below */
    if(AABatteryVoltage < 700){
      
        /* Set output LED's */
        //digitalWrite(AAInitialLED, HIGH);
        //digitalWrite(AAMainLED, LOW);
        //digitalWrite(AAFloatLED, LOW);

        /* Set current setting FET's */
        digitalWrite(AAMainFET, HIGH);     // Main Stage Resistor is not part of circuit
        digitalWrite(AAFloatFET, HIGH);    // Float stage resistor is not part of circuit

        /* Set Charging Current */
        AACurrent = AAInitialCurrent;

        /* Set stage of charging */
        AAStage = 0;

        Serial.println("Initial");
    }
    
    if(AABatteryVoltage >= 540 && AABatteryVoltage < 575){
        /* Set output LED's */
        //digitalWrite(AAInitialLED, LOW);
        //digitalWrite(AAMainLED, HIGH);
        //digitalWrite(AAFloatLED, LOW);

        /* Set current setting FET's */
        digitalWrite(AAMainFET, LOW);       // Main Stage Resistor is part of circuit
        digitalWrite(AAFloatFET, HIGH);     // Float stage resistor is not part of circuit

        /* Set Charging Current */
        AACurrent = AAMainCurrent;

        /* Set stage of charging */
        AAStage = 1;


        Serial.println("Main");
    }

    if(AABatteryVoltage >= 1000){
        /* Set output LED's */
        //digitalWrite(AAInitialLED, LOW);
       // digitalWrite(AAMainLED, LOW);
        //digitalWrite(AAFloatLED, HIGH);
        
        /* Set current setting FET's */
        digitalWrite(AAMainFET, LOW);       // Main Stage Resistor is part of circuit
        digitalWrite(AAFloatFET, LOW);      // Float stage resistor is part of circuit

        /* Set Charging Current */
        AACurrent = AAFloatCurrent;

        /* Set stage of charging */
        AAStage = 2;

        
        Serial.println("Float");
    }
    Serial.println(AABatteryVoltage);
}


void ChargeBattery(void){

    if(AAStage <= 1){
      int AACurrentCheck = analogRead(AACurrentCheckInput);
      if (AACurrentCheck < AACurrent-5){
        digitalWrite(PotDirPin, LOW);
        digitalWrite(IncrementPin, LOW);   // sets the LED on
        digitalWrite(IncrementPin, HIGH);    // sets the LED off
      }
      
      if (AACurrentCheck > AACurrent+5){
        digitalWrite(PotDirPin, HIGH);
        digitalWrite(IncrementPin, LOW);   // sets the LED on
        digitalWrite(IncrementPin, HIGH);    // sets the LED off
      }
      delay(100);
    }

    if(AAStage > 1){
      int AAVoltageCheck = analogRead(AABatteryInputPositive);
      if (AAVoltageCheck < AAFloatVoltage+20){
        digitalWrite(PotDirPin, LOW);
        digitalWrite(IncrementPin, LOW);   // sets the LED on
        digitalWrite(IncrementPin, HIGH);    // sets the LED off        
      }
      
      if (AAVoltageCheck > AAFloatVoltage+20){
        digitalWrite(PotDirPin, HIGH);
        digitalWrite(IncrementPin, LOW);   // sets the LED on
        digitalWrite(IncrementPin, HIGH);    // sets the LED off
      }
      delay(100);
    }
//    Serial.print("Duty Cycle:");
//    Serial.println(DutyCycle);
}

ISR(TIMER1_COMPA_vect){
    
    AACounter++;
    Serial.println(AACounter);
   
    if (AACounter >= 25 && AAStage == 0){
        AACounter = 0;
        /* Set output LED's */
       // digitalWrite(AAInitialLED, LOW);
       // digitalWrite(AAMainLED, HIGH);
       // digitalWrite(AAFloatLED, LOW);

        /* Set current setting FET's */
        digitalWrite(AAMainFET, LOW);       // Main Stage Resistor is not part of circuit
        digitalWrite(AAFloatFET, HIGH);     // Float stage resistor is not part of circuit

        /* Set Charging Current */
        AACurrent = AAMainCurrent;

        AAStage = 1;
        Serial.println("Main");
        Serial.println(AACurrent);
    }

    if (AACounter >=25 && AAStage == 1){
      /* Set output LED's */
      //  digitalWrite(AAInitialLED, LOW);
     //   digitalWrite(AAMainLED, LOW);
       // digitalWrite(AAFloatLED, HIGH);
        
        /* Set current setting FET's */
        digitalWrite(AAMainFET, LOW);   // Main Stage Resistor is not part of circuit
        digitalWrite(AAFloatFET, LOW);  // Float stage resistor is not part of circuit


        AAStage = 2;

        Serial.println("Float");
     }
}

