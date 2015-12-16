/* 
* File: AAand9VCharger.ino
* 
* Project: Portable Battery Recharger
*
* Authors: Matthew E. Chambers and Alexander D. Gregoire
*
* Date: 12/16/15
*
* Brief: This program is made to charge a 9V NiMH battery and 2x AA NiMH 
* rechargeable batteries simulataneously. This code was designed to work
* with an Arduino Nano in accordance with hardware designed by the authors.
 */
 

/* Include Files */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "NineVStruct.h"
#include "AAStruct.h"

/* 9V Digital Pin Defines */ 
#define MainFET                 3
#define FloatFET                2 
#define InitialLED              7
#define MainLED                 8
#define FloatLED                9
#define PWMOutput               5

/* AA Digital Pin Defines */ 
#define AAMainFET               4
#define AAFloatFET              6
#define IncrementPin            10
#define PotDirPin               11
#define AALED                   12

/* 9V Analog Pin Defines */
#define CurrentCheckInput       A7
#define BatteryInput            A6

/* AA Analog Pin Defines */
#define AACurrentCheckInput     A0
#define AABatteryInputPositive  A1
#define AABatteryInputNegative  A2
#define LM317Input              A3

/* 9V Charging Defines */
#define InitialCurrent          760       // Corresponds to 35mA
#define MainCurrent             375       // Corresponds to 17.5mA
#define FloatVoltage            760       // Corresponds to 11.2V

/* AA Charging Defines */
#define AAInitialCurrent        650       // Corresponds to 460mA
#define AAMainCurrent           220       // Corresponds to 230mA
#define AAFloatVoltage          660       // Corresponds to 4.8V

/* Charging timing Defines */
#define InitialTime             10      // Corresponds to 0.5hrs
#define MainTime                10      // Corresponds to 2hrs
#define FloatTime               10      // Corresponds to 5hrs

/* Structures to hold counters, stages, and duty cycle */
NineVStruct NineVStuff = {1,0,0};
AAStruct AAStuff = {0,0};



void setup() {
   
   /* Set PWM frequency to 62kHz */
    TCCR0B = (TCCR0B & 0b11111000) | 0b00000001;

   /* Initialize Timer 1 used for interupt*/
    cli();                                  // disable global interrupts
    TCCR1A = 0;                             // set entire TCCR1A register to 0
    TCCR1B = 0;                             // same for TCCR1B

    /* Set up timer 1 to interrupt every second */
    OCR1A = 15624;                          // set compare match register to desired timer count
    TCCR1B |= (1 << WGM12);                 // turn on CTC mode
    TCCR1B |= (1 << CS10);                  // Set CS10 bits for 1024 prescaler
    TCCR1B |= (1 << CS12);                  // Set CS12 bits for 1024 prescaler
    TIMSK1 |= (1 << OCIE1A);                // enable timer compare interrupt
    sei();                                  // enable global interrupts

   /* Set up 9V output pin modes */
    pinMode(MainFET, OUTPUT);               // sets the Main Stage FET pin as output
    pinMode(FloatFET, OUTPUT);              // sets the Float Stage FET pin as output
    pinMode(InitialLED, OUTPUT);            // sets the Intitial LED pin as output
    pinMode(MainLED, OUTPUT);               // sets the Main LED pin as output
    pinMode(FloatLED, OUTPUT);              // sets the Float LED pin as output
    pinMode(PWMOutput, OUTPUT);             // sets PWM pin as output

    /* Set up AA output pin modes */
    pinMode(AAMainFET, OUTPUT);             // sets the Main Stage FET pin as output
    pinMode(AAFloatFET, OUTPUT);            // sets the Float Stage FET pin as output
    pinMode(AALED, OUTPUT);                 // sets the AALED pin as output
    pinMode(IncrementPin, OUTPUT);          // sets the digital pot increment pin as output
    pinMode(PotDirPin, OUTPUT);             // sets the digital pot directions pin as output

    /* Set up 9V input pin modes */
    pinMode(BatteryInput, INPUT);           // sets battery pin as an input
    pinMode(CurrentCheckInput, INPUT);      // sets current checkpoint as input
    
    /* Set up AA input pin modes */
    pinMode(AABatteryInputPositive, INPUT); // sets AA postive terminal pin as an input
    pinMode(AABatteryInputNegative, INPUT); // sets AA negative terminal pin as an input
    pinMode(AACurrentCheckInput, INPUT);    // sets current checkpoint as input
    pinMode(LM317Input, INPUT);

    /* Configure AA output pin initial conditions */
    digitalWrite(AAMainFET, HIGH);          // Main Stage Resistor is not part of circuit
    digitalWrite(AAFloatFET, HIGH);         // Float stage resistor is not part of circuit
    digitalWrite(IncrementPin, HIGH);       // Digital Pot will be in decrement mode
    digitalWrite(AALED, LOW);               // AA LED is off

    /* Configure 9V output pin initial conditions */
    digitalWrite(MainFET, HIGH);            // Main Stage Resistor is not part of circuit
    digitalWrite(FloatFET, HIGH);           // Float stage resistor is not part of circuit
    digitalWrite(InitialLED, LOW);          // Battery is not connected to boost converter
    digitalWrite(MainLED, HIGH);            // Main Stage Resistor is not part of circuit
    digitalWrite(FloatLED, HIGH);           // Float stage resistor is not part of circuit

    /* Set up serial */
    Serial.begin(19200);
    Serial.println("Configured");
}



void loop() {

  /* Check the voltage of 9V battery and decide charging cycle */
  Check9VBatteryVoltage();

  /* Check the voltage of AA Battery and decide charging cycle */
  CheckAABatteryVoltage();

  /* Enter infinite loop */
  while(1){

      /* Check current through 9V battery and change accordingly */
      Charge9VBattery();

      /* Check current through AA battery and change accordingly */
      ChargeAABattery();
  }
}



void Charge9VBattery(void){ 

    /* Local Variables */
    int HighDuty = 220;                     // Dont let duty cycle exceed ~85%
    int LowDuty = 25;                       // Dont let the duty cycle go below ~10%
    int CurrentHysteresis = 5;              // Used to make sure output is not oscillating
    int VoltageHysteresis = 20;             // Used to make sure output is not oscillating
    
    /* Check if initial stage */
    if(NineVStuff.Stage == 1){

      /* Read current from op-amp */
      int CurrentCheck = analogRead(CurrentCheckInput);

      /* If current is lower than it is supposed to be */
      if (CurrentCheck < InitialCurrent-CurrentHysteresis && NineVStuff.DutyCycle < HighDuty){

        /* Increase Duty Cycle */
        NineVStuff.DutyCycle++;
      }

      /* If current is higher than it is supposed to be */
      if (CurrentCheck > InitialCurrent+CurrentHysteresis && NineVStuff.DutyCycle > LowDuty){

        /* Decrease Duty Cycle */
        NineVStuff.DutyCycle--;
      }

      /* Adjust PWM wave with new duty cycle */
      analogWrite(PWMOutput, NineVStuff.DutyCycle);
    }

     /* Check if initial stage */
    if(NineVStuff.Stage == 2){

      /* Read current from op-amp */
      int CurrentCheck = analogRead(CurrentCheckInput);

      /* If current is lower than it is supposed to be */
      if (CurrentCheck < MainCurrent-CurrentHysteresis && NineVStuff.DutyCycle < HighDuty){

        /* Increase Duty Cycle */
        NineVStuff.DutyCycle++;
      }

      /* If current is higher than it is supposed to be */
      if (CurrentCheck > MainCurrent+CurrentHysteresis && NineVStuff.DutyCycle > LowDuty){

        /* Decrease duty cycle */
        NineVStuff.DutyCycle--;
      }

      /* Adjust PWM wave with new duty cycle */
      analogWrite(PWMOutput, NineVStuff.DutyCycle);
    }

    /* Check if in float stage */
    if(NineVStuff.Stage == 3){

      /* Read voltage from positive node of 9V */
      int VoltageCheck = analogRead(BatteryInput);

      /* If voltage is lower than it should be */
      if (VoltageCheck < FloatVoltage+VoltageHysteresis && NineVStuff.DutyCycle < HighDuty){

        /* Increase Duty Cycle */
        NineVStuff.DutyCycle++;
      }

      /* If voltage is high than it should be */
      if (VoltageCheck > FloatVoltage+VoltageHysteresis && NineVStuff.DutyCycle > LowDuty){

        /* Decrease Duty Cycle */
        NineVStuff.DutyCycle--;
      }

      
      /* Adjust PWM wave with new duty cycle */
      analogWrite(PWMOutput, NineVStuff.DutyCycle);
      
    }
}



void ChargeAABattery(void){

    /* Local variables */
    int AACurrentHysteresis = 0;            // Used to reduce output oscillations
    int AAVoltageHysteresis = 0;            // Used to reduce output oscillations

    /* Read voltage at the LM317 so that we know if the circuit is energized */
    int LM317Voltage = analogRead(LM317Input);

    /* If energized, turn on LED */
    if(LM317Voltage >= 500){
      digitalWrite(AALED,HIGH);
    }

    /* If not energized, turn off LED */
    if(LM317Voltage < 500){
        digitalWrite(AALED, LOW);
    }
    
    /* Check if in initial */
    if(AAStuff.AAStage == 1){

      /* Read current from op-amp */
      int AACurrentCheck = analogRead(AACurrentCheckInput);

      /* If current is lower than it is supposed to be */
      if (AACurrentCheck <= AAInitialCurrent-AACurrentHysteresis){

        /* Change digital pot to increase LM317 output voltage */
        digitalWrite(PotDirPin, LOW);
        delay(1);

        /* Pulse Increment pin */
        digitalWrite(IncrementPin, LOW);   
        digitalWrite(IncrementPin, HIGH);
      }

      /* If current is high than it is supposed to be */
      if (AACurrentCheck >= AAInitialCurrent+AACurrentHysteresis){

        /* Change digital pot to decrease LM317 output voltage */
        digitalWrite(PotDirPin, HIGH);
        delay(1);

        /* Pulse Increment Pin */
        digitalWrite(IncrementPin, LOW);   
        digitalWrite(IncrementPin, HIGH);
      }
    }  

    /* Check if in main */
    if(AAStuff.AAStage == 2){

      /* Read current from op-amp */
      int AACurrentCheck = analogRead(AACurrentCheckInput);

      /* If current is lower than it is supposed to be */
      if (AACurrentCheck <= AAMainCurrent-AACurrentHysteresis){

        /* Change digital pot to increase LM317 output voltage */
        digitalWrite(PotDirPin, LOW);
        delay(1);

        /* Pulse Increment Pin */
        digitalWrite(IncrementPin, LOW);   
        digitalWrite(IncrementPin, HIGH);
      }

      /* If current is high than it is supposed to be */
      if (AACurrentCheck >= AAMainCurrent+AACurrentHysteresis){

        /* Change digital pot to decrease LM317 output voltage */
        digitalWrite(PotDirPin, HIGH);
        delay(1);

        /* Pulse Increment Pin */
        digitalWrite(IncrementPin, LOW);   
        digitalWrite(IncrementPin, HIGH);
      }
    }  
    
    /* Check if in float stage */
    if(AAStuff.AAStage == 3){

      /* Read output voltage of LM317 */
      int AAVoltageCheck = analogRead(AABatteryInputPositive);

      /* If the voltage is lower than it is supposed to be */
      if (AAVoltageCheck < AAFloatVoltage+AAVoltageHysteresis){

        /* Increment digital pot to increase LM317 output voltage */
        digitalWrite(PotDirPin, LOW);
        delay(1);
        
        /* Pulse Increment Pin */
        digitalWrite(IncrementPin, LOW);   
        digitalWrite(IncrementPin, HIGH);           
      }

      /* If the voltage is higher than it is supposed to be */
      if (AAVoltageCheck > AAFloatVoltage+AAVoltageHysteresis){

        /* Increment digital pot to decrease LM317 output voltage */
        digitalWrite(PotDirPin, HIGH);
        delay(1);

        /* Pulse Increment Pin */
        digitalWrite(IncrementPin, LOW);   
        digitalWrite(IncrementPin, HIGH); 
      }
    }
    
}



void Check9VBatteryVoltage(void){
    
    /* Take reading of battery through a 1:3 voltage divider */
    int BatteryVoltage = analogRead(BatteryInput);

    /* If battery voltage is below 1.3V/cell */
    if(BatteryVoltage < 695){

      /* Set up 9V initial charging cycle */
      SetUp9VInitial();
    }

    /* If battery voltage is between 1.3V/cell and 1.4V/cell */
    if(BatteryVoltage >= 695 && BatteryVoltage < 720){

       /* Set up the 9V main charging cycle */
       SetUp9VMain();
    }

    /* If battery voltage is greater than 1.4V/cell */
    if(BatteryVoltage >= 720){

       /* Set up the 9V float charging cycle */
       SetUp9VFloat();
    }
}



void CheckAABatteryVoltage(void){

    /* Read voltage at the LM317 so that we know if the circuit is energized */
    int LM317Voltage = analogRead(LM317Input);
    
    /* Take a differential reading of battery */
    int AABatteryVoltage = (analogRead(AABatteryInputPositive)-analogRead(AABatteryInputNegative));

    /* If battery voltage is below 1.3V/cell */
    if(AABatteryVoltage < 700 && LM317Voltage >= 500){

        /* Set up the AA intial charging cycle */
        SetUpAAInitial();      
    } 

    /* If battery voltage is between 1.3V/cell and 1.4V/cell */
    if(AABatteryVoltage >= 740 && AABatteryVoltage < 775 && LM317Voltage >= 500){ 

        /* Set up AA main charging cycle */
        SetUpAAMain();
    }

    /* If battery voltage is greater than 1.4V/cell */
    if(AABatteryVoltage >= 775 && LM317Voltage >= 500){

        /* Set up AA float stage cycle */
        SetUpAAFloat();
    }
}



void Check9VState(void){
      
    /* Check if 9V batteries are being charged */
    if(NineVStuff.Stage == 0){
        Check9VBatteryVoltage();
    }
    
    /* Check if 9V is in initial stage */
    if(NineVStuff.Stage == 1){
      
      /* Increment timing */
      NineVStuff.Counter++;

      /* Check if stage should be completed */
      if(NineVStuff.Counter >= InitialTime){

          /* Set up 9V main charging cycle */
          SetUp9VMain();

          /* Reset counter */
          NineVStuff.Counter = 0;
      }
    }

    /* check if 9V is in main stage */
    if(NineVStuff.Stage == 2){
             
      /* Increment timing */
      NineVStuff.Counter++;

      /* Check if stage should be completed */
      if(NineVStuff.Counter >= MainTime){

          /* Set up 9V float stage */
          SetUp9VFloat();

          /* Reset counter */
          NineVStuff.Counter = 0;
      }
    }

    /* Check if 9V is in float stage */
    if(NineVStuff.Stage == 3){
      
      /* Increment timing */
      NineVStuff.Counter++;

      /* Check if stage should be completed */
      if(NineVStuff.Counter >= FloatTime){
          Stop9VCharging();
      }
    } 
}



void CheckAAState(void){
     
    /* Check if AA batteries are being charged */
    if(AAStuff.AAStage == 0){
      CheckAABatteryVoltage();
    }
    
    /*Check if AA is in initial charging cycle */
    if(AAStuff.AAStage == 1){
      
      /* Increment timing */
      AAStuff.AACounter++;

      /* Check if stage should be completed */
      if(AAStuff.AACounter >= InitialTime){

          /* Set up AA main charging cycle */
          SetUpAAMain();

          /* Reset counter */
          AAStuff.AACounter = 0;
      }
    }

    /* Check if AA is in main charging stage */
    if(AAStuff.AAStage == 2){
        
      /* Increment timing */
      AAStuff.AACounter++;

      /* Check if stage should be completed */
      if(AAStuff.AACounter >= MainTime){

          /* Set up AA float charging cycle */
          SetUpAAFloat();

          /* Reset Counter */
          AAStuff.AACounter = 0;
      }
    }

    /* check if AA is in float stage */
    if(AAStuff.AAStage == 3){
      
      /* Increment timing */
      AAStuff.AACounter++;

      /* Check if stage should be completed */
      if(AAStuff.AACounter >= FloatTime){
          StopAACharging();
      }
    }
}



void SetUp9VInitial(void){
  
  /* Set output LED's */
  digitalWrite(InitialLED, HIGH);
  digitalWrite(MainLED, LOW);
  digitalWrite(FloatLED, LOW);

  /* Set current setting FET's */
  digitalWrite(MainFET, HIGH);     // Main Stage Resistor is not part of circuit
  digitalWrite(FloatFET, HIGH);    // Float stage resistor is not part of circuit

  /* Set stage of charging to initial */
  NineVStuff.Stage = 1;  
}



void SetUp9VMain(void){
  
   /* Set output LED's */
  digitalWrite(InitialLED, LOW);
  digitalWrite(MainLED, HIGH);
  digitalWrite(FloatLED, LOW);

  /* Set current setting FET's */
  digitalWrite(MainFET, LOW);       // Main Stage Resistor is part of circuit
  digitalWrite(FloatFET, HIGH);     // Float stage resistor is not part of circuit

  /* Set stage of charging to main */
  NineVStuff.Stage = 2;
}



void SetUp9VFloat(void){
  
  /* Set output LED's */
  digitalWrite(InitialLED, LOW);
  digitalWrite(MainLED, LOW);
  digitalWrite(FloatLED, HIGH);
  
  /* Set current setting FET's */
  digitalWrite(MainFET, LOW);       // Main Stage Resistor is part of circuit
  digitalWrite(FloatFET, LOW);      // Float stage resistor is part of circuit

  /* Set stage of charging */
  NineVStuff.Stage = 3;
}



void SetUpAAInitial(void){
  
  /* Set current setting FET's */
  digitalWrite(AAMainFET, HIGH);     // Main Stage Resistor is not part of circuit
  digitalWrite(AAFloatFET, HIGH);    // Float stage resistor is not part of circuit

  /* Set stage of charging to initial */
  AAStuff.AAStage = 1;
}



void SetUpAAMain(void){
  
  /* Set current setting FET's */
  digitalWrite(AAMainFET, LOW);       // Main Stage Resistor is part of circuit
  digitalWrite(AAFloatFET, HIGH);     // Float stage resistor is not part of circuit

  /* Set stage of charging */
  AAStuff.AAStage = 2;  
}



void SetUpAAFloat(void){
  
  /* Set current setting FET's */
  digitalWrite(AAMainFET, LOW);       // Main Stage Resistor is part of circuit
  digitalWrite(AAFloatFET, LOW);      // Float stage resistor is part of circuit

  /* Set stage of charging */
  AAStuff.AAStage = 3;
}



void Stop9VCharging(void){
    NineVStuff.Stage = 4;
}



void StopAACharging(void){
    AAStuff.AAStage = 4;
}



ISR(TIMER1_COMPA_vect){  
    Check9VState();
    CheckAAState();
    if(NineVStuff.Stage == 4){
        digitalWrite(FloatLED, !digitalRead(FloatLED));
        NineVStuff.DutyCycle = 20;
    }
    if(AAStuff.AAStage == 4){
        digitalWrite(AALED, !digitalRead(AALED));
    }
}   

