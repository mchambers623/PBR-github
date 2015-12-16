/* Include Files */
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <AAand9VCharger.h>
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
#define InitialCurrent          760
#define MainCurrent             375
#define FloatVoltage            750

/* AA Charging Defines */
#define AAInitialCurrent        1024
#define AAMainCurrent           280
#define AAFloatVoltage          660

/* Charging timing Defines */
#define InitialTime             1800
#define MainTime                7200
#define FloatTime               18000

///* 9V Global Variables */
//int DutyCycle = 1;
//int Counter = 0;
//int Stage = 0;
//
///* AA Global Variables */
//int AACounter = 0;
//int AAStage = 0;






void setup() {
   
   /* Set PWM frequency to 62kHz */
    TCCR0B = (TCCR0B & 0b11111000) | 0b00000001;

   /* Initialize Timer 1 used for interupt*/
    cli();                               // disable global interrupts
    TCCR1A = 0;                          // set entire TCCR1A register to 0
    TCCR1B = 0;                          // same for TCCR1B

    /* Set up timer 1 to interrupt every second */
    OCR1A = 15624;                       // set compare match register to desired timer count
    TCCR1B |= (1 << WGM12);              // turn on CTC mode
    TCCR1B |= (1 << CS10);               // Set CS10 bits for 1024 prescaler
    TCCR1B |= (1 << CS12);               // Set CS12 bits for 1024 prescaler
    TIMSK1 |= (1 << OCIE1A);             // enable timer compare interrupt
    sei();                               // enable global interrupts

   /* Set up 9V output pin modes */
    pinMode(MainFET, OUTPUT);            // sets the Main Stage FET pin as output
    pinMode(FloatFET, OUTPUT);           // sets the Float Stage FET pin as output
    pinMode(InitialLED, OUTPUT);         // sets the Intitial LED pin as output
    pinMode(MainLED, OUTPUT);            // sets the Main LED pin as output
    pinMode(FloatLED, OUTPUT);           // sets the Float LED pin as output
    pinMode(PWMOutput, OUTPUT);          // sets PWM pin as output

    /* Set up AA output pin modes */
    pinMode(AAMainFET, OUTPUT);            // sets the Main Stage FET pin as output
    pinMode(AAFloatFET, OUTPUT);           // sets the Float Stage FET pin as output
    pinMode(AALED, OUTPUT);
    pinMode(IncrementPin, OUTPUT);      // sets the digital pin as output
    pinMode(PotDirPin, OUTPUT);      // sets the digital pin as output

    /* Set up 9V input pin modes */
    pinMode(BatteryInput, INPUT);        // sets battery pin as an input
    pinMode(CurrentCheckInput, INPUT);   // sets current checkpoint as input
    
    /* Set up AA input pin modes */
    pinMode(AABatteryInputPositive, INPUT);        // sets battery pin as an input
    pinMode(AABatteryInputNegative, INPUT);
    pinMode(AACurrentCheckInput, INPUT);   // sets current checkpoint as input
    pinMode(LM317Input, INPUT);

    /* Configure AA output pin initial conditions */
    digitalWrite(AAMainFET, HIGH);         // Main Stage Resistor is not part of circuit
    digitalWrite(AAFloatFET, HIGH);        // Float stage resistor is not part of circuit
    digitalWrite(IncrementPin, HIGH);
    digitalWrite(AALED, LOW);

    /* Configure 9V output pin initial conditions */
    digitalWrite(MainFET, HIGH);         // Main Stage Resistor is not part of circuit
    digitalWrite(FloatFET, HIGH);        // Float stage resistor is not part of circuit
    digitalWrite(InitialLED, LOW);       // Battery is not connected to boost converter
    digitalWrite(MainLED, HIGH);         // Main Stage Resistor is not part of circuit
    digitalWrite(FloatLED, HIGH);        // Float stage resistor is not part of circuit

    /* Set up serial */
    Serial.begin(19200);
    Serial.println("Configured");

}

void loop() {
  NineVStruct NineVStuff = {1,0,0};
  AAStruct AAStuff = {0,0};
  NineVStuff = Check9VBatteryVoltage(NineVStuff);
  AAStuff = CheckAABatteryVoltage(AAStuff);
  while(1){
      NineVStuff = Charge9VBattery(NineVStuff);
      AAStuff = ChargeAABattery(AAStuff);
  }
}

NineVStruct Charge9VBattery(NineVStruct NineVStuff){   
    int HighDuty = 200;
    int LowDuty = 10;
    int CurrentHysteresis = 5;
    int VoltageHysteresis = 20;
    
    /* Check if initial stage */
    if(NineVStuff.Stage == 1){

      /* Read current from op-amp */
      int CurrentCheck = analogRead(CurrentCheckInput);

      /* If current is lower than it is supposed to be */
      if (CurrentCheck < InitialCurrent-CurrentHysteresis && NineVStuff.DutyCycle < HighDuty){
        NineVStuff.DutyCycle++;
      }

      /* If current is higher than it is supposed to be */
      if (CurrentCheck > InitialCurrent+CurrentHysteresis && NineVStuff.DutyCycle > LowDuty){
        NineVStuff.DutyCycle--;
      }
      analogWrite(PWMOutput, NineVStuff.DutyCycle);
      delay(1);
    }

     /* Check if initial stage */
    if(NineVStuff.Stage == 2){

      /* Read current from op-amp */
      int CurrentCheck = analogRead(CurrentCheckInput);

      /* If current is lower than it is supposed to be */
      if (CurrentCheck < MainCurrent-CurrentHysteresis && NineVStuff.DutyCycle < HighDuty){
        NineVStuff.DutyCycle++;
      }

      /* If current is higher than it is supposed to be */
      if (CurrentCheck > MainCurrent+CurrentHysteresis && NineVStuff.DutyCycle > LowDuty){
        NineVStuff.DutyCycle--;
      }
      analogWrite(PWMOutput, NineVStuff.DutyCycle);
      delay(1);
    }

    /* Check if in float stage */
    if(NineVStuff.Stage > 2){

      /* Read voltage from positive node of 9V */
      int VoltageCheck = analogRead(BatteryInput);

      /* If voltage is lower than it should be */
      if (VoltageCheck < FloatVoltage+VoltageHysteresis && NineVStuff.DutyCycle < HighDuty){
        NineVStuff.DutyCycle++;
      }

      /* If voltage is high than it should be */
      if (VoltageCheck > FloatVoltage+VoltageHysteresis && NineVStuff.DutyCycle > LowDuty){
        NineVStuff.DutyCycle--;
      }
      analogWrite(PWMOutput, NineVStuff.DutyCycle);
      delay(1);
    }

    return NineVStuff;
}

NineVStruct Check9VBatteryVoltage(NineVStruct NineVStuff){

    /* Take reading of battery through a 1:3 voltage divider */
    int BatteryVoltage = analogRead(BatteryInput);

    /* If battery voltage is below */
    if(BatteryVoltage < 695){
      NineVStuff = SetUp9VInitial(NineVStuff);
    }
    
    if(BatteryVoltage >= 695 && BatteryVoltage < 720){
       NineVStuff = SetUp9VMain(NineVStuff);
    }

    if(BatteryVoltage >= 720){
        NineVStuff = SetUp9VFloat(NineVStuff);
    }
    return NineVStuff;
}

AAStruct ChargeAABattery(AAStruct AAStuff){
    int AACurrentHysteresis = 20;
    int AAVoltageHysteresis = 10;
    
    /* Check if in initial */
    if(AAStuff.AAStage == 1){

      /* Read current from op-amp */
      int AACurrentCheck = analogRead(AACurrentCheckInput);

      /* If current is lower than it is supposed to be */
      if (AACurrentCheck <= AAInitialCurrent-AACurrentHysteresis){

        /* Change digital pot to increase LM317 output voltage */
        digitalWrite(PotDirPin, LOW);
        digitalWrite(IncrementPin, LOW);   
        digitalWrite(IncrementPin, HIGH);
      }

      /* If current is high than it is supposed to be */
      if (AACurrentCheck >= AAInitialCurrent+AACurrentHysteresis){

        /* Change digital pot to decrease LM317 output voltage */
        digitalWrite(PotDirPin, HIGH);
        digitalWrite(IncrementPin, LOW);   
        digitalWrite(IncrementPin, HIGH);
      }
      delay(2);
    }  

    /* Check if in main */
    if(AAStuff.AAStage == 2){

      /* Read current from op-amp */
      int AACurrentCheck = analogRead(AACurrentCheckInput);

      /* If current is lower than it is supposed to be */
      if (AACurrentCheck <= AAMainCurrent-AACurrentHysteresis){

        /* Change digital pot to increase LM317 output voltage */
        digitalWrite(PotDirPin, LOW);
        digitalWrite(IncrementPin, LOW);   
        digitalWrite(IncrementPin, HIGH);
      }

      /* If current is high than it is supposed to be */
      if (AACurrentCheck >= AAMainCurrent+AACurrentHysteresis){

        /* Change digital pot to decrease LM317 output voltage */
        digitalWrite(PotDirPin, HIGH);
        digitalWrite(IncrementPin, LOW);   
        digitalWrite(IncrementPin, HIGH);
      }
      delay(2);
    }  
    
    /* Check if in float stage */
    if(AAStuff.AAStage == 3){

      /* Read output voltage of LM317 */
      int AAVoltageCheck = analogRead(AABatteryInputPositive);

      /* If the voltage is lower than it is supposed to be */
      if (AAVoltageCheck < AAFloatVoltage+AAVoltageHysteresis){

        /* Increment digital pot to increase LM317 output voltage */
        digitalWrite(PotDirPin, LOW);
        digitalWrite(IncrementPin, LOW);   // sets the LED on
        digitalWrite(IncrementPin, HIGH);    // sets the LED off        
      }

      /* If the voltage is higher than it is supposed to be */
      if (AAVoltageCheck > AAFloatVoltage+AAVoltageHysteresis){

        /* Increment digital pot to decrease LM317 output voltage */
        digitalWrite(PotDirPin, HIGH);
        digitalWrite(IncrementPin, LOW);   // sets the LED on
        digitalWrite(IncrementPin, HIGH);    // sets the LED off
      }
      delay(2);
    }
    return AAStuff;
}

AAStruct CheckAABatteryVoltage(AAStruct AAStuff){

    /* Read voltage at the LM317 so that we know if the circuit is energized */
    int LM317Voltage = analogRead(LM317Input);
        
    /* Take a differential reading of battery */
    int AABatteryVoltage = (analogRead(AABatteryInputPositive)-analogRead(AABatteryInputNegative));

    /* If battery voltage is below */
    if(AABatteryVoltage < 700 && LM317Voltage >= 200){
        digitalWrite(AALED, HIGH);
        AAStuff = SetUpAAInitial(AAStuff);      
    } 
    
    if(AABatteryVoltage >= 740 && AABatteryVoltage < 775 && LM317Voltage >= 200){ 
        digitalWrite(AALED, HIGH);
        AAStuff = SetUpAAMain(AAStuff);
    }

    if(AABatteryVoltage >= 775 && LM317Voltage >= 200){
        digitalWrite(AALED, HIGH);
        AAStuff = SetUpAAFloat(AAStuff);
    }

    if(LM317Voltage < 200){
        digitalWrite(AALED, LOW);
    }
    
    return AAStuff;
}

NineVStruct Check9VState(NineVStruct NineVStuff){
      
    /* Check if 9V batteries are being charged */
    if(NineVStuff.Stage == 0){
      NineVStuff = Check9VBatteryVoltage(NineVStuff);
    }
    
    /* Check if 9V is being charged */
    if(NineVStuff.Stage = 1){
      
      /* Increment timing */
      NineVStuff.Counter++;

      /* Check if stage should be completed */
      if(NineVStuff.Counter >= InitialTime){
          NineVStuff = SetUp9VMain(NineVStuff);
          NineVStuff.Counter = 0;
      }
    }


    if(NineVStuff.Stage = 2){
             
      /* Increment timing */
      NineVStuff.Counter++;

      /* Check if stage should be completed */
      if(NineVStuff.Counter >= MainTime){
          NineVStuff = SetUp9VFloat(NineVStuff);
          NineVStuff.Counter = 0;
      }
    }

    if(NineVStuff.Stage = 3){
      
      /* Increment timing */
      NineVStuff.Counter++;

      /* Check if stage should be completed */
      if(NineVStuff.Counter >= FloatTime){
          Stop9VCharging();
      }
    } 
    return NineVStuff;
}

AAStruct CheckAAState(AAStruct AAStuff){
      
    /* Check if AA batteries are being charged */
    if(AAStuff.AAStage == 0){
      AAStuff = CheckAABatteryVoltage(AAStuff);
    }
    
    /*Check if AA is being charged */
    if(AAStuff.AAStage = 1){
      
      /* Increment timing */
      AAStuff.AACounter++;

      /* Check if stage should be completed */
      if(AAStuff.AACounter >= InitialTime){
          AAStuff = SetUpAAMain(AAStuff);
          AAStuff.AACounter = 0;
      }
    }


    if(AAStuff.AAStage = 2){
        
      /* Increment timing */
      AAStuff.AACounter++;

      /* Check if stage should be completed */
      if(AAStuff.AACounter >= MainTime){
          AAStuff = SetUpAAFloat(AAStuff);
          AAStuff.AACounter = 0;
      }
    }

    if(AAStuff.AAStage = 3){
      
      /* Increment timing */
      AAStuff.AACounter++;

      /* Check if stage should be completed */
      if(AAStuff.AACounter >= FloatTime){
          StopAACharging();
          AAStuff.AACounter=0;
      }
    }  
    return AAStuff; 
}

NineVStruct SetUp9VInitial(NineVStruct NineVStuff){
  
  /* Set output LED's */
  digitalWrite(InitialLED, HIGH);
  digitalWrite(MainLED, LOW);
  digitalWrite(FloatLED, LOW);

  /* Set current setting FET's */
  digitalWrite(MainFET, HIGH);     // Main Stage Resistor is not part of circuit
  digitalWrite(FloatFET, HIGH);    // Float stage resistor is not part of circuit

  /* Set stage of charging */
  NineVStuff.Stage = 1;  

  return NineVStuff;
}

NineVStruct SetUp9VMain(NineVStruct NineVStuff){
   /* Set output LED's */
  digitalWrite(InitialLED, LOW);
  digitalWrite(MainLED, HIGH);
  digitalWrite(FloatLED, LOW);

  /* Set current setting FET's */
  digitalWrite(MainFET, LOW);       // Main Stage Resistor is part of circuit
  digitalWrite(FloatFET, HIGH);     // Float stage resistor is not part of circuit

  /* Set stage of charging */
  NineVStuff.Stage = 2;

  return NineVStuff;
}

NineVStruct SetUp9VFloat(NineVStruct NineVStuff){
   /* Set output LED's */
  digitalWrite(InitialLED, LOW);
  digitalWrite(MainLED, LOW);
  digitalWrite(FloatLED, HIGH);
  
  /* Set current setting FET's */
  digitalWrite(MainFET, LOW);       // Main Stage Resistor is part of circuit
  digitalWrite(FloatFET, LOW);      // Float stage resistor is part of circuit

  /* Set stage of charging */
  NineVStuff.Stage = 3;

  return NineVStuff;
}

AAStruct SetUpAAInitial(AAStruct AAStuff){
  /* Set current setting FET's */
  digitalWrite(AAMainFET, HIGH);     // Main Stage Resistor is not part of circuit
  digitalWrite(AAFloatFET, HIGH);    // Float stage resistor is not part of circuit

  /* Set stage of charging */
  AAStuff.AAStage = 1;
  
  return AAStuff;
}

AAStruct SetUpAAMain(AAStruct AAStuff){
  /* Set current setting FET's */
  digitalWrite(AAMainFET, LOW);       // Main Stage Resistor is part of circuit
  digitalWrite(AAFloatFET, HIGH);     // Float stage resistor is not part of circuit

  /* Set stage of charging */
  AAStuff.AAStage = 2;  

  return AAStuff;
}

AAStruct SetUpAAFloat(AAStruct AAStuff){
  /* Set current setting FET's */
  digitalWrite(AAMainFET, LOW);       // Main Stage Resistor is part of circuit
  digitalWrite(AAFloatFET, LOW);      // Float stage resistor is part of circuit

  /* Set stage of charging */
  AAStuff.AAStage = 3;

  return AAStuff;
}

void Stop9VCharging(void){
}

void StopAACharging(void){
 
}

ISR(TIMER1_COMPA_vect, NineVStruct NineVStuff, AAStruct AAStuff){  
    NineVStuff = Check9VState(NineVStuff);
    AAStuff = CheckAAState(AAStuff);
}   
