/* Include Files */
#include <avr/io.h>
#include <avr/interrupt.h>


/* Digital Pins */ 
int MainFET = 2;
int FloatFET = 3; 
int InitialLED = 7;
int MainLED = 8;
int FloatLED = 9;
int PWMOutput = 5;

/* Analog Pins */
int CurrentCheckInput = A7;
int BatteryInput = A6;

/* ADC Values */
int InitialCurrent = 800;
int MainCurrent = 400;
int FloatCurrent = 100;
int Current;
int BoostLimit = 880;
int DutyCycle = 1;

void setup() {

    TCCR0B = (TCCR0B & 0b11111000) | 0b00000001;
    // initialize Timer1
    cli();          // disable global interrupts
    TCCR1A = 0;     // set entire TCCR1A register to 0
    TCCR1B = 0;     // same for TCCR1B
 
    // set compare match register to desired timer count:
    OCR1A = 15624;
    // turn on CTC mode:
    TCCR1B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler:
    TCCR1B |= (1 << CS10);
    TCCR1B |= (1 << CS12);
    // enable timer compare interrupt:
    TIMSK1 |= (1 << OCIE1A);
    sei();          // enable global interrupts
    
    /* Set up pin modes */
    pinMode(MainFET, OUTPUT);            // sets the Main Stage FET pin as output
    pinMode(FloatFET, OUTPUT);           // sets the Float Stage FET pin as output
    pinMode(InitialLED, OUTPUT);         // sets the Isolation FET pin as output
    pinMode(MainLED, OUTPUT);            // sets the Main Stage FET pin as output
    pinMode(FloatLED, OUTPUT);           // sets the Float Stage FET pin as output
    pinMode(PWMOutput, OUTPUT);          // sets PWM pin as output
    pinMode(BatteryInput, INPUT);        // sets battery pin as an input
    pinMode(CurrentCheckInput, INPUT);   // sets current checkpoint as input


    /* Configure output pin initial conditions */
    digitalWrite(MainFET, HIGH);   // Main Stage Resistor is not part of circuit
    digitalWrite(FloatFET, HIGH);  // Float stage resistor is not part of circuit
    digitalWrite(InitialLED, LOW);    // Battery is not connected to boost converter
    digitalWrite(MainLED, HIGH);   // Main Stage Resistor is not part of circuit
    digitalWrite(FloatLED, HIGH);  // Float stage resistor is not part of circuit

    
    
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
    Serial.println("In Check Battery");
    int BatteryVoltage = analogRead(BatteryInput);
    
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

        Serial.println("Initial");
    }
    
    if(BatteryVoltage >= 695 && BatteryVoltage < 720){
        /* Set output LED's */
        digitalWrite(InitialLED, LOW);
        digitalWrite(MainLED, HIGH);
        digitalWrite(FloatLED, LOW);

        /* Set current setting FET's */
        digitalWrite(MainFET, LOW);       // Main Stage Resistor is not part of circuit
        digitalWrite(FloatFET, HIGH);     // Float stage resistor is not part of circuit

        /* Set Charging Current */
        Current = MainCurrent;

        Serial.println("Main");
    }

    if(BatteryVoltage >= 720){
        /* Set output LED's */
        digitalWrite(InitialLED, LOW);
        digitalWrite(MainLED, LOW);
        digitalWrite(FloatLED, HIGH);
        
        /* Set current setting FET's */
        digitalWrite(MainFET, LOW);   // Main Stage Resistor is not part of circuit
        digitalWrite(FloatFET, LOW);  // Float stage resistor is not part of circuit

        /* Set Charging Current */
        Current = FloatCurrent;

        Serial.println("Float");
    }
    
    Serial.print("Battery: ");
    Serial.println(BatteryVoltage);
}

void ChargeBattery(void){
    Serial.println("In charger");
    int CurrentCheck = analogRead(CurrentCheckInput);
    if (CurrentCheck < Current && DutyCycle < 200){
      DutyCycle++;
    }
    
    if (CurrentCheck > Current && DutyCycle > 0){
      DutyCycle--;
    }
    analogWrite(PWMOutput, DutyCycle);
    delay(100);
    Serial.print("Duty Cycle: ");
    Serial.println(DutyCycle);
    Serial.print("Current: ");
    Serial.println(CurrentCheck);
}

ISR(TIMER1_COMPA_vect){
    Counter++;
    Serial.println(Counter);
}
