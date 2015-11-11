/* Include Files */
#include <avr/io.h>
#include <avr/interrupt.h>


/* Digital Pins */ 
int IsolationFET = 2;
int MainFET = 3;
int FloatFET = 4; 
int InitialLED = 7;
int MainLED = 8;
int FloatLED = 9;

/* Analog Pins */
int CurrentCheckInput = 7;
int BatteryInput = 6;
int PWMOutput = 5;

/* ADC Values */
int InitialCurrent = 800;
int MainCurrent = 400;
int FloatCurrent = 100;
int Current;
int BoostLimit = 880;

void setup() {
      
    /* Set up pin modes */
    pinMode(IsolationFET, OUTPUT);       // sets the Isolation FET pin as output
    pinMode(MainFET, OUTPUT);            // sets the Main Stage FET pin as output
    pinMode(FloatFET, OUTPUT);           // sets the Float Stage FET pin as output
    pinMode(InitialLED, OUTPUT);         // sets the Isolation FET pin as output
    pinMode(MainLED, OUTPUT);            // sets the Main Stage FET pin as output
    pinMode(FloatLED, OUTPUT);           // sets the Float Stage FET pin as output


    /* Configure output pin initial conditions */
    digitalWrite(IsolationFET, LOW);    // Battery is not connected to boost converter
    digitalWrite(MainFET, HIGH);   // Main Stage Resistor is not part of circuit
    digitalWrite(FloatFET, HIGH);  // Float stage resistor is not part of circuit
    digitalWrite(InitialLED, LOW);    // Battery is not connected to boost converter
    digitalWrite(MainLED, HIGH);   // Main Stage Resistor is not part of circuit
    digitalWrite(FloatLED, HIGH);  // Float stage resistor is not part of circuit

   
    /* Set up serial */
    Serial.begin(19200);
    Serial.println("Configured");
}

int main(void){
    init();
    CheckBatteryVoltage();
    while(1){
        ChargeBattery();
    }
}

void CheckBatteryVoltage(void){
  
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
    }
    
    Serial.print("Battery: ");
    Serial.println(BatteryVoltage);
}

void ChargeBattery(void){
    int DutyCycle = 1;
    int CurrentCheck = analogRead(CurrentCheckInput);
    if (CurrentCheck < Current && DutyCycle < 200 && BatteryVoltage<=BoostLimit){
      DutyCycle++;
    }
    
    if (CurrentCheck > Current && DutyCycle > 0){
      DutyCycle--;
    }
    analogWrite(PWMOutput, DutyCycle);
    delay(50);
    Serial.print("Boost: ");
    Serial.println(BoostVoltage);
}
