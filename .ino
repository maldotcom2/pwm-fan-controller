#include <TimerOne.h> // timer1 library for setting pwm frequency
//For Arduino Pro-Mini 5v 16 MHz
//////////////////////NOTES////////////////////////////////////////////////////////////////////////////////
//vT - temp indexes based on theoretical from voltage divider calculator based on 10k thermistor with 10k fixed resistor
// 2.21 - 20C
// 2.5 - 25C
// 2.78 - 30C
// 3.05 - 35C
// 3.3 - 40C
// 3.52 - 45C
// 3.72 - 50C
// Linear conversion formula
// NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
///////////////////////////////////////////////////////////////////////////////////////////////////////////

float dutyCycle = 0;                  // PWM setting from 0 - 255
float dutyNow = 0;                    // storage of current dutycycle
float fanPercent = 0;                 // percentage of 0 - 255 PWM range - only used for USB logging
float vT = 0;                         // thermistor voltage read
float vMin = 0;                       // vT fan-on threshold
float vMax = 0;                       // vT max fan threshold
float tArr[15];                       // temperature read array initialisation
int tMaxSensor = A0;                  // max pot read pin
int tMinSensor = A1;                  // min pot read pin
int tSensor = A2;                     // thermistor read pin
int offCount = 0;                     // how many cycles have fans been off
int tMaxAnalog = 0;                   // analog read value from Max pot
int tMinAnalog = 0;                   // analog read value from Min pot
int dutyMin = 80;                     // slowest the fan can spin
int pollTime = 500;                   // increase delay time (ms) on each loop frequency
int wasOnReset = 3600000 / pollTime;  // reset wasOn time after fixed period of time regardless of pollTime 
int tIndex = 0;                       // temperature read array index
int tSum = 0;                         // sum of raw analog thermistor voltage reads
bool wasOn = 0;                       // flag for whether fans were on before
bool isOn = 0;                        // flag for whether fans are on now
bool errorState = 0;                  // flag for health state of system
long int startTime = 0;               // millis value
long int startTime2 = 0;              // millis value
volatile unsigned long tacho1 = 0;    // tacho1 interrupt counter
volatile unsigned long tacho2 = 0;    // tacho2 interrupt counter
long int elapsed = 0;                 // time since last rpm evaluation
long int rpmNow1 = 0;                 // fan1 rpm value
long int rpmNow2 = 0;                 // fan2 rpm value
unsigned long tachoCopy1 = 0;         // storage for interrupt count
unsigned long tachoCopy2 = 0;         // storage for interrupt count

void setup() {
  Timer1.initialize(40);    // 40 microseconds = Set timer1 to 25 kHz. 1 trigger every 40 microseconds = (1 / 25000) or (1 / 0.00004)
  Serial.begin(9600);       // start serial communication with 9600 baud rate
  delay(1000);
  pinMode(2, INPUT_PULLUP); // fan 1 tacho
  pinMode(3, INPUT_PULLUP); // fan 2 tacho
  pinMode(4, OUTPUT);       // buzzer
  pinMode(6, OUTPUT);       // vcc to min pot
  pinMode(7, OUTPUT);       // vcc to max pot
  pinMode(8, OUTPUT);       // thermistor power
  pinMode(9, OUTPUT);       // fan pwm
  pinMode(13, OUTPUT);      // LED

  attachInterrupt(digitalPinToInterrupt(2), tachisr1, RISING); // pin 2 is the tacho1 input pin, pulse counter is tachisr1 function and the rising edge is the interrupt trigger
  startTime = millis();

  attachInterrupt(digitalPinToInterrupt(3), tachisr2, RISING); // pin 3 is the tacho2 input pin, pulse counter is tachisr2 function and the rising edge is the interrupt trigger
  startTime2 = millis();
}

void statusled(){
  if (errorState == 1) {
    for (int i = 0; i < 5; i++) {     // long flash status LED and pulse buzzer for 5 iterations
    digitalWrite(13, HIGH);
    digitalWrite(4, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    digitalWrite(4, LOW);
    delay(1000);
    }
  }

  else {                              // short flash status led
    digitalWrite(13, HIGH);
    delay(50);
    digitalWrite(13, LOW);
    delay(50);
  }
}

void tachisr1(){
  tacho1++;
}

void tachisr2(){
  tacho2++;
}

void readpots(){
  digitalWrite(7, HIGH); // pot power
  delay(200);
  tMinAnalog = analogRead(tMinSensor);
  tMaxAnalog = analogRead(tMaxSensor);
  digitalWrite(7, LOW); // pot power
// linear conversion vMin Pot  analog range 0 - 1023 range to 2.1 - 3.05 (approx 20 - 35 degrees) to make full use of the dial 
  vMin = (((tMinAnalog - 0) * (3.05 - 2.21)) / (1023 - 0)) + 2.21;
// linear conversion vMax Pot  analog range 0 - 1023 range to 3.06 - 3.72 (approx 35 - 50 degrees) to make full use of the dial 
  vMax = (((tMaxAnalog - 0) * (3.72 - 3.06)) / (1023 - 0)) + 3.06; 
}

void readtemp(){
  digitalWrite(8, HIGH);                // power the thermistor
  for (int i = 0; i < 15; i++) {        // read the temperature 15 times
    tArr[tIndex] = analogRead(tSensor); // read raw analog value into array element
    tSum += tArr[tIndex];               // add raw analog values together into single total
    tIndex++;                           // increment the array index
    delay(100);                         // wait 100ms between reads
  }
  digitalWrite(8, LOW);
  vT = ((tSum / 15) * (5.0 / 1023.0));  // take average of raw analog reads and convert into voltage value
  tSum = 0;
  tIndex = 0;
}

void fanstart(){
  Serial.print("Fan start");
  Serial.println();
  dutyCycle = 128;
  isOn = 1;
  offCount = 0;
}

void fanstop(){
  Serial.print("Fan stop");
  Serial.println();
  dutyCycle = 0;
  isOn = 0;
  wasOn = 1;
  fanduty();
}

void fanduty(){
  Serial.print("Applying dutycycle ");
  Serial.print(dutyCycle);
  Serial.println();
  analogWrite(9, dutyCycle);      // set PWM dutycycle on fans
  dutyNow = dutyCycle;            // copy current dutycycle for later comparison
}

void readtacho(){
// Fan tacho read
  delay(3000);                                        // allow fans to reach set speed
  elapsed = millis() - startTime;                     // duration since last RPM read
  noInterrupts();                                     // pause interrupts
  tachoCopy1 = tacho1;                                // copy out value for fan1
  tachoCopy2 = tacho2;                                // copy out value for fan2
  tacho1 = 0;                                         // reset isr value
  tacho2 = 0;                                         // reset isr value
  interrupts();                                       // resume interrupts
  rpmNow1 = (tachoCopy1 / 2.0) * (60000 / elapsed);
  rpmNow2 = (tachoCopy2 / 2.0) * (60000 / elapsed);
  startTime = millis();                               // new start time
  startTime2 = millis();                              // new start time
}

void loop() {
  statusled(); 
   
  readpots(); 

  readtemp();
  
// linear conversion of voltage (temperature) values to PWM values for fan speed
  dutyCycle = (((vT - vMin) * (255 - dutyMin)) / (vMax - vMin)) + dutyMin;
  
  if (dutyCycle > 255){
    dutyCycle = 255;
  }

// run fans at 50% on cold start. If the fan was on previously, temp must rise slightly before fan will trigger again
  if (dutyCycle >= dutyMin && isOn == 0 && errorState == 0) {
    if (wasOn == 0) {
      fanstart();
    }
    
    if (wasOn == 1 && (vT >= vMin + 0.03)) {
      fanstart();
    }
  }

// once the fans are on, apply new dutycycle if dutycycle above min and the change will be more than 5 from current setting
  if (dutyCycle >= dutyMin && ((dutyCycle < (dutyNow - 5)) || (dutyCycle > dutyNow + 5)) && isOn == 1) {
    fanduty();    
  }

// turn off fans at min speed threshold
  if (dutyCycle <= dutyMin && isOn == 1) {
    Serial.print("Temp low");
    Serial.println();
    fanstop();
  }

// counter to reset wasOn
  if (isOn == 0 && wasOn == 1) {
    offCount++;    
  }

// reset wasOn after period of time
  if (offCount >= wasOnReset) {
    wasOn = 0;
    offCount = 0;
  }

// get fan % from dutycycle.
  fanPercent = dutyCycle / 255 * 100;
  if (fanPercent < 0 || isOn == 0) {
    fanPercent = 0;
  }

readtacho();

// check fan tachos and if fan stalled set error and turn off
  if ((rpmNow1 == 0 || rpmNow2 == 0) && isOn == 1) {
    Serial.print("Fan stalled or disconnected");
    Serial.println();
    errorState = 1;
    fanstop();
  }
  
//////////////////////DEBUG////////////////////////////////////////////////////////////////////////////////
  Serial.print("vT is ");
  Serial.print(vT);
  Serial.println();
  Serial.print("vMin is ");
  Serial.print(vMin);
  Serial.println();
  Serial.print("vMax is ");
  Serial.print(vMax);
  Serial.println();
  Serial.print("Fan Percent is ");
  Serial.print(fanPercent);
  Serial.println();
  Serial.print("Fan1 tacho reading is ");
  Serial.print(rpmNow1);
  Serial.println();
  Serial.print("Fan2 tacho reading is ");
  Serial.print(rpmNow2);
  Serial.println();
  Serial.print("offCount is ");
  Serial.print(offCount);
  Serial.println();
  Serial.print("wasOn is ");
  Serial.print(wasOn);
  Serial.println();
  Serial.print("isOn is ");
  Serial.print(isOn);
  Serial.println();
  Serial.print("errorState is ");
  Serial.print(errorState);
  Serial.println();
///////////////////////////////////////////////////////////////////////////////////////////////////////////
  delay(pollTime);
}