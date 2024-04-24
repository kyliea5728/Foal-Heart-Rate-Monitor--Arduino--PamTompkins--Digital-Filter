#include <Arduino.h>
#include <avr/sleep.h>
#include <Arduino_LSM6DSOX.h>
#include "thingProperties.h"

//For HeartRate Monitoring
#define SAMPLINGRATE 800 //Hz
#define SAMPLINGPERIOD 1.3 //ms (1/SAMPLINGRATE)
#define ECG_PIN A7
#define REFRACTORY 300 // period to wait before another R-wave is possible (ms). Reduces double counting of peaks 
// Pan-Tompkins parameters
//const int windowSize = 77; // May need tuning -> QRS duration estimated 100ms -> QRS duration(ms)/SAMPLINGPERIOD(ms)
//float integratedSignal[windowSize] = {0}; //buffer to hold values
float threshold = 0.3; // Needs to be tuned according to actual QRS amplitude
long lastRPeakTime = 0; //used to store the timestamp of the last detected R-peak. Initially set to 0
const int N = 2; //filter order 
//Notch filter coefficients 
float a[] = {1, -1.71596743185558, 0.924390491658207};
float b[] = {0.962195245829104	-1.71596743185558	0.962195245829104};
// Arrays for past samples
float x[N+1] = {0};
float y[N+1] = {0};
unsigned long lastSampleTime = 0; // Variable to store the last time a sample was taken, ensures correct sampling period

//For Battery Tracking
const int analogPinBattery = A5;
unsigned long StartTime;

//for respiratory systen
const int analogPin = A2;
volatile float currentValue ;
volatile float previousValue = 0;
volatile bool peakDetected = false;
volatile unsigned long previousPeakTime = 0;
volatile unsigned long peakInterval = 0;
enum State { INIT, INCREASE, DECREASE };
State currentState = INIT;
const float voltageThreshold = 40;

//For Activity Monitoring
float a_x, a_y, a_z; // Declare global variables for accelerometer readings
float accelMagnitude = 0; // Declare a global variable for acceleration magnitude

///////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println(" ");
  Serial.println("Started");
  delay(1000);

  lastSampleTime = millis(); //for heartrate monitoring

  StartTime = micros(); //for battery tracking

//for respiratory
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 64; //interupt trigger every 4ms
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
  previousPeakTime = micros();  

  //For Arduino Cloud
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

}

///////////////////////////////////////
void loop() {
  // put your main code here, to run repeatedly:
     unsigned long currentMillis = millis();
    
    //Heart Rate
    // Check if it's time to sample again for heart rate
    if (currentMillis - lastSampleTime >= SAMPLINGPERIOD) {
        lastSampleTime = currentMillis; // Update the last sampled time

        // Get the current ECG sample
        float ECG = (analogRead(ECG_PIN) * 3.3) / 1023;

        // Apply the notch filter to the ECG sample
        float filteredECG = filter(ECG);

        // Apply QRS detection
        if (detectQRS(filteredECG)) {
            float heartRate = calculateHeartRate(millis()); 
            Serial.println(" ");
            Serial.print("Heart Rate: ");
            Serial.print(heartRate);
            Serial.println(" BPM");
        }
    }
    //Print Statements for testing signal after being filtered
    //Serial.print(filteredECG);
    //Serial.print("     ");

    //Battery Tracking
    if ((micros() - StartTime) >= 1000000){
        StartTime = micros();
        BatteryTracking();
      }

      //Respiratory
      if (peakDetected) {
            float frequency =1000000.0 / peakInterval; 
              if (frequency < 10){
                //Serial.print(currentValue);
                //Serial.print("Frequency: ");
                //Serial.print(frequency); 
                //Serial.println(" Hz");
                Serial.print("Respiratory rate: ");
                Serial.print(60*frequency);
                Serial.println(" breaths/min");
                peakDetected = false;
           }
      }

      //Get Magnitude from IMU data
      if (IMU.accelerationAvailable()) {
            IMU.readAcceleration(a_x, a_y, a_z);
            Accelmagnitude = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
      }

      ArduinoCloud.update(); // Keep the cloud connection updated
}

///////////////////////////////////////
bool detectQRS(float sample) {

    // Differentiate and square 
    static float lastSample = 0;
    float derivative = sample - lastSample;
    lastSample = sample;
    float squared = derivative * derivative;

    // amplify squared value more
    squared = squared * 10;

    //Print Statements for testing signal after derivitive is taken and squared
    //Serial.print(squared);
    //Serial.print("     ");

    // Moving window integration
    /*static int index = 0;
    integratedSignal[index] = squared;


    index = (index + 1) % windowSize;
    float sum = 0;
    for (int i = 0; i < windowSize; i++) {
      sum += integratedSignal[i];
    }
    float integrated = sum / windowSize;*/

    // Thresholding for QRS detection
    long currentTime = millis();


    //if (integrated > threshold && currentTime - lastRPeakTime > REFRACTORY) { // 200 ms refractory period
    if (squared > threshold && currentTime - lastRPeakTime > REFRACTORY) { // 200 ms refractory period
        
        lastRPeakTime = currentTime;
        return true; // QRS detected
      }
    
    return false;
}

///////////////////////////////////////
float calculateHeartRate(long rPeakTime) {
    static long lastRPeakTime = 0; //not reinitialized to 0 each time because static

    if (lastRPeakTime == 0) { //if this is the first R peak detected 
        lastRPeakTime = rPeakTime;
        return 0; 
    }
    
    float rrInterval = (rPeakTime - lastRPeakTime) / 1000.0; // Convert milliseconds to seconds
    lastRPeakTime = rPeakTime;
    return 60.0 / rrInterval; // BPM
}

///////////////////////////////////////
//Function for notch filter
float filter(float input) { 
    // Shift past samples
    for (int i = N; i > 0; --i) {
        x[i] = x[i - 1];
        y[i] = y[i - 1];
    }

    // Add new sample
    x[0] = input;

    // Compute the filter output
    float output = b[0] * x[0]; // Initialize with the first term
    for (int i = 1; i <= N; ++i) {
        output += b[i] * x[i]; // Apply b coefficients
        output -= a[i] * y[i]; // Apply a coefficients, skipping a[0] as it's assumed to be 1
    }

    y[0] = output;
    return output;
}

///////////////////////////////////////
void BatteryTracking(){

  int  BatteryVoltage = analogRead(analogPinBattery);
  float voltage = BatteryVoltage * (5.0 / 1023.0);

  if (voltage <= 2.8){
    Serial.print("Battery is low. Please charge!");
  }

}

///////////////////////////////////////
//Function for respiratory system
ISR(TIMER1_COMPA_vect) {
  // This function will be called by the timer interrupt
  float  rawValue = analogRead(analogPin);
  unsigned long currentTime = micros();

  if (abs(rawValue - previousValue) >= voltageThreshold) {
    currentValue = rawValue;

  switch (currentState) {
    case INIT:
      if (previousValue == 0 ) {
        currentState = INIT;
      }
      if (currentValue > previousValue) {
        currentState = INCREASE;
      } else {
        currentState = DECREASE;
      }
      break;

    case INCREASE:
      if (currentValue < previousValue) {
        if (previousPeakTime > 0) {
          peakInterval = currentTime - previousPeakTime;
          peakDetected = true;
          previousPeakTime = currentTime; 
        } else {
          Serial.println("First peak detected.");
        }
        currentState = DECREASE;
      }
      break;

    case DECREASE:
      if (currentValue > previousValue) {
        currentState = INCREASE;
      }
      break;
  }
  previousValue = currentValue;
}
}
