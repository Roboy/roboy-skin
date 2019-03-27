#include <OSCMessage.h>
#include <WiFiUdp.h>
#include "SimpleKalmanFilter.h"
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define LED_OFF HIGH
#define LED_ON LOW

const int LEDs[] = {
  //27, 29, 31, 33, 35, 37, 39
  // 51, 44 busted
  53, 51, 49, 47, 45, 43, 52, 50, 48, 46, 44, 42
 //51

};
const int Sensors[] = {
  //A6, A5, A4, A3, A2, A1, A0
  A15, A14, A13, A12, A11, A10, A9, A8, A7, A6, A5, A4
  //A0, A1
};

// Samples to be measured for each sensor measurement
const int SAMPLES = 20;
const int LEDs_num = sizeof(LEDs) / sizeof(int);
const int Sensors_num = sizeof(Sensors) / sizeof(int);
float zeroValues[Sensors_num];
float allValues[LEDs_num][Sensors_num];
int lastLED = LEDs[0];
long sum;
float value;
int l, s, i; // Inner loop variables; so they don't need to be reallocated every time
//char buf[15]; // faster printing
//char out[500];
const char* comma = ",";
//SimpleKalmanFilter kf = SimpleKalmanFilter(3, 3, 0.6);
//SimpleKalmanFilter kf = SimpleKalmanFilter(1, 1, 0.007);
SimpleKalmanFilter kf = SimpleKalmanFilter(1, 1, 0.05);

void setup() {

  // Increase ADC sampling rate according to
  // https://yaab-arduino.blogspot.de/2015/02/fast-sampling-from-analog-input.html
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  // Increase resolution of ADCs in the relevant value range
  //analogReference(INTERNAL1V1);
  analogReference(DEFAULT);

  for(int i = 0; i < LEDs_num; i++){
    pinMode(LEDs[i], OUTPUT);
    digitalWrite(LEDs[i], LED_OFF);
  }
  for(int i = 0; i < Sensors_num; i++){
    pinMode(Sensors[i], INPUT);
  }

  Serial.begin(1000000);
  //Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
}

void loop() {
  long t0, t;
  t0 = micros();
  int led, sensor;
  // Take a "snapshot": measure all the LED / Sensor combinations

  // Measure all sensors for their zero-values while LEDs are off
  for(s = 0; s < Sensors_num; s++){
    sensor = Sensors[s];
    // Read sensor once to switch ADC circuit to this pin (discard result)
    analogRead(sensor);

    sum = 0;
    for(i = 0; i < SAMPLES; i++){
      long v = analogRead(sensor);

      sum += v;
      delay(1);
    }
    value = (float) sum / SAMPLES;

    zeroValues[s] = value;
  }

  for(l = 0; l < LEDs_num; l++){
    led = LEDs[l];

    // switch to new LED
    digitalWrite(lastLED, LED_OFF);
    digitalWrite(led, LED_ON);
    delay(4);

    // Measure all sensors
    for(s = 0; s < Sensors_num; s++){
      sensor = Sensors[s];
      /*Serial.print("Reading sensor ");
      Serial.print(Sensors[s]);
      Serial.print(" for LED ");
      Serial.print(led);*/
      // Read sensor once to switch ADC circuit to this pin (discard result)
      analogRead(sensor);

      sum = 0;
      for(i = 0; i < SAMPLES; i++){
        long v = analogRead(sensor);
        //Serial.println(v);

        sum += v;
      }
      value = (float) sum / SAMPLES;

      allValues[l][s] = max(0, value - zeroValues[s]);
      //Serial.print(" : ");
      //Serial.println(allValues[l][s]);
      //Serial.println("-------------------");
    }
    lastLED = led;
  }
  digitalWrite(lastLED, LED_OFF);

  /*-------------------------------------------------*/
  // Print the measurements to Serial
  Serial.print("Snapshot: ");
  Serial.print(LEDs_num);
  Serial.print(',');
  Serial.print(Sensors_num);
  Serial.println();

  //Get rid of values from the same side
 /* allValues[0][0] =  0.0;
  allValues[0][1] =  0.0;
  allValues[0][2] =  0.0;

  allValues[1][0] =  0.0;
  allValues[1][1] =  0.0;
  allValues[1][2] =  0.0;

  allValues[2][0] =  0.0;
  allValues[2][1] =  0.0;
  allValues[2][2] =  0.0;

  allValues[3][3] =  0.0;
  allValues[3][4] =  0.0;
  allValues[3][5] =  0.0;

  allValues[4][3] =  0.0;
  allValues[4][4] =  0.0;
  allValues[4][5] =  0.0;

  allValues[5][3] =  0.0;
  allValues[5][4] =  0.0;
  allValues[5][5] =  0.0;

  allValues[6][6] =  0.0;
  allValues[6][7] =  0.0;
  allValues[6][8] =  0.0;

  allValues[7][6] =  0.0;
  allValues[7][7] =  0.0;
  allValues[7][8] =  0.0;

  allValues[8][6] =  0.0;
  allValues[8][7] =  0.0;
  allValues[8][8] =  0.0;

  allValues[9][9] =  0.0;
  allValues[9][10] =  0.0;
  allValues[9][11] =  0.0;

  allValues[10][9] =  0.0;
  allValues[10][10] =  0.0;
  allValues[10][11] =  0.0;

  allValues[11][9] =  0.0;
  allValues[11][10] =  0.0;
  allValues[11][11] =  0.0;*/

  for(l = 0; l < LEDs_num; l++){
    for(s = 0; s < Sensors_num; s++){
      Serial.print((allValues[l][s]),2);
      //Serial.print(kf.updateEstimate(allValues[l][s]), 2);
      Serial.print(',');
    }
    Serial.println();
  }
  Serial.println();
//  Serial.print(allValues[11][3],2);
//  Serial.print(',');
//  Serial.print(allValues[8][0],2);
//  Serial.print(',');
//  Serial.print(allValues[4][10],2);
//  Serial.println();
  /*-------------------------------------------------*/

  // Wait for the next snapshot to be taken
  delay(100);
}
