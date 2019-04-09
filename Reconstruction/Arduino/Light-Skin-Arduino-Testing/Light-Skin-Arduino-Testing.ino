#include "SimpleKalmanFilter.h"
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define LED_OFF HIGH
#define LED_ON LOW

/*const int LEDs[] = {
  27, 29, 31, 33, 35, 37, 39
};
const int Sensors[] = {
  A6, A5, A4, A3, A2, A1, A0
};*/
float vit[20];
const int LEDs[] = {
  //27, 29, 31, 33, 35, 37, 39
  // 51, 44 busted
  53, 51, 49, 47, 45, 43, 52, 50, 48, 46, 44, 42

};
const int Sensors[] = {
  //A6, A5, A4, A3, A2, A1, A0
  A15, A14, A13, A12, A11, A10, A9, A8, A7, A6, A5, A4
};

// Samples to be measured for each sensor measurement
const int SAMPLES = 20;
const int LEDs_num = sizeof(LEDs) / sizeof(int);
const int Sensors_num = sizeof(Sensors) / sizeof(int);
float zeroValues[Sensors_num];
float allValues_Old[LEDs_num][Sensors_num];
float allValues_delta[LEDs_num][Sensors_num];
float allValues[LEDs_num][Sensors_num];
int lastLED = LEDs[0];
long sum;
float value;
int l, s, i; // Inner loop variables; so they don't need to be reallocated every time
//char buf[15]; // faster printing
//char out[500];
const char* comma = ",";
SimpleKalmanFilter kf = SimpleKalmanFilter(1, 1, 0.05);

void setup() {
  // Increase ADC sampling rate according to
  // https://yaab-arduino.blogspot.de/2015/02/fast-sampling-from-analog-input.html
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  // Increase resolution of ADCs in the relevant value range
  //analogReference(DEFAULT);//INTERNAL1V1);
  analogReference(DEFAULT);//INTERNAL1V1);
  for(int i = 0; i < LEDs_num; i++){
    pinMode(LEDs[i], OUTPUT);
    digitalWrite(LEDs[i], LED_OFF);
  }
  for(int i = 0; i < Sensors_num; i++){
    pinMode(Sensors[i], INPUT);
  }

  Serial.begin(1000000);
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
      //Serial.println(v);
    }
    value = (float) sum / SAMPLES;
    //Serial.println(value);
    zeroValues[s] = value;
  }


// Measure all sensors
    for(s = 0; s < Sensors_num; s++){
      sensor = Sensors[s];

      for(l = 0; l < LEDs_num; l++){
        led = LEDs[l];

    // switch to new LED

        digitalWrite(lastLED, LED_OFF);
        //delay(30);
        digitalWrite(led, LED_ON);
        delay(4);



     /* Serial.print("Reading sensor ");
      Serial.print(Sensors[s]);
      Serial.print(" for LED ");
      Serial.print(led);*/
      // Read sensor once to switch ADC circuit to this pin (discard result)
      analogRead(sensor);

      sum = 0;

      const int vit_leng = sizeof(vit) / sizeof(int);
      for(i = 0; i < SAMPLES; i++){
        long v = analogRead(sensor);
        vit[i] = v;
        sum += v;
        delay(1);
        //Serial.println(v);
      }


     /* if (s == 1){
        for (int i = 0; i < 20; i++){
          Serial.println(vit[i]);
        }
      }*/
      value = (float) sum / SAMPLES;
      // Save last measurements for comparison
      allValues_Old[l][s] = allValues[l][s];
      //Serial.println(value);
      //Serial.println(zeroValues[s]);
      allValues[l][s] = max(0, value - zeroValues[s]);
      //Serial.print(" : ");
      //Serial.println(allValues[l][s]);
      //Serial.println("-------------------");
      lastLED = led;
      digitalWrite(lastLED, LED_OFF);
    }


  }
   digitalWrite(lastLED, LED_OFF);

/*-------------------------------------------------*/
  // Print the measurements to Serial
 /*Serial.print("Matrix snapshot: ");
  //Serial.print(LEDs_num);
  Serial.print("x");
  //Serial.print(Sensors_num);
  Serial.println();

int next_sensor = 0;
  for(s = 0; s < Sensors_num; s++){
    Serial.print("  ");
    Serial.print(Sensors[next_sensor]);
    Serial.print(", ");
    next_sensor += 1;
  }

  int next_led = 27;
  Serial.println();

  for(l = 0; l < LEDs_num; l++){
    //Serial.print(next_led);
    Serial.print("  ");
    next_led += 2;
    for(s = 0; s < Sensors_num; s++){
      Serial.print(allValues[l][s], 2);
       //Serial.print(kf.updateEstimate(allValues[l][s]), 2);
      Serial.print(',');
    }
    Serial.println();
  }
  Serial.println();
  Serial.print(allValues[0][8],2);
  Serial.print(',');
  Serial.print(allValues[3][11],2);
  Serial.println();

/*-------------------------------------------------*/
// Print delta of measurements to Serial
/*Serial.print("Matrix of deltas; ");
Serial.print(LEDs_num);
Serial.print("x");
Serial.print(Sensors_num);
Serial.println();

next_sensor = 0;
  for(s = 0; s < Sensors_num; s++){
    Serial.print("  ");
    Serial.print(Sensors[next_sensor]);
    Serial.print(", ");
    next_sensor += 1;
  }

next_led = 53;
float largest_delta = 0;
  Serial.println();
  for(l = 0; l < LEDs_num; l++){
    Serial.print(next_led);
    Serial.print("  ");
    next_led += 2;
    for(s = 0; s < Sensors_num; s++){
      allValues_delta[l][s] = allValues[l][s] - allValues_Old[l][s];
      if (allValues_delta[l][s] < largest_delta){
        largest_delta = allValues_delta[l][s];
      }
      Serial.print(allValues_delta[l][s], 2);
      Serial.print(',');
    }
    Serial.println();
  }
  Serial.println();
  Serial.println(largest_delta);
  Serial.println("-------------------");

/*-------------------------------------------------*/
/*-------------------------------------------------*/
  // Print the measurements to Serial
 Serial.print("Snapshot: ");
  Serial.print(LEDs_num);
  Serial.print(',');
  Serial.print(Sensors_num);
  Serial.println();

  for(l = 0; l < LEDs_num; l++){
    for(s = 0; s < Sensors_num; s++){
      Serial.print(allValues[l][s], 2);
      //Serial.print(',');
      //Serial.print(kf.updateEstimate(allValues[l][s]), 2);
      Serial.print(',');
    }
    Serial.println();
  }
  Serial.println();
  /*-------------------------------------------------*/

  // Wait for the next snapshot to be taken
  delay(10);
}
