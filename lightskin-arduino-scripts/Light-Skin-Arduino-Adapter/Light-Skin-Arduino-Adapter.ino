#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


#define LED_OFF HIGH
#define LED_ON LOW


const int LEDs[] = {
  27, 29, 31, 33, 35, 37, 39
};
const int Sensors[] = {
  A6, A5, A4, A3, A2, A1, A0
};


// Samples to be measured for each sensor measurement
const int SAMPLES = 20;




const int LEDs_num = sizeof(LEDs) / sizeof(int);
const int Sensors_num = sizeof(Sensors) / sizeof(int);




void setup() {
  // Increase ADC sampling rate according to 
  // https://yaab-arduino.blogspot.de/2015/02/fast-sampling-from-analog-input.html
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  // Increase resolution of ADCs in the relevant value range
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




float zeroValues[Sensors_num];
float allValues[LEDs_num][Sensors_num];
int lastLED = LEDs[0];
long sum;
float value;
int l, s, i; // Inner loop variables; so they don't need to be reallocated every time


//char buf[15]; // faster printing
//char out[500];
const char* comma = ",";

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
      sum += analogRead(sensor);
    }
    value = (float) sum / SAMPLES;

    zeroValues[s] = value;
  }

  for(l = 0; l < LEDs_num; l++){
    led = LEDs[l];
    
    // switch to new LED
    digitalWrite(lastLED, LED_OFF);
    digitalWrite(led, LED_ON);
    
    // Measure all sensors
    for(s = 0; s < Sensors_num; s++){
      sensor = Sensors[s];
      // Read sensor once to switch ADC circuit to this pin (discard result)
      analogRead(sensor);

      sum = 0;
      for(i = 0; i < SAMPLES; i++){
        sum += analogRead(sensor);
      }
      value = (float) sum / SAMPLES;

      allValues[l][s] = max(0, value - zeroValues[s]);
    }
    
    lastLED = led;
  }
  digitalWrite(lastLED, LED_OFF);
  /* */


  /*
  // Measure all sensors
  for(s = 0; s < Sensors_num; s++){
    sensor = Sensors[s];
    // Read sensor once to switch ADC circuit to this pin (discard result)
    analogRead(sensor);

    for(l = 0; l < LEDs_num; l++){
      led = LEDs[l];
  
      // switch to new LED
      digitalWrite(lastLED, LED_OFF);
      digitalWrite(led, LED_ON);
  
      sum = 0;
      for(i = 0; i < SAMPLES; i++){
        sum += analogRead(sensor);
      }
      value = (float) sum / SAMPLES;
  
      allValues[l][s] = value;
      
      lastLED = led;
    }
    digitalWrite(lastLED, LED_OFF);
  }
  /* */

  
  t = micros();

  
  // Print the measurements to Serial
  
  Serial.print("Snapshot: ");
  Serial.print(LEDs_num);
  Serial.print(',');
  Serial.print(Sensors_num);
  Serial.println();
  /* */

  //out[0] = 0;
  //out.reserve(500);
  for(l = 0; l < LEDs_num; l++){
    for(s = 0; s < Sensors_num; s++){
      //dtostrf(allValues[l][s], 7, 3, buf);
      Serial.print(allValues[l][s], 2);
      //out += 'o';
      //strcat(out, "o");
      //strcat(out, comma);
      Serial.print(',');
    }
    Serial.println();
    //buf += '\n';
  }
  //Serial.print(out);
  Serial.println();
  /* */
  t = micros();

  //Serial.println(t-t0);

  // Wait for the next snapshot to be taken
  delay(2);
}
