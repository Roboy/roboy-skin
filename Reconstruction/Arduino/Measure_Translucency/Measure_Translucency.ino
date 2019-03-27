
#define LED 43
#define SENSOR A10



#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // Increase ADC sampling rate according to 
  // https://yaab-arduino.blogspot.de/2015/02/fast-sampling-from-analog-input.html
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  
  // Increase resolution of ADCs in the relevant value range
  analogReference(INTERNAL1V1);
}


const int samplesPerIteration = 40;
const int iterationsPerResult = 40;
const int samplesPerResult = samplesPerIteration * iterationsPerResult;

long onSum = 0, offSum = 0;
float onVal, offVal, val;

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:

  onSum = 0;
  offSum= 0;

  for (int i = 0; i < iterationsPerResult; i++){
    digitalWrite(LED, HIGH);
    delayMicroseconds(50);
    for (int j = 0; j < samplesPerIteration; j++){
      int sensorValue = analogRead(SENSOR);
      offSum += sensorValue;
    }
    digitalWrite(LED, LOW);
    delayMicroseconds(50);
    for (int j = 0; j < samplesPerIteration; j++){
      int sensorValue = analogRead(SENSOR);
      onSum += sensorValue;
    }
  }

  offVal = (float) offSum / samplesPerResult;
  onVal = (float) onSum / samplesPerResult;
  val = onVal - offVal;
  

  //Serial.print("0,500,");
  Serial.println(val);
  Serial.println(val);
  Serial.println(val);
  Serial.println(val);
  //delay(10);
}
