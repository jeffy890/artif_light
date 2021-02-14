#include "arduinoFFT.h"
#include <Servo.h>

#define SAMPLING_FREQUENCY 10000
#define MIC 36

const uint16_t FFTsamples = 256;
double vReal[FFTsamples];
double vImag[FFTsamples];
arduinoFFT FFT = arduinoFFT(vReal, vImag, FFTsamples, SAMPLING_FREQUENCY);
Servo servo;

unsigned int sampling_period_us;
float dmax = 5.0;
float value = 0;

void sample(int nsamples) {
  for (int i = 0; i < nsamples; i++) {
    unsigned long t = micros();
    vReal[i] = (double)analogRead(MIC) / 4095.0 * 3.6 + 0.1132;
    vImag[i] = 0;
    while ((micros() - t) < sampling_period_us) ;
  }
}

void DCRemoval(double *vData, uint16_t samples) {
  double mean = 0;
  for (uint16_t i = 1; i < samples; i++) {
    mean += vData[i];
  }
  mean /= samples;
  for (uint16_t i = 1; i < samples; i++) {
    vData[i] -= mean;
  }
}

bool judgeFreq(int nsamples) {
  for (int band = 0; band < nsamples; band++) {
    float d = vReal[band];
    if (d > dmax) {
      value = (band * 1.0 * SAMPLING_FREQUENCY) / FFTsamples / 1000;
      Serial.print(d);
      Serial.print("  ");
      Serial.println(value);
      if (value > 1.8 && value < 2.2) {
        return true;
      }
    }
  }
  return false;
}

void setup() {
  pinMode(MIC, INPUT);
  pinMode(27, OUTPUT);
  digitalWrite(27, LOW);  // set led pin low(mosfet is off)

  servo.attach(13);
  servo.write(90);  // set servo position

  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  //Serial.begin(9600);
}

void loop() {
  digitalWrite(27, LOW);
  servo.write(90);
  
  sample(FFTsamples);

  DCRemoval(vReal, FFTsamples);
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  bool judge = judgeFreq(FFTsamples / 2);
  if (judge == true) {
    digitalWrite(27, HIGH);
    for (int i = 0; i < 45; i++) {
      servo.write(i);
    }
    delay(6000);
  }
}
