#include "arduinoFFT.h"
#include "pwm.cpp" 

arduinoFFT FFT = arduinoFFT();

const uint16_t samples = 128;
const double samplingFrequency = 64000;

unsigned int sampling_period_us;
unsigned long microseconds;

/*
  These are the input and output vectors
  Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];
double vFreq[samples];

// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;
const int led1 = 13;
const int led2 = 2;

// defines variables
double mag_last[30] = {0};
double mag_max[30] = {0};
double mag_now[30];
int cur_max = 10;
int max_idx = 10;
int target = 10;
long duration;
int distance;

PWM myPWM;

void setup()
{
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  digitalWrite(led1, HIGH);
  analogWriteFrequency(3, 50);     // set the frequency of pin 3 to 50Hz // left wheel
  analogWriteFrequency(4, 50);     // set the frequency of pin 3 to 50Hz // right wheel
  analogWriteResolution(10);       // set the resolution to 10 bits
  ////////////////////////////////////////// ADC SETUP //////////////////////////////////////
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);

  //We want portD 1 for A0, but GPIO is default as analog so lucky us
  //We want to use ADC0, ADC0_SE5b, AD5b
  //Single ended channels AD23- AD4, we want AD5

  //ADC0 Configuration register 1
  ADC0_CFG1 |= _BV(5) | (1U << 6); // clock division of 8. 9MHZ,
  ADC0_CFG1 |= (1U << 2) | (1U << 3); //16 bit conversion mode

  //ADC0 configuration register 2
  ADC0_CFG2 |= _BV(4); //This selects ADXXb, in our case AD05b or just AD5b
  //ADC0_CFG2 |= ADC_CFG2_MUXSEL; //This does the same thing as above

  //ADC0 Programmable gain amplifier
  //ADC0_PGA |= ADC_PGA_PGAEN; //Use this line if you want to use programmable gain amplified that Professor discussed

  //ADC0 Status and control register 3
  ADC0_SC3 |= ADC_SC3_ADCO | ADC_SC3_AVGE; //These are in kinetis.h, enables continuous mode and averaging function. You don't NEED averaging function.

  //ADC0 Status control register 1
  ADC0_SC1A = 5; // or binary 0101, it is equal not OR equal since I disabled the other configurations on purpose.

  //////////////////////////////////////////////////////////////////////////////////////////

  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
  Serial.begin(115200);
  Serial.println("Ready");
  delay(2000);//5s delay for setup
  digitalWrite(led2, HIGH);
  delay(500);
  digitalWrite(led2, LOW);
  myPWM.forward();
  delay(1000);
  myPWM.stay();
}

// Coefficients from Matlab


void loop()
{
  check_ultrasonic();
  delay(100);
  scan();
  get_max();
  if (distance < 30 && distance != 0 && cur_max == 19) //cur_max = current max magnitude to test if it's the goal
  {
    myPWM.stay();
    digitalWrite(led2, HIGH);
    while (1) {}
  } else if (distance < 15 && distance != 0) //too close we move back
  {
    myPWM.backward();
    get_max();
    if (cur_max >= target) // we change target if we reached it
    {
      digitalWrite(led2, HIGH);
      delay(500);
      digitalWrite(led2, LOW);
      target = cur_max + 1 ;
    }
    myPWM.stay();
  } else if (distance >= 15 && distance < 25 && distance != 0) //check if beacon have safe distance for turning
  {
    get_max();
    if (cur_max >= target)
    {
      digitalWrite(led2, HIGH);
      delay(200);
      digitalWrite(led2, LOW);
      target = cur_max + 1 ;
    }
    avoid_beacon();
  } else
  {
    do
    {
      myPWM.left();
      myPWM.stay();
      check_ultrasonic();
      scan();
      target = find_target();
      if (distance < 15 && distance != 0) {
        myPWM.backward();
        myPWM.stay();
      }
    } while (mag_now[target] > 0.9 * mag_last[target]);
    do
    {
      myPWM.right();
      myPWM.stay();
      check_ultrasonic();
      scan();
      target = find_target();
      if (distance < 15 && distance != 0) {
        myPWM.backward();
        myPWM.stay();
      }
    } while (mag_now[target] > 0.9 * mag_last[target]);
    myPWM.left();
    myPWM.stay();
    check_ultrasonic();
    delay(100);
    if (distance < 15 && distance != 0) {
      myPWM.backward();
      myPWM.stay();
    } else
    {
      myPWM.forward();
    }
    myPWM.stay();
  }
}

void adc()
{
  for (int i = 0; i < samples; i++)
  {
    microseconds = micros();    //Overflows after around 70 minutes!
    vReal[i] = ADC0_RA;
    vImag[i] = 0;
    while (micros() < (microseconds + sampling_period_us)) {
      //empty loop
    }
  }
}

void fft_calculate()
{
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */ 
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
}


void scan()
{
  for (int i = 10; i < 20; i++) {
    mag_last[i] = mag_now[i];
    mag_now[i] = 0;
  }
  for (int i = 0; i < 50; i++)
  {
    adc();
    fft_calculate();
    for (int j = 10; j < 20; j++) {
      mag_now[j] += vReal[j];
    }
    delay(25);//you can test remove this line
  }
  for (int i = 10; i < 20; i++)
  {
    mag_now[i] /= 50;
    if (mag_now[i] > mag_max[i])
    {
      mag_max[i] = mag_now[i];
    }
  }
}

void avoid_beacon()
{
  myPWM.left();
  myPWM.stay();
  check_ultrasonic();
  delay(100);
  int left_distance = distance;
  myPWM.right();
  myPWM.right();
  myPWM.stay();
  check_ultrasonic();
  delay(100);
  int right_distance = distance;
  if ((right_distance < left_distance && right_distance != 0) || left_distance == 0 && right_distance > 50) {
    myPWM.forward();
    myPWM.forward();
    myPWM.stay();
  } else {
    myPWM.left();
    myPWM.left();
    while (left_distance < 50) {
      myPWM.left();
      check_ultrasonic();
      delay(100);
      left_distance = distance;
    }
    myPWM.forward();
    myPWM.forward();
    myPWM.stay();
  }
}

void get_max()
{
  for (int i = 10; i < 20; i++) {
    if (mag_now[i] > mag_now[cur_max])
    {
      cur_max = i;
    }
  }
}


void PrintVector(double *vData, double *vFreq, uint16_t bufferSize)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    /* Print frequency value */
    vFreq[i] = ((i * 1.0 * samplingFrequency) / samples);

    Serial.print(vFreq[i], 6);
    Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

int find_target()
{
  for (int i = 11; i < 20; i++)
  {
    if ((mag_now[i] > mag_now[max_idx]) && (i > max_idx))
    {
      max_idx = i;
    }
  }
  return max_idx;
}

void check_ultrasonic()
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
}
