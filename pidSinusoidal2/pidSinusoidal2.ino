#include<Servo.h>
#include<SoftwareSerial.h>
#include<stdarg.h>
#include<stdio.h>

const float pi = 3.14159265;
//int startPos = 90;
int startPos = 1500;
const int numServos = 2;
//int amp = 30;
int amp = 111;
const int testTime = 30000; // 10 seconds
const int minPeriod = 1000;
const int maxPeriod = 4000;
const int timestep = minPeriod/25;
const int pidTimestep = timestep;
const int calibrateTimestep = minPeriod/4;
const int N = testTime/timestep;
float kp = 0.5;
float ki = 0.0001;
float kd = 10.0;
//float tu = pidTimestep;
//float ti = tu*0.5;
//float td = tu*0.1;
int t=0;

int potentBounds[3][numServos];
Servo servo[numServos];
int positions[numServos][N];

void setup() 
{
  Serial.begin(9600);
  randomSeed(analogRead(40));
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i=0;i<numServos;i++)
  {
    servo[i].writeMicroseconds(startPos);
    servo[i].attach(i+2);
    calibrate(servo[i],3,i);
  };
  delay(500);
}

void loop() 
{
  testCycle();
  //while (1){}; 
  //Serial.print(i);
  //Serial.print(", ");
  
  for (int i=0;i<N;i++)
  {
    int pos[numServos];
    for (int ser=0;ser<numServos;ser++)
    {
      pos[ser] = positions[ser][i];
    };
    pidWrite(pos);
    //while(1){}; 
  };
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}

void pidWrite(int posArray[])
{
  float error[numServos] = {100}; 
  float error_prior[numServos] = {0};
  float integral[numServos] = {0};
  float derivative[numServos] = {0};
  int output[numServos];
  for (int i=0;i<numServos;i++){output[i]=posArray[i];};
  int writePos[numServos] = {0};
  //for (int i=0;i<numServos;i++)
  //{
  //  writePos[i] = map(posArray[i],potentBounds[0][i],potentBounds[2][i],startPos-amp,startPos+amp);
  //  servo[ser].writeMicroseconds(writePos[ser]);
  //};
  //delay(pidTimestep);
  //for (int i=0;i<numServos;i++)
  //{
  //  error[ser] = float(posArray[ser]) - float(analogRead(ser));
  //};

  while (findMax(error)>10)
  {
    int potentRead[numServos];
    for (int ser=0;ser<numServos;ser++)
    {
      error[ser] = float(posArray[ser]) - float(analogRead(ser));
      potentRead[ser] = analogRead(ser);
      //error[ser] = float(analogRead(ser)) - float(posArray[ser]);
      integral[ser] += float(error[ser]*pidTimestep);
      //integral[ser] = 0;
      derivative[ser] = (error[ser] - error_prior[ser])/pidTimestep;
      output[ser] += int(kp*error[ser])+int(ki*integral[ser])+int(kd*derivative[ser]);//+posArray[ser];
      //posArray[ser] += output[ser];
      //output[ser] = int(kp*(error[ser]+(1/ti)*integral[ser]+td*derivative[ser]))+posArray[ser];
      error_prior[ser] = error[ser];
      writePos[ser] = map(output[ser],potentBounds[0][ser],potentBounds[2][ser],startPos-amp,startPos+amp);
      servo[ser].writeMicroseconds(writePos[ser]);
    };
    //serialPrint(5,t,posArray[0],output[0],posArray[1],output[1]);
    Serial.print(t);
    Serial.print(", ");
    Serial.print(posArray[0]);
    Serial.print(", ");
    //Serial.print(output[0]);
    Serial.print(potentRead[0]);
    Serial.print(", ");
    Serial.print(error[0]);
    Serial.print(", ");
    Serial.print(posArray[1]);
    Serial.print(", ");
    //Serial.print(output[1]);
    Serial.print(potentRead[1]);
    Serial.print(", ");
    Serial.print(error[1]);
    Serial.print(", ");
    Serial.print(integral[0]);
    Serial.print(", ");
    Serial.print(integral[1]);
    Serial.print(", ");
    Serial.print(derivative[0]);
    Serial.print(", ");
    Serial.print(derivative[1]);
    Serial.print("\n");
    t+=1;
    delay(pidTimestep);
    //delay(timestep*100);
  };
}

void serialPrint(int num, ...)
{
  va_list arguments;
  va_start(arguments,num);
  for (int x;x<num;x++)
  {
    int to_print;
    if (x<num-1)
    { 
      to_print = va_arg(arguments,int);
      Serial.print(to_print);
      Serial.print(", ");
    }
    else
    {
      to_print = va_arg(arguments,int);
      Serial.print(to_print);
      Serial.print("\n");
    };
  };
  va_end(arguments);
}

int findMax(float array[])
{ 
  int theMax;
  for (int i;i<numServos;i++)
  {
    //if (array[i]>theMax)
    //{
    //  theMax = array[i];
    //};
    theMax = max(abs(array[i]),theMax);
  };
  return int(theMax);
}

void calibrate(Servo ser,int cycles,int i) 
{
  int top = 0;
  int bottom = 0;
  //int mean = 0;
  for (int j;j<cycles;j++)
  {
    ser.writeMicroseconds(startPos+amp);
    delay(calibrateTimestep);
    top += analogRead(i);
    ser.writeMicroseconds(startPos-amp);
    delay(calibrateTimestep);
    bottom += analogRead(i);
  }
  potentBounds[0][i] = bottom/cycles;
  potentBounds[2][i] = top/cycles;
  potentBounds[1][i] = (potentBounds[0][i]+potentBounds[2][i])/2;
  //for (int hi;hi<3;hi++)
  //{
  //  Serial.print(potentBounds[hi][i]);
  //  Serial.print("\n");
  //}
};

void testCycle()
{
  for (int i=0;i<numServos;i++)
  {
    int count = 0;
    int phase = pow(-1,random(0,2));
    while (positions[i][N-1]==0)
    {
      float period = random(minPeriod,maxPeriod);
      int n = period/timestep;
      float num = n;
      for (int j=0;j<n;j++)
      {
        float J = j;
        //int val = phase*amp*sin(pi*2*J/num)+startPos;
        int val = phase*(potentBounds[2][i]-potentBounds[1][i])*sin(pi*2*J/num)+potentBounds[1][i];
        positions[i][count]=val;
        count += 1;
        if (count>=N)
        {
          j = n;
        };
      };
    };
  };
}
