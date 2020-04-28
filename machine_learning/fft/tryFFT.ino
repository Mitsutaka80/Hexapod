#include<Servo.h>
#include<SoftwareSerial.h>
#include<Time.h>
#include "period_shift.h"

const float pi = 3.14159265;
float startPos = 1500;
int amp = 140;
const int testTime = 6000; // 6 seconds
const int minPeriod = 1000;
const int maxPeriod = 4000;
int minPeriodArr[numServos];
int maxPeriodArr[numServos];
const int timestep = minPeriod/25;
const int N = testTime/timestep;
int trials = 5;
int predicted[numServos];
float mutate = 0.15;

Servo servo[numServos];

int positions[numServos][N];
int lastWrite[numServos][3];

void setup()
{
  for (int i=0;i<numServos;i++)
  {
    //shifts[i]*=timestep;
    servo[i].write(amp*sin(shifts[i]*timestep)+startPos);
    servo[i].attach(i+2);
    if (periods[i] != 0)
    {
      //lastWrite[i][0] = amp*sin(2*pi*0*timestep/periods[i]+shifts[i]);
      //lastWrite[i][1] = amp*sin(2*pi*2*timestep/periods[i]+shifts[i]);
      //lastWrite[i][2] = amp*sin(2*pi*3*timestep/periods[i]+shifts[i]);
      minPeriodArr[i] = (1.0-mutate)*float(periods[i])*float(timestep);
      maxPeriodArr[i] = (1.0+mutate)*float(periods[i])*float(timestep);
    }
    else
    {
      minPeriodArr[i] = minPeriod;
      maxPeriodArr[i] = maxPeriod; 
    };
  };
  Serial.begin(57600);
  delay(5000);
  unsigned long start = millis();
  for (int tr=0;tr<trials;tr++)
  {
    fillPrediction();
    for (int i=0;i<N;i++)
    {
      int toPrint[numServos];
      for (int ser=0;ser<numServos;ser++)
      {
        servo[ser].write(positions[ser][i]);
        toPrint[ser] = positions[ser][i];
      };
      delay(timestep);
      unsigned long now = millis()-start;
      Serial.print(now);
      for (int ser=0;ser<numServos;ser++)
      {
        Serial.print(", ");
        Serial.print(toPrint[ser]);
      };
      Serial.print("\n");
    };
    resetServos();
  };
}

void loop()
{ 
}

void fillPrediction()
{
  for (int ser=0;ser<numServos;ser++)
  {
    int count = 0;
    int phase = 1;//= pow(-1,random(0,2));
    if (periods[ser]<0){phase=-1;periods[ser]=abs(periods[ser]);};
    while (positions[ser][N-1]==0)
    {
      float period;
      float shift = 0;
      float num;
      int toPhase;
      if (lastWrite[ser][0]==0)
      {
        if (predicted[ser]==0)
        {
          period = float(periods[ser]*timestep);
          shift = shifts[ser]*timestep;
          predicted[ser]=1;
          if (period>0)
          {
            num = ((pi-shift)*period)/(2*pi*timestep);
          }
          else
          {
            num = ((pi+shift)*period)/(2*pi*timestep);
          };
          //if (period<0)
          //{
          //  //period = abs(period);
          //  toPhase = 1;
          //};
        }
        else
        {
          //minPeriod
          period = float(random(minPeriodArr[ser],maxPeriodArr[ser]));
          num = period/timestep;
          //period = float(random(low,high));
          //period = 4000;
        };
        toPhase = phase;
      }
      else
      {
        float x0 = 0;
        float x1 = 1;
        float y0;
        float y1;
        if (lastWrite[ser][1] != lastWrite[ser][2])
        {
          y0 = float(lastWrite[ser][1]);
          y1 = float(lastWrite[ser][2]);
        }
        else
        {
          y0 = float(lastWrite[ser][0]);
          y1 = float(lastWrite[ser][2]);
          x1 = 3;
        };
        float lhs0 = asin((y0-float(startPos))/float(amp));
        float lhs1 = asin((y1-float(startPos))/float(amp));
        period = (2*pi*timestep*(x1-x0))/(lhs1-lhs0);
        shift = lhs1 - (2*pi*x1)/period;
        if (period>0)
        {
          num = ((pi-shift)*period)/(2*pi*timestep);
        }
        else
        {
          num = ((pi+shift)*period)/(2*pi*timestep);
        };
        //Serial.print(ser);
        //Serial.print(", ");
        //Serial.print(num);
        //Serial.print("\n");
        phase = 1;
        lastWrite[ser][0]=0;
        lastWrite[ser][1]=0;
        lastWrite[ser][2]=0;
        if (num<0)
        {toPhase = 1;}
        else {toPhase = -1;};
      };
      int n = abs(num); 
      for (int j=0;j<n;j++)
      {
        float J = j;
        int val = phase*amp*sin(pi*2*J*timestep/period+shift)+startPos;
        positions[ser][count]=val;
        count += 1;
        if (count>=N)
        {
          j = n;
        };
      };
      phase = toPhase;
    };
  };
}

void resetServos()
{
  for (int i=0;i<numServos;i++)
  {
    lastWrite[i][0]=positions[i][N-4];
    lastWrite[i][1]=positions[i][N-2];
    lastWrite[i][2]=positions[i][N-1];
    for (int j=0;j<N;j++)
    {
      positions[i][j]=0;
    };
  };
}
