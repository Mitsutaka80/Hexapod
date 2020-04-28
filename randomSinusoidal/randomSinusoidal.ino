#include<Servo.h>
#include<SoftwareSerial.h>
#include<Time.h>

const float pi = 3.14159265;
float startPos = 1500;
const int numServos = 18;
int amp = 140;
const int testTime = 6000; // 6 seconds
const int minPeriod = 1000;
const int maxPeriod = 4000;
const int timestep = minPeriod/25;
const int N = testTime/timestep;
int trials = 5;
//int factor = 1;

Servo servo[numServos];

int positions[numServos][N];
int lastWrite[numServos][3];
//int sensorPin[numServos];

void setup()
{
  for (int i=0;i<numServos;i++)
  {
    servo[i].write(startPos);
    servo[i].attach(i+2);
    //servo[i].write(startPos);
    //sensorPin[i] = i;
  };
  //while(1){};
  Serial.begin(57600);
  randomSeed(analogRead(40));
  delay(5000);
  //int t=0;
  //time_t start = time(0);
  unsigned long start = millis();
  for (int tr=0;tr<trials;tr++)
  {
    fillCycle();
    
    for (int i=0;i<N;i++)
    {
      //Serial.print(t);
      //Serial.print(difftime(time(0),start));
      int toPrint[numServos];
      for (int ser=0;ser<numServos;ser++)
      {
        servo[ser].write(positions[ser][i]);
        toPrint[ser] = positions[ser][i];
        //Serial.print(positions[ser][i]);
        //Serial.print(", ");
      };
      //t+=1;
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
    //for (int i=0;i<numServos;i++)
    //{
    //  servo[i].write(startPos);
    //};
    //delay(2000);
  };
}

void loop()
{ 
  //while(1){};
}

void fillCycle()
{
  for (int ser=0;ser<numServos;ser++)
  {
    int count = 0;
    int phase = pow(-1,random(0,2));
    while (positions[ser][N-1]==0)
    {
      float period;
      float shift = 0;
      float num;
      int toPhase;
      if (lastWrite[ser][0]==0)
      {
        period = float(random(minPeriod,maxPeriod));
        num = period/timestep;
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
        //phase = 1;
        //if (period<0){period*=-1;};
        shift = lhs1 - (2*pi*x1)/period;
        //Serial.print(ser);
        //Serial.print(", ");
        //Serial.print(y0);
        //Serial.print(", ");
        //Serial.print(y1);
        //Serial.print(", ");
        //Serial.print(lhs0);
        //Serial.print(", ");
        //Serial.print(lhs1);
        //Serial.print(", ");
        //Serial.print(shift);
        //Serial.print("\n");
        if (period>0)
        {
          num = ((pi-shift)*period)/(2*pi*timestep);
        }
        else
        {
          num = ((pi+shift)*period)/(2*pi*timestep);
        };
        phase = 1;
        lastWrite[ser][0]=0;
        lastWrite[ser][1]=0;
        lastWrite[ser][2]=0;
        if (num<0)
        {toPhase = 1;}
        else {toPhase = -1;};
      };
      int n = abs(num); 
      //Serial.print(ser);
      //Serial.print(", ");
      //Serial.print(period);
      //Serial.print(", ");
      //Serial.print(shift);
      //Serial.print(", ");
      //Serial.print(num);
      //Serial.print(", ");
      //Serial.print(n);
      //Serial.print("\n");
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
  //startPos -= 1;
  for (int i=0;i<numServos;i++)
  {
    //servo[i].write(startPos);
    //delay(timestep*8);
    lastWrite[i][0]=positions[i][N-4];
    lastWrite[i][1]=positions[i][N-2];
    lastWrite[i][2]=positions[i][N-1];
    //Serial.print(lastWrite[i][0]);
    //Serial.print(", ");
    //Serial.print(lastWrite[i][1]);
    //Serial.print("\n");
    for (int j=0;j<N;j++)
    {
      positions[i][j]=0;
    };
    //positions[i][-2]=0;
    //positions[i][-1]=0;
  };
  //delay(1000);
}

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
        int val = phase*amp*sin(pi*2*J/num)+startPos;
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



