#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

int anglepot0= 34;
int anglepot1=35;

int a1=19;
int b1=18;
int a2=5;
int b2=4;
int motor1Pin1 = 14; 
int motor1Pin2 = 12; 
int motor2Pin1=33;
int motor2Pin2=25;
int pos[2];
int setpos[2];
float kp=15,kd=0.010,ki=0;
long prevt[2];
float eprev[2],einteg[2],deltaT[2],dervative[2],pidOutput[2];
float rpm[2],prevTime[2];
const int pwmChannel_11 = 0;
const int pwmChannel_12 = 1;
const int pwmChannel_21 = 2;
const int pwmChannel_22 = 3;
float pre;


void readencoder1()
{
  int f= digitalRead(b1);
  if(f>0)
  pos[0]++;
  else
  pos[0]--;
}
void readencoder2()
{
  int f= digitalRead(b2);
  if(f>0)
  pos[1]++;
  else
  pos[1]--;
}

float pid(int i)
{
 float calc;
 long currt = micros();
 deltaT[i]= ((float)(currt-prevt[i]))/1.0e6;
 prevt[i]=currt;
 int e=setpos[i]-rpm[i];
 dervative[i]= (e-eprev[i])/(deltaT[i]);
 einteg[i] = einteg[i]+ e*deltaT[i];
 eprev[i]=e;
 return calc= kp*e+kd*dervative[i]+ki*einteg[i];
}

void motorwrite(int channel1,int channel2,int val)
{
 int dutyc = constrain(fabs(val),0,255);
 if(val<0)
 {analogWrite(channel2,0);
 analogWrite(channel1,dutyc);}
 else
 {analogWrite(channel1,0);
 analogWrite(channel2,dutyc);
 }
}
void ENCODER_RPM(int i)
{
  // Calculate RPM every second
  float currentTime = millis();
  if (currentTime - prevTime[i] >= 1000)
  {
    // Calculate RPM
    float rpm[i] = (float)pos[i] * (60000.0 / 493.9) / (currentTime - prevTime[i]);
    // Reset the encoder count and update the previous time
    pos[i] = 0;
    prevTime[i] = currentTime;
  }
}

void setup()
{
 pinMode(anglepot0,INPUT);
 pinMode(anglepot1,INPUT); 
 pinMode(b1,INPUT);
 pinMode(b2,INPUT);
 pinMode(motor1Pin1,OUTPUT);
 pinMode(motor1Pin2,OUTPUT);
 pinMode(motor2Pin1,OUTPUT);
 pinMode(motor2Pin2,OUTPUT);


 attachInterrupt(digitalPinToInterrupt(a1), readencoder1, RISING);
 attachInterrupt(digitalPinToInterrupt(a2), readencoder2, RISING);
 lcd.init();
 lcd.backlight();
 Serial.begin(115200);
}

void loop() {

setpos[0]=map(analogRead(anglepot0),0,4096,-187,187);
setpos[1]=map(analogRead(anglepot1),0,4096,-187,187);
ENCODER_RPM(0);
ENCODER_RPM(1);
pidOutput[0]= pid(0);
pidOutput[1]= pid(1);
motorwrite(pwmChannel_11,pwmChannel_12,pidOutput[0]);
motorwrite(pwmChannel_21,pwmChannel_22,pidOutput[1]);


long t=millis();
if((t-pre)>400){

lcd.setCursor(0, 0);
lcd.print("M1:");
lcd.print(map(pos[0],-187,187,-135,135));
lcd.print("  ");
lcd.print("M2:");
lcd.print(map(pos[1],-187,187,-135,135));
lcd.print("  ");
lcd.setCursor(0, 1);
lcd.print("M3:");
lcd.print(map(pos[2],-187,187,-135,135));
lcd.print("  ");
pre=t;
}


}