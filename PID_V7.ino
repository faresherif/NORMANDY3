#include<Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#define FORWARD_RM    11          // RIGHT MOTORS FORWARD MOVEMENT
#define FORWARD_LM     6         // LEFT MOTORS FORWARD MOVEMENT
#define BACKWARD_RM   10        // RIGHT MOTORS BACKWARD MOVEMENT
#define BACKWARD_LM    9       // LEFT MOTORS BACKWARD MOVEMENT
#define BUTTON         4      // ON/OFF BUTTON
#define BUZZER        12     // BACKGROUND MUSIC
#define SERVO          5    //SERVO ARM OBSTECALE REMOVE
#define USS_TRIG      8   //ULTRASONIC SENSOR SIGNAL TRANSFER PIN
#define USS_ECHO       13  //ULTRASONIC SENSOR SIGNAL RECIVE PIN
int IR[5] = { A0, A1, A2, A3, 7 };  
int IR_READ[5],speed,currentError,prevError,Flag=1,mspeed=150,Mode,setdistance=10,POS=0; 
String serial_input= "";
float kp,kd,ki,distance; 
Servo SERVO_1;
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
byte HEART[8] ={
    0b00001010,
    0b00010101,
    0b00010001,
    0b00001010,
    0b00000100,
    0b00000000,
    0b00000000,
    0b00000000}; //heart shape character

  
void getSerial()
{
    while (Serial.available())
 {
  char x=Serial.read();
  serial_input +=x;
  delay(2);
 }
 if (serial_input.length()>0)
 {
  movement();
  getpidvalue();
  serial_input= "";
 }
}
void movement()
{
  switch(serial_input[0]){
  case'a':
   Mode=1;
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Line Follower");
   break;
  case'm':
   Mode=0;
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Manual");
   break;
  case'q':
   Flag=0;
   break;
  case'f':
   Forward();
   break;
  case'l':
   Left();
   break;
  case'r':
   Right();
   break;
  case'b':
   Backward();
   break;
  case's':
   STOP();
   break;
  case'v':
   STOP();
   Flag=1;
   break;
  case'o':
   serial_input.remove(0,1);
   mspeed=serial_input.toInt();
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("speed=");
   lcd.print(mspeed);
   break;
  case'h':
   digitalWrite(BUZZER,HIGH);
   break;
  case'j':
   digitalWrite(BUZZER,LOW);
   break;
  }

  }
void getpidvalue()
{
 if(serial_input.substring(0,1)=="k")
 {
  if(serial_input.substring(0,2)=="kp")
  {serial_input.remove(0,2);
  kp=serial_input.toFloat();
    }
  if(serial_input.substring(0,2)=="ki")
  {serial_input.remove(0,2);
  ki=serial_input.toFloat();
    }
  if(serial_input.substring(0,2)=="kd")
  {serial_input.remove(0,2);
  kd=serial_input.toFloat();
    }
  if(serial_input.substring(0,2)=="ks")
  {serial_input.remove(0,2);
  speed=serial_input.toInt();
    }
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("kp=");
   lcd.print(kp,4);
   lcd.setCursor(0,1);
   lcd.print("ki=");
   lcd.print(ki,4);
   delay(1500);
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("kd=");
   lcd.print(kd);
   lcd.setCursor(0,1);
   lcd.print("speed=");
   lcd.print(speed);
 }
}

void Forward()
{ 
  analogWrite(FORWARD_RM, mspeed);
  analogWrite(FORWARD_LM, mspeed);
}

void Backward()
{ 
  analogWrite(BACKWARD_RM, mspeed);
  analogWrite(BACKWARD_LM, mspeed);
}

void Right()
{ 
  analogWrite(BACKWARD_RM, mspeed);
  analogWrite(FORWARD_LM, mspeed);
}

void Left()
{ 
  analogWrite(FORWARD_RM, mspeed);
  analogWrite(BACKWARD_LM, mspeed);
}

void STOP() 
{
  analogWrite(FORWARD_RM, LOW);
  analogWrite(FORWARD_LM, LOW);
  analogWrite(BACKWARD_RM, LOW);
  analogWrite(BACKWARD_LM, LOW);
}
//OBSTACLE DISTANCE DETECTION
void USS() 
{
  float DURATION, DISTANCE;
  digitalWrite(USS_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(USS_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(USS_TRIG, LOW);
  DURATION = pulseIn(USS_ECHO, HIGH,5000);
  if (DURATION>0)
  distance = (DURATION * 0.0343) / 2;
  else
  distance = 60;
  //delay(100);
}
//OBSTACLE REMOVAL
int SERVO_M() 
{
  for (POS = 0; POS <= 180; POS += 1) {
    SERVO_1.write(POS);
    delayMicroseconds(1000);
  }
  for (POS = 180; POS >= 0; POS -= 1) {
    SERVO_1.write(POS);
    delayMicroseconds(1000);
  }
}
void FollowLine()
{int rspeed,lspeed,I,D,PIDOut;
   readIR();
   error();
 if(currentError==9)
 {
   STOP();
   Flag=1;
 }
 else{

   I=currentError+prevError;
   D=currentError-prevError;
   prevError=currentError;
   PIDOut=kp*currentError+ki*I+kd*D; 

   rspeed=speed-PIDOut;
   lspeed=speed+PIDOut;
 if(rspeed>=0)
 {  
   digitalWrite(BACKWARD_RM,LOW);
   analogWrite(FORWARD_RM,constrain(rspeed,0,120));
  }
 else
 {
   digitalWrite(FORWARD_RM,LOW);
   analogWrite(BACKWARD_RM,constrain(-rspeed,0,120));
  }
 if(lspeed>=0)
 {  
   digitalWrite(BACKWARD_LM,LOW);
   analogWrite(FORWARD_LM,constrain(lspeed,0,120));
  }
 else
 {
   digitalWrite(FORWARD_LM,LOW);
   analogWrite(BACKWARD_LM,constrain(-lspeed,0,120));
  }
 }
}

void readIR()
{
  for (int i = 0; i < 5; i++) 
        IR_READ[i] = digitalRead(IR[i]);  //STORE IR READINGS IN ARRAY     
}

void error()
{
  switch(IR_READ[0]*16 + IR_READ[1]*8 + IR_READ[2]*4 + IR_READ[3]*2 + IR_READ[4]*1){
  case 24:
  currentError=5;
  break;
  case 30:
  currentError=4;
  break;
  case 28:
  currentError=3;
  break;
  case 29:
  currentError=2;
  break;
  case 25:
  currentError=1;
  break;
  case 27:
  currentError=0;
  break;
  case 19:
  currentError=-1;
  break;
  case 23:
  currentError=-2;
  break;
  case 7:
  currentError=-3;
  break;
  case 15:
  currentError=-4;
  break;
  case 3:
  currentError=-5;
  break;
}
  
}

void setup()
{ Serial.begin(9600);
  pinMode(FORWARD_RM, OUTPUT);
  pinMode(FORWARD_LM, OUTPUT);
  pinMode(BACKWARD_RM, OUTPUT);
  pinMode(BACKWARD_LM, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(USS_TRIG, OUTPUT);
  pinMode(USS_ECHO, INPUT);
  lcd.init();                      // initialize the lcd 
  lcd.backlight(); 
//LCD CODE START
  lcd.begin(16, 2);
  lcd.createChar(0,HEART);
  lcd.setCursor(3, 0);
  lcd.print("FARES SHERIF");
  lcd.setCursor(4,1);
  lcd.print("20106051");
  delay(500);
  for (int i = 0; i < 16; i++) {
    lcd.scrollDisplayLeft();
    delay(300);
  }
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("SEIF ELDIN");
  lcd.setCursor(4,1);
  lcd.print("20106729");
  delay(500);
  for (int i = 0; i < 16; i++) {
    lcd.scrollDisplayLeft();
    delay(300);
  }
  lcd.clear();
    lcd.setCursor(3, 0);
  lcd.print("YOUSSEF EHAB");
  lcd.setCursor(4,1);
  lcd.print("20104535");
  delay(500);
  for (int i = 0; i < 16; i++) {
    lcd.scrollDisplayLeft();
    delay(300);
  }
  lcd.clear();
  delay(500);
  lcd.setCursor(2, 0);
  lcd.print("ENG.MOHAMED G.");
  lcd.setCursor(1,1);
  lcd.print("DR.MOSTAFA FOUZ");
  delay(2000);
  for(int i=0;i<16;i++)
  {
    delay(500);
     lcd.setCursor(i,0);
  lcd.write((byte)0);
     lcd.setCursor(i,1);
  lcd.write((byte)0);
  }
  delay(1000);
  lcd.clear();
 //LCD CODE END
}

void loop()
{
  getSerial();
  SERVO_1.write(POS);
  USS();
  if(digitalRead(BUTTON)==LOW)
    Flag=0;
  if(Flag==0)
  {;
   FollowLine(); 
   while(distance<setdistance && distance>0)
   {USS();
   STOP();
   SERVO_M();
   }
  }
}
