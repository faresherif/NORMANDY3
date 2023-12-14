//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Wire.h> 
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#define FORWARD_RM    11          // RIGHT MOTORS FORWARD MOVEMENT
#define FORWARD_LM     6         // LEFT MOTORS FORWARD MOVEMENT
#define BACKWARD_RM   10        // RIGHT MOTORS BACKWARD MOVEMENT
#define BACKWARD_LM    9       // LEFT MOTORS BACKWARD MOVEMENT
#define BUTTON         4      // ON/OFF BUTTON
#define BUZZER        12     // BACKGROUND MUSIC
#define SERVO          5    //SERVO ARM OBSTECALE REMOVE
#define USS_TRIG       3   //ULTRASONIC SENSOR SIGNAL TRANSFER PIN
#define USS_ECHO       2  //ULTRASONIC SENSOR SIGNAL RECIVE PIN
#define LED_BRAKE      7 //RED LED
int IR[5] = { A0, A1, A2, A3, A4 };  
int IR_READ[5],speed,currentPos,setPos=12,currentError,prevError,I,D,Flag=1,mspeed=150,Mode,setdistance=10,POS=0; 
String serial_input= "";
float kp,kd,ki,PIDOut,distance,USS_READ; 
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
void getSerial(){
   while (Serial.available())
 {
  char x=Serial.read();
  serial_input +=x;
  delay(2);
 }
 if (serial_input.length()>0)
 {
  //movement();
  getpidvalue();
  serial_input= "";
 }
}
void movement()
{
  if(serial_input.substring(0,1)=="a")
  Mode=1;
  if(serial_input.substring(0,1)=="m")
  Mode=0;
  if(serial_input.substring(0,1)=="q")
  Flag=0;
  if(serial_input.substring(0,1)=="f")
  Forward();
  if(serial_input.substring(0,1)=="l")
  Left();
  if(serial_input.substring(0,1)=="r")
  Right();
  if(serial_input.substring(0,1)=="b")
  Backward();
  if(serial_input.substring(0,1)=="s")
  STOP();
  if(serial_input.substring(0,1)=="o")
  {
    serial_input.remove(0,1);
    mspeed=serial_input.toInt();
    }
  if(serial_input.substring(0,1)=="h")
  digitalWrite(BUZZER,HIGH);
  if(serial_input.substring(0,1)=="j")
  digitalWrite(BUZZER,LOW);
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

void readIR()
{
  for (int i = 0; i < 5; i++) 
        IR_READ[i] = digitalRead(IR[i]);  //STORE IR READINGS IN ARRAY     
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
}
//OBSTACLE DISTANCE DETECTION
int USS() 
{
  float DURATION, DISTANCE;
  digitalWrite(USS_TRIG, LOW);
  delayMicroseconds(1);
  digitalWrite(USS_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(USS_TRIG, LOW);
  DURATION = pulseIn(USS_ECHO, HIGH);
  DISTANCE = (DURATION * 0.0343) / 1;
  //delay(100);
  return DISTANCE;
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
{
   readIR();
  currentPos=IR_READ[0]*1+IR_READ[1]*2+IR_READ[2]*3+IR_READ[3]*4+IR_READ[4]*5;
 if(currentPos==0)
 {
   STOP();
   Flag=1;
 }
 else if(currentPos==15)
 {
  //asm(NOP);
  }
 else
 {
   currentError= setPos-currentPos;
   I=currentError+prevError;
   D=currentError-prevError;
   prevError=currentError;
   PIDOut=kp*currentError+ki*I+kd*D; 

   analogWrite(FORWARD_RM,constrain(speed-PIDOut,0,255));
   analogWrite(FORWARD_LM,constrain(speed+PIDOut,0,255));
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
  lcd.print("55555555");
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
  distance=USS();
  if(digitalRead(BUTTON)==LOW)
    Flag=0;
  if(Flag==0 && Mode==1)
  {
   FollowLine(); 
   while(distance<setdistance&&distance>0)
   {
   STOP();
   SERVO_M();
   }
  }
}
