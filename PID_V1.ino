//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#define FORWARD_RM    11          // RIGHT MOTORS FORWARD MOVEMENT
#define FORWARD_LM     6         // LEFT MOTORS FORWARD MOVEMENT
#define BACKWARD_RM   10        // RIGHT MOTORS BACKWARD MOVEMENT
#define BACKWARD_LM    9       // LEFT MOTORS BACKWARD MOVEMENT
#define BUTTON         4      // ON/OFF BUTTON
int IR[5] = { A0, A1, A2, A3, A4 };  
int IR_READ[5],speed,currentPos,setPos=12,currentError,prevError,I,D,Flag=1; 
String serial_input= "";
float kp,kd,ki,PIDOut; 
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void getSerial(){
   while (Serial.available())
 {
  char x=Serial.read();
  serial_input +=x;
  delay(2);
 }
 if (serial_input.length()>0)
 {
  getpidvalue();
 }
}
void getpidvalue()
{
  if(serial_input.substring(0,2)=="kp")
  {serial_input.remove(0,2);
  kp=serial_input.toFloat();
   serial_input= "";
    }
  if(serial_input.substring(0,2)=="ki")
  {serial_input.remove(0,2);
  ki=serial_input.toFloat();
   serial_input= "";
    }
  if(serial_input.substring(0,2)=="kd")
  {serial_input.remove(0,2);
  kd=serial_input.toFloat();
  serial_input= "";
    }
  if(serial_input.substring(0,2)=="sp")
  {serial_input.remove(0,2);
  speed=serial_input.toInt();
   serial_input= "";
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

void readIR()
{
  for (int i = 0; i < 5; i++) 
        IR_READ[i] = digitalRead(IR[i]);  //STORE IR READINGS IN ARRAY     
}

void STOP() {
  analogWrite(FORWARD_RM, LOW);
  analogWrite(FORWARD_LM, LOW);
}

void FollowLine(){
   readIR();
  currentPos=IR_READ[0]*1+IR_READ[1]*2+IR_READ[2]*3+IR_READ[3]*4+IR_READ[4]*5;
 if(currentPos==0)
 {
   STOP();
   Flag=1;
 }
 else if(currentPos==15)
 {
  }
 else
 {
   currentError= setPos-currentPos;
   I+=currentError;
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
  lcd.init();                      // initialize the lcd 
  lcd.backlight(); 
  lcd.print("kp=");
  delay(1000);
}


void loop()
{
  getSerial();
 
  if(digitalRead(BUTTON)==LOW)
    Flag=0;
  if(Flag==0){
  FollowLine();
  }
  I=0;
}
