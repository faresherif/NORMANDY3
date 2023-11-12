#include <pitches.h>                           //PIANO NOTES FREQUENCIES IN Hz
#include <Servo.h>                            //SERVO LIBRARY
#include"CAR_INTERFACE.h"                    //LINE FOLLOWER LIBRARY
#define IR[5] = { A0, A1, A2, A3, A4 };     //IR SENSOR SIGNAL PINS 
int IR_READ[5];                            //IR SENSOR READINGS SAVED IN ARRAY
int PWM_LOW = 20;                         //MOTOR SPEED CONTROL "SLOW"
int PWM_MED = 120;                       //MOTOR SPEED CONTROL "MODERATE"
int PWM_HIGH = 140;                     //MOTOR SPEED CONTROL "FAST"
int OLD_STATE;                         //OLD STATE FOR BUTTON
int flag = 0;                         // "START/STOP" FLAG TO AVOID THE CODE AFTER COMPLETING THE MISSION
Servo SERVO_1;
int POS=0;
float USS_READ;
float dist=2;
float songSpeed = 0.5;
  //SONG NOTES & DURTION OF EACH NOTE
int MELODY[] = {NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4,NOTE_D4,NOTE_F4, NOTE_AS3, NOTE_DS4, NOTE_D4, NOTE_F4, NOTE_AS3,NOTE_DS4, NOTE_D4, NOTE_C4,NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_D4, NOTE_F4, NOTE_AS3, NOTE_DS4, NOTE_D4, NOTE_F4, NOTE_AS3, NOTE_DS4, NOTE_D4, NOTE_C4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_G4,  NOTE_C4, NOTE_DS4, NOTE_F4,NOTE_D4,NOTE_F4, NOTE_AS3,NOTE_D4, NOTE_DS4, NOTE_D4, NOTE_AS3,NOTE_C4,NOTE_C5,NOTE_AS4,NOTE_C4,NOTE_G4,NOTE_DS4, NOTE_DS4, NOTE_F4, NOTE_G4,  NOTE_C5,NOTE_AS4,NOTE_C4, NOTE_G4,  NOTE_DS4, NOTE_DS4, NOTE_D4,NOTE_C5, NOTE_G4, NOTE_GS4, NOTE_AS4, NOTE_C5, NOTE_G4, NOTE_GS4, NOTE_AS4, NOTE_C5, NOTE_G4, NOTE_GS4, NOTE_AS4, NOTE_C5, NOTE_G4, NOTE_GS4, NOTE_AS4,REST, NOTE_GS5, NOTE_AS5, NOTE_C6, NOTE_G5, NOTE_GS5, NOTE_AS5, NOTE_C6, NOTE_G5, NOTE_GS5, NOTE_AS5, NOTE_C6, NOTE_G5, NOTE_GS5, NOTE_AS5};
int DURATIONS[] = {  8, 8, 16, 16, 8, 8, 16, 16,  8, 8, 16, 16, 8, 8, 16, 16,  8, 8, 16, 16, 8, 8, 16, 16,  8, 8, 16, 16, 8, 8, 16, 16, 4, 4,16, 16, 4, 4, 16, 16,  1,  4, 4,  16, 16, 4, 4,  16, 16, 1,    4, 4,    16, 16, 4, 4, 16, 16,  1,  4, 4,  16, 16, 4, 4,  16, 16, 1,  4, 4, 16, 16, 4, 4, 16, 16,    2, 4, 4, 8, 8, 8, 8,  1, 2, 2, 2, 2, 2,4, 4, 1,   2, 2, 2, 2, 2, 4, 4, 8, 8, 16, 16, 8, 8, 16, 16, 8, 8, 16, 16, 8, 8, 16, 16,4, 16, 16, 8, 8, 16, 16, 8, 16, 16, 16, 8, 8, 16, 16};
void setup() {
  pinMode(FORWARD_RM, OUTPUT);
  pinMode(FORWARD_LM, OUTPUT);
  pinMode(BACKWARD_RM, OUTPUT);
  pinMode(BACKWARD_LM, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  SERVO_1.attach(5);
  pinMode(LED_BRAKE, OUTPUT);
  pinMode(USS_TRIG, OUTPUT);
  pinMode(USS_ECHO, INPUT);
  Serial.begin(9600);
}
void MUSIC() {
    int SIZE = sizeof(DURATIONS) / sizeof(int);

  for (int NOTE = 0; NOTE < SIZE; NOTE++) {
    int duration = 1000 / DURATIONS[NOTE];
    tone(BUZZER, MELODY[NOTE], duration); //PLAY SONG

    int PAUSE_BETWEEN_NOTES = duration * 1.30; //TIME TO  SEPARATE BETWEEN NOTES= NOTE DURATION +"30% DURATION" "tried it and it works well"
    delay(PAUSE_BETWEEN_NOTES);
    
    noTone(BUZZER);//STOP SONG
  }
}
void FORWARD() {
  analogWrite(FORWARD_RM, 100);
  analogWrite(FORWARD_LM,100);
}
void STOP() {
  analogWrite(FORWARD_RM, LOW);
  analogWrite(FORWARD_LM, LOW);
  digitalWrite(LED_BRAKE, HIGH);
}
void TURN_RIGHT() {
  analogWrite(FORWARD_RM, PWM_LOW);
  analogWrite(FORWARD_LM, PWM_MED);
}
void TURN_LEFT() {
  analogWrite(FORWARD_RM, PWM_MED);
  analogWrite(FORWARD_LM, PWM_LOW);
}
void TURN_RIGHT_FAST() {
  analogWrite(FORWARD_RM, PWM_LOW);
  analogWrite(FORWARD_LM, PWM_HIGH);
}
void TURN_LEFT_FAST() {
  analogWrite(FORWARD_RM, PWM_HIGH);
  analogWrite(FORWARD_LM, PWM_LOW);
}
//FUNCTION TO GET THE IR READINGS AND SEND PWM SIGNALS TO THE MOTORS AND CONTROL IT'S MOVEMENT DIRECTION
void CAR_STATE(int IR_READ[]) {   
  int USS_READ = USS();
    while (USS_READ <= dist) {  // OBSTACLE CHECK
      STOP();
      SERVO_M();
      USS_READ = USS();
      if (USS_READ > dist) {
        dist = 10;
      }
      else
      {
        dist--;
        FORWARD();
      }
    } 

   if (IR_READ[0] == 1 && IR_READ[1] == 1 && IR_READ[1] == 0 && IR_READ[3] == 1 && IR_READ[2] == 1 && USS_READ>2) {
    FORWARD();
  } else if (IR_READ[0] == 1 && IR_READ[1] == 1 && IR_READ[1] == 1 && IR_READ[3] == 0 && IR_READ[2] == 1 && USS_READ>2 ) {
    TURN_RIGHT();
  } else if (IR_READ[0] == 1 && IR_READ[1] == 1 && IR_READ[1] == 1 && IR_READ[3] == 1 && IR_READ[2] == 0 && USS_READ>2 ) {
    TURN_RIGHT_FAST();
  } else if (IR_READ[0] == 1 && IR_READ[1] == 0 && IR_READ[1] == 1 && IR_READ[3] == 1 && IR_READ[2] == 1 && USS_READ>2 ) {
    TURN_LEFT();
  } else if (IR_READ[0] == 0 && IR_READ[1] == 1 && IR_READ[1] == 1 && IR_READ[3] == 1 && IR_READ[2] == 1  && USS_READ>2) {
    TURN_LEFT_FAST();
  } else if (IR_READ[0] == 0 && IR_READ[1] == 0 && IR_READ[1] == 0 && IR_READ[3] == 0 && IR_READ[2] == 0) {
    STOP();
    flag = 1;
  }
}
// ON/OFF FUNCTION
int BUTTON_STATE() {
  int NEW_STATE = digitalRead(BUTTON);
  if (NEW_STATE != OLD_STATE) {
    OLD_STATE = NEW_STATE;
    flag = 0;
    return NEW_STATE;
  } else {
    analogWrite(FORWARD_RM, LOW);  //STOP THE CAR
    analogWrite(FORWARD_LM, LOW);
    flag = 1;  //FLAG
  }
}
//OBSTACLE DISTANCE DETECTION
int USS() {
  float DURATION, DISTANCE;
  digitalWrite(USS_TRIG, LOW);
  //delayMicroseconds(1);
  digitalWrite(USS_TRIG, HIGH);
   //delayMicroseconds(10);
  digitalWrite(USS_TRIG, LOW);
  DURATION = pulseIn(USS_ECHO, HIGH);
  DISTANCE = (DURATION * 0.0343) / 1;
  //delay(100);
  return DISTANCE;
}
//OBSTACLE REMOVAL
int SERVO_M() {
  for (POS = 0; POS <= 180; POS += 1) {
    SERVO_1.write(POS);
    delayMicroseconds(1000);
  }
  for (POS = 180; POS >= 0; POS -= 1) {
    SERVO_1.write(POS);
    delayMicroseconds(1000);
  }
}
void loop() {
  SERVO_1.write(POS);
  int STATE = BUTTON_STATE();  //IF BUTTON STATE "ON"
  if (STATE == LOW) {
    
    USS_READ=USS();
    while (flag == 0) 
    {
      MUSIC();
      for (int i = 0; i < 5; i++) {
        IR_READ[i] = digitalRead(IR[i]);  //STORE IR READINGS IN ARRAY
      }
      CAR_STATE(IR_READ);    
  }
}
}
