#include <Servo.h>                 //SERVO LIBRARY
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
#define NOTE_C4 262     //PIANO NOTES FREQUENCIES IN Hz
#define NOTE_D4 294    
#define NOTE_E4 330   
#define NOTE_F4 349  
#define NOTE_G4 392 
#define NOTE_A4 440           
#define NOTE_B4 494       
#define NOTE_C5 523      
#define NOTE_D5 587      
#define NOTE_E5 659    
#define NOTE_F5 698    
#define NOTE_G5 784   
#define NOTE_A5 880  
#define NOTE_B5 988 
int IR[5] = { A0, A1, A2, A3, A4 };//IR SENSOR SIGNAL PINS 
int IR_READ[5];              //IR SENSOR READINGS SAVED IN ARRAY
int PWM_LOW = 20;           //MOTOR SPEED CONTROL "SLOW"
int PWM_MED = 120;         //MOTOR SPEED CONTROL "MODERATE"
int PWM_HIGH = 140;       //MOTOR SPEED CONTROL "FAST"
int OLD_STATE;           //OLD STATE FOR BUTTON
int flag = 0;           // "START/STOP" FLAG TO AVOID THE CODE AFTER COMPLETING THE MISSION
Servo SERVO_1;
int POS=0;
float USS_READ;
float dist=15;
 float SONG_SPEED = 1.0;
  //SONG NOTES
   int NOTES[] = {
    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0, NOTE_A4, NOTE_G4, NOTE_A4, 0, NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0, NOTE_A4, NOTE_G4, NOTE_A4, 0, NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0, NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, NOTE_D5, NOTE_E5, NOTE_A4, 0, NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0, NOTE_C5, NOTE_A4, NOTE_B4, 0
  }; 
 // DURATION FOR EACH NOTE IN MILLISECOND
 int duration[] = {
  125, 125, 250, 125, 125, 125, 125, 250, 125, 125, 125, 125, 250, 125, 125, 125, 125, 375, 125, 125, 125, 250, 125, 125, 125, 125, 250, 125, 125, 125, 125, 250, 125, 125, 125, 125, 375, 125, 125, 125, 250, 125, 125, 125, 125, 250, 125, 125, 125, 125, 250, 125, 125, 125, 125, 125, 250, 125, 125, 125, 250, 125, 125, 250, 125, 250, 125, 125, 125, 250, 125, 125, 125, 125, 375, 375
 };
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
 int TOTAL_NOTES = sizeof(NOTES) / sizeof(int);
  // Loop through each note
  for (int i = 0; i < TOTAL_NOTES; i++) {
    int CURRENT_TONE = NOTES[i];
    float WAIT = duration[i] / SONG_SPEED;
    // Play tone if CURRENT_TONE is not 0 frequency, otherwise pause (noTone)
    if (CURRENT_TONE != 0) {
      tone(BUZZER, NOTES[i], WAIT);  // tone(pin, frequency, duration)
    } else {
      noTone(BUZZER);
    }
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

   if (IR_READ[0] == 1 && IR_READ[1] == 1 && IR_READ[2] == 0 && IR_READ[3] == 1 && IR_READ[4] == 1 && USS_READ>15) {
    FORWARD();
  } else if (IR_READ[0] == 1 && IR_READ[1] == 1 && IR_READ[2] == 1 && IR_READ[3] == 0 && IR_READ[4] == 1 && USS_READ>15 ) {
    TURN_RIGHT();
  } else if (IR_READ[0] == 1 && IR_READ[1] == 1 && IR_READ[2] == 1 && IR_READ[3] == 1 && IR_READ[4] == 0 && USS_READ>15 ) {
    TURN_RIGHT_FAST();
  } else if (IR_READ[0] == 1 && IR_READ[1] == 0 && IR_READ[2] == 1 && IR_READ[3] == 1 && IR_READ[4] == 1 && USS_READ>15 ) {
    TURN_LEFT();
  } else if (IR_READ[0] == 0 && IR_READ[1] == 1 && IR_READ[2] == 1 && IR_READ[3] == 1 && IR_READ[4] == 1  && USS_READ>15) {
    TURN_LEFT_FAST();
  } else if (IR_READ[0] == 0 && IR_READ[1] == 0 && IR_READ[2] == 0 && IR_READ[3] == 0 && IR_READ[4] == 0) {
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
  //delayMicroseconds(2);
  digitalWrite(USS_TRIG, HIGH);
   //delayMicroseconds(10);
  digitalWrite(USS_TRIG, LOW);
  DURATION = pulseIn(USS_ECHO, HIGH);
  DISTANCE = (DURATION * 0.0343) / 2;
  //delay(100);
  return DISTANCE;
}
//OBSTACLE REMOVAL
int SERVO_M() {
  for (POS = 0; POS <= 180; POS += 1) {
    SERVO_1.write(POS);
    // Serial.print("A7A");
    delayMicroseconds(1000);
  }
  for (POS = 180; POS >= 0; POS -= 1) {
    SERVO_1.write(POS);
    //Serial.print("A7777A");
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
      for (int i = 0; i < 5; i++) {
        IR_READ[i] = digitalRead(IR[i]);  //STORE IR READINGS IN ARRAY
      }
      CAR_STATE(IR_READ);    
  }
}
}
