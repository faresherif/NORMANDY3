#define FRONT_R 11
#define FRONT_L  6
#define BACK_R  10
#define BACK_L   9
#define BUTTON   4
int IR[5] = { A0, A1, A2, A3, A4 };
int READING[5];
int PWM = 90;
int PWM1 =120;
int OLD_STATE;
//#define SERVO 5
//#define USS_R 3
//#define USS_T 2
//#define BT_TX 1
//#define BT_RX 0

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(FRONT_R, OUTPUT);
  pinMode(FRONT_L, OUTPUT);
  pinMode(BACK_R, OUTPUT);
  pinMode(BACK_L, OUTPUT);
  pinMode(BUTTON,INPUT_PULLUP);
  Serial.begin(9600);
}

void FORWARD() {
  analogWrite(FRONT_R, PWM);
  analogWrite(FRONT_L, PWM);
}
void STOP() {
  analogWrite(FRONT_R, LOW);
  analogWrite(FRONT_L, LOW);
}
void TURN_RIGHT() {
  analogWrite(FRONT_R, LOW);
  analogWrite(FRONT_L, PWM1);
}
void TURN_LEFT() {
  analogWrite(FRONT_R, PWM1);
  analogWrite(FRONT_L, LOW);
}


void loop() {
  int read[5];
  int f=0;

  int NEW_STATE = digitalRead(BUTTON);
  if (NEW_STATE != OLD_STATE) {
    OLD_STATE = NEW_STATE;
    if (NEW_STATE == LOW) {
      while (f == 0) {
        for (int i = 0; i < 5; i++) {
          read[i] = digitalRead(IR[i]);
        }
        if (read[1] == 1 && read[2] == 0 && read[3] == 1) {
          FORWARD();
        } else if (read[1] == 1 && read[2] == 1 && read[3] == 0) {
          TURN_RIGHT();
        } else if (read[1] == 0 && read[2] == 1 && read[3] == 1) {
          TURN_LEFT();
        } else if (read[1] == 0 && read[2] == 0 && read[3] == 0) {
          STOP();
          f = 1;
        }
      }
    }
  }
else {
  STOP();
  f=1;
}
}
