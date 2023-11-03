#define FRONT_R 11
#define FRONT_L 6
#define BACK_R 10
#define BACK_L 9
#define BUTTON 4
int IR[5] = { A0, A1, A2, A3, A4 };
int IR_READ[5];
int PWM_LOW = 90;
int PWM_MED = 160;
int PWM_HIGH = 200;
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
  pinMode(BUTTON, INPUT_PULLUP);
  Serial.begin(9600);
}
void FORWARD() {
  analogWrite(FRONT_R, PWM_LOW);
  analogWrite(FRONT_L, PWM_LOW);
}
void STOP() {
  analogWrite(FRONT_R, LOW);
  analogWrite(FRONT_L, LOW);
}
void TURN_RIGHT() {
  analogWrite(FRONT_R, PWM_LOW);
  analogWrite(FRONT_L, PWM_MED);
}
void TURN_LEFT() {
  analogWrite(FRONT_R, PWM_MED);
  analogWrite(FRONT_L, PWM_LOW);
}
void TURN_RIGHT_FAST() {
  analogWrite(FRONT_R, PWM_LOW);
  analogWrite(FRONT_L, PWM_HIGH);
}
void TURN_LEFT_FAST() {
  analogWrite(FRONT_R, PWM_HIGH);
  analogWrite(FRONT_L, PWM_LOW);
}
int CAR_STATE(int IR_READ[], int f) {
  f=0;
  if (IR_READ[0] == 1 && IR_READ[1] == 1 && IR_READ[2] == 0 && IR_READ[3] == 1 && IR_READ[4] == 1) {
    FORWARD();
  } else if (IR_READ[0] == 1 && IR_READ[1] == 1 && IR_READ[2] == 1 && IR_READ[3] == 0 && IR_READ[4] == 1) {
    TURN_RIGHT();
  } else if (IR_READ[0] == 1 && IR_READ[1] == 1 && IR_READ[2] == 1 && IR_READ[3] == 1 && IR_READ[4] == 0) {
    TURN_RIGHT_FAST();
  } else if (IR_READ[0] == 1 && IR_READ[1] == 0 && IR_READ[2] == 1 && IR_READ[3] == 1 && IR_READ[4] == 1) {
    TURN_LEFT();
  } else if (IR_READ[0] == 0 && IR_READ[1] == 1 && IR_READ[2] == 1 && IR_READ[3] == 1 && IR_READ[4] == 1) {
    TURN_LEFT_FAST();
  } else if (IR_READ[0] == 0 && IR_READ[1] == 0 && IR_READ[2] == 0 && IR_READ[3] == 0 && IR_READ[4] == 0) {
    STOP();
    f= 1;
  }
  return f;
}
int BUTTON_STATE(int flag) {
  int NEW_STATE = digitalRead(BUTTON);
  if (NEW_STATE != OLD_STATE) {
    OLD_STATE = NEW_STATE;
    flag = 0;
    return NEW_STATE;
  } else {
    analogWrite(FRONT_R, LOW);  //STOP THE CAR
    analogWrite(FRONT_L, LOW);
    flag = 1;  //FLAG
  }
  return flag;
}
void loop() {
  int f;
  int STATE = BUTTON_STATE(f);
  if (STATE == LOW) {
    while (f == 0) {
      for (int i = 0; i < 5; i++) {
        IR_READ[i] = digitalRead(IR[i]);
      }
      int CAR_MOVE = CAR_STATE(IR_READ,f);
    }
  }
}
