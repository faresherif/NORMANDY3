#define USS_TRIG      8   //ULTRASONIC SENSOR SIGNAL TRANSFER PIN
#define USS_ECHO       13
float distance;
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

  Serial.print("distance=");
  Serial.println(distance);
  //delay(100);
}
void setup(){
    pinMode(USS_TRIG, OUTPUT);
  pinMode(USS_ECHO, INPUT);
  Serial.begin(9600);
  }
  void loop(){
    USS();
    }
