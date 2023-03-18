

const int trig_Pin = 2;
const int echo_Pin = 4;

#define SPEED_OF_SOUND 0.034

long duration;
float distance_in_cm;


void setup(){
  pinMode(trig_Pin, OUTPUT);
  pinMode(echo_Pin,INPUT);
  Serial.begin(115200);
}

void loop(){
  digitalWrite(trig_Pin,LOW);
  delayMicroseconds(2);
  digitalWrite(trig_Pin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_Pin,LOW);

    duration = pulseIn(echo_Pin,HIGH);
    distance_in_cm = (duration * SPEED_OF_SOUND)/2;
    Serial.println(distance_in_cm);
    delay(500);
}
