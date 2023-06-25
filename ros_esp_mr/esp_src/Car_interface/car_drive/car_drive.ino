/*

*/
//MOTOR 1 (right motor)
const uint8_t  MOTOR_A1_PIN = 26;  //Driver Board IN1
const uint8_t  MOTOR_B1_PIN = 27;  //Driver Board IN2
const uint8_t  PWM_MOTOR_1 = 25;   //Driver Board PWM
//MOTOR 2 (left motor)
const uint8_t  MOTOR_A2_PIN = 12;  //Driver Board IN1
const uint8_t  MOTOR_B2_PIN = 13;  //Driver Board IN2
const uint8_t  PWM_MOTOR_2 = 14;   //Driver Board PWM

const uint8_t  channel_L = 0;
const uint8_t  channel_R = 1;

void setup(){
  Serial.begin(115200);
  pin_def();
  stop();
  Serial.println("Get ready");
  delay(2000);

}

void pin_def(){
  const int freq = 5000;
  const int res = 8;

  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);

  ledcSetup(channel_R, freq, res);
  ledcSetup(channel_L, freq, res);
  ledcAttachPin(PWM_MOTOR_1, channel_R);
  ledcAttachPin(PWM_MOTOR_2, channel_L);
}

void loop(){
  move_forward();
  delay(1000);

  move_backward();
  delay(1000);

  move_left();
  delay(1000);

  move_right();
  delay(1000);

}

void move_forward(){
  Serial.println("Moving forward");
  digitalWrite(MOTOR_A1_PIN, LOW);
  digitalWrite(MOTOR_B1_PIN, HIGH);
  //analogWrite(PWM_MOTOR_1, velocity_set_right);
  ledcWrite(channel_R, 200);

  digitalWrite(MOTOR_A2_PIN, LOW);
  digitalWrite(MOTOR_B2_PIN, HIGH);
  //analogWrite(PWM_MOTOR_2, velocity_set_right);
  ledcWrite(channel_L, 200);
}

void move_backward(){
  Serial.println("Moving backward");
  digitalWrite(MOTOR_A1_PIN, HIGH);
  digitalWrite(MOTOR_B1_PIN, LOW);
  //analogWrite(PWM_MOTOR_1, velocity_set_right);
  ledcWrite(channel_R, 200);

  digitalWrite(MOTOR_A2_PIN, HIGH);
  digitalWrite(MOTOR_B2_PIN, LOW);
  //analogWrite(PWM_MOTOR_2, velocity_set_right);
  ledcWrite(channel_L, 200);
}

void move_left(){
  Serial.println("Moving left");
  digitalWrite(MOTOR_A1_PIN, LOW);
  digitalWrite(MOTOR_B1_PIN, HIGH);
  //analogWrite(PWM_MOTOR_1, velocity_set_right);
  ledcWrite(channel_R, 200);

  digitalWrite(MOTOR_A2_PIN, HIGH);
  digitalWrite(MOTOR_B2_PIN, LOW);
  //analogWrite(PWM_MOTOR_2, velocity_set_right);
  ledcWrite(channel_L, 0);
}

void move_right(){
  Serial.println("Moving right");
  digitalWrite(MOTOR_A1_PIN, HIGH);
  digitalWrite(MOTOR_B1_PIN, LOW);
  //analogWrite(PWM_MOTOR_1, velocity_set_right);
  ledcWrite(channel_R, 0);

  digitalWrite(MOTOR_A2_PIN, LOW);
  digitalWrite(MOTOR_B2_PIN, HIGH);
  //analogWrite(PWM_MOTOR_2, velocity_set_right);
  ledcWrite(channel_L, 200);
}

void stop(){
  digitalWrite(MOTOR_A1_PIN, LOW);
  digitalWrite(MOTOR_B1_PIN, LOW);
  //analogWrite(PWM_MOTOR_1, velocity_set_right);

  digitalWrite(MOTOR_A2_PIN, LOW);
  digitalWrite(MOTOR_B2_PIN, LOW);
  //analogWrite(PWM_MOTOR_2, velocity_set_right);

}
