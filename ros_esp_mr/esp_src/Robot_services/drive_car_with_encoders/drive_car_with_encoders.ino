

#define ROSSERIAL_ARDUINO_TCP
#include"WiFi.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

const int enc_r = 2;
const int enc_l = 4;
int count_r =0;
int count_l =0;

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

float left_wheel;
float right_wheel;

IPAddress server(192, 168, 203, 83);
uint16_t serverPort = 11411;
const char* ssid = "++++++";
const char* password = "++++++";

void cmdVel_to_pwm_cb(const geometry_msgs::Twist& velocity_msg);
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVel_to_pwm_cb);


std_msgs::Int16 enc_r_msg;
std_msgs::Int16 enc_l_msg;
ros::Publisher right_enc("encr_r_values", &enc_r_msg );
ros::Publisher left_enc("encr_l_values", &enc_l_msg );

void setup(){
    Serial.begin(115200);
    setupWiFi();
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.advertise(right_enc);
    nh.advertise(left_enc);
    nh.subscribe(sub);

    pin_definition();
    stop();
    Serial.println("Get ready");
    delay(2000);
}

void loop(){
    right_enc.publish(&enc_r_msg);
    left_enc.publish(&enc_l_msg);
    nh.spinOnce();
    delay(200);
}

void setupWiFi(){
    WiFi.begin(ssid, password);
    while(WiFi.status() != WL_CONNECTED){ delay(500);Serial.print("."); }
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
}

void cmdVel_to_pwm_cb(const geometry_msgs::Twist& velocity_msg){
    right_wheel = (velocity_msg.linear.x+velocity_msg.angular.z)/2;
    left_wheel = (velocity_msg.linear.x-velocity_msg.angular.z)/2;
    direction();
    speed();
    if(velocity_msg.linear.x==0.0 & velocity_msg.angular.z==0.0){
        stop();
    }
    Serial.print(left_wheel);Serial.print(" / ");Serial.print(right_wheel);
}

void pin_definition(){
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

    pinMode(enc_r, INPUT);
    pinMode(enc_l, INPUT);
    attachInterrupt(digitalPinToInterrupt(enc_r), Update_encR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_l), Update_encL, CHANGE);
}

void Update_encR(){
    enc_r_msg.data=count_r ++;
}

void Update_encL(){
    enc_l_msg.data=count_l ++;
}

void direction(){
    digitalWrite(MOTOR_B1_PIN, right_wheel > 0);//right_wheel
    digitalWrite(MOTOR_A1_PIN, right_wheel < 0);
    digitalWrite(MOTOR_B2_PIN, left_wheel > 0);//left_wheel
    digitalWrite(MOTOR_A2_PIN, left_wheel < 0);
}

void speed(){
    ledcWrite(channel_R, 200);
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
