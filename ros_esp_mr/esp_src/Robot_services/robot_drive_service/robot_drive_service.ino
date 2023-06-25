/*

*/
#define ROSSERIAL_ARDUINO_TCP
#include"WiFi.h"
#include <ros.h>
#include <rosserial_arduino/Test.h>
using rosserial_arduino::Test;

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


IPAddress server(192, 168, 203, 83);
uint16_t serverPort = 11411;

const char* ssid = "++++++";
const char* password = "++++++";

void srv_cb(const Test::Request & req, Test::Response & res);
ros::NodeHandle nh;
ros::ServiceServer<Test::Request, Test::Response> car_motion_time_srv("sec_for_movement",&srv_cb);

void setup(){
    Serial.begin(115200);
    setupWiFi();
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.advertiseService(car_motion_time_srv);

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
    nh.spinOnce();
}

void srv_cb(const Test::Request & req, Test::Response & res){
    String value_in = req.input;
    time_in_seconds = value_in.toInt();

    move_forward();
    delay(time_in_seconds + 1000);
    stop();
    res.output = "Service Completed";
}

void setupWiFi(){
    WiFi.begin(ssid, password);
    while(WiFi.status() != WL_CONNECTED){ delay(500);Serial.print("."); }
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
}

void move_forward(){
    digitalWrite(MOTOR_B1_PIN, 1);//right_wheel
    digitalWrite(MOTOR_A1_PIN, 0);
    digitalWrite(MOTOR_B2_PIN, 1);//left_wheel
    digitalWrite(MOTOR_A2_PIN, 0);
    ledcWrite(channel_R, 200);
    ledcWrite(channel_L, 200);
}

void stop(){
    ledcWrite(channel_R, 0);
    ledcWrite(channel_L, 0);
}
