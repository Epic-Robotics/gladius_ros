/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;
#define blue_led 32 
void messageCb( const std_msgs::Empty& toggle_msg){
  if(digitalRead(blue_led) == HIGH){
    delay(2000);
    digitalWrite(blue_led,LOW);
  }else{
    digitalWrite(blue_led,HIGH);
  }

}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{ 
  pinMode(blue_led, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
