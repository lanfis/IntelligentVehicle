/*
used for communicate between ros and other device
*/
/*
Undone with TCP bi-directional ACK between ros and bridige
Maybe occur error when using different baudrate !
*/
#define PROG_NAME "ROS_ARDUINO_COM"
#define vers 1.0

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#define delayTime 20
#define timeOut delayTime
#define ROS_SERIAL_BAUD_RATE 57600
#define ROS_RX_PIN 
#define ROS_TX_PIN

#define BRIDGE_SERIAL_BAUD_RATE 9600
#define BRIDGE_RX_PIN 10
#define BRIDGE_TX_PIN 11

#define ROS_STRING_SIZE 2
#define BRIDGE_STRING_SIZE 2


#include <SoftwareSerial.h>
SoftwareSerial BridgeSerial(BRIDGE_RX_PIN, BRIDGE_TX_PIN);//Rx, TX

#include <ros.h>
#include <std_msgs/String.h>

#define ROS_TOPIC_NAME_SUB "ros_arduino_msgs"
#define ROS_TOPIC_NAME_PUB "arduino_ros_msgs"

ros::NodeHandle  nh;

//General
void stringSplit(String& string_cont, String& split);
void stringMerge(String& string_cont, String& merge_cont);
String str;
bool stat = false;
char terminator_char = '\n';


String rosToBridgeMsg(String& ros_msg);

//ROS
//String ros_str[ROS_STRING_SIZE];
void rosSubCallback( const std_msgs::String& str_msg);
void rosPub(String& msg);
void rosAck(String& msg);

String ros_topic_name_sub = ROS_TOPIC_NAME_SUB;
String ros_topic_name_pub = ROS_TOPIC_NAME_PUB;
std_msgs::String str_msg;
ros::Subscriber<std_msgs::String> ros_sub(ros_topic_name_sub.c_str(), rosSubCallback);
ros::Publisher ros_pub(ros_topic_name_pub.c_str(), &str_msg);


//BRIDGE
String bridge_str[BRIDGE_STRING_SIZE];
void bridgeWrite(String& bridge_msg);   
bool bridgeRead(String& bridge_msg);

void setup() 
{
  Serial.begin(ROS_SERIAL_BAUD_RATE); 
  BridgeSerial.begin(BRIDGE_SERIAL_BAUD_RATE);
  Serial.setTimeout(timeOut);
  BridgeSerial.setTimeout(timeOut);
  
  pinMode(13, OUTPUT);
  
  nh.initNode();
  nh.subscribe(ros_sub);
  nh.advertise(ros_pub);
  
  str = PROG_NAME;  str.concat(" On-Line !\n");
  Serial.write(str.c_str());
  rosPub(str);
}

void loop() 
{  
  digitalWrite(13, LOW);
  if(bridgeRead(str))
    rosPub(str);
  nh.spinOnce();
  delay(delayTime);
}


void rosSubCallback( const std_msgs::String& str_msg)
{
  str = str_msg.data;
  //rosAck(str);
  //Program
  //str = rosToBridgeMsg(str);
  bridgeWrite(str);
  //Program
}

void rosPub(String& msg)
{
  str_msg.data = msg.c_str();
  ros_pub.publish(&str_msg);
  msg = "";
}

void rosAck(String& msg)
{
  String ack_msg = "ACK ";
  stringMerge(ack_msg, msg);
  rosPub(ack_msg);
}


void bridgeWrite(String& bridge_msg)   
{
  digitalWrite(13, HIGH);
  bridge_msg = bridge_msg + "\n";
  BridgeSerial.write(bridge_msg.c_str());
  //delay(delayTime);
}

bool bridgeRead(String& bridge_msg)
{
  if(BridgeSerial.available() > 0)
  {
    digitalWrite(13, HIGH);
    bridge_msg = BridgeSerial.readStringUntil(terminator_char);
    return true;
  }
  return false;
}

void stringSplit(String& string_cont, String& split)
{
  string_cont.trim();
  int i;
  for(i = 0; i < string_cont.length(); i++)
  {
    if(isSpace(string_cont[i]))
      break;
  }

  split = string_cont.substring(0, i);
  string_cont = string_cont.substring(i, string_cont.length());
  string_cont.trim();
  return split.trim();
}

void stringMerge(String& string_cont, String& merge_cont)
{
  string_cont.concat(merge_cont);
}
/////////////////////////////////////////////

