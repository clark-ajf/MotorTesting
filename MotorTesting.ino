#include <ros.h>
#include <std_msgs/String.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


//LED Pin
#define LED 13

//Motor Pins
#define EN_A 8 //Enable pin for first motor
#define IN1 9   //control pin for first motor
#define IN2 10   //control pin for first motor
#define IN3 11   //control pin for second motor
#define IN4 12   //control pin for second motor
#define EN_B 13 //Enable pin for second motor

uint8_t motorcontrol[3];

#define DIRECTION 2
#define MOTOR_RIGHT 1
#define MOTOR_LEFT 0

//nRF24L01 Pins
#define CE_PIN 53
#define SCK_PIN 52
#define MISO_PIN 50
#define CSN_PIN 49
#define MOSI_PIN 51

RF24 radio(CE_PIN, CSN_PIN); // CE, CSN
const byte address[6] = "00001";

bool joystick_enabled = true;


ros::NodeHandle nh;
std_msgs::String str_msg;
void joystickCallback(const std_msgs::String& joystick_command_msg) {
    nh.loginfo(joystick_command_msg.data);

    if(strcmp(joystick_command_msg.data,"LEFT") == 0 && !joystick_enabled) {     
        drive(0, 150, 50);
    } else if (strcmp(joystick_command_msg.data,"RIGHT") == 0 && !joystick_enabled) {        
        drive(0, 50, 150);
    } else if (strcmp(joystick_command_msg.data,"BACKWARD") == 0 && !joystick_enabled) {
        drive(1, 100, 100);
    } else if (strcmp(joystick_command_msg.data,"FORWARD") == 0 && !joystick_enabled) {
        drive(0, 100, 100);
    } else if (strcmp(joystick_command_msg.data,"STOP") == 0 && !joystick_enabled) {
        stop(); 
    } else if (strcmp(joystick_command_msg.data,"DISABLE_JOYSTICK") == 0) {
        joystick_enabled = false;
    } else if (strcmp(joystick_command_msg.data,"ENABLE_JOYSTICK") == 0) {
        joystick_enabled = true;
    }

    nh.loginfo(joystick_enabled ? "TRUE" : "FALSE");
    publish_joystick_sensor(joystick_command_msg);
}
ros::Publisher pub_joystick("joystick_sensor", &str_msg);
ros::Subscriber<std_msgs::String> sub_joystick("joystick_commands", joystickCallback);

void setup ( ) {
  //Starting the serial communication at 9600 baud rate
  Serial.begin (9600);

  // Initializing Radio Receiver
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  //Initializing the motor pins as output
  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);  
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);  
  pinMode(IN4, OUTPUT);
  pinMode(EN_B, OUTPUT);

  //Initializing LED pin
  pinMode(LED, OUTPUT);

  //Setup ROS
  nh.initNode();
  nh.advertise(pub_joystick);
  nh.subscribe(sub_joystick);
  nh.loginfo("Node Initiated");
}

void loop () {
    if(joystick_enabled){
        if (radio.available()) {
            radio.read(&motorcontrol, sizeof(motorcontrol));
            drive(motorcontrol[DIRECTION], motorcontrol[MOTOR_RIGHT], motorcontrol[MOTOR_LEFT]);
            Serial.print("Motor LEFT: ");
            Serial.print(motorcontrol[MOTOR_LEFT]);
            Serial.print(" - Motor RIGHT: ");
            Serial.print(motorcontrol[MOTOR_RIGHT]);
            Serial.print(" - Direction: ");
            Serial.println(motorcontrol[DIRECTION]);
            publish_joystick_sensor("RADIO AVAILABLE");
            publish_joystick_sensor(motorcontrol[DIRECTION]);
        } else {
            publish_joystick_sensor("RADIO NOT AVAILABLE");
            Serial.print("Not Connected");
        }
    }
    
    nh.spinOnce();    
}

void publish_joystick_sensor(const std_msgs::String& joystick_sensor_msg) {
    if(nh.connected()) {
        Serial.println("Connected");
        pub_joystick.publish(&joystick_sensor_msg);
    } else {
        Serial.println("Not Connected");
    }
}

void drive(int direction, int motor_right_speed, int motor_left_speed) {
    if (direction == 1){ // Backward
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    } else { // Forward
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }

    // Drive Motors        
    analogWrite(EN_A, motor_right_speed);
    analogWrite(EN_B, motor_left_speed);
}

void stop() {
    analogWrite(EN_A, 0);
    analogWrite(EN_B, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(LED, LOW);
}