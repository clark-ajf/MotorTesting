#include <ros.h>
#include <std_msgs/String.h>

//Joystick Pins
int x_key = A0;                                               
int y_key = A1;                                               
int x_pos;
int y_pos;

bool joystick_enabled = true;

//LED Pin
int LED = 13;

//Motor Pins
int EN_A = 11;      //Enable pin for first motor
int IN1 = 9;       //control pin for first motor
int IN2 = 8;       //control pin for first motor
int IN3 = 7;        //control pin for second motor
int IN4 = 6;        //control pin for second motor
int EN_B = 10;      //Enable pin for second motor

//Initializing variables to store data
int motor_horizontal;
int motor_vertical;

ros::NodeHandle nh;
std_msgs::String str_msg;
void joystickCallback(const std_msgs::String& joystick_command_msg) {
    nh.loginfo(joystick_command_msg.data);
    if(strcmp(joystick_command_msg.data,"LEFT") == 0 && !joystick_enabled) {
        x_pos = 200;
        left();
    } else if (strcmp(joystick_command_msg.data,"RIGHT") == 0 && !joystick_enabled) {
        x_pos = 800;
        right(); 
    } else if (strcmp(joystick_command_msg.data,"DOWN") == 0 && !joystick_enabled) {
        y_pos = 200;
	    down();
    } else if (strcmp(joystick_command_msg.data,"UP") == 0 && !joystick_enabled) {
        y_pos = 800;
        up();
    } else if (strcmp(joystick_command_msg.data,"STOP") == 0 && !joystick_enabled) {
        x_pos = 500;
        y_pos = 500;
        stay(); 
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
  Serial.begin (9600); //Starting the serial communication at 9600 baud rate
  //Initializing the motor pins as output
  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);  
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);  
  pinMode(IN4, OUTPUT);
  pinMode(EN_B, OUTPUT);

  //Initializing the joystick pins as input
  pinMode(x_key, INPUT) ;                     
  pinMode(y_key, INPUT) ;    

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
        x_pos = analogRead (x_key) ;  //Reading the horizontal movement value
        y_pos = analogRead (y_key) ;  //Reading the vertical movement value
    
        if (x_pos < 400){     //Rotating the left motor in clockwise direction
            left();
            str_msg.data = "LEFT";
            publish_joystick_sensor(str_msg);
        }
        else if (x_pos>400 && x_pos <600){  //Motors will not move when the joystick will be at center
            stay();
        }  
        else if (x_pos > 600){    //Rotating the left motor in anticlockwise direction
            right();
            str_msg.data = "RIGHT";
            publish_joystick_sensor(str_msg);
        }   
        if (y_pos < 400){         //Rotating the right motor in clockwise direction
            down();
            str_msg.data = "DOWN";
            publish_joystick_sensor(str_msg);
        }
        else if (y_pos>400 && y_pos <600){
            stay();   
        }  
        else if (y_pos > 600){        //Rotating the right motor in anticlockwise direction
            up();
            str_msg.data = "UP";
            publish_joystick_sensor(str_msg);
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

void left() {
    motor_horizontal = map(x_pos, 400, 0, 0, 255);   //Mapping the values to 0-255 to move the motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN_A, motor_horizontal);
    digitalWrite(LED, HIGH);
}

void right() {
    motor_horizontal = map(x_pos, 600, 1023, 0, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN_A, motor_horizontal);
    digitalWrite(LED, HIGH);
}

void up() {
    motor_vertical = map(y_pos, 600, 1023, 0, 255);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(EN_B, motor_vertical);
    digitalWrite(LED, HIGH);
}

void down() {
    motor_vertical = map(y_pos, 400, 0, 0, 255);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(EN_B, motor_vertical);
    digitalWrite(LED, HIGH);
}

void stay() {
    analogWrite(EN_A, 0);
    analogWrite(EN_B, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(LED, LOW);
}