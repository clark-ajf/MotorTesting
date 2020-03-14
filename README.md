# MotorTesting
Arduino rosserial testing

## Pre-requisites

ROS (Tested w/ Melodic)

[Arduino CLI](https://arduino.github.io/arduino-cli/installation/) (Tested w/ V 15)


ROSSerial
```
apt-get install ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial 
```

[IDE Setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
```
cd <ros_catkin_ws>/src
git clone https://github.com/ros-drivers/rosserial.git
cd <ros_catkin_ws>
catkin_make
catkin_make install
```
```  
cd /home/ubuntu/Arduino (vi /home/ubuntu/.arduino15/arduino-cli.yaml)
rm -rf ros_lib 
rosrun rosserial_arduino make_libraries.py . 
```


## Build

```
arduino-cli compile --fqbn arduino:avr:mega MotorTesting 
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega MotorTesting 
```

## Run (Using ROS)

```
roscore
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 (check with arduino-cli board list)
rostopic echo /joystick_sensor
rostopic pub joystick_commands std_msgs/String {UP,LEFT,RIGHT,DOWN,STOP,ENABLE_JOYSTICK,DISABLE_JOYSTICK} -1
```
