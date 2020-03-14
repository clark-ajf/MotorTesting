# MotorTesting
Arduino rosserial testing


arduino-cli compile --fqbn arduino:avr:mega MotorTesting 

arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega MotorTesting 


https://arduino.github.io/arduino-cli/installation/

apt-get install ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial 

cd /home/ubuntu/Arduino (vi /home/ubuntu/.arduino15/arduino-cli.yaml)
rm -rf ros_lib 
rosrun rosserial_arduino make_libraries.py . 
