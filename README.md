# ROS2 TEST NAVIGATION
## ROS INSTALLATIONS
sudo snap install code --classic

sudo apt install ros-humble-joint-state-publisher

sudo apt install ros-humble-rmw-cyclonedds-cpp

sudo apt install python3-serial

pip install pyserial

sudo apt install ros-humble-ros2-control

sudo apt install ros-humble-ros2-controllers

sudo apt install ros-humble-ros2-gazebo-control

sudo apt install ros-humble-gazebo-ros2-control

sudo apt install -y ros-humble-twist-mux

## TEST COMMUNICATION BTW DEV MACHINE AND BOT
ros2 run serial_motor_demo driver --ros-args -p serial_port:=/dev/ttyACM0 -p baud_rate:=57600 -p loop_rate:=30 -p encoder_cpr:=583

ros2 run serial_motor_demo gui
## Run bot in realtime and get feeds in rviz
ros2 launch articubot_one launch_robot.launch.py 
rviz2
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
