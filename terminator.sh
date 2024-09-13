#!/bin/bash

ssh_login() {
    xdotool type "ssh kratos@192.168.1.10"
    xdotool key Return
    sleep 1
    xdotool type "kratos123"
    xdotool key Return
    sleep 1
    xdotool type "clear"
    xdotool key Return
    sleep 1
}

simulate_input() {
    xdotool type "$1"
    xdotool key Return
    sleep 1
}

split_vertical() {
    xdotool key Ctrl+Shift+E
    sleep 1
}

split_horizontal() {
    xdotool key Ctrl+Shift+O
    sleep 1
}

# Function to bring the specified terminator window to the front
bring_to_front() {
    wmctrl -r :ACTIVE: -e 0,0,0,-1,-1
}

python3 sample.py
# sleep 4
# pkill -f 'python3 sample.py'

terminator_width=$(xdpyinfo | awk '/dimensions:/ {print $2}' | cut -d'x' -f1)
terminator_height=$(expr $(xdpyinfo | awk '/dimensions:/ {print $2}' | cut -d'x' -f2) / 2)

# First window
terminator --geometry=${terminator_width}x${terminator_height}+0+0 &
bring_to_front
sleep 1
ssh_login
simulate_input "roscore"
split_horizontal
ssh_login
simulate_input "sudo chmod a+rw /dev/ttyACM*"
simulate_input "kratos123"
split_vertical
ssh_login
simulate_input "rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=57600"
split_vertical
ssh_login
simulate_input "roslaunch usb_cam usb_cam-test.launch"
split_vertical
ssh_login
simulate_input "roslaunch darknet_ros yolov3_custom.launch"
xdotool key Alt+Up
split_vertical
ssh_login
simulate_input "rosrun urc2022 videofeed1 0"
split_vertical
ssh_login
# simulate_input "sudo chmod 666 /dev/ttyACM0"
# simulate_input "kratos123"
simulate_input "roslaunch mavros px4.launch"

# Second Window
command_cd="cd catkin_ws/src/autonomous_irc/controls_auto/scripts/"
terminator --geometry=${terminator_width}x${terminator_height}+0+${terminator_height} &
bring_to_front
sleep 1
ssh_login
simulate_input "$command_cd"
simulate_input "./search_arrow_final_try.py"
split_horizontal
ssh_login
simulate_input "$command_cd"
simulate_input "./follow_arrow_final.py"
split_vertical
ssh_login
simulate_input "$command_cd"
simulate_input "> lat_long_arrow.txt"
simulate_input "./new_rotate_90.py"
command_cd="cd catkin_ws_2/src/autonomous_irc/controls_auto/scripts/"
split_vertical
ssh_login
simulate_input "$command_cd"
simulate_input "> lat_long.txt"
simulate_input "./new_gps.py"
xdotool key Alt+Up
split_vertical
ssh_login
simulate_input "$command_cd"
simulate_input "./main_control_final.py"
split_vertical
simulate_input "rosrun rqt_image_view rqt_image_view"
