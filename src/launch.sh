#!/bin/sh
xterm  -e  "roslaunch gmapping slam_gmapping_pr2.launch " &
sleep 5
xterm  -e  " rosrun multiranger test5.py " & 
sleep 5
# xterm  -e  "  rosrun multiranger test6.py " & 
# sleep 5
xterm  -e  " rosrun rviz rviz" 