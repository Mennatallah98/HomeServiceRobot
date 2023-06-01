#! /bin/bash

x-terminal-emulator -e roslaunch my_robot world.launch  2>/dev/null &&

sleep 8 &&

x-terminal-emulator -e roslaunch my_robot amcl.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e rosrun pick_objects pick_objects 2>/dev/null &&
sleep 5 &&

x-terminal-emulator -e rosrun add_markers add_markers 2>/dev/null &&
sleep 10 &
