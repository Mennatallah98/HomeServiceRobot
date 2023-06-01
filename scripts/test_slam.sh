#! /bin/bash

x-terminal-emulator -e roslaunch my_robot world.launch  2>/dev/null &&

sleep 8 &&

x-terminal-emulator -e roslaunch my_robot joy.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch gmapping gmapping.launch 2>/dev/null &&
sleep 5 &
