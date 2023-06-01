#! /bin/bash

x-terminal-emulator -e roslaunch my_robot world.launch  2>/dev/null &&

sleep 8 &&

x-terminal-emulator -e roslaunch my_robot amcl.launch 2>/dev/null &&

sleep 8 &

