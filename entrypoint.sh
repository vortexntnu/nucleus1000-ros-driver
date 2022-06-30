#!/bin/bash

source /nucleus_ws/devel/setup.bash
roslaunch nucleus1000_driver nucleus1000_driver.launch ip:="$@"
