#!/bin/bash

source /nucleus_ws/devel/setup.bash
roslaunch uns_driver uns_driver.launch ip:="$@"