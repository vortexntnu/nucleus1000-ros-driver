# Nortek-UNS-ROS-Driver
Repo containing the ROS driver for the Nortek UNS. Currently on a need-to-know license.

The uns_ros_driver.py is a wrapper for the uns_driver.py provided by Nortek. The provided driver
could be altered directly to work well with ROS (remove most of the threading stuff and such), but
I have decided to create a wrapper so that it will be easier to receive updates to the core driver
from Nortek, should that ever happen. Doing this also gives us a nice separation between the serial,
crc and other stuff that we don't need to think about for the ROS node.
