#!/usr/bin/env python3
"""
A ROS wrapper for the UNS driver, based on example code provided by Nortek.
The UNS driver is inherited into a new class where it is modified in order to better organize how the
extracted packages from the UNS are handled.
"""

import rospy
from sensor_msgs.msg import Imu
from uns_driver import UnsDriver
from tf.transformations import quaternion_from_euler

import time
import serial

IMU_DATA_ID = "0x82"
MAG_DATA_ID = "0x87"
GNSS_DATA_ID = "0x91"
ALT_DATA_ID = "0xAA"
BOTTOM_DATA_ID = "0xB4"
WATER_TRACK_DATA_ID = "0xBE"
AHRS_DATA_ID = "0xD2" 

class UnsRosDriver(UnsDriver):

    def __init__(self):
        # TODO: Get as rosparam
        port = "/dev/ttyUSB0"
        baud = 115200
        use_queues = False
        self.uns_frame_id = "uns_link"

        ser = serial.Serial(port=port,
                            baudrate=baud,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            timeout=1)

        UnsDriver.__init__(self, serial_connection=ser, use_queues=use_queues)
        self.start()
        self.start_uns()

        self.imu_data_pub = rospy.Publisher("/uns/imu_data", Imu, queue_size=10)
        self.imu_pub_seq = 0
    
    def __del__(self):
        self.stop_uns()
        self.join(timeout=10)
        rospy.loginfo("Stopping UNS...")

    def write_package(self, package):
        """
        This function is executed whenever a package with sensor data is extracted from the UNS data stream. Overwriting
        this function in this class allow a user to handle these packages as they see fit.
        """
        id = package['id']

        if id == IMU_DATA_ID:
            imu_msg = Imu()

            imu_msg.header.seq = self.imu_pub_seq
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = self.uns_frame_id

            imu_msg.linear_acceleration.x = package['accelerometer_x']
            imu_msg.linear_acceleration.y = package['accelerometer_y']
            imu_msg.linear_acceleration.z = package['accelerometer_z']
            imu_msg.linear_acceleration_covariance = [0, 0, 0,
                                                      0, 0, 0,
                                                      0, 0, 0] # Row major about x, y, z

            imu_msg.angular_velocity.x = package['gyro_x']
            imu_msg.angular_velocity.y = package['gyro_y']
            imu_msg.angular_velocity.z = package['gyro_z']
            imu_msg.angular_velocity_covariance = [0, 0, 0,
                                                   0, 0, 0,
                                                   0, 0, 0] # Row major about x, y, z

            self.imu_data_pub.publish(imu_msg)
            self.imu_pub_seq += 1


    def write_condition(self, error_message, package):
        """
        This function is executed whenever an error occurs either in the UNS or with the parsing of its data. Overwriting
        this function in this class allow a user to handle these packages as they see fit.
        """
        rospy.logerr("UNS error: %s" % error_message)

    def write_ascii(self, ascii_package):
        """
        This function is executed whenever an ascii string is sent from the UNS. Overwriting his function in this class
        allow a user to handle these packages as they see fit.
        """
        ascii_string = "".join([chr(v) for v in ascii_package])
        rospy.loginfo("ASCII string received from UNS: %s" % ascii_string)


if __name__ == "__main__":

    rospy.init_node("uns_driver", anonymous=False)

    uns_ros_driver = UnsRosDriver()

    while uns_ros_driver.driver_running and not rospy.is_shutdown():
        # This just loops while the driver is running in a separate thread
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            break

    uns_ros_driver.driver_running = False
