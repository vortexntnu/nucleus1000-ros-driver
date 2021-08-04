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


class NORTEK_DEFINES:
    """
    This class just serves as a pseudo-namespace for constants defined by nortek
    """
    IMU_DATA_ID = "0x82"
    MAG_DATA_ID = "0x87"
    GNSS_DATA_ID = "0x91"
    ALT_DATA_ID = "0xaa"
    DVL_DATA_ID = "0xb4"
    WATER_TRACK_DATA_ID = "0xbe"
    AHRS_DATA_ID = "0xd2" 

    INVALID_FOM = 9.9   # Data sheet says 10.0, set lower here to be on the safe side :)
    INVALID_DISTANCE = 0.0
    INVALID_VELOCITY = -30.0 # Data sheet says -32.768, set higher here to be on the safe side :)

    # See page 15 of the communication interface spec
    AHRS_CALIBRATING = 0
    AHRS_INITIALIZING = 1
    AHRS_REGULAR_MODE = 2

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


        if id == NORTEK_DEFINES.IMU_DATA_ID:
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
        
        elif id == NORTEK_DEFINES.MAG_DATA_ID:

            magnetometer_x = package['magnetometer_x']
            magnetometer_y = package['magnetometer_y']
            magnetometer_z = package['magnetometer_z']

        elif id == NORTEK_DEFINES.DVL_DATA_ID:
            # Data from the three angled transducers        
            v_b   = [package['velocity_beam_0'], package['velocity_beam_1'], package['velocity_beam_2']]
            d_b   = [package['distance_beam_0'], package['distance_beam_1'], package['distance_beam_2']]
            fom_b = [package['fom_beam_0'], package['fom_beam_1'], package['fom_beam_2']]
            fom_xyz = [package['fom_x'], package['fom_y'], package['fom_z']]

            # Check validity of incoming data, note that we avoid == to account for precision errors
            invalid_data = ""
            if any(velocity <= NORTEK_DEFINES.INVALID_VELOCITY for velocity in v_b):
                invalid_data += "velocity "

            if any(distance <= NORTEK_DEFINES.INVALID_DISTANCE for distance in d_b):
                invalid_data += "distance "
            
            if any(fom >= NORTEK_DEFINES.INVALID_FOM for fom in fom_b):
                invalid_data += "beam-fom "
            
            if any(fom >= NORTEK_DEFINES.INVALID_FOM for fom in fom_xyz):
                invalid_data += "xyz-fom "

            if invalid_data != "":
                #rospy.logwarn("Invalid { %s} received. Ignoring package..." % invalid_data)
                return
            
            v_x = package['velocity_x']
            v_y = package['velocity_y']
            v_z = package['velocity_z']
            pressure = package['pressure']

            # TODO: Make TwistStamped message out of velocities (not Odom like the dvl1000, because we get depth from ahrs here)
            rospy.loginfo("Velocity XYZ: %.4f, %.4f, %.4f" % (v_x, v_y, v_z))
            rospy.loginfo("Pressure: %.4f" % pressure)

        elif id == NORTEK_DEFINES.AHRS_DATA_ID:

            op_mode = package['operation_mode']

            # Currently always in calibrating mode? TODO: Ask :)
            #if op_mode == AHRS_CALIBRATING:
            #    rospy.logwarn("AHRS calibrating...")
            #    return
            #
            #if op_mode == AHRS_INITIALIZING:
            #    rospy.logwarn("AHRS initializing...")
            #    return
            
            fom_ahrs = package['fom_ahrs']
            fom_field_calib = package['fom_fc1']

            x = package['quaternion_0']
            y = package['quaternion_1']
            z = package['quaternion_2']
            w = package['quaternion_3']

            R_bn = [ [package['dcm_11'], package['dcm_12'], package['dcm_13']],
                     [package['dcm_21'], package['dcm_22'], package['dcm_23']],
                     [package['dcm_31'], package['dcm_32'], package['dcm_33']]]

            declination = package['declination']
            depth = package['depth']
            
            # TODO: Make PosedStamped message from orientation and depth
            rospy.loginfo(depth)
            rospy.loginfo("%.4f, %.4f, %.4f, %.4f" % (x, y, z, w))


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
