#!/usr/bin/env python3
"""
A ROS wrapper for the Nucleus1000 DVL driver, based on example code provided by Nortek.
The Nucleus1000 DVL driver is inherited into a new class where it is modified in order to better organize how the
extracted packages from the Nucleus1000 DVL are handled.
"""

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

import socket
import time

from NucleusDriver import NucleusDriver

class NORTEK_DEFINES:
    """
    This class just serves as a pseudo-namespace for constants defined by nortek
    """
    ID_IMU = "0x82"
    ID_MAGNETOMETER = "0x87"
    ID_BOTTOMTRACK = "0xb4"
    ID_WATERTRACK = "0xbe"
    ID_ALTIMETER = "0xaa"
    ID_AHRS = "0xd2"
    ID_FIELD_CALIBRATION = "0x8B"

    INVALID_FOM = 9.9   # Data sheet says 10.0, set lower here to be on the safe side :)
    INVALID_DISTANCE = 0.0
    INVALID_VELOCITY = -30.0 # Data sheet says -32.768, set higher here to be on the safe side :)

    # See page 15 of the communication interface spec
    AHRS_CALIBRATING = 0
    AHRS_INITIALIZING = 1
    AHRS_REGULAR_MODE = 2

class NucleusRosDriver():

    def __init__(self):

        self.imu_data_pub = rospy.Publisher("/dvl/imu_data", Imu, queue_size=10)
        self.imu_pub_seq = 0

        self.dvl_twist_pub = rospy.Publisher("/dvl/dvl_data", TwistWithCovarianceStamped, queue_size=10)
        self.dvl_pub_seq = 0

        self.ahrs_pose_pub = rospy.Publisher("/dvl/ahrs_pose", PoseWithCovarianceStamped, queue_size=10)
        self.ahrs_pub_seq = 0

        self.altitude_pub = rospy.Publisher("/dvl/altitude", Float32, queue_size=10)

        self.sensor_frame_id = "nucleus_link"
        self.map_frame_id = "odom"

        self.hostname = rospy.get_param("/nucleus1000_driver/dvl_ip")
        if self.hostname == "":
            self.hostname = "169.254.15.123"
        self.port = 9000
        self.use_queues = True

        rospy.loginfo(f"Nucleus configured as: {self.hostname}:{self.port}")

        self.nucleus_driver = NucleusDriver()

        self.nucleus_driver.connection.set_tcp_configuration(host=self.hostname)
        self.nucleus_driver.connection.set_tcp_configuration(port=int(self.port))
        self.nucleus_driver.connection.connect(connection_type="tcp")
        self.nucleus_driver.parser.set_queuing(packet=self.use_queues)

        if self.nucleus_driver.connection.get_connection_status() is not True:
            rospy.logerr("Nucleus is not connected!")
            rospy.signal_shutdown("Nucleus is not connected!")
            exit()


        self.nucleus_driver.thread.start()
        #self.nucleus_driver.logging.start()
        self.nucleus_driver.commands.start() # Send START to Nucleus1000


    def spin(self):
        while not rospy.is_shutdown():
            try:
                packet = self.nucleus_driver.parser.read_packet()
                self.parse_packet_ros(packet)
            except Exception as err:
                pass
            time.sleep(0.01)
            
        rospy.loginfo("Stopping Nucleus1000 DVL...")
        #self.nucleus_driver.logging.stop()
        self.nucleus_driver.thread.stop()
        self.nucleus_driver.commands.stop() # Send STOP to Nucleus1000

    def parse_packet_ros(self, packet):
        """
        This function is executed whenever a package with sensor data is extracted from the Nucleus1000 DVL data stream. Overwriting
        this function in this class allow a user to handle these packages as they see fit.
        """
        id = packet['id']

        if id == NORTEK_DEFINES.ID_IMU:
            imu_msg = Imu()

            status = packet['status']

            imu_msg.header.seq = self.imu_pub_seq
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = self.sensor_frame_id

            imu_msg.linear_acceleration.x = packet['accelerometer_x']
            imu_msg.linear_acceleration.y = packet['accelerometer_y']
            imu_msg.linear_acceleration.z = packet['accelerometer_z']
            imu_msg.linear_acceleration_covariance = [0, 0, 0,
                                                      0, 0, 0,
                                                      0, 0, 0] # Row major about x, y, z

            imu_msg.angular_velocity.x = packet['gyro_x']
            imu_msg.angular_velocity.y = packet['gyro_y']
            imu_msg.angular_velocity.z = packet['gyro_z']
            imu_msg.angular_velocity_covariance = [0, 0, 0,
                                                   0, 0, 0,
                                                   0, 0, 0] # Row major about x, y, z

            self.imu_data_pub.publish(imu_msg)
            self.imu_pub_seq += 1

        elif id == NORTEK_DEFINES.ID_MAGNETOMETER:

            magnetometer_x = packet['magnetometer_x']
            magnetometer_y = packet['magnetometer_y']
            magnetometer_z = packet['magnetometer_z']

        elif id == NORTEK_DEFINES.ID_BOTTOMTRACK:
            # Data from the three angled transducers
            # This could be simplified using the status bit, see page 11 of the communication spec,
            # but is left like this since we explicitly retrieve the data

            #status = package['status']
                    
            v_b   = [packet['velocity_beam_0'], packet['velocity_beam_1'], packet['velocity_beam_2']]
            d_b   = [packet['distance_beam_0'], packet['distance_beam_1'], packet['distance_beam_2']]
            fom_b = [packet['fom_beam_0'], packet['fom_beam_1'], packet['fom_beam_2']]
            fom_xyz = [packet['fom_x'], packet['fom_y'], packet['fom_z']]

            #any_invalid_data = (~status & 0xFFFFFFFF) & 0x3FFFF # First & is to limit to 32 bit precision and second is to cut off the 18 bits we use
            # Alternatively: any_invalid_data = status < 0x3FFFF
            # Check validity of incoming data, note that we avoid == to account for precision errors
            invalid_data = ""
            if any(velocity <= NORTEK_DEFINES.INVALID_VELOCITY for velocity in v_b):
                invalid_data += "beam-velocity "

            if any(distance <= NORTEK_DEFINES.INVALID_DISTANCE for distance in d_b):
                invalid_data += "beam-distance "
            
            if any(fom >= NORTEK_DEFINES.INVALID_FOM for fom in fom_b):
                invalid_data += "beam-fom "
            
            if any(fom >= NORTEK_DEFINES.INVALID_FOM for fom in fom_xyz):
                invalid_data += "xyz-fom "

            if invalid_data != "":
                rospy.logwarn("Invalid { %s} received. Ignoring package..." % invalid_data)
                return
            
            #pressure = package['pressure']

            # TODO: Make TwistStamped message out of velocities (not Odom like the dvl1000, because we get depth from ahrs here)
            dvl_msg = TwistWithCovarianceStamped()

            dvl_msg.header.seq = self.dvl_pub_seq
            dvl_msg.header.stamp = rospy.Time.now()
            dvl_msg.header.frame_id = self.sensor_frame_id

            dvl_msg.twist.twist.linear.x = packet['velocity_x']
            dvl_msg.twist.twist.linear.y = packet['velocity_y']
            dvl_msg.twist.twist.linear.z = packet['velocity_z']

            var_x = fom_xyz[0] * fom_xyz[0]
            var_y = fom_xyz[1] * fom_xyz[1]
            var_z = fom_xyz[2] * fom_xyz[2]

            dvl_msg.twist.covariance = [var_x,  0,    0,   0, 0, 0,
                                          0,  var_y,  0,   0, 0, 0,
                                          0,    0,  var_z, 0, 0, 0,
                                          0,    0,    0,   0, 0, 0,
                                          0,    0,    0,   0, 0, 0,
                                          0,    0,    0,   0, 0, 0]

            self.dvl_twist_pub.publish(dvl_msg)
            self.dvl_pub_seq += 1


        elif id == NORTEK_DEFINES.ID_AHRS:

            status = packet['status']
            op_mode = packet['operation_mode'] # == status?

            # Currently always in calibrating mode? TODO: Ask :)
            #if op_mode == AHRS_CALIBRATING:
            #    rospy.logwarn("AHRS calibrating...")
            #    return
            #
            #if op_mode == AHRS_INITIALIZING:
            #    rospy.logwarn("AHRS initializing...")
            #    return
            
            fom_ahrs = packet['fom_ahrs']
            fom_field_calib = packet['fom_fc1']

            ahrs_pose = PoseWithCovarianceStamped()

            ahrs_pose.header.seq = self.ahrs_pub_seq
            ahrs_pose.header.stamp = rospy.Time.now()
            ahrs_pose.header.frame_id = self.map_frame_id
            
            ahrs_pose.pose.pose.position.z = -packet['depth']
            
            ahrs_pose.pose.pose.orientation.x = packet['quaternion_1']
            ahrs_pose.pose.pose.orientation.y = packet['quaternion_2']
            ahrs_pose.pose.pose.orientation.z = packet['quaternion_3']
            ahrs_pose.pose.pose.orientation.w = packet['quaternion_0']

            var_z = 0.000001
            var_rx = 0.000001
            var_ry = 0.000001
            var_rz = 0.000001
            ahrs_pose.pose.covariance = [0, 0,   0,     0,     0,     0,
                                         0, 0,   0,     0,     0,     0,
                                         0, 0, var_z,   0,     0,     0,
                                         0, 0,   0,  var_rx,   0,     0,
                                         0, 0,   0,     0,  var_ry,   0,
                                         0, 0,   0,     0,     0,  var_rz]

            self.ahrs_pose_pub.publish(ahrs_pose)
            self.ahrs_pub_seq += 1

        elif id == NORTEK_DEFINES.ID_ALTIMETER:

            quality = packet['altimeter_quality']

            distance = Float32()
            distance.data = packet['altimeter_distance']

            self.altitude_pub.publish(distance)


if __name__ == "__main__":
    rospy.init_node("nucleus_ros_driver", anonymous=False)

    nucleus_ros_driver = NucleusRosDriver()
    nucleus_ros_driver.spin()
