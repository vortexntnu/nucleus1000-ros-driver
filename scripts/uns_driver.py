import threading
import time
import serial
import serial.tools.list_ports
import queue
import argparse
from itertools import zip_longest
from datetime import datetime
from struct import unpack, error

START = "START\r\n".encode()
STOP = "STOP\r\n".encode()


class UnsDriver(threading.Thread):
    """
    The UnsDriver class inherits threading.Thread class, thus it is intended to run as its own seperate thread.

    To initiate the driver create an object of UnsDriver and execute the start() function. Upon object creation the
    user can specify 2 arguments. The first is serial_connection which is a pyserial object for serial communication.
    If the user do not wish to provide this argument a serial connection can be created through the connect_uns()
    function.

    The second argument is use_queues. If this argument is set to True, the packages from the UNS will be stored in
    queues. The packages can then be obtained through get_package(), get_condition(), and get_ascii() for packages,
    condition messages and ascii messages. If the user do NOT intend to obtain the packages through these queues this
    argument should remain False in order to prevent the stored data to overflow memory.

    This class could also be inherited into another class where the functions write_package(self, package),
    write_condition(self, error_message, package), and write_ascii(self, ascii_package) are overwritten. These functions
    are executed whenever the driver extracts a package of the given type, thus allowing a user to handle a package
    as they see fit whenever it is extraced.
    """

    def __init__(self, serial_connection=None, use_queues=False):

        threading.Thread.__init__(self)

        self.uns_serial = serial.Serial()
        self.serial_port = None
        self.baudrate = 115200

        if serial_connection is not None:
            self.uns_serial = serial_connection

        self.package_queue = queue.Queue()
        self.ascii_queue = queue.Queue()
        self.condition_queue = queue.Queue()
        self.queuing = use_queues

        self.packet_counter_previous = 1e10

        self.log_init_time = datetime.now()

        self.driver_running = True

    def set_serial_port(self, serial_port):

        self.serial_port = serial_port

    def set_baudrate(self, baudrate):

        self.baudrate = baudrate

    def connect_uns(self, serial_port=None, baudrate=None, timeout=1):

        if self.uns_serial.is_open:
            self.uns_serial.close()

        if serial_port is not None:
            self.serial_port = serial_port

        if baudrate is not None:
            self.baudrate = baudrate

        try:
            self.uns_serial = serial.Serial(port=self.serial_port,
                                            baudrate=self.baudrate,
                                            parity=serial.PARITY_NONE,
                                            stopbits=serial.STOPBITS_ONE,
                                            timeout=timeout)
            self.uns_serial.reset_input_buffer()
            condition = True
        except serial.SerialException:
            condition = False

        return condition

    def disconnect_uns(self):

        if self.uns_serial.is_open:
            self.uns_serial.close()

    def send_command(self, command):

        if self.uns_serial.is_open:
            try:
                self.uns_serial.write(command)
            except Exception as e:
                print('sending serial command to UNS failed: {}'.format(e))
                pass

    def start_uns(self):

        self.send_command(START)

    def stop_uns(self):

        self.send_command(STOP)

    def get_package(self):

        package = None

        if not self.package_queue.empty():
            try:
                package = self.package_queue.get_nowait()
            except Exception as e:
                print('failed to retrieve package from package queue: {}'.format(e))
                pass

        return package

    def get_condition(self):

        package = None

        if not self.condition_queue.empty():
            try:
                package = self.condition_queue.get_nowait()
            except Exception as e:
                print('failed to retrieve package from condition queue: {}'.format(e))
                pass

        return package

    def get_ascii(self):

        package = None

        if not self.ascii_queue.empty():
            try:
                package = self.ascii_queue.get_nowait()
            except Exception as e:
                print('failed to retrieve package from ascii queue: {}'.format(e))
                pass

        return package

    def write_package(self, package):

        if self.queuing:
            self.package_queue.put(package)

    def write_condition(self, error_message, package):

        failed_package = {'timestamp_python': datetime.now().timestamp() - self.log_init_time.timestamp(),
                          'error_message': error_message,
                          'failed_packet': package}

        if self.queuing:
            self.condition_queue.put(failed_package)

    def write_ascii(self, ascii_package):

        ascii_string = self._data_to_ascii(ascii_package)
        ascii_package = {'message': ascii_string}

        if self.queuing:
            self.ascii_queue.put(ascii_package)

    @staticmethod
    def _data_to_ascii(data):

        ascii_string = ''.join(chr(i) for i in data if 0 <= i <= 0x7e)

        return ascii_string

    @staticmethod
    def checksum(package):

        checksum = 0xb58c

        for u, v in zip_longest(package[::2], package[1::2], fillvalue=None):

            if v is not None:
                checksum += u | v << 8
            else:
                checksum += u << 8 | 0x00

            checksum &= 0xffff

        return checksum

    @staticmethod
    def check_header(data):

        checksum_result = False
        header_size = data[1]

        if UnsDriver.checksum(data[:header_size - 2]) == data[header_size - 2] | data[header_size - 1] << 8:
            checksum_result = True

        return checksum_result

    @staticmethod
    def check_packet(data):

        checksum_result = False
        header_size = data[1]

        if UnsDriver.checksum(data[header_size:]) == data[header_size - 4] | data[header_size - 3] << 8:
            checksum_result = True

        return checksum_result

    @staticmethod
    def extract_packet(data):

        header_size = data[1]
        id = data[2]
        packet_counter = data[6] | data[7] << 8

        sensor_data, condition = UnsDriver.get_sensor_data(id, packet_counter, data[header_size:])

        return sensor_data, condition

    @staticmethod
    def get_sensor_data(sensor_id, packet_counter, data):

        packet = None
        condition = 'packet id not recognized'

        if sensor_id == 0x82:  # IMU
            try:
                version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                if version == 1:
                    unpacked_data = list(unpack('<BBHIIIfffffff', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
                        'packet_counter': packet_counter,
                        'timestamp_python': datetime.now().timestamp(),
                        'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                        'status': hex(unpacked_data[5]),
                        'accelerometer_x': unpacked_data[6],
                        'accelerometer_y': unpacked_data[7],
                        'accelerometer_z': unpacked_data[8],
                        'gyro_x': unpacked_data[9],
                        'gyro_y': unpacked_data[10],
                        'gyro_z': unpacked_data[11],
                        'temperature': unpacked_data[12]
                    }
                    condition = True
            except error:
                print('failed imu')
                condition = 'failed to unpack IMU data'

        elif sensor_id == 0x87:  # Magnetometer
            try:
                version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                if version == 1:
                    unpacked_data = list(unpack('<BBHIIIfff', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
                        'packet_counter': packet_counter,
                        'timestamp_python': datetime.now().timestamp(),
                        'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                        'status': hex(unpacked_data[5]),
                        'magnetometer_x': unpacked_data[6],
                        'magnetometer_y': unpacked_data[7],
                        'magnetometer_z': unpacked_data[8],
                    }
                    condition = True
            except error:
                print('failed mag')
                condition = 'failed to unpack magnetometer data'

        elif sensor_id == 0xb4:  # DVL
            try:
                version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                if version == 1:
                    unpacked_data = list(unpack('<BBHIIIIIffffffffffffffffffffffffff', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
                        'packet_counter': packet_counter,
                        'timestamp_python': datetime.now().timestamp(),
                        'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                        'status': hex(unpacked_data[5]),
                        'serial_number': unpacked_data[6],
                        'error': hex(unpacked_data[7]),
                        'sound_speed': unpacked_data[8],
                        'temperature': unpacked_data[9],
                        'pressure': unpacked_data[10],
                        'velocity_beam_0': unpacked_data[11],
                        'velocity_beam_1': unpacked_data[12],
                        'velocity_beam_2': unpacked_data[13],
                        'distance_beam_0': unpacked_data[14],
                        'distance_beam_1': unpacked_data[15],
                        'distance_beam_2': unpacked_data[16],
                        'fom_beam_0': unpacked_data[17],
                        'fom_beam_1': unpacked_data[18],
                        'fom_beam_2': unpacked_data[19],
                        'dt_beam_0': unpacked_data[20],
                        'dt_beam_1': unpacked_data[21],
                        'dt_beam_2': unpacked_data[22],
                        'time_velocity_estimate_0': unpacked_data[23],
                        'time_velocity_estimate_1': unpacked_data[24],
                        'time_velocity_estimate_2': unpacked_data[25],
                        'velocity_x': unpacked_data[26],
                        'velocity_y': unpacked_data[27],
                        'velocity_z': unpacked_data[28],
                        'fom_x': unpacked_data[29],
                        'fom_y': unpacked_data[30],
                        'fom_z': unpacked_data[31],
                        'dt_xyz': unpacked_data[32],
                        'time_velocity_estimate_xyz': unpacked_data[33]
                    }
                    condition = True
            except error:
                print('failed dvl')
                condition = 'failed to unpack DVL data'

        elif sensor_id == 0xaa:  # Altimeter
            try:
                version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                if version == 1:
                    unpacked_data = list(unpack('<BBHIIIIIffffH', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
                        'packet_counter': packet_counter,
                        'timestamp_python': datetime.now().timestamp(),
                        'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                        'status': hex(unpacked_data[5]),
                        'serial_number': unpacked_data[6],
                        'error': hex(unpacked_data[7]),
                        'sound_speed': unpacked_data[8],
                        'temperature': unpacked_data[9],
                        'pressure': unpacked_data[10],
                        'altimeter_distance': unpacked_data[11],
                        'altimeter_quality': unpacked_data[12]
                    }
                    condition = True
            except error:
                print('failed altimeter')
                condition = 'failed to unpack altimeter data'

        elif sensor_id == 0xd2:  # AHRS
            try:
                version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                if version == 1:
                    unpacked_data = list(unpack('<BBHIIIIIBBBBffffffffffffffffff', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
                        'packet_counter': packet_counter,
                        'timestamp_python': datetime.now().timestamp(),
                        'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                        'status': hex(unpacked_data[5]),
                        'serial_number': unpacked_data[6],
                        'error': hex(unpacked_data[7]),
                        'operation_mode': unpacked_data[8],
                        'fom_ahrs': unpacked_data[9],
                        'fom_fc1': unpacked_data[10],
                        'euler_angles_roll': unpacked_data[12],
                        'euler_angles_pitch': unpacked_data[13],
                        'euler_angles_heading': unpacked_data[14],
                        'quaternion_0': unpacked_data[15],
                        'quaternion_1': unpacked_data[16],
                        'quaternion_2': unpacked_data[17],
                        'quaternion_3': unpacked_data[18],
                        'dcm_11': unpacked_data[19],
                        'dcm_12': unpacked_data[20],
                        'dcm_13': unpacked_data[21],
                        'dcm_21': unpacked_data[22],
                        'dcm_22': unpacked_data[23],
                        'dcm_23': unpacked_data[24],
                        'dcm_31': unpacked_data[25],
                        'dcm_32': unpacked_data[26],
                        'dcm_33': unpacked_data[27],
                        'declination': unpacked_data[28],
                        'depth': unpacked_data[29],
                    }
                    condition = True
            except error:
                print('failed ahrs')
                condition = 'failed to unpack AHRS data'

        elif sensor_id == 0xdc:  # INS
            try:
                version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                if version == 1:
                    unpacked_data = list(unpack('<BBHIIIIIBBBBffffffffffffffffffffffffffffffffffffffffffffffffffffff', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
                        'packet_counter': packet_counter,
                        'timestamp_python': datetime.now().timestamp(),
                        'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                        'status': hex(unpacked_data[5]),
                        'serial_number': unpacked_data[6],
                        'error': hex(unpacked_data[7]),
                        'operation_mode': unpacked_data[8],
                        'fom_ahrs': unpacked_data[9],
                        'fom_fc1': unpacked_data[10],
                        'euler_angles_roll': unpacked_data[12],
                        'euler_angles_pitch': unpacked_data[13],
                        'euler_angles_heading': unpacked_data[14],
                        'quaternion_0': unpacked_data[15],
                        'quaternion_1': unpacked_data[16],
                        'quaternion_2': unpacked_data[17],
                        'quaternion_3': unpacked_data[18],
                        'dcm_11': unpacked_data[19],
                        'dcm_12': unpacked_data[20],
                        'dcm_13': unpacked_data[21],
                        'dcm_21': unpacked_data[22],
                        'dcm_22': unpacked_data[23],
                        'dcm_23': unpacked_data[24],
                        'dcm_31': unpacked_data[25],
                        'dcm_32': unpacked_data[26],
                        'dcm_33': unpacked_data[27],
                        'declination': unpacked_data[28],
                        'depth': unpacked_data[29],
                        # End of AHRS part
                        'delta_quaternion_0': unpacked_data[30],
                        'delta_quaternion_1': unpacked_data[31],
                        'delta_quaternion_2': unpacked_data[32],
                        'delta_quaternion_3': unpacked_data[33],
                        'course_over_ground': unpacked_data[34],
                        'temperature': unpacked_data[35],
                        'pressure': unpacked_data[36],
                        'altitude': unpacked_data[37],
                        'latitude': unpacked_data[38],
                        'longitude': unpacked_data[39],
                        'height': unpacked_data[40],
                        'position_frame_x': unpacked_data[41],
                        'position_frame_y': unpacked_data[42],
                        'position_frame_z': unpacked_data[43],
                        'delta_position_frame_x': unpacked_data[44],
                        'delta_position_frame_y': unpacked_data[45],
                        'delta_position_frame_z': unpacked_data[46],
                        'delta_position_uns_x': unpacked_data[47],
                        'delta_position_uns_y': unpacked_data[48],
                        'delta_position_uns_z': unpacked_data[49],
                        'velocity_ned_x': unpacked_data[50],
                        'velocity_ned_y': unpacked_data[51],
                        'velocity_ned_z': unpacked_data[52],
                        'velocity_uns_x': unpacked_data[53],
                        'velocity_uns_y': unpacked_data[54],
                        'velocity_uns_z': unpacked_data[55],
                        'delta_velocity_ned_x': unpacked_data[56],
                        'delta_velocity_ned_y': unpacked_data[57],
                        'delta_velocity_ned_z': unpacked_data[58],
                        'delta_velocity_uns_x': unpacked_data[59],
                        'delta_velocity_uns_y': unpacked_data[60],
                        'delta_velocity_uns_z': unpacked_data[61],
                        'velocity_sog': unpacked_data[62],
                        'turn_rate_x': unpacked_data[63],
                        'turn_rate_y': unpacked_data[64],
                        'turn_rate_z': unpacked_data[65],
                    }
                    condition = True
            except error:
                print('failed ins')
                condition = 'failed to unpack INS data'

        elif sensor_id == 0xf0:  # field calibration
            try:
                version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                if version == 1:
                    unpacked_data = list(unpack('<BBHIIIffffffffff', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
                        'packet_counter': packet_counter,
                        'timestamp_python': datetime.now().timestamp(),
                        'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                        'status': hex(unpacked_data[5]),
                        'hard_bias_x': unpacked_data[6],
                        'hard_bias_y': unpacked_data[7],
                        'hard_bias_z': unpacked_data[8],
                        'a_mat_11': unpacked_data[9],
                        'a_mat_12': unpacked_data[10],
                        'a_mat_13': unpacked_data[11],
                        'a_mat_21': unpacked_data[12],
                        'a_mat_22': unpacked_data[13],
                        'a_mat_23': unpacked_data[14],
                        'duration_s': unpacked_data[15]
                    }
                    condition = True
            except error:
                print('failed field_cal')
                condition = 'failed to unpack field_cal data'

        return packet, condition

    def check_package_counter(self, packet):

        packet_counter = packet['packet_counter']
        if self.packet_counter_previous + 1 < packet_counter:
            self.write_condition(error_message='lost packets between packet number ' + str(
                self.packet_counter_previous) + ' and ' + str(packet_counter), package=packet)

        self.packet_counter_previous = packet_counter

    def add_binary_package(self, binary_package, ascii_package):

        head_len = binary_package[1]
        pkg_len = head_len + binary_package[4]

        if self.check_header(binary_package):

            if self.check_packet(binary_package):
                package, condition = self.extract_packet(binary_package[:pkg_len])
            else:
                package = binary_package
                condition = 'data checksum failed'

            if condition is True:
                self.check_package_counter(package)
                self.write_package(package=package)

            else:
                self.write_condition(error_message=condition, package=binary_package[:pkg_len])

            binary_package = binary_package[pkg_len:]
            reading_package = False

        elif 0xa5 in binary_package[1:]:  # This gets evaluated if checksum fails, and checks for another wrapper byte 0xa5 within the package
            start_index = next(i + 1 for i, x in enumerate(binary_package[1:]) if x == 0xa5)
            ascii_package.extend(binary_package[:start_index])
            binary_package = binary_package[start_index:]
            reading_package = True

        else:
            self.write_condition(error_message='header checksum failed', package=binary_package[:pkg_len])
            binary_package = binary_package[pkg_len:]
            reading_package = False

        return binary_package, reading_package

    def run(self):
        """
        This is the function that is executed when the uns driver starts
        """
        reading_package = False
        binary_package = list()
        ascii_package = list()

        while self.driver_running:

            if self.uns_serial.in_waiting:

                data = self.uns_serial.read(self.uns_serial.in_waiting)

                for value in data:

                    if value == 0xa5 and not reading_package:
                        reading_package = True
                        ascii_package = list()

                    if reading_package:
                        binary_package.append(value)
                    else:
                        ascii_package.append(value)

                    if len(binary_package) > 4 and len(binary_package) >= binary_package[1] + binary_package[4]:

                        binary_package, reading_package = self.add_binary_package(binary_package, ascii_package)

                    if len(ascii_package) >= 2 and ascii_package[-2] == 0x0d and ascii_package[-1] == 0x0a:

                        self.write_ascii(ascii_package=ascii_package)
                        ascii_package = list()

            else:
                time.sleep(0.01)


def select_com_port():
    """
    Helper function to set the com port.
    """
    com_port = None

    com_info = serial.tools.list_ports.comports(include_links=False)
    ports = [com.device for com in com_info]

    if bool(ports):
        print('\nSelect com port:')

        for i, key in enumerate(ports):
            print('[' + str(i) + '] ' + str(key))

        device_selection = input("Input integer value in the range [0:" + str(len(ports) - 1) + "]: ")

        try:
            if not 0 <= int(device_selection) < len(ports):
                print('input value out of range')
                return None
        except ValueError:
            print('input value is not integer')
            return None

        com_port = ports[int(device_selection)]

    return com_port


def main():
    """
    a simple example on how to use the driver. In this case the queues are enabled and all packages are stored in their
    respective queues. The packages in this case could be obtained through the get_package(), get_condition, and
    get_queue() commands.
    """

    if not uns_driver.connect_uns():
        print('failed to connect to the UNS. exiting...')
        exit()

    uns_driver.start()

    uns_driver.start_uns()

    print('Input "data" to see how many packages that has been collected. Input "quit" to quit')
    while uns_driver.driver_running:
        try:
            command = input('command: ')

            if command == 'quit':
                uns_driver.driver_running = False
            elif command == 'data':
                print(uns_driver.package_queue.qsize())

        except KeyboardInterrupt:
            uns_driver.driver_running = False

    uns_driver.stop_uns()

    uns_driver.join(timeout=10)

    uns_driver.disconnect_uns()

    print('Ending script')


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=None, help='com port (default=None)')
    args = parser.parse_args()

    if args.port is not None:
        serial_port = args.port
    else:
        serial_port = select_com_port()

    if serial_port is None:
        print('Incorrect selection of com port. Exiting...')
        exit()

    uns_driver = UnsDriver(use_queues=True)
    uns_driver.set_serial_port(serial_port=serial_port)

    main()
