from threading import Thread
import time
import serial
import serial.tools.list_ports
import queue
import argparse
import socket
from itertools import zip_longest
from datetime import datetime
from struct import unpack, error

# Sensor ID's
ID_IMU = 0x82
ID_MAGNETOMETER = 0x87
ID_BOTTOMTRACK = 0xb4
ID_WATERTRACK = 0xbe
ID_ALTIMETER = 0xaa
ID_AHRS = 0xd2
ID_FIELD_CALIBRATION = 0x8B

CONNECTION_TYPES = ['serial', 'tcp', 'udp']
UNS_UDP_IP = '192.168.2.2'  # UDP adress of ROV running on ArduSub software

# UNS commands
START = "START\r\n".encode()
STOP = "STOP\r\n".encode()
FIELDCAL = "FIELDCAL\r\n".encode()

# UNS responses
ACK = "ACK\r\n".encode()
OK = "OK\r\n".encode()
ERROR = "ERROR\r\n".encode()
START_UP = "Nortek Fusion DVL1000\r\n".encode()
FAIL = "FAIL\r\n".encode()

# communication
CRLF = "\r\n".encode()
LF = '\n'.encode()


class UnsConnect:

    def __init__(self, connection_type='serial', serial_port=None, serial_baudrate=None, tcp_ip=None, tcp_hostname=None, tcp_port=None, udp_port=None, use_queues=False, timeout=3):

        # connection variables
        self.connection_type = None
        self.set_connection_type(connection_type)
        self.uns_connected = False
        self.timeout = timeout

        # serial variables
        self.uns_serial = serial.Serial()
        self.serial_port = serial_port
        self.serial_baudrate = serial_baudrate

        # tcp variables
        self.tcp_socket = socket.socket()
        self.tcp_ip = tcp_ip
        self.tcp_hostname = tcp_hostname
        self.tcp_port = tcp_port
        self.tcp_buffer = b''

        # udp variables
        self.udp_socket = socket.socket()
        self.udp_ip = UNS_UDP_IP
        self.udp_port = udp_port

        # queue variables
        self.packet_queue = queue.Queue()
        self.ascii_queue = queue.Queue()
        self.condition_queue = queue.Queue()
        self.queuing = use_queues

    def _write_message(self, message):

        print(message)

    def _input(self) -> bytes:

        return input('> ').encode()

    def set_connection_type(self, connection_type):

        if connection_type in CONNECTION_TYPES:
            self.connection_type = connection_type

        else:
            self._write_message('UnsDriver does not support the following connection type: {}'.format(connection_type))

    def set_serial_port(self, serial_port: str):

        self.serial_port = serial_port

    def set_serial_baudrate(self, serial_baudrate: int):

        self.serial_baudrate = serial_baudrate

    def set_tcp_port(self, tcp_port: int):

        self.tcp_port = tcp_port

    def set_tcp_hostname(self, tcp_hostname: str):

        self.tcp_hostname = tcp_hostname

    def set_tcp_ip(self, tcp_ip=None):

        if tcp_ip is not None:
            self.tcp_ip = tcp_ip
        elif self.tcp_hostname is not None:
            try:
                self.tcp_ip = socket.gethostbyname(self.tcp_hostname)
            except socket.gaierror as e:
                self._write_message('Failed to obtain IP from hostname: {}'.format(e))

    def set_udp_ip(self, udp_ip: str):

        self.udp_ip = udp_ip

    def set_udp_port(self, udp_port: int):

        self.udp_port = udp_port

    def set_use_queues(self, use_queues: bool):

        self.queuing = use_queues

    def set_timeout(self, timeout: float):

        self.timeout = timeout

        if self.connection_type == 'serial':
            self.uns_serial.timeout = self.timeout

        elif self.connection_type == 'tcp':
            self.tcp_socket.settimeout(self.timeout)

        elif self.connection_type == 'udp':
            self.udp_socket.settimeout(self.timeout)

    def get_timeout(self):

        return self.timeout

    def connect_uns(self, serial_port=None, serial_baudrate=None, timeout=None, tcp_ip=None, tcp_hostname=None,
                    tcp_port=None, udp_port=None, blocking: bool = False, ignore_connection_error=False) -> bool:

        if self.uns_connected:
            self.disconnect_uns()
            if self.uns_connected:
                self._write_message('Failed to disconnect current UNS connection before reconnecting')

        if self.connection_type == 'tcp':
            self.tcp_socket = socket.socket()

        if self.connection_type == 'udp':
            self.udp_socket = socket.socket()

        if timeout is not None:
            self.set_timeout(timeout)

        if serial_port is not None:
            self.set_serial_port(serial_port=serial_port)

        if serial_baudrate is not None:
            self.set_serial_baudrate(serial_baudrate=serial_baudrate)

        if tcp_hostname is not None:
            self.set_tcp_hostname(tcp_hostname=tcp_hostname)
            self.set_tcp_ip()

        if tcp_ip is not None:
            self.set_tcp_ip(tcp_ip=tcp_ip)

        if tcp_port is not None:
            self.set_tcp_port(tcp_port=tcp_port)

        if udp_port is not None:
            self.set_udp_port(udp_port=udp_port)

        def _connect_serial():

            try:
                self.uns_serial = serial.Serial(port=self.serial_port,
                                                baudrate=self.serial_baudrate,
                                                parity=serial.PARITY_NONE,
                                                stopbits=serial.STOPBITS_ONE,
                                                timeout=self.timeout)
                time.sleep(0.1)
                self.serial_reset_buffer(buffer='both')
                condition = True

            except serial.SerialException as e:
                if not ignore_connection_error:
                    self._write_message('Failed to connect through serial with error: {}'.format(e))
                condition = False

            return condition

        def _connect_tcp():

            try:
                if self.tcp_ip is None:
                    raise Exception('TCP IP is not defined')

                if self.tcp_port is None:
                    raise Exception('TCP port is not defined')

                self.tcp_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
                self.tcp_socket.connect((self.tcp_ip, self.tcp_port))

                if blocking:
                    self.tcp_socket.settimeout(0)
                else:
                    self.tcp_socket.settimeout(self.timeout)

                condition = True

            except Exception as e:
                if not ignore_connection_error:
                    self._write_message('Failed to connect with TCP with error: {}'.format(e))
                condition = False

            return condition

        def _connect_udp():

            try:
                self.udp_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

                if blocking:
                    self.udp_socket.settimeout(0)
                else:
                    self.udp_socket.settimeout(self.timeout)

                condition = True

            except Exception as e:
                if not ignore_connection_error:
                    self._write_message('Failed to connect with UDP with error: {}'.format(e))
                condition = False

            return condition

        def _login():

            login_status = False

            if not self.uns_connected:
                self._write_message("UNS not connected")
                return login_status

            login = self.uns_read()

            if not b'Please enter password:\r\n' in login:
                raise Exception('Did not receive login message when connecting to TCP')

            while b'Please enter password:\r\n' in login:
                self._write_message(login.decode())
                #password = self._input()
                password = "nortek".encode()
                self.uns_write(password + b'\r\n')
                login = self.uns_read()
                if login in b'Welcome to Nortek Fusion DVL1000\r\n':
                    self._write_message(login.decode())
                    login_status = True
                    break
                else:
                    self._write_message('Incorrect password')

            return login_status

        if self.connection_type == 'serial':
            self.uns_connected = _connect_serial()

        if self.connection_type == 'tcp':
            self.uns_connected = _connect_tcp()
            if not _login():
                self.disconnect_uns()

        if self.connection_type == 'udp':
            self.uns_connected = _connect_udp()

        return self.uns_connected

    def disconnect_uns(self) -> bool:

        def _disconnect_serial():

            if self.uns_serial.is_open:
                try:
                    self.uns_serial.close()
                except Exception as e:
                    self._write_message('Exception occurred when disconnecting from serial: {}'.format(e))

        def _disconnect_tcp():

            try:
                self.tcp_socket.close()
            except Exception as e:
                self._write_message('Exception occurred when disconnecting from TCP: {}'.format(e))

        def _disconnect_udp():

            try:
                self.udp_socket.close()
            except Exception as e:
                self._write_message('Exception occurred when disconnecting from UDP: {}'.format(e))

        if self.connection_type == 'serial':
            self.uns_connected = False
            _disconnect_serial()

        elif self.connection_type == 'tcp':
            self.uns_connected = False
            _disconnect_tcp()

        elif self.connection_type == 'udp':
            self.uns_connected = False
            _disconnect_udp()

        return self.uns_connected

    def login_tcp(self):

        login_status = False

        if not self.uns_connected:
            self._write_message("UNS not connected")
            return login_status

        login = self.uns_read()

        if not b'Please enter password:\r\n' in login:
            raise Exception('Did not receive login message when connecting to TCP: {}'.format(login))

        while b'Please enter password:\r\n' in login:
            self._write_message(login.decode())
            password = "nortek".encode()
            self.uns_write(password + b'\r\n')
            login = self.uns_read()
            if b'Welcome to Nortek Fusion DVL1000\r\n' in login:
                self._write_message(login.decode())
                login_status = True
                break
            else:
                self._write_message('Incorrect password')

        return login_status

    def serial_reset_buffer(self, buffer: str):

        if self.connection_type != 'serial' and self.connection_type != 'tcp':
            return

        try:
            if self.connection_type == 'serial':
                if buffer == 'input' or buffer == 'both':
                    self.uns_serial.reset_input_buffer()

                if buffer == 'output' or buffer == 'both':
                    self.uns_serial.reset_output_buffer()

            elif self.connection_type == 'tcp':
                self.tcp_buffer = b''

            time.sleep(0.1)  # wait for reset to be executed

        except Exception as e:
            self._write_message('Failed to reset buffer: {}'.format(e))

    def uns_write(self, command: bytes):

        if not self.uns_connected:
            self._write_message("UNS not connected")
            return

        def _send_serial_command():

            try:
                self.uns_serial.write(command)
            except Exception as e:
                print('sending serial command to UNS failed: {}'.format(e))
                pass

        def _send_tcp_command():

            try:

                self.tcp_socket.sendall(command)
            except Exception as e:
                print('sending TCP command to UNS failed: {}'.format(e))
                pass

        def _send_udp_command():

            try:
                self.udp_socket.sendto(command, (self.udp_ip, self.udp_port))
            except Exception as e:
                print('sending UDP command to UNS failed: {}'.format(e))
                pass

        if self.connection_type == 'serial':
            _send_serial_command()

        elif self.connection_type == 'udp':
            _send_udp_command()

        elif self.connection_type == 'tcp':
            _send_tcp_command()

    def uns_read(self, size=None, terminator=None, timeout=None) -> bytes:

        data = b''

        def _serial_read() -> bytes:

            if timeout is not None:
                self.uns_serial.timeout = timeout

            serial_data = b''

            try:
                if size is None and terminator is None:
                    if self.uns_serial.in_waiting:
                        serial_data = self.uns_serial.read(self.uns_serial.in_waiting)

                elif terminator is not None:
                    #serial_data = self.uns_serial.read_until(expected=terminator, size=size)
                    serial_data = self.uns_serial.read_until(terminator, size)

                elif size is not None:
                    serial_data = self.uns_serial.read(size=size)

                if isinstance(serial_data, str):
                    serial_data = serial_data.encode()

            except Exception as e:
                if self.uns_connected:
                    self._write_message('Serial read error: {}'.format(e))

            self.uns_serial.timeout = self.timeout

            return serial_data

        def _tcp_read() -> bytes:

            tcp_data = b''

            if timeout is not None:
                self.tcp_socket.settimeout(timeout)

            try:
                data = self.tcp_socket.recv(4096)

                # TODO: Deal with no data received

                self.tcp_buffer += data

                if terminator is not None:
                    line, separator, self.tcp_buffer = self.tcp_buffer.partition(terminator)
                    tcp_data = line + separator

                elif size is not None:
                    tcp_data = self.tcp_buffer[:size]
                    self.tcp_buffer = self.tcp_buffer[size:]

                else:
                    tcp_data = self.tcp_buffer
                    self.tcp_buffer = b''

            except socket.timeout:
                pass
            except Exception as e:
                if self.uns_connected:
                    self._write_message('TCP read error: {}'.format(e))

            if timeout is not None:
                self.tcp_socket.settimeout(self.timeout)

            return tcp_data

        def _udp_read() -> bytes:

            udp_data = b''

            if timeout is not None:
                self.udp_socket.settimeout(timeout)

            try:
                if terminator is not None:
                    for i in range(4096):
                        udp_data += self.udp_socket.recv(1)
                        if terminator in udp_data:
                            break
                elif size is not None:
                    udp_data = self.udp_socket.recv(size)
                else:
                    udp_data = self.udp_socket.recv(4096)
            except socket.timeout:
                pass
            except Exception as e:
                if self.uns_connected:
                    self._write_message('UDP read error: {}'.format(e))

            if timeout is not None:
                self.udp_socket.settimeout(self.timeout)

            return udp_data

        if self.connection_type == 'serial':
            data = _serial_read()

        elif self.connection_type == 'tcp':
            data = _tcp_read()

        elif self.connection_type == 'udp':
            data = _udp_read()

        return data

    def uns_readline(self, timeout=None) -> bytes:

        return self.uns_read(terminator=LF, timeout=timeout)


class UnsDriver(UnsConnect):

    def __init__(self, connection_type='serial', serial_port=None, serial_baudrate=None, tcp_ip=None, tcp_hostname=None, tcp_port=None, udp_port=None, use_queues=False, timeout=3):

        UnsConnect.__init__(self, connection_type=connection_type, serial_port=serial_port, serial_baudrate=serial_baudrate, tcp_ip=tcp_ip, tcp_hostname=tcp_hostname, tcp_port=tcp_port, udp_port=udp_port, use_queues=use_queues, timeout=timeout)

        # operation variables
        self.log_init_time = datetime.now()
        self.driver_running = True

    def start_uns(self):

        self.uns_write(START)

    def field_calibration(self):

        self.uns_write(FIELDCAL)

    def stop_uns(self):

        self.uns_write(STOP)


    def write_packet(self, packet):

        if self.queuing:
            self.packet_queue.put(packet)

    def read_packet(self, timeout=None):

        packet = None

        try:
            if timeout is not None:
                packet = self.packet_queue.get(timeout=timeout)

            elif not self.packet_queue.empty():
                packet = self.packet_queue.get_nowait()

        except Exception as e:
            print('failed to retrieve packet from packet queue: {}'.format(e))
            pass

        return packet

    def write_condition(self, error_message, packet):

        failed_packet = {'timestamp_python': datetime.now().timestamp() - self.log_init_time.timestamp(),
                          'error_message': error_message,
                          'failed_packet': packet}

        if self.queuing:
            self.condition_queue.put(failed_packet)

    def read_condition(self, timeout=None):

        packet = None

        try:
            if timeout is not None:
                packet = self.condition_queue.get(timeout=timeout)

            elif not self.condition_queue.empty():
                packet = self.condition_queue.get_nowait()

        except Exception as e:
            print('failed to retrieve packet from condition queue: {}'.format(e))
            pass

        return packet

    def write_ascii(self, ascii_packet):

        ascii_string = self._data_to_ascii(ascii_packet)
        ascii_packet = {'message': ascii_string}

        if self.queuing:
            self.ascii_queue.put(ascii_packet)

    def read_ascii(self, timeout=None):

        packet = None

        try:
            if timeout is not None:
                packet = self.ascii_queue.get(timeout=timeout)

            elif not self.ascii_queue.empty():
                packet = self.ascii_queue.get_nowait()

        except Exception as e:
            print('failed to retrieve packet from ascii queue: {}'.format(e))
            pass

        return packet

    def clear_queue(self, queue_name):

        if queue_name == 'packet' or queue_name == 'all':
            for i in range(self.packet_queue.qsize()):
                self.packet_queue.get_nowait()

        if queue_name == 'condition' or queue_name == 'all':
            for i in range(self.condition_queue.qsize()):
                self.condition_queue.get_nowait()

        if queue_name == 'ascii' or queue_name == 'all':
            for i in range(self.ascii_queue.qsize()):
                self.ascii_queue.get_nowait()

    @staticmethod
    def _data_to_ascii(data):

        ascii_string = ''.join(chr(i) for i in data if 0 <= i <= 0x7e)

        return ascii_string

    @staticmethod
    def checksum(packet):

        checksum = 0xb58c

        for u, v in zip_longest(packet[::2], packet[1::2], fillvalue=None):

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

    def extract_packet(self, data):

        header_size = data[1]
        id = data[2]

        sensor_data, condition = self.get_sensor_data(id, data[header_size:])

        return sensor_data, condition

    @staticmethod
    def get_sensor_data(sensor_id, data):

        packet = None
        condition = 'packet id not recognized'

        if sensor_id == ID_IMU:  # IMU
            try:
                version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                if version == 1:
                    unpacked_data = list(unpack('<BBHIIIfffffff', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
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

        elif sensor_id == ID_MAGNETOMETER:  # Magnetometer
            try:
                version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                if version == 1:
                    unpacked_data = list(unpack('<BBHIIIfff', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
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

        elif sensor_id in (ID_BOTTOMTRACK, ID_WATERTRACK):
            try:
                version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                if version == 1:
                    unpacked_data = list(unpack('<BBHIIIIIffffffffffffffffffffffffff', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
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

        elif sensor_id == ID_ALTIMETER:  # Altimeter
            try:
                version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                if version == 1:
                    unpacked_data = list(unpack('<BBHIIIIIffffH', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
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

        elif sensor_id == ID_AHRS:  # AHRS
            try:
                version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                if version == 1:
                    unpacked_data = list(unpack('<BBHIIIIIBBBBffffffffffffffffff', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
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
                elif version == 2:
                    unpacked_data = list(unpack('<BBHIIIIIBBBBffffffffffffffffffff', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
                        'timestamp_python': datetime.now().timestamp(),
                        'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                        'status': hex(unpacked_data[5]),
                        'serial_number': unpacked_data[6],
                        'error': hex(unpacked_data[7]),
                        'operation_mode': unpacked_data[8],
                        'fom_ahrs': unpacked_data[12],
                        'fom_fc1': unpacked_data[13],
                        'euler_angles_roll': unpacked_data[14],
                        'euler_angles_pitch': unpacked_data[15],
                        'euler_angles_heading': unpacked_data[16],
                        'quaternion_0': unpacked_data[17],
                        'quaternion_1': unpacked_data[18],
                        'quaternion_2': unpacked_data[19],
                        'quaternion_3': unpacked_data[20],
                        'dcm_11': unpacked_data[21],
                        'dcm_12': unpacked_data[22],
                        'dcm_13': unpacked_data[23],
                        'dcm_21': unpacked_data[24],
                        'dcm_22': unpacked_data[25],
                        'dcm_23': unpacked_data[26],
                        'dcm_31': unpacked_data[27],
                        'dcm_32': unpacked_data[28],
                        'dcm_33': unpacked_data[29],
                        'declination': unpacked_data[30],
                        'depth': unpacked_data[31],
                    }
                    condition = True
            except error:
                print('failed ahrs')
                condition = 'failed to unpack AHRS data'

        elif sensor_id == ID_FIELD_CALIBRATION:  # field calibration
            try:
                version = unpack('<B', data[0].to_bytes(1, 'little'))[0]
                if version == 1:
                    unpacked_data = list(unpack('<BBHIIIfffffffffffffffff', bytearray(data)))
                    packet = {
                        'id': hex(sensor_id),
                        'timestamp_python': datetime.now().timestamp(),
                        'timestamp': unpacked_data[3] + unpacked_data[4] * 1e-6,
                        'status': hex(unpacked_data[5]),
                        'hard_iron_x': unpacked_data[6],
                        'hard_iron_y': unpacked_data[7],
                        'hard_iron_z': unpacked_data[8],
                        's_axis_00': unpacked_data[9],
                        's_axis_01': unpacked_data[10],
                        's_axis_02': unpacked_data[11],
                        's_axis_10': unpacked_data[12],
                        's_axis_11': unpacked_data[13],
                        's_axis_12': unpacked_data[14],
                        's_axis_20': unpacked_data[15],
                        's_axis_21': unpacked_data[16],
                        's_axis_22': unpacked_data[17],
                        'new_point_0': unpacked_data[18],
                        'new_point_1': unpacked_data[19],
                        'new_point_2': unpacked_data[20],
                        'fom': unpacked_data[21],
                        'coverage': unpacked_data[22],
                    }
                    condition = True
            except error:
                print('failed field_cal')
                condition = 'failed to unpack field_cal data'


        return packet, condition

    def add_binary_packet(self, binary_packet, ascii_packet):

        head_len = binary_packet[1]
        pkg_len = head_len + binary_packet[4]

        if self.check_header(binary_packet[:pkg_len]):

            if self.check_packet(binary_packet[:pkg_len]):
                packet, condition = self.extract_packet(binary_packet[:pkg_len])
            else:
                packet = binary_packet[:pkg_len]
                condition = 'data checksum failed'

            if condition is True:
                self.write_packet(packet=packet)

            else:
                self.write_condition(error_message=condition, packet=packet)

            binary_packet = binary_packet[pkg_len:]
            reading_packet = False

        elif 0xa5 in binary_packet[1:]:
            start_index = next(i + 1 for i, x in enumerate(binary_packet[1:]) if x == 0xa5)
            ascii_packet.extend(binary_packet[:start_index])
            binary_packet = binary_packet[start_index:]
            reading_packet = True

        else:
            self.write_condition(error_message='header checksum failed', packet=binary_packet[:pkg_len])
            binary_packet = binary_packet[pkg_len:]
            reading_packet = False

        return binary_packet, reading_packet

    def add_ascii_packet(self, ascii_packet):

        self.write_ascii(ascii_packet=ascii_packet)

    @staticmethod
    def get_com_ports():

        com_info = serial.tools.list_ports.comports(include_links=False)
        ports = [com.device for com in com_info]

        return ports

    @staticmethod
    def select_com_port():

        com_port = None

        ports = UnsDriver.get_com_ports()

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


class UnsDriverThread(UnsDriver, Thread):

    def __init__(self, connection_type='serial', serial_port=None, serial_baudrate=None, tcp_ip=None, tcp_hostname=None, tcp_port=None, udp_port=None, use_queues=False, timeout=3):

        UnsDriver.__init__(self, connection_type=connection_type, serial_port=serial_port, serial_baudrate=serial_baudrate, tcp_ip=tcp_ip, tcp_hostname=tcp_hostname, tcp_port=tcp_port, udp_port=udp_port, use_queues=use_queues, timeout=timeout)
        Thread.__init__(self)

    def stop(self):

        self.driver_running = False

    def run(self):

        reading_packet = False
        binary_packet = list()
        ascii_packet = list()
        self.driver_running = True

        while self.driver_running:

            if not self.uns_connected:
                time.sleep(0.1)
                continue

            data = self.uns_read()

            if data:

                for value in data:

                    if value == 0xa5 and not reading_packet:
                        reading_packet = True
                        ascii_packet = list()

                    if reading_packet:
                        binary_packet.append(value)
                    else:
                        ascii_packet.append(value)

                    if len(binary_packet) > 5 and len(binary_packet) >= binary_packet[1] + (binary_packet[4] & 0xff) | ((binary_packet[5] & 0xff) << 8):

                        binary_packet, reading_packet = self.add_binary_packet(binary_packet, ascii_packet)

                    if len(ascii_packet) >= 2 and ascii_packet[-2] == 0x0d and ascii_packet[-1] == 0x0a:

                        self.add_ascii_packet(ascii_packet)
                        ascii_packet = list()

            else:
                time.sleep(0.01)


def main():

    if not uns_driver.connect_uns():
        print('failed to connect to the UNS. exiting...')
        exit()

    uns_driver.start()

    while uns_driver.driver_running:
        try:
            command = input('> ')

            if command == 'quit':
                uns_driver.stop()
            elif command == 'start':
                uns_driver.start_uns()
            elif command == 'stop':
                uns_driver.stop_uns()
            elif command == 'data':
                print(uns_driver.packet_queue.qsize())
            else:
                command = command + '\r\n'
                uns_driver.uns_write(command.encode())
                response = uns_driver.uns_read()
                print(response)

        except KeyboardInterrupt:
            uns_driver.driver_running = False

    uns_driver.join(timeout=10)

    uns_driver.disconnect_uns()

    print('Ending script')


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--connection-type', default='serial', help='connection type (default=serial)')
    parser.add_argument('--serial-port', default=None, help='serial com port (default=None)')
    parser.add_argument('--serial-baudrate', default=None, help='serial baudrate (default=None)')
    parser.add_argument('--tcp-hostname', default=None, help='tcp hostname (default=None)')
    parser.add_argument('--tcp-port', default=None, help='tcp port (default=None)')
    parser.add_argument('--udp-port', default=None, help='udp port (default=None)')
    args = parser.parse_args()

    connection_type = args.connection_type

    if args.serial_port is not None and connection_type == 'serial':
        serial_port = args.serial_port
    elif connection_type == 'serial':
        serial_port = UnsDriver.select_com_port()
    else:
        serial_port = None

    if args.serial_baudrate is not None and connection_type == 'serial':
        serial_baudrate = int(args.serial_baudrate)
    elif connection_type == 'serial':
        serial_baudrate = int(input('Input baudrate: '))
    else:
        serial_baudrate = None

    if args.tcp_hostname is not None and connection_type == 'tcp':
        tcp_hostname = args.tcp_hostname
    elif connection_type == 'tcp':
        tcp_hostname = input('Input TCP hostname: ')
    else:
        tcp_hostname = None

    if args.tcp_port is not None and connection_type == 'tcp':
        tcp_port = int(args.tcp_port)
    elif connection_type == 'tcp':
        tcp_port = int(input('Input TCP port: '))
    else:
        tcp_port = None

    if args.udp_port is not None and connection_type == 'udp':
        udp_port = int(args.udp_port)
    elif connection_type == 'udp':
        udp_port = int(input('Input UDP port: '))
    else:
        udp_port = None

    if serial_port is None and udp_port is None and tcp_hostname is None:
        print('Neither serial port or udp port is specified. Exiting...')
        exit()

    uns_driver = UnsDriverThread(connection_type=connection_type, serial_port=serial_port, serial_baudrate=serial_baudrate, tcp_hostname=tcp_hostname, tcp_port=tcp_port, udp_port=udp_port, use_queues=True)

    if uns_driver.tcp_ip is None and uns_driver.tcp_hostname is not None:
        uns_driver.set_tcp_ip()

    main()
