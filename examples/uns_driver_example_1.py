#!/usr/bin/env python3

"""
In this example the UNS driver is inherited into a new class where it is modified in order to better organize how the
extracted packages from the UNS are handled.
"""

from uns_driver import UnsDriver
import serial
import argparse
import time


class MyUnsDriver(UnsDriver):

    def __init__(self, print_packages=False, serial_connection=None, use_queues=False):
        UnsDriver.__init__(self, serial_connection=serial_connection, use_queues=use_queues)

        self.print_packages = print_packages

        self.previous_timestamp = 1e10

    def write_package(self, package):
        """
        This function is executed whenever a package with sensor data is extracted from the UNS data stream. Overwriting
        this function in this class allow a user to handle these packages as they see fit. In this case they are simply
        printed to the terminal
        """
        if self.print_packages:
            print(package)

    def write_condition(self, error_message, package):
        """
        This function is executed whenever an error occurs either in the UNS or with the parsing of its data. Overwriting
        this function in this class allow a user to handle these packages as they see fit. In this case they are simply
        printed to the terminal
        """
        if self.print_packages:
            print(error_message)

    def write_ascii(self, ascii_package):
        """
        This function is executed whenever an ascii string is sent from the UNS. Overwriting his function in this class
        allow a user to handle these packages as they see fit. In this case they are simply printed to the terminal
        """
        if self.print_packages:
            print(ascii_package)


def main():

    my_uns_driver.start()
    my_uns_driver.start_uns()

    while my_uns_driver.driver_running:
        # This just loops while the driver is running in a separate thread
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            my_uns_driver.driver_running = False
            break

    my_uns_driver.stop_uns()
    my_uns_driver.join(timeout=10)
    print('Ending script')


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=None, help='com port (default=None)')
    args = parser.parse_args()

    if args.port is None:
        print('Specify which port the UNS is connected to with the --port argument')
        exit()

    # Defining the serial connection outside of the class because you can
    ser = serial.Serial(port=args.port,
                        baudrate=115200,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=1)

    my_uns_driver = MyUnsDriver(print_packages=True, serial_connection=ser, use_queues=False)

    main()
