#!/usr/bin/env python3

"""
In this example the UNS driver is used directly and packages are obtained through the get_package() function from the
class
"""

from uns_driver import UnsDriver
import argparse


def main():

    uns_driver.start()
    uns_driver.start_uns()

    while uns_driver.driver_running:
        # This just loops while the driver is running in a separate thread
        try:
            package = uns_driver.get_package()
            if package is not None:
                print(package)
        except KeyboardInterrupt:
            uns_driver.driver_running = False
            break

    uns_driver.stop_uns()
    uns_driver.join(timeout=10)
    print('Ending script')


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=None, help='com port (default=None)')
    args = parser.parse_args()

    if args.port is None:
        print('Specify which port the UNS is connected to with the --port argument')
        exit()

    uns_driver = UnsDriver(use_queues=True)
    uns_driver.connect_uns(args.port)

    main()
