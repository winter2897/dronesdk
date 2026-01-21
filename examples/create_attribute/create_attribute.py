#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronesdk import connect, Vehicle
from my_vehicle import MyVehicle  # Our custom vehicle class
import time


# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(
    description="Demonstrates how to create attributes from MAVLink messages. "
)
parser.add_argument(
    "--connect",
    help="Vehicle connection target string. If not specified, SITL automatically started and used.",
)
args = parser.parse_args()

connection_string = args.connect

# Connect to the Vehicle
print("Connecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True, vehicle_class=MyVehicle)

# Add observer for the custom attribute


def raw_imu_callback(self, attr_name, value):
    # attr_name == 'raw_imu'
    # value == vehicle.raw_imu
    print(value)


vehicle.add_attribute_listener("raw_imu", raw_imu_callback)

print("Display RAW_IMU messages for 5 seconds and then exit.")
time.sleep(5)

# The message listener can be unset using ``vehicle.remove_message_listener``

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
