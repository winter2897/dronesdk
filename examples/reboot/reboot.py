#!/usr/bin/env python
# -*- coding: utf-8 -*-


from dronesdk import connect
import time

# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(description="Reboots vehicle")
parser.add_argument(
    "--connect",
    help="Vehicle connection target string. If not specified, SITL automatically started and used.",
)
args = parser.parse_args()

connection_string = args.connect

# Connect to the Vehicle
print("Connecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)

vehicle.reboot()
time.sleep(1)
