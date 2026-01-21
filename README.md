# Drone SDK Python

A Python SDK for developing powerful UAV applications using MAVLink.

## Overview

Drone SDK Python provides a high-level API to communicate with drones over MAVLink. Access telemetry, control flight modes, manage missions, and build autonomous drone applications.

**Use Cases:**
- Onboard companion computers (computer vision, path planning, 3D modeling)
- Ground station applications
- Autonomous flight systems

## Requirements

- Python 3.9+
- pymavlink >= 2.2.20
- monotonic >= 1.3

## Installation

```bash
pip install dronesdk
```

**Development install:**

```bash
git clone https://github.com/winter2897/dronesdk.git
cd dronesdk-python
pip install -e .
```

## Quick Start

```python
from dronesdk import connect

# Connect to vehicle
vehicle = connect('tcp:192.168.2.2:5763', wait_ready=True)

# Read telemetry
print(f"Mode: {vehicle.mode.name}")
print(f"Armed: {vehicle.armed}")
print(f"Location: {vehicle.location.global_frame}")
print(f"Battery: {vehicle.battery}")

# Close connection
vehicle.close()
```

## Connection Strings

| Type | Example |
|------|---------|
| UDP | `connect('127.0.0.1:14550')` |
| UDP Input | `connect('udpin:0.0.0.0:14550')` |
| TCP | `connect('tcp:127.0.0.1:5760')` |
| Serial (Linux) | `connect('/dev/ttyUSB0', baud=57600)` |
| Serial (Windows) | `connect('com14', baud=57600)` |

## Common Operations

```python
from dronesdk import connect, VehicleMode, LocationGlobalRelative

vehicle = connect('tcp:192.168.2.2:5763', wait_ready=True)

# Change mode
vehicle.mode = VehicleMode("GUIDED")

# Arm vehicle
vehicle.armed = True

# Takeoff to 10 meters
vehicle.simple_takeoff(10)

# Navigate to location
target = LocationGlobalRelative(-35.36, 149.17, 20)
vehicle.simple_goto(target)

# Access parameters
print(vehicle.parameters['THR_MIN'])
vehicle.parameters['THR_MIN'] = 100

vehicle.close()
```

## Event Listeners

```python
# Attribute listener
@vehicle.on_attribute('mode')
def on_mode_change(self, attr_name, value):
    print(f"Mode changed to: {value}")

# MAVLink message listener  
@vehicle.on_message('HEARTBEAT')
def on_heartbeat(self, name, message):
    print("Heartbeat received")
```

## Vehicle Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `mode` | `VehicleMode` | Current flight mode |
| `armed` | `bool` | Armed state |
| `location` | `Locations` | GPS location (global/local frames) |
| `attitude` | `Attitude` | Pitch, yaw, roll |
| `velocity` | `list[float]` | Velocity [vx, vy, vz] m/s |
| `heading` | `int` | Heading in degrees |
| `airspeed` | `float` | Airspeed m/s |
| `groundspeed` | `float` | Groundspeed m/s |
| `battery` | `Battery` | Battery status |
| `gps_0` | `GPSInfo` | GPS information |
| `rangefinder` | `Rangefinder` | Distance sensor |
| `ekf_ok` | `bool` | EKF health status |
| `is_armable` | `bool` | Ready to arm |
| `system_status` | `SystemStatus` | System status |
| `version` | `Version` | Autopilot version |
| `parameters` | `ParameterManager` | Vehicle parameters |
| `commands` | `CommandSequence` | Mission waypoints |
| `gimbal` | `GimbalController` | Gimbal control |
| `channels` | `Channels` | RC channels |

## Vehicle Methods

| Method | Description |
|--------|-------------|
| `simple_takeoff(alt)` | Takeoff to altitude |
| `simple_goto(location)` | Navigate to location |
| `send_mavlink(message)` | Send raw MAVLink |
| `wait_ready(*attrs, timeout)` | Wait for ready state |
| `close()` | Close connection |

## Location Types

```python
from dronesdk import LocationGlobal, LocationGlobalRelative, LocationLocal

# Absolute altitude (MSL)
loc = LocationGlobal(lat=-35.36, lon=149.17, alt=30)

# Relative altitude (above home)
loc = LocationGlobalRelative(lat=-35.36, lon=149.17, alt=20)

# Local NED frame
loc = LocationLocal(north=10.0, east=5.0, down=-15.0)
```

## Project Structure

```
dronesdk/
├── core/           # Exceptions, observer pattern, events
├── models/         # Data models (location, attitude, battery)
├── datalink/       # MAVLink communication
├── flight_control/ # Mode, arming, takeoff, navigation
├── sensors/        # Sensor data management
├── mission/        # Waypoint management
├── gimbal/         # Gimbal control
├── parameters/     # Parameter management
├── channels/       # RC channel control
├── power/          # Battery monitoring
├── health/         # EKF, system health
└── compat/         # Vehicle facade, connect()
```

## Examples

See the `examples/` directory:

- `simple_goto/` - Basic navigation
- `vehicle_state/` - Reading telemetry
- `mission_basic/` - Mission upload/download
- `drone_delivery/` - Delivery simulation
- `follow_me/` - GPS following
- `channel_overrides/` - RC channel control

## Testing

```bash
# All tests
nosetests dronesdk/test/

# Unit tests only
nosetests dronesdk/test/unit/
```

## Contributing

We welcome contributions! Please submit pull requests with tests.

## License

GNU GPL v3