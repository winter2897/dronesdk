# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

dronesdk-Python is a Python API library for communicating with UAVs/drones over the MAVLink protocol. It provides access to telemetry, state, parameters, mission management, and direct vehicle control. Primarily used for onboard companion computers and ground station applications.

## Build and Test Commands

```bash
# Install in development mode
pip install -e .

# Install dependencies
pip install -r requirements.txt

# Run all tests (uses nose framework)
nosetests dronesdk/test/

# Run unit tests only
nosetests dronesdk/test/unit/

# Run SITL (Software-in-the-Loop) tests
nosetests dronesdk/test/sitl/

# Run a specific test file
nosetests dronesdk/test/unit/test_api.py

# Run a specific test
nosetests dronesdk/test/unit/test_api.py:TestClassName.test_method_name
```

## Architecture

### Core Module Structure

- `dronesdk/__init__.py` - Main API containing all public classes (~3200 lines)
- `dronesdk/mavlink.py` - MAVLink protocol communication layer
- `dronesdk/util.py` - Logging utilities

### Key Classes

**Vehicle** (`__init__.py:997`) - Primary API class for UAV interaction
- Manages vehicle state, telemetry, parameters, and commands
- Key attributes: `mode`, `armed`, `location`, `attitude`, `velocity`, `battery`, `parameters`
- Key methods: `simple_takeoff()`, `simple_goto()`, `send_mavlink()`, `wait_ready()`

**HasObservers** (`__init__.py:560`) - Base class providing observer pattern
- Used by Vehicle and Parameters classes
- Methods: `add_attribute_listener()`, `on_attribute()` decorator, `notify_attribute_listeners()`

**Location Classes**:
- `LocationGlobal` - WGS84 coordinates with MSL altitude
- `LocationGlobalRelative` - Global coordinates with altitude relative to home
- `LocationLocal` - Local NED frame (North, East, Down)

**Control Classes**:
- `Parameters` - MutableMapping for vehicle parameters (case-insensitive keys)
- `CommandSequence` - Mission waypoint management
- `Channels`/`ChannelsOverride` - RC channel reading/override

### Connection Entry Point

`connect()` function (`__init__.py:3130`) - Main entry point for vehicle connections
```python
from dronesdk import connect, VehicleMode, LocationGlobalRelative
vehicle = connect('127.0.0.1:14550', wait_ready=True)
```

### Design Patterns

**Observer Pattern** - Two styles for state change notifications:
```python
# Callback style
vehicle.add_attribute_listener('mode', callback_fn)

# Decorator style
@vehicle.on_attribute('mode')
def mode_callback(self, attr_name, value):
    pass

# Message listener
@vehicle.on_message('ATTITUDE')
def attitude_callback(self, name, message):
    pass
```

**Lazy Initialization** - `wait_ready()` ensures attributes are populated before use. Default wait: `['parameters', 'gps_0', 'armed', 'mode', 'attitude']`

### Dependencies

- `pymavlink>=2.2.20` - MAVLink protocol implementation
- `monotonic>=1.3` - Monotonic time tracking

### Logging

Two loggers available:
- `logging.getLogger('dronesdk')` - dronesdk operations
- `logging.getLogger('autopilot')` - Autopilot messages

## Examples

The `examples/` directory contains 16 working examples including simple_goto, vehicle_state, mission_basic, drone_delivery, follow_me, and more. Each demonstrates specific API usage patterns.
