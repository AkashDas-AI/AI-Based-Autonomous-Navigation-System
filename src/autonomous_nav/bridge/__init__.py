"""High-fidelity simulation bridge package for autonomous navigation integration."""

from autonomous_nav.bridge.config import BridgeConfig
from autonomous_nav.bridge.contracts import (
    CoordinateFrame,
    GnssObservation,
    ImuObservation,
    LaserScanObservation,
    NavigationCommand,
    OdometryObservation,
    SimulationLifecycleState,
    SimulatorCapabilities,
    SimulatorSensorPacket,
    TimestampedRobotState,
)
from autonomous_nav.bridge.interface import SimulationBridge
from autonomous_nav.bridge.mapping import (
    control_to_nav_command,
    euler_to_quaternion,
    laser_scan_to_grid_map,
    packet_to_sensor_measurements,
    quaternion_to_euler,
    robot_state_to_3d_state,
    state_3d_to_2d_robot_state,
)
from autonomous_nav.bridge.mock_adapter import OfflineTestAdapter

__all__ = [
    "BridgeConfig",
    "CoordinateFrame",
    "GnssObservation",
    "ImuObservation",
    "LaserScanObservation",
    "NavigationCommand",
    "OdometryObservation",
    "OfflineTestAdapter",
    "SimulationBridge",
    "SimulationLifecycleState",
    "SimulatorCapabilities",
    "SimulatorSensorPacket",
    "TimestampedRobotState",
    "control_to_nav_command",
    "euler_to_quaternion",
    "laser_scan_to_grid_map",
    "packet_to_sensor_measurements",
    "quaternion_to_euler",
    "robot_state_to_3d_state",
    "state_3d_to_2d_robot_state",
]

