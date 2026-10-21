"""Abstract bridge interface defining simulation communication contracts."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from autonomous_nav.bridge.contracts import (
    NavigationCommand,
    SimulationLifecycleState,
    SimulatorCapabilities,
    SimulatorSensorPacket,
    TimestampedRobotState,
)


class SimulationBridge(ABC):
    """Abstract interface decoupling autonomous navigation from external 3D simulators.

    Adheres to dependency inversion: the navigation stack interacts exclusively
    through this protocol, enabling zero-code changes whether connecting to a
    deterministic offline test harness, ROS 2 Nav2 node graph, Gazebo, or CARLA.
    """

    @abstractmethod
    def connect(self, timeout_sec: Optional[float] = None) -> bool:
        """Establish transport connection to the simulator process/socket.

        Returns:
            bool: True if connection succeeded and handshake verified; False otherwise.
        """
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Gracefully terminate connection and release simulator resources."""
        pass

    @abstractmethod
    def initialize(self, initial_state: Optional[TimestampedRobotState] = None) -> bool:
        """Prepare the simulation environment, configure sensors, and place robot.

        Args:
            initial_state: Optional starting 3D pose and velocities.

        Returns:
            bool: True if environment was initialized cleanly; False on failure.
        """
        pass

    @abstractmethod
    def step(self, dt: float) -> bool:
        """Advance the simulation time by discrete delta dt (synchronous lockstep).

        Args:
            dt: Simulation timestep in seconds.

        Returns:
            bool: True if step completed and sensor buffers refreshed; False on simulation error.
        """
        pass

    @abstractmethod
    def reset(self, initial_state: Optional[TimestampedRobotState] = None) -> bool:
        """Reset the simulation world and vehicle state to initial conditions.

        Args:
            initial_state: Optional starting 3D pose and velocities.

        Returns:
            bool: True if reset succeeded; False otherwise.
        """
        pass

    @abstractmethod
    def send_command(self, command: NavigationCommand) -> bool:
        """Dispatch a validated actuator control command to the simulator vehicle.

        Args:
            command: NavigationCommand containing linear and angular velocity setpoints.

        Returns:
            bool: True if command was received and accepted by simulator; False otherwise.
        """
        pass

    @abstractmethod
    def get_sensor_observations(self) -> SimulatorSensorPacket:
        """Retrieve the latest synchronized packet of noisy sensor observations.

        Returns:
            SimulatorSensorPacket: Contains odometry, IMU, GNSS, and range scan data.
        """
        pass

    @abstractmethod
    def get_ground_truth_state(self) -> TimestampedRobotState:
        """Retrieve true physical state of the vehicle from simulator physics engine.

        CRITICAL ARCHITECTURAL INVARIANT: Ground truth is isolated for evaluation
        and validation; it MUST NOT be used as sensor feedback for navigation.

        Returns:
            TimestampedRobotState: Exact physical 3D pose and velocity.
        """
        pass

    @abstractmethod
    def get_capabilities(self) -> SimulatorCapabilities:
        """Query simulator capabilities, supported frames, sensors, and health."""
        pass

    @abstractmethod
    def get_lifecycle_state(self) -> SimulationLifecycleState:
        """Query current lifecycle state of the bridge connection."""
        pass

    @abstractmethod
    def is_healthy(self) -> bool:
        """Check if bridge transport and simulation engine are operational."""
        pass

