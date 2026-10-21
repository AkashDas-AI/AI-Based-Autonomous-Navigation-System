"""Configuration specifications for the high-fidelity simulation bridge."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class BridgeConfig:
    """Configuration parameters and safety limits for the simulation bridge.

    Attributes:
        allowed_frames: Set of valid coordinate frame identifiers accepted by the bridge.
        min_velocity: Minimum allowed commanded forward velocity in m/s.
        max_velocity: Maximum allowed commanded forward velocity in m/s.
        max_angular_velocity: Maximum allowed commanded yaw rate in rad/s.
        clamp_commands: If True, commands exceeding bounds are clamped; if False, ValueError is raised.
        strict_timestamp_monotonicity: If True, time-travel or duplicate step timestamps raise exceptions.
        laser_angle_min: Start angle of planar laser scan in radians (default -pi/2, 90 deg right).
        laser_angle_max: End angle of planar laser scan in radians (default +pi/2, 90 deg left).
        laser_num_rays: Number of discrete angular range samples in laser scan (default 181 for 1 deg res).
        laser_range_min: Minimum valid LiDAR return distance in meters.
        laser_range_max: Maximum range of LiDAR sensor in meters.
        laser_noise_std: Additive Gaussian noise on range returns in meters.
        gnss_update_interval: Interval between valid GNSS fixes in seconds (default 1.0s / 1 Hz).
        timeout_sec: Maximum time to wait for simulator connection responses in seconds.
    """
    allowed_frames: Tuple[str, ...] = (
        "world",
        "map",
        "odom",
        "base_link",
        "base_laser",
        "base_imu",
        "base_gnss",
    )
    min_velocity: float = 0.0
    max_velocity: float = 2.0
    max_angular_velocity: float = 1.5
    clamp_commands: bool = True
    strict_timestamp_monotonicity: bool = True
    laser_angle_min: float = -1.5707963267948966  # -pi/2
    laser_angle_max: float = 1.5707963267948966   # +pi/2
    laser_num_rays: int = 181
    laser_range_min: float = 0.1
    laser_range_max: float = 10.0
    laser_noise_std: float = 0.02
    gnss_update_interval: float = 1.0
    timeout_sec: float = 5.0

