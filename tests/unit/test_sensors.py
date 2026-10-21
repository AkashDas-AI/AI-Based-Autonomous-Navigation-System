"""Unit tests for sensor simulation models (Odometry, IMU, GNSS, SensorSuite)."""

import math
import numpy as np
import pytest

from autonomous_nav.sensors.sensor_models import (
    GNSSReceiver,
    IMUSensor,
    SensorMeasurements,
    SensorSuite,
    WheelOdometrySensor,
)
from autonomous_nav.simulation.robot import RobotState


def test_wheel_odometry_noise_distribution():
    """Verify wheel odometry produces zero-mean Gaussian noise matching configured standard deviation."""
    noise_std = 0.05
    odom = WheelOdometrySensor(noise_std=noise_std, scale_factor_error=1.0)
    rng = np.random.RandomState(42)

    true_v = 1.5
    samples = [odom.sample(true_v, rng) for _ in range(5000)]
    mean_v = np.mean(samples)
    std_v = np.std(samples)

    assert abs(mean_v - true_v) < 0.01
    assert abs(std_v - noise_std) < 0.005


def test_wheel_odometry_scale_factor():
    """Verify systematic wheel scale factor alters the base measurement."""
    scale = 1.05
    odom = WheelOdometrySensor(noise_std=0.0, scale_factor_error=scale)
    rng = np.random.RandomState(42)

    true_v = 2.0
    measured_v = odom.sample(true_v, rng)
    assert abs(measured_v - (true_v * scale)) < 1e-9


def test_imu_gyro_noise_and_bias_drift():
    """Verify IMU rate gyro exhibits white noise and slow random-walk bias drift."""
    rate_noise_std = 0.02
    bias_init = 0.03
    bias_drift_std = 0.005
    imu = IMUSensor(
        rate_noise_std=rate_noise_std,
        bias_init=bias_init,
        bias_drift_std=bias_drift_std,
    )
    rng = np.random.RandomState(42)

    assert imu.bias == pytest.approx(0.03)

    # Over 500 steps of 0.1s, bias should evolve as random walk
    dt = 0.1
    biases = []
    readings = []
    true_omega = 0.5
    for _ in range(500):
        val = imu.sample(true_omega, dt, rng)
        biases.append(imu.bias)
        readings.append(val)

    # Bias should have drifted from initial value
    assert abs(biases[-1] - bias_init) > 0.001
    # Average reading should reflect true_omega + average bias
    avg_bias = np.mean(biases)
    assert abs(np.mean(readings) - (true_omega + avg_bias)) < 0.02

    # Reset should restore initial bias
    imu.reset()
    assert imu.bias == pytest.approx(0.03)


def test_gnss_sample_rate_decimation():
    """Verify GNSS emits fixes strictly at configured time intervals (e.g. 1 Hz vs 10 Hz sim)."""
    update_interval = 1.0
    gnss = GNSSReceiver(pos_noise_std=0.2, update_interval=update_interval)
    rng = np.random.RandomState(42)

    dt = 0.1
    fix_times = []
    for step in range(35):  # 3.5 seconds
        t = round(step * dt, 2)
        meas, valid = gnss.sample(true_x=10.0, true_y=5.0, current_time=t, rng=rng)
        if valid:
            assert meas is not None
            fix_times.append(t)

    # Fixes should occur at 0.0, 1.0, 2.0, 3.0
    assert len(fix_times) == 4
    for i in range(1, len(fix_times)):
        assert abs((fix_times[i] - fix_times[i - 1]) - update_interval) < 1e-4


def test_gnss_outage_window():
    """Verify GNSS position fixes are suppressed during outage intervals."""
    outages = [(1.5, 3.5)]
    gnss = GNSSReceiver(pos_noise_std=0.2, update_interval=1.0, outages=outages)
    rng = np.random.RandomState(42)

    dt = 0.1
    valid_times = []
    for step in range(50):  # 5.0 seconds
        t = round(step * dt, 2)
        meas, valid = gnss.sample(true_x=5.0, true_y=5.0, current_time=t, rng=rng)
        if valid:
            valid_times.append(t)

    # At t=0.0 and t=1.0, valid; t=2.0 and t=3.0 fall within [1.5, 3.5], so suppressed; t=4.0 valid
    assert 0.0 in valid_times
    assert 1.0 in valid_times
    assert 2.0 not in valid_times
    assert 3.0 not in valid_times
    assert 4.0 in valid_times


def test_sensor_suite_determinism():
    """Verify SensorSuite seeded determinism produces bitwise identical measurement streams."""
    suite_a = SensorSuite(seed=12345)
    suite_b = SensorSuite(seed=12345)

    robot = RobotState(x=5.0, y=10.0, theta=0.5, v=1.2, omega=0.2)
    stream_a = [suite_a.sample(robot, dt=0.1, current_time=i * 0.1) for i in range(20)]
    stream_b = [suite_b.sample(robot, dt=0.1, current_time=i * 0.1) for i in range(20)]

    for a, b in zip(stream_a, stream_b):
        assert a.timestamp == b.timestamp
        assert a.v_odom == b.v_odom
        assert a.omega_imu == b.omega_imu
        assert a.gnss_valid == b.gnss_valid
        assert a.gnss_pos == b.gnss_pos


def test_sensor_suite_reset_reproducibility():
    """Verify reset() restores deterministic stream from beginning."""
    suite = SensorSuite(seed=999)
    robot = RobotState(x=1.0, y=2.0, theta=0.0, v=1.0, omega=0.0)

    run_1 = [suite.sample(robot, dt=0.1, current_time=i * 0.1) for i in range(15)]
    suite.reset()
    run_2 = [suite.sample(robot, dt=0.1, current_time=i * 0.1) for i in range(15)]

    for r1, r2 in zip(run_1, run_2):
        assert r1.v_odom == r2.v_odom
        assert r1.omega_imu == r2.omega_imu
        assert r1.gnss_pos == r2.gnss_pos

