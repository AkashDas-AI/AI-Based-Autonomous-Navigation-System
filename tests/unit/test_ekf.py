"""Unit tests for Extended Kalman Filter (EKF) state estimation."""

import math
import numpy as np
import pytest

from autonomous_nav.estimation.ekf import ExtendedKalmanFilter
from autonomous_nav.simulation.robot import RobotState


def test_ekf_initialization_and_reset():
    """Verify state vector shape, initial values, and covariance initialization."""
    ekf = ExtendedKalmanFilter(
        initial_pos_std=0.4,
        initial_theta_std=0.1,
        initial_v_std=0.2,
        initial_bias_std=0.05,
    )
    assert ekf.x.shape == (5,)
    assert ekf.P.shape == (5, 5)
    assert np.allclose(ekf.x, 0.0)

    # Check initial diagonal variances
    assert ekf.P[0, 0] == pytest.approx(0.4**2)
    assert ekf.P[1, 1] == pytest.approx(0.4**2)
    assert ekf.P[2, 2] == pytest.approx(0.1**2)
    assert ekf.P[3, 3] == pytest.approx(0.2**2)
    assert ekf.P[4, 4] == pytest.approx(0.05**2)

    # Test reset
    ekf.reset(x=10.0, y=20.0, theta=math.pi / 2, v=1.5, bias_gyro=0.02)
    assert ekf.x[0] == pytest.approx(10.0)
    assert ekf.x[1] == pytest.approx(20.0)
    assert ekf.x[2] == pytest.approx(math.pi / 2)
    assert ekf.x[3] == pytest.approx(1.5)
    assert ekf.x[4] == pytest.approx(0.02)


def test_ekf_jacobian_numerical_check():
    """Verify analytical state transition Jacobian F against numerical finite differences."""
    ekf = ExtendedKalmanFilter()
    dt = 0.1
    eps = 1e-7

    test_states = [
        (5.0, 5.0, 0.0, 1.5, 0.01),
        (10.0, 2.0, math.pi / 4, 2.0, -0.02),
        (0.0, 0.0, -math.pi / 3, 0.5, 0.0),
        (15.0, 25.0, 2.8, 1.0, 0.05),
    ]

    for x, y, theta, v, bg in test_states:
        f_analytical = ekf.compute_jacobian_f(v=v, theta=theta, dt=dt)
        f_numerical = np.zeros((5, 5), dtype=np.float64)

        base_state = np.array([x, y, theta, v, bg], dtype=np.float64)
        omega_imu = 0.2

        def process_model(s: np.ndarray) -> np.ndarray:
            px, py, pth, pv, pbg = s
            unbiased_w = omega_imu - pbg
            return np.array([
                px + pv * math.cos(pth) * dt,
                py + pv * math.sin(pth) * dt,
                pth + unbiased_w * dt,
                pv,
                pbg,
            ])

        for col in range(5):
            pert_pos = base_state.copy()
            pert_neg = base_state.copy()
            pert_pos[col] += eps
            pert_neg[col] -= eps

            diff = (process_model(pert_pos) - process_model(pert_neg)) / (2.0 * eps)
            f_numerical[:, col] = diff

        max_err = np.max(np.abs(f_analytical - f_numerical))
        assert max_err < 1e-6, f"Jacobian mismatch at state {base_state}: max error = {max_err}"


def test_ekf_covariance_symmetry_and_positive_definiteness():
    """Verify error covariance matrix remains symmetric and positive-definite across simulation steps."""
    ekf = ExtendedKalmanFilter()
    ekf.reset(x=0.0, y=0.0, theta=0.0, v=1.0, bias_gyro=0.01)

    rng = np.random.RandomState(42)
    dt = 0.1

    for step in range(100):
        # Predict
        w_imu = 0.1 + rng.normal(0, 0.01)
        ekf.predict(omega_imu=w_imu, dt=dt)

        # Odometry update
        v_meas = 1.0 + rng.normal(0, 0.05)
        ekf.update_odometry(v_meas)

        # GNSS update every 10 steps
        if step % 10 == 0:
            x_meas = ekf.x[0] + rng.normal(0, 0.2)
            y_meas = ekf.x[1] + rng.normal(0, 0.2)
            ekf.update_gnss(x_meas, y_meas)

        # Verify symmetry
        assert np.allclose(ekf.P, ekf.P.T, atol=1e-10)

        # Verify positive semi-definiteness (all eigenvalues > 0)
        eigvals = np.linalg.eigvalsh(ekf.P)
        assert np.all(eigvals > 1e-12), f"Non-positive eigenvalue at step {step}: {eigvals}"


def test_ekf_joseph_form_numerical_stability():
    """Verify Joseph-form covariance update avoids numerical degeneration with tiny measurement noise."""
    ekf = ExtendedKalmanFilter()
    ekf.reset(x=5.0, y=5.0, theta=0.0, v=1.0)

    # Perform updates with very small measurement variance
    for _ in range(20):
        ekf.predict(omega_imu=0.0, dt=0.1)
        ekf.update_odometry(v_odom=1.0, r_odom=1e-4)
        ekf.update_gnss(x_gnss=5.0, y_gnss=5.0, r_gnss=1e-3)

        assert not np.isnan(ekf.P).any()
        assert not np.isinf(ekf.P).any()
        eigvals = np.linalg.eigvalsh(ekf.P)
        assert np.all(eigvals > 0.0)


def test_ekf_heading_angle_normalization():
    """Verify heading angles wrap correctly within [-pi, pi]."""
    ekf = ExtendedKalmanFilter()
    assert abs(abs(ekf.normalize_angle(math.pi)) - math.pi) < 1e-9
    assert abs(abs(ekf.normalize_angle(3.0 * math.pi)) - math.pi) < 1e-9
    assert abs(abs(ekf.normalize_angle(-3.0 * math.pi)) - math.pi) < 1e-9
    assert ekf.normalize_angle(math.pi / 2) == pytest.approx(math.pi / 2)
    assert ekf.normalize_angle(-math.pi / 2) == pytest.approx(-math.pi / 2)

    # Predict beyond pi
    ekf.reset(x=0.0, y=0.0, theta=3.0, v=0.0, bias_gyro=0.0)
    ekf.predict(omega_imu=1.0, dt=1.0)  # theta = 3.0 + 1.0 = 4.0 rad => 4.0 - 2*pi ~= -2.283 rad
    assert -math.pi <= ekf.x[2] <= math.pi
    assert ekf.x[2] == pytest.approx(4.0 - 2.0 * math.pi)


def test_ekf_odometry_update_reduces_variance():
    """Verify that applying an odometry measurement update contracts velocity uncertainty."""
    ekf = ExtendedKalmanFilter(initial_v_std=0.5, r_odom=0.05)
    ekf.reset(x=0.0, y=0.0, theta=0.0, v=0.0)

    p_v_prior = ekf.P[3, 3]
    ekf.update_odometry(v_odom=1.2)
    p_v_post = ekf.P[3, 3]

    assert p_v_post < p_v_prior
    assert abs(ekf.x[3] - 1.2) < 0.2


def test_ekf_gnss_update_reduces_variance():
    """Verify that GNSS measurement updates contract 2D position uncertainty."""
    ekf = ExtendedKalmanFilter(initial_pos_std=1.0, r_gnss=0.3)
    ekf.reset(x=0.0, y=0.0, theta=0.0, v=0.0)

    p_pos_prior = ekf.P[0, 0]
    ekf.update_gnss(x_gnss=1.0, y_gnss=2.0)
    p_pos_post = ekf.P[0, 0]

    assert p_pos_post < p_pos_prior
    assert ekf.x[0] > 0.0
    assert ekf.x[1] > 0.0


def test_ekf_uncertainty_ellipse_geometry():
    """Verify uncertainty ellipse dimensions are mathematically sound."""
    ekf = ExtendedKalmanFilter(initial_pos_std=0.5)
    ekf.reset(x=10.0, y=10.0)

    semi_major, semi_minor, phi = ekf.get_position_covariance_ellipse(conf_level=0.95)
    assert semi_major > 0.0
    assert semi_minor > 0.0
    assert semi_major >= semi_minor
    assert -math.pi <= phi <= math.pi


def test_ekf_stationary_convergence():
    """Verify stationary robot position estimate converges under repeated noisy GNSS measurements."""
    ekf = ExtendedKalmanFilter(initial_pos_std=2.0, r_gnss=0.4)
    true_x, true_y = 12.0, 8.0
    ekf.reset(x=0.0, y=0.0)  # Started with 14m initial error

    rng = np.random.RandomState(42)
    for _ in range(30):
        ekf.predict(omega_imu=0.0, dt=0.1)
        z_x = true_x + rng.normal(0, 0.4)
        z_y = true_y + rng.normal(0, 0.4)
        ekf.update_gnss(z_x, z_y)

    assert abs(ekf.x[0] - true_x) < 0.25
    assert abs(ekf.x[1] - true_y) < 0.25
    assert ekf.P[0, 0] < 0.1
    assert ekf.P[1, 1] < 0.1
