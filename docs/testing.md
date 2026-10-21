# Testing & Quality Assurance Framework

## 1. Test Architecture

The testing suite is structured according to testing scopes and functional domains:

```
tests/
├── conftest.py                       # Shared test fixtures (grids, models, configs)
├── unit/                             # Isolated algorithmic & mathematical verification
│   ├── test_grid_map.py              # Transforms, bounds, obstacle insertion & inflation
│   ├── test_astar.py                 # Graph search optimality, obstacles, unreachable goals
│   ├── test_collision.py             # Point, disc, and path collision detection
│   ├── test_controller.py            # Kinematic integration, controller outputs, waypoints
│   ├── test_dwa.py                   # Dynamic Window limits, trajectory simulation, DWA scoring & safety
│   ├── test_dynamic_obstacle.py      # Moving obstacle kinematics, prediction, manager queries
│   ├── test_temporal_collision.py    # Continuous relative swept-distance, tunneling elimination
│   ├── test_sensors.py               # Wheel odometry noise, IMU bias drift, GNSS decimation & outages
│   ├── test_ekf.py                   # Analytical Jacobians, covariance stability, Joseph updates, ellipses
│   ├── test_metrics.py               # CTE calculation, path lengths, estimation error metrics
│   ├── test_perception.py           # Image preprocessing, cameras, detector, map adapter
│   ├── test_visualizer.py           # Pygame visualizer, step interface, headless rendering & HUD
│   ├── test_bridge_contracts.py     # 3D state, command validation, sensor contracts
│   ├── test_bridge_adapter.py       # OfflineTestAdapter lifecycle, replay, LiDAR raycasting
│   ├── test_dataset_adapter.py      # Dataset config, ground truth parsing, SLERP, depth unprojection
│   └── test_calibration_sensitivity.py # 6-DoF extrinsic perturbations, analytical shifts, height coupling
├── integration/                      # End-to-end component handoff tests
│   ├── test_navigation.py            # Static closed-loop engine runs without crashes/collisions
│   ├── test_dynamic_navigation.py    # Comparative tests: Pure Pursuit collision vs DWA goal reaching
│   ├── test_ekf_navigation.py        # Closed-loop EKF navigation, feedback isolation, DR drift vs EKF bound
│   ├── test_bridge_navigation.py     # Closed-loop bridge waypoint tracking, EKF fusion, LiDAR projection
│   └── test_dataset_integration.py   # Real-world TUM RGB-D trajectory and mapping integration
└── scenarios/                        # Standard scenario verification
    └── test_scenario_validation.py   # Asserting all predefined benchmarks (static + dynamic) are well-formed
```

---

## 2. Test Suites Summary

| Test Module | Test Name | Assertion / Invariant |
| :--- | :--- | :--- |
| **`test_grid_map.py`** | `test_grid_map_initialization` | Width, height, resolution, and origin correctly set; defaults to zero. |
| | `test_invalid_dimensions` | Raises `ValueError` on non-positive dimensions. |
| | `test_coordinate_transforms` | Roundtrip mapping between metric world and discrete grid cell centers. |
| | `test_obstacle_setting_and_queries`| Dynamic cell state toggling and out-of-bounds boundary guards. |
| | `test_rectangular_and_circular_obstacles` | Geometric shape rasterization onto grid. |
| | `test_obstacle_inflation` | Dilation radius covers neighboring cells. |
| **`test_astar.py`** | `test_astar_empty_field` | Finds shortest diagonal path on 8-connected grid. |
| | `test_astar_around_obstacle` | Navigates around wall obstacle without crossing occupied cells. |
| | `test_astar_unreachable_goal` | Returns `None` gracefully when enclosed by obstacles. |
| | `test_astar_start_equals_goal` | Returns single start node without hanging. |
| | `test_astar_world_planning` | Metric path coordinates align with world endpoints. |
| | `test_astar_strictly_forbids_diagonal_cutting_single_blocked_corner` | Enforces that diagonal moves cannot cut through a blocked corner cell. |
| **`test_collision.py`** | `test_point_collision` | Inside obstacle returns `True`, outside returns `False`. |
| | `test_disc_collision` | Circle-to-AABB detects penetration at distance $\le R$. |
| | `test_path_collision` | Sequence of waypoints detects obstacle intersection. |
| | `test_distance_to_nearest_obstacle` | Minimum Euclidean distance query within search radius. |
| **`test_controller.py`**| `test_robot_kinematics_straight` | Unicycle moves along forward heading with $\omega = 0$. |
| | `test_robot_kinematics_turn` | Robot heading updates proportionally to commanded angular rate. |
| | `test_pure_pursuit_heading_correction` | Generates steering demand toward left-shifted target point. |
| | `test_waypoint_manager_progression` | Advances active index within acceptance radius. |
| **`test_metrics.py`** | `test_point_to_segment_distance` | Projection and Euclidean clamping to line segment. |
| | `test_calculate_path_and_trajectory_length` | Total Euclidean arc length calculation. |
| | `test_calculate_cross_track_error_orthogonal` | Validates 0.0 error when robot is exactly on reference segment. |
| | `test_evaluate_run_metrics_dictionary` | Validates scorecard generation and termination labeling. |
| | `test_simulation_logger` | Verifies step telemetry logging and CSV/JSON disk export. |
| | `test_calculate_tracking_errors` | Verifies synthesis of orthogonal CTE, path efficiency, and actual vs planned lengths. |
| | `test_calculate_estimation_errors_zero_for_identical` | Asserts exact 0.0 error across all metrics when feedback matches true trajectory. |
| **`test_perception.py`** | `test_valid_image_preprocessing` | Dimension preservation, blur smoothing, and grayscale conversion. |
| | `test_color_conversions` | Valid BGR to grayscale single-channel channel reduction. |
| | `test_normalization_dynamic_range` | Verifies intensity scaling maps to $[0, 255]$ with $\text{dtype}=\text{uint8}$. |
| | `test_invalid_image_inputs` | Asserts `ValueError` on empty, single-channel, 4D, or non-uint8 inputs. |
| | `test_empty_and_uniform_images` | Returns zero detections without errors on plain/blank frames. |
| | `test_detect_known_shapes` | Accurately identifies single synthetic rectangle centroid and area. |
| | `test_detect_multiple_obstacles` | Detects all discrete geometric obstacles in synthetic multi-obstacle scene. |
| | `test_orthographic_camera_roundtrip_transforms` | Roundtrip $(x, y) \leftrightarrow (u, v)$ with $< 0.01\text{ m}$ coordinate error. |
| | `test_planar_perspective_homography_transforms` | IPM homography inverse projection recovers 4 ground coplanar points. |
| | `test_deterministic_output_for_identical_inputs` | 100% identical detection count, centroids, and areas across runs. |
| | `test_noise_filtering_min_max_area` | Area filters correctly reject small noise specks ($< A_{\min}$) and background. |
| | `test_grid_map_updates_from_detections` | Confirms `MapAdapter` successfully sets occupancy cells in `GridMap`. |
| | `test_complete_perception_to_path_planning_integration` | End-to-end: synthetic scene $\to$ perception $\to$ map adapter $\to$ A* planner finding valid path. |
| | `test_orthographic_camera_boundary_and_corners` | Audits 4 image corners, vertical axis inversion, metric scale, and boundary checks. |
| | `test_planar_perspective_homography_from_correspondences` | Computes homography from 4 coplanar correspondences and tests roundtrip projection. |
| | `test_planar_perspective_degenerate_and_singular_inputs` | Asserts `ValueError` on collinear points, singular matrices, and out-of-bounds horizon. |
| | `test_obstacle_footprint_mapping_modes` | Verifies `MapAdapter` fills full interior footprint cells (> 50 cells for 4x4m square). |
| | `test_obstacle_detection_precision_recall_and_centroid_accuracy` | Confirms 100% precision/recall and sub-5cm centroid accuracy on synthetic fixtures. |
| | `test_obstacle_footprint_iou_and_coverage` | Measures footprint IoU ($\ge 0.88$) and zero false negatives (no under-estimation). |
| | `test_obstacles_touching_and_crossing_map_boundaries` | Confirms off-grid obstacles do not corrupt unrelated map cells or create fake borders. |
| | `test_binary_mask_affine_warp_projection` | Validates `update_from_binary_mask` via affine warp directly onto `GridMap`. |
| | `test_perceived_obstacle_clearance_and_inflation_safety` | Verifies obstacle inflation preserves robot clearance $> 0.5\text{ m}$ for all path waypoints. |
| | `test_perceived_obstacle_unreachable_goal_reporting` | Confirms A* planner safely reports failure (`path is None`) for enclosed goals. |
| **`test_visualizer.py`** | `test_engine_step_interface` | Verifies fine-grained $\Delta t$ stepping, state evolution, and termination signaling. |
| | `test_engine_replan_on_obstacle_mutation` | Confirms dynamic mid-simulation obstacle mutation triggers successful A* replanning. |
| | `test_visualizer_headless_initialization` | Verifies headless Pygame surface allocation, metrics binding, and coordinate limits. |
| | `test_world_to_screen_and_inverse_conversion` | Validates bidirectional screen $\leftrightarrow$ world transforms with vertical inversion ($< 0.01\text{ m}$). |
| | `test_rendering_does_not_mutate_state` | Asserts that rendering frames does not alter robot coordinates, velocities, or step counters. |
| | `test_visualizer_pause_step_and_reset_logic` | Verifies pause toggle, single-step execution, and reset restore mechanisms. |
| | `test_visualizer_scenario_switching` | Confirms seamless scenario transitions (1-4) without orphaned state or memory leaks. |
| | `test_visualizer_planning_failure_and_collision_status` | Validates HUD telemetry status flags (`PLANNING_FAILURE`, `COLLISION`, `GOAL_REACHED`). |
| | `test_visualizer_run_and_save_screenshot` | Runs bounded headless loop, renders UI/HUD, and exports valid PNG image artifact. |
| | `test_replan_obstacle_forces_valid_detour` | Verifies placing an obstacle across direct route forces an obstacle-avoiding detour. |
| | `test_replan_blocking_only_available_route` | Confirms completely blocking corridor route halts simulation and clears stale path. |
| | `test_replan_obstacle_on_goal_cell` | Confirms placing obstacle on goal cell safely flags planning failure and clears path. |
| | `test_replan_and_collision_at_robot_location` | Asserts obstacle placed at robot pose triggers collision, and clearing it restores safe state. |
| | `test_replan_recovery_from_planning_failure_on_obstacle_removal` | Verifies removing blocking obstacle allows valid recovery and normal step progression. |
| | `test_failed_replan_clears_stale_route_and_renders_safely` | Asserts failed replan empties planned path and renders UI without crashing. |
| | `test_reported_termination_state_agreement` | Verifies 100% agreement between engine termination reasons and GUI status badges. |
| | `test_controls_prevent_unintended_advancement` | Confirms pause, step, reset, and scenario switching prevent accidental physics steps. |
| | `test_rendering_invariance_across_all_metrics` | Asserts repeated rendering and snapshot calls leave all simulation state unchanged. |
| **`test_navigation.py`**| `test_closed_loop_navigation_empty_field` | Complete end-to-end closed-loop run on open field. |
| | `test_closed_loop_navigation_obstacle_avoidance` | Complete closed-loop run around obstacles without collision. |
| | `test_navigation_planning_failure` | Confirms `planning_failed=True` and `success=False` on unreachable goal. |
| | `test_navigation_timeout_failure` | Confirms `timeout=True` and `success=False` when step budget is exceeded. |
| | `test_navigation_collision_detection` | Confirms `collision=True` and `success=False` if trajectory hits obstacle. |
| **`test_dynamic_obstacle.py`** | `test_dynamic_obstacle_initialization` | Verifies obstacle ID, radius, coordinates, speed, and velocity decomposition. |
| | `test_linear_ping_pong_kinematics_and_reversal` | Confirms linear ping-pong reverses heading at endpoint and steps forward. |
| | `test_analytical_future_prediction_determinism` | Confirms `predict_position()` matches step-by-step numerical simulation. |
| | `test_predict_trajectory_horizon` | Returns discrete list of future positions over specified prediction horizon. |
| | `test_circular_motion_pattern` | Verifies circular orbit around center maintaining constant radial distance. |
| | `test_reset_restores_initial_state` | Asserts `reset()` restores identical starting position, velocity, and state. |
| | `test_dynamic_obstacle_manager` | Audits manager collision checks, minimum clearance queries, and prediction. |
| **`test_dwa.py`** | `test_dynamic_window_bounds` | Bounds velocity search space by acceleration limits and max speeds. |
| | `test_simulate_trajectory_straight` | Evaluates forward unicycle trajectory integration with $\omega=0$. |
| | `test_trajectory_static_collision_filtering` | Rejects trajectory candidates penetrating static occupancy grid obstacles. |
| | `test_trajectory_dynamic_collision_detection` | Detects intersection with moving obstacle's predicted position along trajectory. |
| | `test_emergency_stop_when_fully_trapped` | Returns $(v=0, \omega=0)$ with `is_safe=False` when completely surrounded. |
| | `test_dwa_determinism` | Identical inputs produce bit-exact commands and trajectory points. |
| **`test_temporal_collision.py`** | `test_temporal_tunneling_continuous_relative_distance` | Verifies continuous swept distance detects tunneling missed by discrete endpoint sampling. |
| | `test_close_pass_and_near_tangency` | Evaluates closest approach distance on near-tangent passing paths. |
| | `test_obstacle_moving_toward_and_away` | Confirms relative continuous distance for head-on and receding obstacle paths. |
| | `test_fast_crossing_obstacle_swept_rejection_in_dwa` | Asserts DWA rejects candidate arcs intersecting fast-crossing obstacle swept volume. |
| | `test_stopping_when_all_candidates_unsafe` | Asserts DWA emergency stop when completely surrounded by obstacles. |
| | `test_dwa_velocity_and_acceleration_limits` | Confirms commands strictly respect maximum velocity and acceleration limits. |
| | `test_dwa_determinism_identical_inputs` | Verifies bit-exact repeatability of commands across duplicate invocations. |
| | `test_obstacle_and_engine_telemetry_reset` | Asserts engine reset restores initial obstacle states, clocks, and counters. |
| **`test_dynamic_navigation.py`** | `test_dynamic_crossing_pure_pursuit_collides` | Baseline Pure Pursuit fails on dynamic crossing due to blind route following. |
| | `test_dynamic_crossing_dwa_succeeds` | DWA safely slows and maneuvers around crossing obstacles to reach goal. |
| | `test_dynamic_chokepoint_pure_pursuit_vs_dwa` | Pure Pursuit collides at door; DWA pauses for opening and traverses cleanly. |
| | `test_dynamic_corridor_dwa_success` | DWA completes diagonal crossing corridor detour with zero collisions. |
| | `test_nav_mode_engine_telemetry` | Validates active trajectory arc and DWA telemetry reporting in engine. |
| **`test_sensors.py`** | `test_wheel_odometry_noise_distribution` | Validates zero-mean Gaussian noise matching configured standard deviation. |
| | `test_wheel_odometry_scale_factor` | Confirms systematic scale factor calibration alters base velocity measurement. |
| | `test_imu_gyro_noise_and_bias_drift` | Verifies rate gyro white noise, random-walk bias drift, and reset logic. |
| | `test_gnss_sample_rate_decimation` | Verifies GNSS emits position fixes strictly at periodic 1 Hz epoch schedule. |
| | `test_gnss_outage_window` | Asserts GNSS fixes are suppressed during designated signal loss intervals. |
| | `test_sensor_suite_determinism` | Confirms identical seeds produce bitwise duplicate sensor measurement streams. |
| | `test_sensor_suite_reset_reproducibility`| Verifies suite reset restores initial random generator sequence. |
| **`test_ekf.py`** | `test_ekf_initialization_and_reset` | Checks 5-state vector, covariance diagonals, and state reset restores values. |
| | `test_ekf_jacobian_numerical_check` | Validates analytical Jacobian matrix F against numerical finite differences (< 1e-6 error). |
| | `test_ekf_covariance_symmetry_and_positive_definiteness` | Enforces P = P^T and all eigenvalues > 0 across repeated predict/update cycles. |
| | `test_ekf_joseph_form_numerical_stability` | Confirms Joseph-form covariance update avoids degeneration with small measurement noise. |
| | `test_ekf_heading_angle_normalization` | Verifies angular wrapping across the [-pi, pi] branch cut. |
| | `test_ekf_odometry_update_reduces_variance` | Confirms velocity measurement update reduces velocity variance and updates estimate. |
| | `test_ekf_gnss_update_reduces_variance` | Confirms 2D position fix reduces positional uncertainty covariance terms. |
| | `test_ekf_uncertainty_ellipse_geometry` | Validates semi-major and semi-minor axis calculations and Chi-square scaling. |
| | `test_ekf_stationary_convergence` | Confirms stationary robot converges under noisy GNSS with bounded error. |
| **`test_ekf_navigation.py`** | `test_closed_loop_ekf_navigation_success` | Full closed-loop navigation episode in EKF mode with bounded RMSE and 0 collisions. |
| | `test_state_feedback_isolation` | Verifies controller strictly consumes EKF estimated state without ground-truth leakage. |
| | `test_dead_reckoning_drift_vs_ekf_bounded` | Confirms dead reckoning drifts over time while EKF fusion bounds position error. |
| | `test_gnss_outage_divergence_and_reconvergence` | Verifies error escalation during outage and rapid re-convergence upon GNSS recovery. |
| | `test_ekf_navigation_determinism` | Asserts identical seeds produce bitwise duplicate navigation metrics and paths. |
| | `test_ground_truth_feedback_zero_estimation_error` | Verifies ground_truth feedback mode produces exact 0.0m estimation error by proper path construction. |
| | `test_dead_reckoning_strictly_zero_gnss_leakage` | Verifies dead-reckoning mode processes 0 GNSS fixes and isolates estimator from satellite data. |
| | `test_metric_distinction_estimation_vs_tracking` | Enforces distinct data structures and formulations for estimation accuracy vs tracking performance. |
| | `test_trajectory_timestamp_semantics_and_terminal_alignment` | Verifies identical length and matching timestamps across ground truth, estimated, and feedback histories. |
| | `test_early_collision_termination_timestamp_alignment` | Confirms early termination preserves exact timestamp alignment without orphan unstepped estimates. |
| **`test_bridge_contracts.py`** | `test_timestamped_robot_state_valid_and_projection` | Validates 3D state instantiation, finite bounds, and 2D projection `to_2d_robot_state()`. |
| | `test_timestamped_robot_state_non_finite_rejected` | Asserts `ValueError` when X, Y, Z, yaw, velocities, or timestamp are NaN or Inf. |
| | `test_timestamped_robot_state_negative_timestamp_rejected` | Rejects negative timestamps. |
| | `test_navigation_command_validation_and_clamping` | Validates linear/angular velocity bounds and clamping policy. |
| | `test_navigation_command_non_finite_rejected` | Rejects non-finite velocity commands. |
| | `test_sensor_observation_contracts` | Validates Odometry, IMU, GNSS, LaserScan, and SensorPacket contracts. |
| | `test_laser_scan_invalid_parameters` | Asserts `ValueError` on negative increment or inverted range bounds. |
| | `test_sensor_packet_intra_packet_timestamp_desynchronization_rejected` | Asserts `ValueError` when child observations are desynchronized from packet timestamp. |
| | `test_timestamped_robot_state_quaternion_conversions` | Validates `as_quaternion()` and `from_quaternion()` bidirectional consistency. |
| | `test_quaternion_normalization_and_degenerate_rejection` | Asserts quaternion normalization and rejection of degenerate/non-finite quaternions. |
| **`test_bridge_adapter.py`** | `test_offline_test_adapter_lifecycle_state_transitions` | State machine transitions: UNINITIALIZED -> READY -> RUNNING -> READY -> STOPPED. |
| | `test_offline_test_adapter_unconnected_operations_raise_error` | Pre-connection calls to initialize/step/reset/send_command raise `ConnectionError`. |
| | `test_offline_test_adapter_connection_timeout_failure` | Non-positive timeout fails connection and transitions to ERROR state. |
| | `test_offline_test_adapter_deterministic_replay` | Identical seeds produce bitwise identical states, odometry, and laser scans. |
| | `test_offline_test_adapter_command_validation_rules` | Rejects invalid coordinate frames, out-of-bound speeds, and past timestamps. |
| | `test_offline_test_adapter_laser_raycasting_against_map` | Raymarches planar LiDAR against `GridMap` obstacles and verifies range accuracy. |
| | `test_offline_test_adapter_ground_truth_isolation` | Verifies ground truth is distinct from noisy sensor observations. |
| **`test_bridge_navigation.py`** | `test_bridge_closed_loop_navigation_pure_pursuit` | Closed-loop waypoint navigation through the bridge adapter to goal. |
| | `test_bridge_sensor_packet_to_ekf_fusion` | Streams bridge sensor packet directly into EKF estimator, verifying convergence. |
| | `test_laser_scan_projection_to_grid_map` | Projects bridge LiDAR returns into an empty `GridMap`, marking obstacle cells. |
| | `test_quaternion_and_euler_roundtrip_transforms` | Bidirectional consistency of unit quaternion <-> Euler angles. |
| | `test_bridge_closed_loop_ekf_feedback_isolated_navigation` | Verifies closed-loop navigation through bridge using ONLY EKF feedback with 0 ground-truth leakage. |
| **`test_dataset_adapter.py`** | `test_dataset_config_validation` | Validates default Freiburg 2 intrinsics, depth scale, and custom overrides. |
| | `test_groundtruth_pose_yaw_and_robot_state` | Converts quaternion to yaw angle and exports planar `RobotState`. |
| | `test_groundtruth_trajectory_parsing` | Parses synthetic TUM-format ground truth, verifies timestamp indexing and driven length. |
| | `test_groundtruth_slerp_interpolation` | Tests spherical linear interpolation of poses and orientations at arbitrary query times. |
| | `test_depth_unprojection_geometry` | Back-projects synthetic depth pixels and checks known 3D camera coordinates. |
| | `test_optical_to_body_transform` | Verifies REP-105 body frame transformation ($X_b = Z_c + d_x$, $Y_b = -X_c + d_y$, $Z_b = -Y_c + d_z$). |
| | `test_height_filtering_floor_and_ceiling` | Slices points within $[0.05, 1.20]\text{ m}$, filtering floor and ceiling points. |
| | `test_insert_points_into_grid_map` | Inserts transformed obstacle points into `GridMap` and verifies occupied cells. |
| | `test_adapter_synthetic_manifest_processing` | Full synthetic adapter pipeline: parses trajectory, processes depth frames, and tracks error. |
| | `test_depth_projection_nan_inf_and_negative_rejection` | Asserts non-finite/negative depths are filtered during back-projection. |
| | `test_floor_plane_rejection_invariance` | Verifies 100% rejection of floor plane points ($Z_b=0.0\text{ m}$) under $[0.05, 1.20]\text{ m}$ slicing. |
| | `test_provenance_manifest_depth_directory_absence` | Validates adapter manifest correctly reports `has_depth_dir=False` without fabricating frame counts. |
| | `test_corrupt_or_truncated_png_handling` | Asserts `ValueError` when loading truncated or malformed PNG files. |
| | `test_invalid_depth_pixel_filtering_comprehensive` | Comprehensive filtering of sensor dropout, near-field noise ($<0.4\text{ m}$), and far-field ($>5.0\text{ m}$). |
| | `test_timestamp_association_and_interpolation` | Verifies temporal association and interpolation of sensor timestamps with ground truth poses within $0.05\text{ s}$. |
| | `test_projection_and_grid_map_bounds` | Confirms points falling outside discrete grid bounds are clipped/ignored safely without `IndexError`. |
| | `test_depth_projection_determinism` | Verifies identical point clouds and grid occupancy states on identical repeated invocations. |
| | `test_manifest_parsing_and_provenance` | Validates reading and parsing of authentic depth frame cryptographic manifest (`manifest.json`). |
| | `test_pixel_reconciliation_exact_breakdown` | Verifies exact arithmetic reconciliation across raw pixels ($307,200 = \text{zero} + \text{out-of-range} + \text{valid}$) and height filtering ($\text{valid} = \text{floor} + \text{ceiling} + \text{retained}$). |
| | `test_timestamp_association_nearest_pose_and_brackets` | Verifies nearest MoCap pose timestamp reporting accurately non-zero physical $\Delta t$ and valid bounding bracket SLERP interpolation. |
| | `test_grid_boundary_clipping_diagnostics` | Verifies `insert_points_into_grid_map(return_diagnostics=True)` correctly tracks in-bounds vs boundary-clipped points. |
| | `test_quaternion_component_ordering_and_normalization` | Verifies (qx, qy, qz, qw) ordering with qw scalar real, normalizes non-unit quaternions, and rejects zero-norm or non-finite inputs. |
| | `test_quaternion_to_yaw_four_quadrants_and_wrapping` | Tests quaternion-to-yaw conversion across all four quadrants with correct wrapping in $(-\pi, \pi]$. |
| | `test_optical_to_body_axis_handedness_and_so3_determinant` | Verifies optical-to-body coordinate transform preserves right-handedness with SO(3) determinant $= +1$ and orthonormal check $R^T R = I$. |
| | `test_end_to_end_projection_analytical_hand_calculation` | Validates complete depth-to-world projection pipeline against independently derived analytical hand calculations with non-zero heading ($30^\circ$ and $90^\circ$). |
| | `test_transformation_composition_no_double_offset` | Proves mounting offsets are rotated with vehicle heading and applied exactly once without duplication. |
| | `test_grid_semantics_obstacle_endpoint_rasterization_only` | Validates that GridMap insertion marks obstacle point hits only and does not perform free-space ray tracing. |
| **`test_calibration_sensitivity.py`** | `test_camera_extrinsics_defaults_and_validation` | Validates default engineering hypothesis parameters and rejects non-finite/NaN extrinsics. |
| | `test_analytical_pure_translation_displacement` | Asserts exact predictable Euclidean displacements under pure dx, dy, and dz translation offsets. |
| | `test_analytical_pure_yaw_rotation` | Validates chord length and small-angle arc displacement under pure camera yaw rotation. |
| | `test_analytical_pure_roll_rotation` | Verifies camera roll rotation about forward longitudinal axis Xb. |
| | `test_height_filtering_coupling_pitch_vs_yaw` | Proves physical coupling: pitch rotates obstacle points into floor rejection, while yaw leaves height retention invariant. |
| | `test_grid_map_overlap_and_iou_zero_perturbation` | Verifies zero-perturbation identity yields exactly 1.0 IoU and 100% recall. |
| | `test_production_config_immutability` | Guarantees default production `CameraExtrinsics` and `DatasetConfig` are immutable and protected from sweep mutations. |
| **`test_dataset_integration.py`** | `test_tum_rgbd_groundtruth_file_integration` | Loads local `data/tum_rgbd/groundtruth.txt`, verifying 33,188 poses, 115.63s duration, and 23.33m driven path. |
| | `test_tum_rgbd_adapter_evaluation_pipeline` | Executes end-to-end evaluation pipeline on local TUM dataset files. |
| | `test_authentic_depth_frame_manifest_and_files` | Verifies file existence, sizes, SHA-256 hashes, and dimensions ($480 \times 640$) of extracted authentic depth PNGs. |
| | `test_authentic_depth_frame_grid_mapping` | Projects authentic TUM depth frames into 2D `GridMap` with interpolated ground truth poses, confirming valid occupied cells. |
| | `test_authentic_depth_frame_pixel_reconciliation_and_spatial_bounds` | Audits authentic TUM depth frames against exact pixel count reconciliation, height-filtering reconciliation, sub-2 ms nearest MoCap offset, and spatial grid bounds clipping. |
| **`test_scenario_validation.py`** | `test_scenario_validity` | Parametrized across all 7 scenarios (4 static + 3 dynamic) to ensure validity. |

---

## 3. Running Tests

### Standard Test Execution
```powershell
pytest -v
```

### Running Specific Test Groups
```powershell
# Unit tests only
pytest tests/unit/ -v

# Integration tests only
pytest tests/integration/ -v

# Scenario validation only
pytest tests/scenarios/ -v
```

### Running with Code Coverage
```powershell
pytest --cov=src --cov-report=term-missing tests/
```

