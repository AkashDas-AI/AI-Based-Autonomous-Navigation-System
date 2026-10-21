"""Interactive 2D Pygame visualizer for autonomous navigation simulation."""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import numpy as np

# Ensure Pygame can run headlessly in environments without a physical display
if "SDL_VIDEODRIVER" not in os.environ and os.environ.get("HEADLESS", "").lower() in ("1", "true", "yes"):
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    os.environ["SDL_AUDIODRIVER"] = "dummy"

import pygame
from autonomous_nav.config import SimulationConfig
from autonomous_nav.environment.grid_map import GridMap
from autonomous_nav.environment.scenarios import get_all_scenarios, get_scenario_by_name
from autonomous_nav.evaluation.metrics import calculate_cross_track_error, calculate_trajectory_length
from autonomous_nav.simulation.engine import SimulationEngine
from autonomous_nav.simulation.robot import RobotState


# ---------------------------------------------------------------------------
# Visual Palette & Color Constants
# ---------------------------------------------------------------------------
COLOR_BG = (24, 26, 32)               # Dark slate main background
COLOR_PANEL = (33, 37, 46)            # Sidebar and card background
COLOR_PANEL_BORDER = (48, 54, 67)     # Border outlines
COLOR_MAP_BG = (245, 247, 250)        # Free space light background
COLOR_GRID_LINE = (226, 230, 237)     # Map cell grid lines
COLOR_OBSTACLE = (42, 47, 58)         # Occupied obstacle cells
COLOR_OBSTACLE_BORDER = (30, 34, 42)  # Obstacle cell borders

COLOR_PLANNED_PATH = (33, 115, 243)   # Royal blue planned route
COLOR_WAYPOINT = (0, 188, 212)        # Cyan waypoint markers
COLOR_TRAJECTORY = (46, 204, 113)     # Emerald green driven path
COLOR_LOOKAHEAD = (255, 140, 0)       # Amber lookahead target

COLOR_START = (39, 174, 96)           # Start marker green
COLOR_GOAL = (231, 76, 60)            # Goal marker crimson red

COLOR_ROBOT_BODY = (241, 196, 15)     # Unicycle chassis yellow/gold
COLOR_ROBOT_BORDER = (180, 140, 10)   # Chassis outline
COLOR_ROBOT_HEADING = (192, 57, 43)   # Heading indicator red arrow

COLOR_DYNAMIC_OBSTACLE = (235, 87, 87)       # Coral red dynamic moving obstacles
COLOR_DYNAMIC_BORDER = (180, 40, 40)         # Border for dynamic obstacles
COLOR_DYNAMIC_PREDICT = (242, 153, 74)        # Amber predicted trajectory trail
COLOR_LOCAL_TRAJECTORY = (0, 220, 255)        # Bright cyan DWA planned arc

COLOR_EST_ROBOT = (0, 220, 255)              # Cyan estimated robot chassis outline
COLOR_EST_HEADING = (50, 255, 200)           # Mint green estimated heading arrow
COLOR_GNSS = (255, 105, 180)                 # Hot pink GNSS position fixes

# Status Badge Colors
STATUS_COLORS = {
    "RUNNING": (46, 204, 113),         # Green
    "PAUSED": (241, 196, 15),          # Amber
    "GOAL_REACHED": (0, 206, 209),     # Cyan
    "COLLISION": (231, 76, 60),        # Red
    "TIMEOUT": (230, 126, 34),         # Orange
    "PLANNING_FAILURE": (155, 89, 182),# Purple
}


class InteractiveVisualizer:
    """Interactive 2D Pygame visualizer displaying real-time vehicle simulation.

    Features:
        - Real-time 2D occupancy grid rendering with exact metric scaling.
        - Start, Goal, A* planned path, lookahead target, and driven trajectory.
        - Kinematic unicycle chassis with collision radius disk and heading quiver.
        - Telemetry HUD dashboard: speed, angular rate, cross-track error, sim time.
        - Interactive controls: Pause/Resume, Reset, Step, Scenario Switch, Obstacle Click-Toggle.
        - Headless execution support for automated testing and CI.
    """

    def __init__(
        self,
        engine: Optional[SimulationEngine] = None,
        scenario_name: str = "simple_obstacles",
        window_width: int = 1100,
        window_height: int = 720,
        fps: int = 30,
        headless: bool = False,
    ) -> None:
        self.headless = headless
        if self.headless:
            os.environ["SDL_VIDEODRIVER"] = "dummy"
            os.environ["SDL_AUDIODRIVER"] = "dummy"

        pygame.init()
        if pygame.font.get_init():
            self.font_large = pygame.font.SysFont("Segoe UI, Arial", 20, bold=True)
            self.font_main = pygame.font.SysFont("Segoe UI, Arial", 14)
            self.font_bold = pygame.font.SysFont("Segoe UI, Arial", 14, bold=True)
            self.font_small = pygame.font.SysFont("Consolas, Courier", 12)
        else:
            self.font_large = None
            self.font_main = None
            self.font_bold = None
            self.font_small = None

        self.window_width = int(window_width)
        self.window_height = int(window_height)
        self.fps = int(fps)
        self.clock = pygame.time.Clock()

        # Display surface
        self.screen = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption("AI-Based Autonomous Navigation - Interactive Simulator")

        # Viewport geometry: map area on left, sidebar HUD on right
        self.sidebar_width = 280
        self.margin = 16
        self.map_rect = pygame.Rect(
            self.margin,
            self.margin,
            self.window_width - self.sidebar_width - (self.margin * 3),
            self.window_height - (self.margin * 2),
        )
        self.sidebar_rect = pygame.Rect(
            self.window_width - self.sidebar_width - self.margin,
            self.margin,
            self.sidebar_width,
            self.window_height - (self.margin * 2),
        )

        # Simulation state
        self.engine = engine if engine is not None else SimulationEngine()
        self.scenario_name = scenario_name
        self.scenarios_list = list(get_all_scenarios().keys())
        self.paused = False
        self.running = True

        # Coordinate transform parameters
        self.scale: float = 20.0
        self.offset_u: float = 0.0
        self.offset_v: float = 0.0

        # Load active scenario
        self.load_scenario(self.scenario_name)

    def load_scenario(self, scenario_name: str) -> bool:
        """Load and initialize a benchmark scenario in the simulation engine."""
        if scenario_name not in get_all_scenarios():
            return False

        self.scenario_name = scenario_name
        scenario = get_scenario_by_name(scenario_name)

        # Reset simulation engine
        success = self.engine.reset(
            grid_map=scenario.grid_map,
            start_pose=scenario.start_world,
            goal_position=scenario.goal_world,
            dynamic_obstacles=scenario.dynamic_obstacles,
        )

        self._update_coordinate_transforms()
        return success

    def next_scenario(self) -> None:
        """Cycle to the next available scenario."""
        if not self.scenarios_list:
            return
        idx = (self.scenarios_list.index(self.scenario_name) + 1) % len(self.scenarios_list)
        self.load_scenario(self.scenarios_list[idx])

    def _update_coordinate_transforms(self) -> None:
        """Compute scale and viewport offsets for world-to-screen mapping."""
        if self.engine.grid_map is None:
            return

        grid = self.engine.grid_map
        world_w = grid.width * grid.resolution
        world_h = grid.height * grid.resolution

        # Compute uniform scale fitting within map_rect
        padding = 10.0
        avail_w = max(10.0, float(self.map_rect.width) - (padding * 2.0))
        avail_h = max(10.0, float(self.map_rect.height) - (padding * 2.0))

        self.scale = min(avail_w / world_w, avail_h / world_h)

        # Center map in viewport
        rendered_w = world_w * self.scale
        rendered_h = world_h * self.scale
        self.offset_u = float(self.map_rect.left) + padding + (avail_w - rendered_w) / 2.0
        self.offset_v = float(self.map_rect.top) + padding + (avail_h - rendered_h) / 2.0

    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convert continuous metric world coordinates (x, y) to screen pixels (u, v)."""
        if self.engine.grid_map is None:
            return int(x), int(y)

        grid = self.engine.grid_map
        x0, y0 = grid.origin
        world_h = grid.height * grid.resolution
        ymax = y0 + world_h

        u = self.offset_u + (float(x) - x0) * self.scale
        # Invert vertical axis: world y points up, screen v points down
        v = self.offset_v + (ymax - float(y)) * self.scale
        return int(round(u)), int(round(v))

    def screen_to_world(self, u: int, v: int) -> Tuple[float, float]:
        """Convert screen pixels (u, v) to continuous metric world coordinates (x, y)."""
        if self.engine.grid_map is None or self.scale <= 0.0:
            return float(u), float(v)

        grid = self.engine.grid_map
        x0, y0 = grid.origin
        world_h = grid.height * grid.resolution
        ymax = y0 + world_h

        x = x0 + (float(u) - self.offset_u) / self.scale
        y = ymax - (float(v) - self.offset_v) / self.scale
        return float(x), float(y)

    def handle_events(self) -> None:
        """Process keyboard, mouse, and window management events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return

            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    self.running = False
                    return
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key in (pygame.K_s, pygame.K_RIGHT):
                    # Single step forward when paused
                    if self.paused and not self.engine.is_done:
                        self.engine.step()
                elif event.key == pygame.K_m:
                    # Toggle navigation mode between pure_pursuit and dwa
                    self.engine.config.nav_mode = "dwa" if self.engine.config.nav_mode == "pure_pursuit" else "pure_pursuit"
                elif event.key == pygame.K_e:
                    # Cycle state feedback mode: ground_truth -> ekf -> dead_reckoning
                    if self.engine.config.state_feedback == "ground_truth":
                        self.engine.config.state_feedback = "ekf"
                    elif self.engine.config.state_feedback == "ekf":
                        self.engine.config.state_feedback = "dead_reckoning"
                    else:
                        self.engine.config.state_feedback = "ground_truth"
                elif event.key == pygame.K_r:
                    self.load_scenario(self.scenario_name)
                elif event.key == pygame.K_n:
                    self.next_scenario()
                elif pygame.K_1 <= event.key <= pygame.K_9:
                    idx = event.key - pygame.K_1
                    if idx < len(self.scenarios_list):
                        self.load_scenario(self.scenarios_list[idx])

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    mu, mv = event.pos
                    if self.map_rect.collidepoint(mu, mv):
                        wx, wy = self.screen_to_world(mu, mv)
                        grid = self.engine.grid_map
                        if grid and grid.is_in_bounds(*grid.world_to_grid(wx, wy)):
                            gx, gy = grid.world_to_grid(wx, wy)
                            # Toggle obstacle state
                            is_occ = grid.is_occupied(gx, gy)
                            grid.set_occupied(gx, gy, not is_occ)
                            # Replan path avoiding modified cell
                            self.engine.replan()

    def update_simulation(self) -> None:
        """Advance the physics simulation when not paused."""
        if not self.paused and not self.engine.is_done:
            self.engine.step()

    def render(self) -> None:
        """Draw the entire visual scene: grid map, entities, routes, and sidebar HUD."""
        self.screen.fill(COLOR_BG)

        # Draw map viewport and sidebar containers
        pygame.draw.rect(self.screen, COLOR_PANEL, self.map_rect, border_radius=8)
        pygame.draw.rect(self.screen, COLOR_PANEL_BORDER, self.map_rect, width=2, border_radius=8)

        pygame.draw.rect(self.screen, COLOR_PANEL, self.sidebar_rect, border_radius=8)
        pygame.draw.rect(self.screen, COLOR_PANEL_BORDER, self.sidebar_rect, width=2, border_radius=8)

        # Render Map Elements
        self._render_grid_map()
        self._render_planned_path()
        self._render_dynamic_obstacles()
        self._render_local_trajectory()
        self._render_trajectory()
        self._render_gnss_measurements()
        self._render_markers()
        self._render_estimated_robot()
        self._render_robot()

        # Render Telemetry and Controls Sidebar
        self._render_sidebar()

        if not self.headless:
            pygame.display.flip()

    def _render_grid_map(self) -> None:
        """Render discrete occupancy grid cells and boundary lines."""
        grid = self.engine.grid_map
        if grid is None:
            return

        # Render free-space map background rect
        x0, y0 = grid.origin
        u_top_left, v_top_left = self.world_to_screen(x0, y0 + grid.height * grid.resolution)
        u_bot_right, v_bot_right = self.world_to_screen(x0 + grid.width * grid.resolution, y0)
        map_surf_rect = pygame.Rect(
            u_top_left,
            v_top_left,
            u_bot_right - u_top_left,
            v_bot_right - v_top_left,
        )
        pygame.draw.rect(self.screen, COLOR_MAP_BG, map_surf_rect)

        # Draw obstacle cells
        cell_size = grid.resolution * self.scale
        occ_indices = np.argwhere(grid.grid == 1)
        for gy, gx in occ_indices:
            wx = x0 + gx * grid.resolution
            wy = y0 + (gy + 1) * grid.resolution
            cu, cv = self.world_to_screen(wx, wy)
            cell_rect = pygame.Rect(
                cu, cv,
                int(math.ceil(cell_size)), int(math.ceil(cell_size))
            )
            pygame.draw.rect(self.screen, COLOR_OBSTACLE, cell_rect)
            pygame.draw.rect(self.screen, COLOR_OBSTACLE_BORDER, cell_rect, width=1)

        # Draw outer bounding border
        pygame.draw.rect(self.screen, (100, 110, 125), map_surf_rect, width=2)

    def _render_planned_path(self) -> None:
        """Draw global planned A* route."""
        path = self.engine.planned_path
        if not path or len(path) < 2:
            return

        screen_pts = [self.world_to_screen(wx, wy) for wx, wy in path]
        pygame.draw.lines(self.screen, COLOR_PLANNED_PATH, False, screen_pts, width=3)

        # Draw discrete waypoints
        for pt in screen_pts:
            pygame.draw.circle(self.screen, COLOR_WAYPOINT, pt, 3)

    def _render_dynamic_obstacles(self) -> None:
        """Render active moving dynamic obstacles, footprints, and motion prediction trails."""
        for obs in self.engine.dynamic_obstacles:
            ou, ov = self.world_to_screen(obs.x, obs.y)
            r_px = max(4, int(round(obs.radius * self.scale)))

            # 1. Render projected future trajectory trail (up to 1.5s ahead)
            pred_pts = obs.predict_trajectory(horizon=1.5, dt=0.2)
            if pred_pts:
                pred_screen = [self.world_to_screen(px, py) for px, py in pred_pts]
                if len(pred_screen) >= 2:
                    pygame.draw.lines(self.screen, COLOR_DYNAMIC_PREDICT, False, pred_screen, width=1)
                for ppt in pred_screen:
                    pygame.draw.circle(self.screen, COLOR_DYNAMIC_PREDICT, ppt, 2)

            # 2. Render moving obstacle body disk
            pygame.draw.circle(self.screen, COLOR_DYNAMIC_OBSTACLE, (ou, ov), r_px)
            pygame.draw.circle(self.screen, COLOR_DYNAMIC_BORDER, (ou, ov), r_px, width=2)

            # 3. Render velocity arrow
            vel_len = math.hypot(obs.vx, obs.vy)
            if vel_len > 1e-4:
                arrow_len = max(r_px * 1.3, r_px + vel_len * self.scale * 0.4)
                arrow_u = ou + int(round(arrow_len * (obs.vx / vel_len)))
                arrow_v = ov - int(round(arrow_len * (obs.vy / vel_len)))
                pygame.draw.line(self.screen, (255, 255, 255), (ou, ov), (arrow_u, arrow_v), width=2)

    def _render_local_trajectory(self) -> None:
        """Render the DWA candidate trajectory selected for immediate execution."""
        traj = self.engine.current_candidate_trajectory
        if not traj or len(traj) < 2:
            return

        screen_pts = [self.world_to_screen(wx, wy) for wx, wy in traj]
        color = COLOR_LOCAL_TRAJECTORY if self.engine.dwa_safe else (231, 76, 60)
        pygame.draw.lines(self.screen, color, False, screen_pts, width=2)

    def _render_trajectory(self) -> None:
        """Draw historical path traversed by the vehicle."""
        traj = self.engine.trajectory
        if not traj or len(traj) < 2:
            return

        screen_pts = [self.world_to_screen(s.x, s.y) for s in traj]
        pygame.draw.lines(self.screen, COLOR_TRAJECTORY, False, screen_pts, width=2)

    def _render_markers(self) -> None:
        """Draw Start and Goal landmark targets."""
        # Start marker
        start_u, start_v = self.world_to_screen(self.engine.start_pose[0], self.engine.start_pose[1])
        pygame.draw.circle(self.screen, COLOR_START, (start_u, start_v), 7)
        pygame.draw.circle(self.screen, (255, 255, 255), (start_u, start_v), 3)

        # Goal target
        goal_u, goal_v = self.world_to_screen(self.engine.goal_position[0], self.engine.goal_position[1])
        pygame.draw.circle(self.screen, COLOR_GOAL, (goal_u, goal_v), 9, width=2)
        pygame.draw.circle(self.screen, COLOR_GOAL, (goal_u, goal_v), 4)

        # Lookahead target point and tether line
        if self.engine.current_target and not self.engine.is_done:
            tx, ty = self.engine.current_target
            tu, tv = self.world_to_screen(tx, ty)
            rx, ry = self.world_to_screen(self.engine.robot.state.x, self.engine.robot.state.y)
            pygame.draw.line(self.screen, COLOR_LOOKAHEAD, (rx, ry), (tu, tv), width=1)
            pygame.draw.circle(self.screen, COLOR_LOOKAHEAD, (tu, tv), 5)

    def _render_robot(self) -> None:
        """Draw the unicycle robot chassis disk and heading arrow."""
        state = self.engine.robot.state
        ru, rv = self.world_to_screen(state.x, state.y)
        r_px = max(4, int(round(self.engine.config.collision_radius * self.scale)))

        # Draw chassis disk
        pygame.draw.circle(self.screen, COLOR_ROBOT_BODY, (ru, rv), r_px)
        pygame.draw.circle(self.screen, COLOR_ROBOT_BORDER, (ru, rv), r_px, width=2)

        # Draw heading direction quiver
        heading_len = r_px * 1.5
        hu = ru + int(round(heading_len * math.cos(state.theta)))
        # Note: screen y is inverted, so sin(theta) subtracts
        hv = rv - int(round(heading_len * math.sin(state.theta)))
        pygame.draw.line(self.screen, COLOR_ROBOT_HEADING, (ru, rv), (hu, hv), width=3)
        pygame.draw.circle(self.screen, COLOR_ROBOT_HEADING, (hu, hv), 3)

    def _render_gnss_measurements(self) -> None:
        """Draw recent GNSS measurement fixes as crosshairs."""
        if not hasattr(self.engine, "gnss_measurements"):
            return
        for gx, gy in self.engine.gnss_measurements[-15:]:
            gu, gv = self.world_to_screen(gx, gy)
            pygame.draw.line(self.screen, COLOR_GNSS, (gu - 4, gv), (gu + 4, gv), width=1)
            pygame.draw.line(self.screen, COLOR_GNSS, (gu, gv - 4), (gu, gv + 4), width=1)

    def _render_estimated_robot(self) -> None:
        """Draw estimated robot pose and 95% position uncertainty ellipse."""
        if not hasattr(self.engine, "ekf"):
            return

        est_state = self.engine.ekf.get_estimated_robot_state()
        ru, rv = self.world_to_screen(est_state.x, est_state.y)
        r_px = max(4, int(round(self.engine.config.collision_radius * self.scale)))

        # 95% positional uncertainty ellipse
        semi_major, semi_minor, phi = self.engine.ekf.get_position_covariance_ellipse(conf_level=0.95)
        if 0.05 < semi_major < 25.0 and 0.05 < semi_minor < 25.0:
            angles = np.linspace(0, 2 * math.pi, 24)
            pts = []
            for a in angles:
                lx = semi_major * math.cos(a)
                ly = semi_minor * math.sin(a)
                wx = est_state.x + (lx * math.cos(phi) - ly * math.sin(phi))
                wy = est_state.y + (lx * math.sin(phi) + ly * math.cos(phi))
                pts.append(self.world_to_screen(wx, wy))
            if len(pts) >= 3:
                pygame.draw.polygon(self.screen, (0, 180, 220), pts, width=1)

        # Draw estimated chassis outline
        pygame.draw.circle(self.screen, COLOR_EST_ROBOT, (ru, rv), r_px, width=1)

        # Draw estimated heading direction quiver
        heading_len = r_px * 1.5
        hu = ru + int(round(heading_len * math.cos(est_state.theta)))
        hv = rv - int(round(heading_len * math.sin(est_state.theta)))
        pygame.draw.line(self.screen, COLOR_EST_HEADING, (ru, rv), (hu, hv), width=2)

    def _render_sidebar(self) -> None:
        """Render real-time telemetry scorecard and interactive keyboard guide."""
        if not self.font_main:
            return

        x = self.sidebar_rect.left + 16
        y = self.sidebar_rect.top + 16
        w = self.sidebar_rect.width - 32

        # Header Title
        title_surf = self.font_large.render("AUTONOMOUS NAV", True, (255, 255, 255))
        self.screen.blit(title_surf, (x, y))
        y += 28

        scenario_surf = self.font_main.render(f"Scenario: {self.scenario_name}", True, (160, 175, 195))
        self.screen.blit(scenario_surf, (x, y))
        y += 28

        # Status Badge
        status = self._get_status_label()
        status_color = STATUS_COLORS.get(status, (200, 200, 200))
        badge_rect = pygame.Rect(x, y, w, 26)
        pygame.draw.rect(self.screen, status_color, badge_rect, border_radius=4)
        status_text = self.font_bold.render(status.replace("_", " "), True, (20, 20, 25))
        text_rect = status_text.get_rect(center=badge_rect.center)
        self.screen.blit(status_text, text_rect)
        y += 38

        # Divider
        pygame.draw.line(self.screen, COLOR_PANEL_BORDER, (x, y), (x + w, y), 1)
        y += 12

        # Telemetry Section
        traj = self.engine.trajectory
        actual_dist = calculate_trajectory_length(traj) if len(traj) > 1 else 0.0
        v_cmd, omega_cmd = self.engine.current_cmd
        if self.engine.planned_path:
            mean_cte, _ = calculate_cross_track_error([self.engine.robot.state], self.engine.planned_path)
            cte = mean_cte
        else:
            cte = 0.0

        if self.engine.waypoint_mgr:
            dist_to_goal = self.engine.waypoint_mgr.distance_to_goal(self.engine.robot.state.x, self.engine.robot.state.y)
        else:
            dist_to_goal = 0.0

        # State estimation telemetry
        if hasattr(self.engine, "ekf"):
            est_state = self.engine.ekf.get_estimated_robot_state()
            pos_err = math.hypot(self.engine.robot.state.x - est_state.x, self.engine.robot.state.y - est_state.y)
            d_th = math.atan2(math.sin(self.engine.robot.state.theta - est_state.theta), math.cos(self.engine.robot.state.theta - est_state.theta))
            head_err_deg = math.degrees(abs(d_th))
            bg_est = self.engine.ekf.x[4]
            gnss_count = len(self.engine.gnss_measurements)
        else:
            pos_err = 0.0
            head_err_deg = 0.0
            bg_est = 0.0
            gnss_count = 0

        feedback_label = {
            "ground_truth": "GROUND_TRUTH",
            "ekf": "EKF_FUSION",
            "dead_reckoning": "DEAD_RECKONING",
        }.get(self.engine.config.state_feedback, self.engine.config.state_feedback.upper())

        telemetry_items = [
            ("Nav Mode", "DWA_LOCAL" if self.engine.config.nav_mode == "dwa" else "PURE_PURSUIT"),
            ("Feedback", feedback_label),
            ("Simulated Time", f"{self.engine.simulated_time:.2f} s"),
            ("Sim Steps", f"{self.engine.steps_taken} / {self.engine.config.max_steps}"),
            ("Linear Speed", f"{self.engine.robot.state.v:.2f} m/s"),
            ("Pos Error", f"{pos_err:.2f} m"),
            ("Heading Err", f"{head_err_deg:.1f} deg"),
            ("Gyro Bias", f"{bg_est:+.3f} rad/s"),
            ("GNSS Fixes", f"{gnss_count}"),
            ("Cross-Track Err", f"{cte:.3f} m"),
            ("Dist to Goal", f"{dist_to_goal:.2f} m"),
            ("Collision State", "COLLIDED" if self.engine.collision_occurred else "SAFE"),
        ]

        for label, val in telemetry_items:
            lbl_surf = self.font_main.render(label + ":", True, (160, 175, 195))
            if label == "Collision State" and val == "COLLIDED":
                val_color = (231, 76, 60)
            elif label == "Nav Mode":
                val_color = (0, 220, 255) if val == "DWA_LOCAL" else (241, 196, 15)
            elif label == "Feedback":
                val_color = (0, 220, 255) if val == "EKF_FUSION" else ((241, 196, 15) if val == "GROUND_TRUTH" else (235, 87, 87))
            else:
                val_color = (240, 245, 255)
            val_surf = self.font_small.render(val, True, val_color)
            self.screen.blit(lbl_surf, (x, y))
            self.screen.blit(val_surf, (x + w - val_surf.get_width(), y + 2))
            y += 18

        # Divider
        y += 4
        pygame.draw.line(self.screen, COLOR_PANEL_BORDER, (x, y), (x + w, y), 1)
        y += 8

        # Interactive Controls Guide
        guide_title = self.font_bold.render("CONTROLS", True, (255, 255, 255))
        self.screen.blit(guide_title, (x, y))
        y += 18

        controls = [
            ("[SPACE]", "Pause / Resume"),
            ("[M]", "Toggle Nav Mode (DWA/PP)"),
            ("[E]", "Toggle Feedback Mode"),
            ("[S] / [->]", "Step Frame (Paused)"),
            ("[R]", "Reset Episode"),
            ("[1 - 7]", "Select Scenario"),
            ("[N]", "Next Scenario"),
            ("[Click]", "Toggle Obstacle Cell"),
            ("[ESC / Q]", "Exit Simulator"),
        ]

        for key, desc in controls:
            key_surf = self.font_small.render(key, True, (0, 206, 209))
            desc_surf = self.font_main.render(desc, True, (190, 200, 215))
            self.screen.blit(key_surf, (x, y))
            self.screen.blit(desc_surf, (x + 85, y))
            y += 18

    def _get_status_label(self) -> str:
        """Return standardized status label string strictly synchronized with engine termination state."""
        reason = self.engine.termination_reason
        if reason == "planning_failure" or self.engine.planning_failed:
            return "PLANNING_FAILURE"
        elif reason == "collision" or self.engine.collision_occurred:
            return "COLLISION"
        elif reason == "goal_reached" or self.engine.goal_reached:
            return "GOAL_REACHED"
        elif reason == "timeout" or self.engine.timeout_occurred:
            return "TIMEOUT"
        elif self.paused:
            return "PAUSED"
        elif not self.engine.is_done:
            return "RUNNING"
        else:
            return "IDLE"

    def save_screenshot(self, filepath: str | Path) -> Path:
        """Capture the current frame buffer and save to an image file."""
        path = Path(filepath)
        path.parent.mkdir(parents=True, exist_ok=True)
        # Ensure latest frame is rendered before saving
        self.render()
        pygame.image.save(self.screen, str(path))
        return path

    def run(self, max_frames: Optional[int] = None) -> None:
        """Main simulation execution and rendering loop.

        Args:
            max_frames: Optional integer limit on total frames to render (useful for automated testing).
        """
        frame_count = 0
        while self.running:
            self.handle_events()
            self.update_simulation()
            self.render()

            frame_count += 1
            if max_frames is not None and frame_count >= max_frames:
                break

            self.clock.tick(self.fps)

    def close(self) -> None:
        """Cleanly shutdown Pygame."""
        pygame.quit()
