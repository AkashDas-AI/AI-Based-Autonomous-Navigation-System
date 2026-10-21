# Installation and Environment Setup

## 1. Prerequisites

- **Python**: Python 3.11 or 3.12
- **Operating System**: Windows, Linux, or macOS
- **Package Manager**: `pip` (version $\ge 23.0$)

---

## 2. Virtual Environment Setup

To isolate dependencies, set up a dedicated Python virtual environment:

### Windows (PowerShell)
```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

### Linux / macOS (Bash / Zsh)
```bash
python3 -m venv .venv
source .venv/bin/activate
```

---

## 3. Dependency Installation

### Core Runtime Dependencies
Install the project in editable mode with core dependencies (`numpy`, `matplotlib`, `opencv-python`, `pygame`):
```powershell
pip install -e .
```
Or via `requirements.txt`:
```powershell
pip install -r requirements.txt
```

### Development and Testing Tools
To run test suites and coverage analysis:
```powershell
pip install -r requirements-dev.txt
```

---

## 4. Verification

Confirm installation by running the automated test suite (160 tests passing):
```powershell
pytest -v
```

Run a sample simulation to verify graphics rendering and trajectory tracking:
```powershell
python scripts/run_simulation.py --scenario simple_obstacles --plot
```
Visual outputs will be generated in `outputs/figures/simple_obstacles_trajectory.png`.

Run the vision-based perception pipeline demonstration:
```powershell
python scripts/run_perception_demo.py
```
Visual outputs will be generated in `outputs/figures/perception_demo.png` and summary metrics in `outputs/metrics/perception_demo_summary.json`.

Run the interactive 2D Pygame simulator:
```powershell
python scripts/run_interactive.py --scenario simple_obstacles
```
Visual screenshot artifact will be saved in `outputs/figures/interactive_sim_screenshot.png`.

---

## 5. High-Fidelity Simulation Bridge Environment & Testing

The **Simulation Bridge** (`autonomous_nav.bridge`) provides a decoupled abstraction layer connecting the navigation stack with simulation backends:

### In-Process Offline Testing (Zero-Dependency)
The included `OfflineTestAdapter` is an in-process, deterministic test adapter requiring **no external 3D simulators or robotics middleware**. It runs on any standard CPU using the existing Python/NumPy environment:
```powershell
pytest -v tests/unit/test_bridge_contracts.py tests/unit/test_bridge_adapter.py tests/integration/test_bridge_navigation.py
```

### Future External 3D Simulator Requirements (Optional / Phased)
Connecting this bridge to an external 3D simulator runtime in future phases will require:
1. **CARLA Autonomous Driving Simulator**:
   - Host: Ubuntu 20.04/22.04 LTS or Windows 10/11 with WSL2
   - Hardware: Dedicated NVIDIA GPU ($\ge 8\text{ GB}$ VRAM, DirectX 11 / Vulkan), $\ge 25\text{ GB}$ disk space
   - Middleware: `carla` Python client library matching simulator server version
2. **ROS 2 / Gazebo**:
   - Host: Ubuntu 22.04 LTS (Jammy Jellyfish) or ROS 2 container
   - Middleware: ROS 2 Humble / Iron (`rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`), Gazebo (Gz Sim)
   - Packages: Nav2 integration stack

> [!NOTE]
> In Milestone 6, no external simulator binaries are downloaded or installed, preserving the lightweight, portable CPU-only development environment while providing a completely tested integration boundary.

