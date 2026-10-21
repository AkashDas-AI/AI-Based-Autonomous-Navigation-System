# Mathematical and Algorithmic Methodology

## 1. 2D Occupancy Grid Mapping & Coordinate Transforms

The spatial environment is discretized into a regular 2D grid matrix $\mathbf{M} \in \{0, 1\}^{H \times W}$, where $0$ denotes traversable free space and $1$ denotes an obstacle.

### 1.1 Forward Mapping: World to Grid
Given continuous world position $(x, y) \in \mathbb{R}^2$, map origin $(x_0, y_0)$, and metric cell resolution $\delta > 0$:
$$g_x = \left\lfloor \frac{x - x_0}{\delta} \right\rfloor, \quad g_y = \left\lfloor \frac{y - y_0}{\delta} \right\rfloor$$

### 1.2 Inverse Mapping: Grid Cell Center to World
Given discrete cell indices $(g_x, g_y) \in [0, W-1] \times [0, H-1]$:
$$x = x_0 + (g_x + 0.5)\delta, \quad y = y_0 + (g_y + 0.5)\delta$$

### 1.3 Obstacle Inflation Layer
To account for the non-zero physical radius of the mobile base and provide a safety buffer for the controller, obstacles are dilated via a discrete disk structuring element of radius $R_{\text{inf}}$:
$$\mathbf{M}_{\text{inf}}(g_x, g_y) = \bigvee_{(nx, ny) \in \mathcal{N}_{R_{\text{inf}}}(g_x, g_y)} \mathbf{M}(nx, ny)$$
where $\mathcal{N}_{R_{\text{inf}}}(g_x, g_y) = \{(nx, ny) \mid (nx - g_x)^2 \delta^2 + (ny - g_y)^2 \delta^2 \le R_{\text{inf}}^2\}$.

---

## 2. A* Path Planning Algorithm

The pathfinder computes the minimum-cost obstacle-free path across the graph of grid nodes.

### 2.1 Node Evaluation Function
$$f(n) = g(n) + h(n)$$
- $g(n)$: Exact accumulated cost from start node $s$ to node $n$.
- $h(n)$: Admissible heuristic estimate from node $n$ to goal $t$:
  $$h_{\text{euclidean}}(n, t) = \sqrt{(n_x - t_x)^2 + (n_y - t_y)^2}$$

### 2.2 8-Connectivity Motion Primitives
Nodes expand into 8 neighbors with transition costs:
- Orthogonal moves: $\Delta g = 1.0$
- Diagonal moves: $\Delta g = \sqrt{2} \approx 1.4142$

### 2.3 Corner-Cutting Prevention
To prevent diagonal tunneling through touching diagonal obstacle corners, a diagonal transition from $(x, y)$ to $(x + dx, y + dy)$ is permitted if and only if both adjacent orthogonal cells are free:
$$\neg (\mathbf{M}(x + dx, y) = 1 \land \mathbf{M}(x, y + dy) = 1)$$

---

## 3. Robot Kinematic Model

The mobile base is modeled as a differential-drive / unicycle robot constrained by non-holonomic kinematics:
$$\dot{x}(t) = v(t) \cos \theta(t)$$
$$\dot{y}(t) = v(t) \sin \theta(t)$$
$$\dot{\theta}(t) = \omega(t)$$

### 3.1 Numerical Integration (Runge-Kutta 2nd Order / Midpoint)
Given time step $\Delta t$, linear command $v_k$, and angular command $\omega_k$:
$$\theta_{\text{mid}} = \theta_k + \frac{1}{2} \omega_k \Delta t$$
$$x_{k+1} = x_k + v_k \cos(\theta_{\text{mid}}) \Delta t$$
$$y_{k+1} = y_k + v_k \sin(\theta_{\text{mid}}) \Delta t$$
$$\theta_{k+1} = \text{atan2}\left(\sin(\theta_k + \omega_k \Delta t), \cos(\theta_k + \omega_k \Delta t)\right)$$

Velocity commands are subjected to acceleration limits:
$$|v_{k+1} - v_k| \le a_{\max} \Delta t, \quad |\omega_k| \le \omega_{\max}$$

---

## 4. Trajectory Tracking & Control Law

The system utilizes an enhanced **Pure Pursuit** controller with adaptive deceleration.

### 4.1 Target Angle and Heading Error
Given robot pose $(x, y, \theta)$ and lookahead target $(x_L, y_L)$:
$$\theta_{\text{target}} = \text{atan2}(y_L - y, x_L - x)$$
$$\alpha = \text{atan2}\left(\sin(\theta_{\text{target}} - \theta), \cos(\theta_{\text{target}} - \theta)\right)$$

### 4.2 Control Commands
$$\omega_{\text{cmd}} = K_\omega \cdot \alpha$$
$$v_{\text{cmd}} = v_{\text{target}} \cdot \max(0.2, \cos \alpha) \cdot \min\left(1.0, \max\left(0.1, \frac{d_{\text{goal}}}{2.0}\right)\right)$$
- If the heading error $\alpha$ is large, the vehicle decelerates smoothly to prevent lateral overshoot.
- As the remaining distance to goal $d_{\text{goal}}$ drops below $2\text{ m}$, the vehicle smoothly ramps down speed to achieve accurate terminal positioning.

---

## 5. Collision Detection (Circle-to-AABB Exact Intersection)

The vehicle footprint is approximated as a circular disk of radius $R_{\text{veh}}$ centered at $(x, y)$.

For each grid cell $(g_x, g_y)$ marked occupied within the search bounding box:
- Cell horizontal bounds: $[x_{\min}, x_{\max}] = [x_0 + g_x \delta, x_0 + (g_x + 1)\delta]$
- Cell vertical bounds: $[y_{\min}, y_{\max}] = [y_0 + g_y \delta, y_0 + (g_y + 1)\delta]$

The closest point on the cell $(c_x, c_y)$ to $(x, y)$ is computed via coordinate clamping:
$$c_x = \max(x_{\min}, \min(x, x_{\max})), \quad c_y = \max(y_{\min}, \min(y, y_{\max}))$$

The vehicle collides if:
$$(x - c_x)^2 + (y - c_y)^2 \le R_{\text{veh}}^2$$
This formulation guarantees zero penetration errors across cell faces, edges, and corners.

---

## 6. Vision-Based Obstacle Perception & Camera Geometry

The perception extension extracts traversable space and obstacle boundaries from synthetic camera observations without requiring deep neural networks.

### 6.1 Image Preprocessing & Filtering
Camera observations $\mathbf{I} \in \mathbb{R}^{H \times W \times 3}$ (BGR format) are converted to single-channel intensity maps $\mathbf{I}_{\text{gray}}$:
$$\mathbf{I}_{\text{gray}}(u, v) = 0.114 B(u, v) + 0.587 G(u, v) + 0.299 R(u, v)$$

To attenuate sensor noise while preserving structural obstacle boundaries, Gaussian smoothing is applied:
$$\mathbf{I}_{\text{blur}}(u, v) = \mathbf{I}_{\text{gray}} * G_\sigma(u, v), \quad G_\sigma(u, v) = \frac{1}{2\pi \sigma^2} \exp\left(-\frac{u^2 + v^2}{2\sigma^2}\right)$$

Contrast enhancement scales the dynamic range across the full $[0, 255]$ spectrum:
$$\mathbf{I}_{\text{norm}}(u, v) = 255 \cdot \frac{\mathbf{I}_{\text{blur}}(u, v) - \min(\mathbf{I}_{\text{blur}})}{\max(\mathbf{I}_{\text{blur}}) - \min(\mathbf{I}_{\text{blur}}) + \epsilon}$$

### 6.2 Segmentation & Morphological Filtering
Obstacle pixels are segmented via Otsu's thresholding, which deterministically finds the optimal threshold $t^*$ maximizing inter-class variance $\sigma_b^2$:
$$t^* = \arg\max_{t} \left\{ \omega_0(t) \omega_1(t) \left[\mu_0(t) - \mu_1(t)\right]^2 \right\}$$
where $\omega_0(t), \omega_1(t)$ are cumulative class probabilities and $\mu_0(t), \mu_1(t)$ are class means.

Morphological closing ($A \bullet B = (A \oplus B) \ominus B$) followed by opening ($A \circ B = (A \ominus B) \oplus B$) with an elliptical structuring element $B_{3 \times 3}$ bridges pixel discontinuities and suppresses isolated salt-and-pepper noise.

### 6.3 Contour Extraction and Spatial Moments
Connected obstacle contours are parameterized using 2D spatial moments:
$$m_{pq} = \sum_{u} \sum_{v} u^p v^q \mathbf{M}_{\text{bin}}(u, v)$$

The obstacle centroid in pixel space is given by:
$$\bar{u} = \frac{m_{10}}{m_{00}}, \quad \bar{v} = \frac{m_{01}}{m_{00}}, \quad \text{Area} = m_{00}$$

Spurious noise detections with area $A < A_{\min}$ or background boundaries with $A > A_{\max}$ are rejected.

### 6.4 Camera Projection Models

#### 6.4.1 Orthographic Top-Down Projection
For overhead surveillance cameras, the mapping is affine with vertical axis inversion (matching image top-left origin to world Cartesian origin):
$$u = (x - x_{\min}) \cdot s_x, \quad v = (y_{\max} - y) \cdot s_y$$
Inverse metric reconstruction:
$$x = x_{\min} + \frac{u}{s_x}, \quad y = y_{\max} - \frac{v}{s_y}$$

#### 6.4.2 Planar Perspective Projection (Homography & IPM)
For a forward-facing camera mounted at height $h > 0$ and tilted downward by pitch angle $\alpha \in (0, \pi/2)$ observing a strictly planar ground ($Z = 0$), the transformation from ground coordinates $\mathbf{x}_{\text{ground}} = [X_g, Y_g, 1]^T$ (where $X_g$ is forward and $Y_g$ is lateral left) to pixel coordinates $[u, v, 1]^T$ is governed by the 3D optical projection:
$$Z_c = X_g \cos\alpha + h \sin\alpha, \quad X_c = -Y_g, \quad Y_c = -X_g \sin\alpha + h \cos\alpha$$
$$u = c_x + f_x \frac{X_c}{Z_c} = c_x - f_x \frac{Y_g}{Z_c}$$
$$v = c_y + f_y \frac{Y_c}{Z_c} = c_y + f_y \frac{-X_g \sin\alpha + h \cos\alpha}{X_g \cos\alpha + h \sin\alpha}$$

In homogeneous matrix form, $[u', v', w']^T = \mathbf{H} [X_g, Y_g, 1]^T$ where:
$$\mathbf{H} = \begin{bmatrix} c_x \cos\alpha & -f_x & c_x h \sin\alpha \\ c_y \cos\alpha - f_y \sin\alpha & 0 & c_y h \sin\alpha + f_y h \cos\alpha \\ \cos\alpha & 0 & h \sin\alpha \end{bmatrix}$$

Inverse Perspective Mapping (IPM) computes ground coordinates via the matrix inverse $\mathbf{H}^{-1}$:
$$\lambda' \begin{bmatrix} X_g \\ Y_g \\ 1 \end{bmatrix} = \mathbf{H}^{-1} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}$$
with metric recovery: $X_g = \frac{(\mathbf{H}^{-1} \mathbf{x}_{\text{img}})_1}{(\mathbf{H}^{-1} \mathbf{x}_{\text{img}})_3}$ and $Y_g = \frac{(\mathbf{H}^{-1} \mathbf{x}_{\text{img}})_2}{(\mathbf{H}^{-1} \mathbf{x}_{\text{img}})_3}$, valid for $X_g > 0$ (in front of camera horizon) and points within sensor bounds.

For empirical calibration, $\mathbf{H}$ can also be estimated directly from $N \ge 4$ coplanar point correspondences using Direct Linear Transformation (DLT) or OpenCV's `getPerspectiveTransform` / `findHomography`, provided no 3 points are collinear.

#### 6.4.3 Critical Raised-Obstacle Parallax Limitation
A planar homography assumes all projected pixels originate from the ground surface ($Z = 0$). 
If an obstacle extends into 3D space with height $H_{\text{obs}} > 0$:
- The elevated top of the obstacle at $(X, Y, H_{\text{obs}})$ projects to a higher optical position in the image.
- When back-projected through $\mathbf{H}^{-1}$ under the false assumption that $Z = 0$, the projected point lands at a ground distance $X' > X$ behind the obstacle.
- Consequently, 2D planar homography smears and elongates raised obstacles along the camera's radial lines of sight. It cannot reconstruct the true ground footprint without 3D depth sensing (stereo, LiDAR, or depth cameras).

---

## 7. Error Decoupling in Vision-to-Grid Mapping

To evaluate perception and mapping rigor, errors are categorized into three orthogonal components:
1. **Image Segmentation Error ($\epsilon_{\text{seg}}$)**: Pixel-level discrepancies between true projected obstacle boundaries and Otsu/morphological contour masks in image space. In high-contrast synthetic scenes, this is sub-pixel ($\approx 0$).
2. **Coordinate Transformation Error ($\epsilon_{\text{proj}}$)**: Geometric mapping inaccuracies between pixel $(u, v)$ and continuous world $(x, y)$. For calibrated cameras, this error is governed by numerical floating-point precision ($< 10^{-4}\text{ m}$).
3. **Grid Discretization Error ($\epsilon_{\text{grid}}$)**: Spatial quantization arising from rasterizing continuous metric shapes into discrete cells of width $\delta = 0.5\text{ m}$. A cell is occupied if any portion of the obstacle intersects its bounds, introducing a boundary-cell uncertainty bounded by $\delta / \sqrt{2} \approx 0.35\text{ m}$.

Centroid localization accuracy ($< 0.05\text{ m}$) evaluates center of mass, whereas footprint accuracy evaluates filled polygon Intersection-over-Union ($\text{IoU} \ge 0.90$). Centroid accuracy must not be conflated with footprint coverage.


