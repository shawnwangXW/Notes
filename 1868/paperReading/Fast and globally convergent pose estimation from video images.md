## Introduction
Orthogonal Iteration (OI)—commonly referred to as the LHM algorithm after its creators Chien-Ping Lu, Gregory D. Hager, and Eric Mjolsness—is an iterative algorithm for solving the Perspective-n-Point (PnP) camera pose estimation problem. Unlike traditional nonlinear least-squares solvers (such as Gauss-Newton or Levenberg-Marquardt) that minimize 2D image-space reprojection error, OI minimizes 3D object-space collinearity error while strictly enforcing the orthogonality of the camera rotation matrix at every step

## Image space vs. Object Space
In the influential paper "Fast and Globally Convergent Pose Estimation from Video Images" (by Chien-Ping Lu, Gregory Hager, and Evan Mjolsness), object-space co-linearity is a foundational mathematical reformulation used to solve the Perspective-n-Point (PnP) pose estimation problem.   Instead of measuring errors on the 2D camera sensor (image plane), this approach measures errors directly in 3D space.   1. The Geometric Intuition: Image-Space vs. Object-SpaceUnder an ideal pinhole camera model, three points must lie on a straight line (they must be collinear):The 3D scene point ($p_i$) transformed into camera coordinates ($q_i = R p_i + t$).The corresponding 2D projection of that point on the image plane ($v_i$).The camera's center of projection (the optical origin).Image-Space Collinearity (Traditional Approach): Classical algorithms (like Gauss-Newton or Levenberg-Marquardt) project the estimated 3D point back onto the 2D image plane and minimize the distance between this projection and the observed 2D image feature. This creates a complex, highly non-linear objective function that is sensitive to initial guesses.   Object-Space Collinearity (Lu et al.'s Approach): Rather than pulling 3D points down to the 2D image plane, object-space collinearity treats the 2D image point as defining a 3D line of sight extending outward from the camera origin. The error is then measured as the shortest distance from the actual 3D point ($q_i$) in camera coordinates to that 3D line of sight.2. Mathematical FormulationGiven a 3D reference point $p_i$ transformed into camera coordinates as:$$q_i = R p_i + t$$where $R$ is the rotation matrix and $t$ is the translation vector.Let $v_i$ be the normalized 2D image vector corresponding to the point. The line of sight can be represented using a line-of-sight projection matrix $V_i$ (an orthogonal projection matrix onto the direction of $v_i$).The object-space collinearity condition states that the orthogonal projection of the 3D point $q_i$ onto the ray vector $v_i$ should equal $q_i$ itself:$$q_i = V_i q_i \quad \text{or} \quad (I - V_i)q_i = 0$$The pose estimation problem is then framed as minimizing the aggregate object-space squared error:$$\min_{R, t} \sum_{i} \Vert{} q_i - V_i q_i \Vert{}^2$$3. Why Object-Space Collinearity is a BreakthroughReformulating the error in object space rather than image space yields major computational advantages:Quadratic Translation Structure: For any fixed rotation matrix $R$, the object-space error function becomes completely quadratic with respect to translation $t$. This allows the optimal translation to be solved in closed form.Decoupling and Orthogonal Iteration (OI): It transforms a messy non-linear optimization problem into an alternating series of simpler sub-problems. The algorithm updates rotation and translation iteratively by solving standard 3D-3D absolute orientation problems using Singular Value Decomposition (SVD).   Guaranteed Global Convergence: Because the intermediate rotation estimates are always constrained to be valid orthogonal matrices via SVD (rather than relying on vulnerable parameterizations like Euler angles), the authors were able to mathematically prove that the algorithm is globally convergent—meaning it will find the correct global optimum from almost any arbitrary initial guess, typically converging in just 5 to 10 iterations.


## Core Mathematical Formulation
Given n known 3D reference points \(\mathbf{p}_{i}\) in the object/world frame and their corresponding normalized 2D image coordinates \(\mathbf{v}_i = (u_i, v_i, 1)^T\), the algorithm estimates the rigid transformation (rotation \(\mathbf{R} \in SO(3)\) and translation \(\mathbf{t} \in \mathbb{R}^3\)).

• Line-of-Sight Projection Matrix. For each observed 2D point, an orthogonal projection matrix \(\mathbf{V}_{i}\) projects any 3D point onto the line of sight passing through the camera optical center and the image point:
\(\mathbf{V}_{i}=\frac{\mathbf{v}_{i}\mathbf{v}_{i}^{T}}{\mathbf{v}_{i}^{T}\mathbf{v}_{i}}\)


• Object-Space Collinearity Error. Under an exact pose, the transformed 3D point \(\mathbf{q}_i = \mathbf{R}\mathbf{p}_i + \mathbf{t}\) lies directly on its corresponding line of sight, meaning \(\mathbf{V}_i \mathbf{q}_i = \mathbf{q}_i\). OI minimizes the sum of squared orthogonal distances from the transformed 3D points to their respective lines of sight:


$$J(R, t) =\sum _{i=1}^{n}\|{}(\mathbf{I}-\mathbf{V}_{i})(\mathbf{Rp}_{i}+\mathbf{t})\|{}^{2}= \sum_{i=1}^n \Vert{} W_i (R p_i + t) \Vert{}^2$$


## How the Algorithm Works

The optimization alternates between updating translation in closed form and updating rotation via an Orthogonal Procrustes step:
1. Eliminate Translation. Taking the derivative of \(J(\mathbf{R}, \mathbf{t})\) with respect to \(\mathbf{t}\) and setting it to zero.
   $$\frac{\partial J}{\partial t} = 2 \sum_{i=1}^n W_i^T W_i (R p_i + t) = 0$$

   Using the properties of the projection matrix ($W_i^T W_i = W_i$), this simplifies to:$$\sum_{i=1}^n W_i (R p_i + t) = 0$$Expanding and rearranging terms to isolate $t$:$$\left( \sum_{i=1}^n W_i \right) t = - \sum_{i=1}^n W_i R p_i$$

    Thus, the optimal translation $t$ as a direct function of the rotation matrix $R$ is:$$t(R) = - \left( \sum_{i=1}^n W_i \right)^{-1} \sum_{i=1}^n W_i R p_i$$
    
    This shows that translation is never searched iteratively; it is analytically locked to the current estimate of $R$. 

2. Eliminating $t$ to Form the Rotation Cost Function
   Substituting the expression for $t(R)$ back into the original error function $J(R, t)$ eliminates translation entirely, leaving a cost function $J(R)$ that depends only on the rotation matrix $R$:$$J(R) = \sum_{i=1}^n \Vert{} W_i R p_i - W_i \left( \sum_{j=1}^n W_j \right)^{-1} \sum_{j=1}^n W_j R p_j \Vert{}^2$$By defining a composite projection operator $H_i$ that accounts for both the point-specific projection and the collective translation baseline:$$H_i = W_i - W_i \left( \sum_{j=1}^n W_j \right)^{-1} W_i \quad (\text{simplified formulation})$$The cost function can be compactly expressed as a minimization over the rotation matrix:$$\min_{R} \sum_{i=1}^n \Vert{} H_i R p_i \Vert{}^2 \quad \text{subject to } R^T R = I, \; \det(R) = 1$$


3. The Orthogonal Iteration (OI) Update via SVD
   To minimize $\sum \Vert{} H_i R p_i \Vert{}^2$ with respect to $R$, Lu et al. reformulate this into a standard weighted absolute orientation problem (similar to Wahba's problem).Define modified 3D points in the camera frame: $q_i' = H_i R p_i$.The problem can be cast as finding a rotation $R$ that aligns two sets of 3D vectors. This is achieved by constructing a $3 \times 3$ cross-covariance matrix $M$:$$M = \sum_{i=1}^n (H_i p_i) (H_i q_i)^T$$Compute the Singular Value Decomposition (SVD) of $M$:$$M = U D V^T$$The globally optimal rotation update for that iteration is computed directly from the orthogonal matrices:$$R_{\text{new}} = V U^T$$

4.  Summary of the Algorithmic Loop
   * Initialize an initial guess for rotation $R$ (e.g., $R = I$).
   * Compute Translation: Calculate $t(R)$ using the closed-form equation from Step 
   * Update Rotation: Form the matrix $M$, compute its SVD, and update $R_{\text{new}} = V U^T$.
   * Check Convergence: If the change in error falls below a threshold, stop; otherwise, repeat from Step 2.
  Because $R$ is strictly enforced as a rotation matrix via SVD at every single step, the algorithm avoids local minima traps common in Euler-angle or quaternion-based non-linear optimizations, ensuring rapid and globally convergent behavior.

1. Project onto Lines of Sight. Using the current pose estimate \((\mathbf{R}^{(k)}, \mathbf{t}^{(k)})\), each transformed 3D point is projected onto its observed line of sight to create an intermediate target point \(\hat{\mathbf{q}}_i^{(k)} = \mathbf{V}_i (\mathbf{R}^{(k)}\mathbf{p}_i + \mathbf{t}^{(k)})\).
2. Update Rotation via SVD. The algorithm finds the next rotation matrix \(\mathbf{R}^{(k+1)}\) that best aligns the original centered 3D points \(\mathbf{p}_{i}\) with the projected target points \(\^{\mathbf{q}}_{i}^{(k)}\) using Singular Value Decomposition (SVD), guaranteeing that \(\mathbf{R}^{(k+1)}\) remains a valid orthonormal rotation matrix (\(\mathbf{R}^T\mathbf{R} = \mathbf{I}, \det(\mathbf{R}) = 1\)).
3. Repeat Until Convergence. Steps 2 and 3 repeat until the change in object-space error falls below a threshold.

## Key Advantages and Trade-offs

• Global Convergence. Lu et al. demonstrated that the algorithm is globally convergent from almost any initial rotation estimate, making it far less prone to diverging than standard gradient-based methods.
• Guaranteed Orthonormality. Because rotation is updated via SVD at every step, it avoids parameterization singularities (like gimbal lock) and never drifts off the SO(3) manifold.
• Planar Ambiguity Sensitivity. For planar targets (n ≥ 4 coplanar points), the object-space error surface can exhibit two local minima (pose ambiguity) when viewed from certain angles, requiring multi-start initialization or a planar PnP initializer (like IPPE) to ensure the global minimum is chosen.
• Computational Cost. Because it runs an SVD inside an iterative loop, OI is slower than modern non-iterative O(n) solvers such as EPnP or OPnP, though it remains fast enough for real-time tracking when initialized from a previous video frame.


## How to handle planar target ambiguity?
Planar target ambiguities occur when a 2D image or sensor reading of a flat object can be interpreted in more than one 3D orientation or position. This is most common in computer vision (PnP pose estimation), augmented reality (AR), and radar/sonar tracking.

1. Computer Vision & Augmented Reality (Pose Estimation)

When estimating the 3D pose (position and rotation) of a flat marker, fiducial tag (like ArUco or AprilTag), or planar surface from a single camera, perspective projection creates geometric ambiguities:
• The Necker Cube / Reflection Ambiguity: At a distance or with weak perspective, a tilted plane can appear to tilt either toward or away from the camera, creating two valid mathematical solutions for the rotation matrix.
• Rotational Symmetry: Square markers or symmetrical logos look identical when rotated by 90°, 180°, or 270°.
How to resolve them:
• Use Specialized Solvers: Instead of generic Perspective-n-Point (PnP) solvers, use algorithms designed specifically for planar targets, such as IPPE (Infinitesimal Plane-Based Pose Estimation). IPPE explicitly calculates both valid pose solutions and ranks them by reprojection error.
• Break Visual Symmetry: Design markers or targets with asymmetric features (such as a missing corner, an internal binary bit pattern, or an off-center dot) so the system can immediately determine "up" from "down."
• Temporal Tracking (Kalman Filters): Assume physical objects do not teleport or flip instantaneously between frames. Use the object's pose from the previous frame to predict its current pose, discarding any mathematical solution that requires an impossible jump in rotation.
• Sensor Fusion (IMU/Stereo): Combine camera data with an Inertial Measurement Unit (IMU) to establish a known gravity vector, or use stereo cameras to measure true depth across the plane's corners.
• Perspective Distortion Clues: Ensure the target occupies enough pixels in the camera frame so that foreshortening (the closer edge appearing slightly larger than the farther edge) is stronger than image sensor noise.



2. Radar, Sonar, and Autonomous Navigation

In radar and tracking systems, "planar target ambiguity" often refers to range/Doppler ambiguities (where pulse timing makes a target's distance or velocity wrap around) or elevation ambiguity in 2D planar arrays (where a sensor detects azimuth and range, but cannot tell if the target is high or low).
How to resolve them:
• Staggered PRF (Pulse Repetition Frequency): Vary the timing between radar pulses. True targets remain consistent across different pulse intervals, while "ghost" or ambiguous targets shift position and can be filtered out.
• Multi-Hypothesis Tracking (MHT): Maintain multiple possible tracks for an ambiguous target over several scans until new movement data proves which trajectory is physically real.
• Waveform Diversity: Modulate the frequency or phase of outgoing pulses (such as chirp waveforms) so each pulse is uniquely identifiable upon return.

## Handle Noise and Deal with Outliers
The original Orthogonal Iteration (OI) algorithm formulated by Lu, Hager, and Mjolsness relies on a least-squares ($L_2$ norm) framework. While this provides elegant math and rapid convergence for clean data, standard least-squares is inherently sensitive to gross errors.Handling noise and outliers requires distinguishing between how the algorithm naturally manages Gaussian measurement noise and how it is extended to deal with outliers (mismatched feature points).

1. Handling Gaussian Noise (The Object-Space Advantage)
   When feature points are perturbed by small, random sensor or extraction errors (Gaussian noise), the object-space collinearity formulation offers distinct mathematical advantages over traditional 2D image-space methods:
   * Avoiding Perspective Compression Bias: In image-space optimization, points far away from the camera have tiny pixel footprints, meaning a small pixel error can translate to huge 3D spatial errors. Object-space error measures distance directly along the 3D ray, treating spatial geometry more uniformly.
   * Statistical Optimality: Minimizing the sum of squared object-space distances ($\sum \Vert{}W_i q_i\Vert{}^2$) corresponds directly to Maximum Likelihood Estimation (MLE) assuming the noise on the normalized line-of-sight vectors is isotropic and Gaussian.
   * Uncertainty-Weighted Extensions: Later adaptations of the OI framework introduced custom weight matrices $w_i$ for each point. If certain feature points have higher localization uncertainty (e.g., blurry edges or occlusions), a diagonal weighting matrix can be embedded into the cost function:$$J(R, t) = \sum_{i=1}^n w_i \Vert{} W_i (R p_i + t) \Vert{}^2$$

2. Handling Outliers (Gross Mismatches)
   Because the standard cost function uses a quadratic ($L_2$) penalty, a single severe outlier (e.g., a completely incorrect feature match) will introduce a massive squared error, pulling the estimated pose heavily off target.
   To achieve robust outlier rejection, the OI algorithm is typically paired with one of two standard computer vision paradigms:
   
   A. RANSAC (Random Sample Consensus) Wrapper
   In real-world video pipelines (such as visual SLAM or object tracking), OI is rarely run raw on a set containing outliers. Instead, it sits inside a RANSAC loop:
   * RANSAC randomly samples a minimal subset of points (e.g., 3 or 4 points) to compute an initial hypothesis.
   * The Orthogonal Iteration algorithm quickly refines this hypothesis using the subset.
   * The resulting pose is tested against the entire set of points to count "inliers" (points whose object-space residual error falls below a strict pixel/distance threshold).
   * Because OI typically converges in just 5 to 10 iterations, running it thousands of times inside a RANSAC framework remains computationally efficient for real-time video frames.
   
   B. M-Estimators and Iteratively Reweighted Least Squares (IRLS)
   To handle outliers without completely discarding data via RANSAC, the cost function can be modified using robust loss functions (such as Huber loss, Tukey's biweight, or Cauchy loss) instead of a pure quadratic penalty:
   * Small errors are treated quadratically (like standard least-squares for normal noise).
   * Large errors (outliers) are capped or heavily penalized linearly, preventing them from skewing the gradient.
     
  In an IRLS implementation, the algorithm alternates between updating the pose via OI and recalculating point weights based on their current residuals—automatically scaling down or zeroing out the weights of outlier points as the iterations progress.

  ## $$M = \sum_{i=1}^n (H_i p_i) (H_i q_i)^T$$

This equation is the 
3×3 cross-covariance matrix that serves as the computational engine driving every step of the Orthogonal Iteration (OI) algorithm.

By structuring the problem into this matrix form, Lu, Hager, and Mjolsness transformed a messy, non-linear Perspective-n-Point (PnP) problem into a standard absolute orientation (Procrustes) problem that can be solved instantly using Singular Value Decomposition (SVD).   
The Computable Plant

1. Anatomy of the Equation
   * $p_i$: The known 3D coordinates of the $i$-th feature point in the object's local coordinate frame.
   * $q_i$: The corresponding 3D position of the point mapped into the camera frame ($q_i = R p_i + t$).
   * $H_i$: The composite projection operator ($H_i = W_i - W_i \left(\sum W_j\right)^{-1} W_i$), where $W_i = I - \hat{v}_i \hat{v}_i^T$ is the matrix that projects points orthogonally away from the 3D line of sight.
  
  The operator $H_i$ is crucial because it mathematically "bakes in" both the line-of-sight constraint and the closed-form solution for translation, isolating how the rotation matrix $R$ affects the spatial alignment.

  2. What $M$ Actually DoesIn statistics and linear algebra, a cross-covariance matrix measures the degree of correlation between two sets of vector coordinates.In this algorithm, $M$ accumulates the orientation relationships across all $n$ point correspondences after they have been filtered through the object-space geometry operator $H_i$. It answers the question: Given our current pose estimate, how are the object-space points correlated with the camera-space observations?
   
3. Solving for the Optimal Rotation via SVDOnce the matrix $M$ is computed for the current iteration, finding the updated rotation matrix $R_{\text{new}}$ relies on a theorem from absolute orientation (often solved via SVD):   
   * Compute the SVD of $M$:$$M = U D V^T$$where $U$ and $V$ are orthogonal matrices, and $D$ is a diagonal matrix of singular values.
   * Extract the Optimal Rotation:
The rotation matrix that maximizes alignment (maximizing the trace $\text{tr}(R^T M)$) is extracted directly as:$$R_{\text{new}} = V U^T$$

4. Why This is a MasterstrokeInstead of tweaking Euler angles or dealing with non-linear gradient steps that can get trapped in local minima, the OI algorithm leverages $M$ to force the rotation matrix onto the space of true orthogonal matrices ($R^T R = I, \det(R) = 1$) at every single iteration. This guarantees that the algorithm remains stable, highly efficient, and globally convergent.   



## Multicamera system

Using a multi-camera rig on a robot fundamentally alters the geometry and robustness of pose estimation—whether you are tracking the robot's global trajectory (Visual SLAM / VIO) or estimating the 6-DOF pose of external targets.

Transitioning from a single camera to a synchronized multi-camera system introduces several architectural, calibration, and mathematical layers:
1. Rigid Extrinsic Calibration and Rig Invariance
   In a single-camera setup, you estimate the camera's pose relative to the world. In a multi-camera rig, the cameras are rigidly mounted to the robot body frame.
   * The Mathematical Constraint: The relative rotation and translation between any two cameras ($R_{c_i}^{c_j}, t_{c_i}^{c_j}$) are treated as constant knowns determined during offline or online calibration.
   * The State Vector: Instead of solving an isolated pose for each camera, the optimization backend solves for a single unified robot body pose ($R_{\text{body}}, t_{\text{body}}$). The pose of the $k$-th camera is simply:$$R_k = R_{c_k}^{\text{body}} R_{\text{body}}, \quad t_k = R_{c_k}^{\text{body}} t_{\text{body}} + t_{c_k}^{\text{body}}$$
   * Tools: Standard toolchains like Kalibr use continuous-time trajectory estimation to simultaneously solve spatial extrinsics and temporal offsets between cameras and inertial sensors.
2. Multi-Camera PnP and Extended Object-Space Collinearity
   If you are extending the Perspective-n-Point (PnP) principles we discussed earlier to a multi-camera rig:
   * Expanded Field of View (FoV) & Degeneracy Mitigation: A single camera often suffers from geometric degeneracy when features are coplanar or when turning away from landmarks. Multi-camera setups (e.g., forward-facing stereo paired with fish-eye side/rear cameras) ensure that features are continuously tracked across camera boundaries.
   * Joint Cost Function: The optimization problem aggregates object-space or reprojection errors across all active cameras simultaneously:$$J(R_{\text{body}}, t_{\text{body}}) = \sum_{k=1}^{M} \sum_{i=1}^{n_k} \Vert{} W_{k,i} (R_k p_{k,i} + t_k) \Vert{}^2$$where $M$ is the number of cameras and $n_k$ is the number of features observed by camera $k$. Because features are viewed from completely different spatial vantage points, the depth observability and conditioning of the Jacobian matrix improve dramatically.
3. Hardware vs. Software Temporal Synchronization
   When a robot moves at high speeds, asynchronous cameras introduce motion skew. If Camera A captures a frame at $t=0.00$s and Camera B captures at $t=0.01$s, the robot has physically moved between exposures, violating the rigid-body assumption.
   * Hardware Triggering: High-performance robot rigs utilize a central microcontroller (or a dedicated FPGA/Jetson GPIO pin) to broadcast a hardware pulse (PPS or trigger signal) simultaneously to the image sensors' shutter pins.
   * Rolling vs. Global Shutter: Utilizing global shutter sensors is critical in multi-camera setups to prevent scan-line distortion artifacts from corrupting cross-camera feature matching.
4. Integration into Factor Graphs (Visual-Inertial Odometry)
   In modern robotic state estimation (using frameworks like OpenVINS, VINS-Fusion, or custom GTSAM/Ceres factor graphs), multi-camera measurements are integrated alongside IMU data:
   * Visual Factors: Each landmark observed across any camera at any timestamp adds multi-view reprojection factor edges to the sliding-window optimization graph.
   * Stereo/Baseline Advantage: Cameras mounted with overlapping fields of view provide immediate metric scale (eliminating the scale drift inherent in monocular setups), while non-overlapping cameras maximize the overall tracking workspace and redundancy.

### Optimize $J(R_{\text{body}}, t_{\text{body}}) = \sum_{k=1}^{M} \sum_{i=1}^{n_k} \Vert{} W_{k,i} (R_k p_{k,i} + t_k) \Vert{}^2$

Solving the joint multi-camera optimization problem:$$J(R_{\text{body}}, t_{\text{body}}) = \sum_{k=1}^{M} \sum_{i=1}^{n_k} \Vert{} W_{k,i} (R_k p_{k,i} + t_k) \Vert{}^2$$can be elegantly achieved by extending the Orthogonal Iteration (OI) framework. By leveraging the rigid kinematic linkage between the cameras and the robot body, we can collapse the multi-camera system into a unified single-body coordinate system, allowing us to solve for translation in closed form and rotation via Singular Value Decomposition (SVD).

#### Step 1: Unifying the Kinematic Chain
Let the robot body pose be parameterized by rotation $R_b = R_{\text{body}}$ and translation $t_b = t_{\text{body}}$.The absolute pose of the $k$-th camera is rigidly linked to the body pose via its known fixed extrinsics $(R_{bk}, t_{bk})$:$$R_k = R_b R_{bk}, \quad t_k = R_b t_{bk} + t_b$$Substituting these into the argument of the cost function for the $i$-th point observed by camera $k$:$$q_{k,i} = R_k p_{k,i} + t_k = (R_b R_{bk}) p_{k,i} + (R_b t_{bk} + t_b)$$Rearranging terms by factoring out the body rotation $R_b$:$$q_{k,i} = R_b (R_{bk} p_{k,i} + t_{bk}) + t_b$$If we define a body-referenced local point $\tilde{p}_{k,i}$ as the 3D feature point transformed into the body frame using the camera's fixed extrinsics:$$\tilde{p}_{k,i} = R_{bk} p_{k,i} + t_{bk}$$Then the camera-frame point simplifies to:$$q_{k,i} = R_b \tilde{p}_{k,i} + t_b$$This substitution is the key breakthrough: it transforms a multi-camera rig problem into a standard absolute pose estimation problem where all observations are mapped relative to a single moving robot body frame.

#### Step 2: Closed-Form Solution for Body Translation ($t_b$)
Rewriting the joint cost function with our simplified terms:$$J(R_b, t_b) = \sum_{k=1}^{M} \sum_{i=1}^{n_k} \Vert{} W_{k,i} (R_b \tilde{p}_{k,i} + t_b) \Vert{}^2$$Because the cost function is completely quadratic with respect to $t_b$, we find the optimal translation for any fixed body rotation $R_b$ by taking the partial derivative with respect to $t_b$ and setting it to zero:$$\frac{\partial J}{\partial t_b} = 2 \sum_{k=1}^{M} \sum_{i=1}^{n_k} W_{k,i}^T W_{k,i} (R_b \tilde{p}_{k,i} + t_b) = 0$$Using the property of projection matrices ($W_{k,i}^T W_{k,i} = W_{k,i}$):$$\sum_{k=1}^{M} \sum_{i=1}^{n_k} W_{k,i} (R_b \tilde{p}_{k,i} + t_b) = 0$$Isolating $t_b$ yields the analytical closed-form solution for body translation as a function of body rotation:$$t_b(R_b) = - \left( \sum_{k=1}^{M} \sum_{i=1}^{n_k} W_{k,i} \right)^{-1} \sum_{k=1}^{M} \sum_{i=1}^{n_k} W_{k,i} R_b \tilde{p}_{k,i}$$

#### Step 3: Eliminating Translation to Form the Body Rotation Cost
Substituting $t_b(R_b)$ back into the cost function eliminates translation, leaving an optimization problem dependent solely on the body rotation $R_b$:$$\min_{R_b} \sum_{k=1}^{M} \sum_{i=1}^{n_k} \Vert{} H_{k,i} R_b \tilde{p}_{k,i} \Vert{}^2$$subject to $R_b^T R_b = I, \; \det(R_b) = 1$, where $H_{k,i}$ is the multi-camera composite projection operator:$$H_{k,i} = W_{k,i} - W_{k,i} \left( \sum_{u=1}^{M} \sum_{v=1}^{n_u} W_{u,v} \right)^{-1} W_{k,i}$$

#### Step 4: Solving for Body Rotation via SVD
To minimize this cost function, we construct the aggregate $3 \times 3$ cross-covariance matrix $M_{\text{body}}$ across all cameras and points:$$M_{\text{body}} = \sum_{k=1}^{M} \sum_{i=1}^{n_k} (H_{k,i} \tilde{p}_{k,i}) (H_{k,i} q_{k,i})^T$$Computing the Singular Value Decomposition (SVD) of this matrix:$$M_{\text{body}} = U D V^T$$The globally optimal updated body rotation matrix is extracted directly:$$R_{b,\text{new}} = V U^T$$


#### Summary of the Multi-Camera Optimization Loop
* Transform Points: Map all 3D features into the robot body frame using known extrinsics ($\tilde{p}_{k,i} = R_{bk} p_{k,i} + t_{bk}$).
* Compute Translation: Solve for $t_b(R_b)$ analytically using the summed projection weights across all cameras.
* Compute Cross-Covariance: Build $M_{\text{body}}$ by aggregating contributions from every camera view.
* Update Rotation: Perform SVD on $M_{\text{body}}$ to extract the orthogonal body rotation update.
* Iterate: Repeat until convergence (typically 5–10 iterations).