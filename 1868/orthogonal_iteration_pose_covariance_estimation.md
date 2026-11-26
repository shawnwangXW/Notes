# Covariance Estimation for Orthogonal Iteration (OI) Pose Estimation

To add covariance estimation to an Orthogonal Iteration (OI) pose solver (Lu, Hager, and Mjolsness), you need to bridge the **object-space collinearity** optimization that OI solves with the **image-space measurement noise** (2D pixel or normalized ray uncertainty) that drives actual pose variance in downstream filters or factor graphs.

Because OI enforces $\mathbf{R} \in SO(3)$ via SVD at every step rather than parameterizing $\mathbf{R}$ directly inside a single stacked Gauss–Newton solve, the cleanest way to obtain a physically meaningful $6 \times 6$ pose covariance is a **post-convergence first-order (Gauss–Newton / Laplace) approximation on $\mathfrak{se}(3)$ or $\mathfrak{so}(3) \times \mathbb{R}^3$** evaluated at the converged pose $(\hat{\mathbf{R}}, \hat{\mathbf{t}})$—optionally paired with a **weighted OI** variant when points have heterogeneous noise.

---

## 1. Problem Setup & Residual Choice

Let 3D model points be $\mathbf{P}_i \in \mathbb{R}^3$ ($i = 1, \dots, N$), and let the pose mapping world to camera be:
$$\mathbf{X}_i = \mathbf{R}\mathbf{P}_i + \mathbf{t} = \begin{bmatrix} X_i \\ Y_i \\ Z_i \end{bmatrix} \tag{1}$$

In OI, each normalized image measurement $\tilde{\mathbf{v}}_i = [u_i,\, v_i,\, 1]^\top$ defines a unit viewing ray $\mathbf{v}_i = \tilde{\mathbf{v}}_i / \|\tilde{\mathbf{v}}_i\|_2$, and OI minimizes the **object-space collinearity error**:
$$\mathbf{e}_i^{\text{obj}}(\mathbf{R}, \mathbf{t}) = (\mathbf{I}_3 - \mathbf{V}_i)\mathbf{X}_i, \qquad \mathbf{V}_i = \mathbf{v}_i \mathbf{v}_i^\top \tag{2}$$

### Crucial Distinction for Covariance
* **Object-space residuals $\mathbf{e}_i^{\text{obj}}$** scale with depth $Z_i$ and mix tangential/radial ray geometry, so applying $\sigma_{\text{pix}}^2 \mathbf{I}_3$ directly on $\mathbf{e}_i^{\text{obj}}$ over-weights distant points and distorts covariance units.
* **Image-space reprojection residuals** $\mathbf{r}_i(\mathbf{R}, \mathbf{t}) = \hat{\mathbf{u}}_i(\mathbf{R}, \mathbf{t}) - \mathbf{u}_i^{\text{obs}} \in \mathbb{R}^2$ (where $\mathbf{u}_i^{\text{obs}} \in \mathbb{R}^2$ has 2D pixel covariance $\boldsymbol{\Sigma}_{u,i}$, typically $\sigma_u^2 \mathbf{I}_2$) directly tie pose uncertainty to sensor noise.

Once OI converges to $(\hat{\mathbf{R}}, \hat{\mathbf{t}})$, compute the $6 \times 6$ covariance via **image-space reprojection Gauss–Newton** (recommended for downstream fusion).

---

## 2. Recommended Method: Image-Space Gauss–Newton Covariance

Use OI for fast, globally convergent pose estimation, then perform **one** Jacobian evaluation (and optionally 1–2 damped refinement steps) around $(\hat{\mathbf{R}}, \hat{\mathbf{t}})$ with a minimal 6-DOF perturbation $\delta\boldsymbol{\xi} = [\delta\boldsymbol{\omega}^\top,\, \delta\mathbf{t}^\top]^\top \in \mathbb{R}^6$:

* **Left-perturbed rotation:**
  $$\mathbf{R} \leftarrow \exp([\delta\boldsymbol{\omega}]_\times)\hat{\mathbf{R}} \implies \delta\mathbf{X}_i \approx -[\hat{\mathbf{X}}_i]_\times \delta\boldsymbol{\omega} + \delta\mathbf{t} \tag{3}$$
  where $[\mathbf{a}]_\times$ denotes the $3 \times 3$ skew-symmetric cross-product matrix of $\mathbf{a}$.

* **Camera projection & Jacobian:**
  $$\hat{\mathbf{u}}_i = \begin{bmatrix} f_x \frac{X_i}{Z_i} + c_x \\ f_y \frac{Y_i}{Z_i} + c_y \end{bmatrix}, \qquad \frac{\partial \hat{\mathbf{u}}_i}{\partial \mathbf{X}_i} = \begin{bmatrix} \frac{f_x}{Z_i} & 0 & -\frac{f_x X_i}{Z_i^2} \\ 0 & \frac{f_y}{Z_i} & -\frac{f_y Y_i}{Z_i^2} \end{bmatrix} \tag{4}$$

By the chain rule, the $2 \times 6$ Jacobian for point $i$ is:
$$\mathbf{J}_i = \frac{\partial \hat{\mathbf{u}}_i}{\partial \mathbf{X}_i} \begin{bmatrix} -[\hat{\mathbf{X}}_i]_\times & \mathbf{I}_3 \end{bmatrix} \in \mathbb{R}^{2 \times 6} \tag{5}$$

Equation (5) follows directly from differentiating the reprojection residual about the converged pose. First, let
$$\mathbf{X}_i(\delta\boldsymbol{\omega},\delta\mathbf{t})=\exp([\delta\boldsymbol{\omega}]_\times)\hat{\mathbf{R}}\mathbf{P}_i+\hat{\mathbf{t}}+\delta\mathbf{t} \tag{5a}$$
and use $\exp([\delta\boldsymbol{\omega}]_\times)\approx\mathbf{I}_3+[\delta\boldsymbol{\omega}]_\times$ and $\hat{\mathbf{X}}_i=\hat{\mathbf{R}}\mathbf{P}_i+\hat{\mathbf{t}}$. Expanding to first order gives
$$\mathbf{X}_i\approx\hat{\mathbf{X}}_i+[\delta\boldsymbol{\omega}]_\times\hat{\mathbf{X}}_i+\delta\mathbf{t}
=\hat{\mathbf{X}}_i-[\hat{\mathbf{X}}_i]_\times\delta\boldsymbol{\omega}+\delta\mathbf{t}, \tag{5b}$$
where $[\delta\boldsymbol{\omega}]_\times\hat{\mathbf{X}}_i=-[\hat{\mathbf{X}}_i]_\times\delta\boldsymbol{\omega}$ was used. Thus the camera-point derivative is
$$\frac{\partial\mathbf{X}_i}{\partial\delta\boldsymbol{\xi}}
=\begin{bmatrix}-[\hat{\mathbf{X}}_i]_\times & \mathbf{I}_3\end{bmatrix},
\qquad
\delta\boldsymbol{\xi}=\begin{bmatrix}\delta\boldsymbol{\omega}\\\delta\mathbf{t}\end{bmatrix}. \tag{5c}$$

Next, differentiating the pinhole projection
$$\hat{\mathbf{u}}_i
=\begin{bmatrix}
f_xX_i/Z_i+c_x\\
f_yY_i/Z_i+c_y
\end{bmatrix} \tag{5d}$$
with respect to $\mathbf{X}_i=[X_i,Y_i,Z_i]^\top$ yields
$$\frac{\partial\hat{\mathbf{u}}_i}{\partial\mathbf{X}_i}
=\begin{bmatrix}
\frac{f_x}{Z_i}&0&-\frac{f_xX_i}{Z_i^2}\\
0&\frac{f_y}{Z_i}&-\frac{f_yY_i}{Z_i^2}
\end{bmatrix}. \tag{5e}$$
Multiplying this $2\times3$ projection derivative by the $3\times6$ pose derivative in (5c) gives
$$\delta\hat{\mathbf{u}}_i
=\mathbf{J}_i\,\delta\boldsymbol{\xi}
=\frac{\partial\hat{\mathbf{u}}_i}{\partial\mathbf{X}_i}\left(-[\hat{\mathbf{X}}_i]_\times\delta\boldsymbol{\omega}+\delta\mathbf{t}\right). \tag{5f}$$
Therefore, **the left $2\times3$ block of $\mathbf{J}_i$ contains the rotational sensitivity and the right $2\times3$ block contains the translational sensitivity**:
$$\mathbf{J}_{i,\boldsymbol{\omega}}
=-\frac{\partial\hat{\mathbf{u}}_i}{\partial\mathbf{X}_i}[\hat{\mathbf{X}}_i]_\times,
\qquad
\mathbf{J}_{i,\mathbf{t}}
=\frac{\partial\hat{\mathbf{u}}_i}{\partial\mathbf{X}_i}. \tag{5g}$$
The two blocks are $2\times3$, so stacking them horizontally gives $\mathbf{J}_i=[\mathbf{J}_{i,\boldsymbol{\omega}}\;\mathbf{J}_{i,\mathbf{t}}]\in\mathbb{R}^{2\times6}$, exactly equation (5). If a right perturbation $\mathbf{R}\leftarrow\hat{\mathbf{R}}\exp([\delta\boldsymbol{\omega}]_\times)$ is used instead, the rotational block becomes the corresponding right-perturbation expression and must be kept consistent with the covariance tangent convention.

With measurement noise covariance $\boldsymbol{\Sigma}_{u,i} \in \mathbb{R}^{2 \times 2}$ (and information matrix $\mathbf{W}_i = \boldsymbol{\Sigma}_{u,i}^{-1}$), the $6 \times 6$ Fisher information (Gauss–Newton Hessian) and pose covariance matrices are:
$$\mathbf{H} = \sum_{i=1}^{N} \mathbf{J}_i^\top \mathbf{W}_i \mathbf{J}_i + \boldsymbol{\Lambda}_{\text{prior}}, \qquad \boldsymbol{\Sigma}_{\boldsymbol{\xi}} = \mathbf{H}^{-1} \tag{6}$$

### Practical Considerations
* **Planar / degenerate targets:** For coplanar points or near-axis viewing angles, $\mathbf{H}$ can be ill-conditioned or rank-deficient (e.g., depth/yaw ambiguity on a flat fiducial tag). Adding a small diagonal prior $\boldsymbol{\Lambda}_{\text{prior}}$ or using SVD/Cholesky with condition-number checks prevents numerical blow-ups and flags observability issues.
* **Outlier handling:** Scale $\mathbf{W}_i$ by an IRLS/Huber/Tukey weight $w_i$ if using robust correspondences.
* **Unknown pixel noise scale:** If $\sigma_u^2$ is unknown a priori, estimate the posterior variance factor from the residual sum of squares at convergence:
  $$\hat{\sigma}_u^2 = \frac{1}{2N - 6} \sum_{i=1}^{N} \|\hat{\mathbf{u}}_i - \mathbf{u}_i^{\text{obs}}\|_2^2 \tag{7}$$
  and scale
  $$\boldsymbol{\Sigma}_{\boldsymbol{\xi}} = \hat{\sigma}_u^2 \left(\sum_{i=1}^N \mathbf{J}_i^\top \mathbf{J}_i\right)^{-1}. \tag{8}$$

---

## 3. Alternative: Weighted OI When Point Uncertainties Differ

If feature points or corners have known heterogeneous noise (e.g., blurred corners, fiducial corner confidence, or depth-derived uncertainty), standard unweighted OI treats every point equally.

In the OI update step, each point $i$ has an orthogonal projector $\mathbf{V}_i = \mathbf{v}_i \mathbf{v}_i^\top$ and a scalar weight $w_i$ derived from its inverse variance. Using weighted centroids:
$$\bar{\mathbf{P}}_w = \frac{\sum_{i=1}^N w_i \mathbf{P}_i}{\sum_{i=1}^N w_i}, \qquad \bar{\mathbf{X}}_w = \frac{\sum_{i=1}^N w_i \mathbf{V}_i \hat{\mathbf{X}}_i}{\sum_{i=1}^N w_i} \tag{9}$$
the weighted Procrustes cross-covariance matrix becomes:
$$\mathbf{C} = \sum_{i=1}^{N} w_i (\mathbf{V}_i \hat{\mathbf{X}}_i - \bar{\mathbf{X}}_w)(\mathbf{P}_i - \bar{\mathbf{P}}_w)^\top \tag{10}$$
This preserves the fast SVD rotation update while prioritizing higher-precision keypoints. You can still compute $\boldsymbol{\Sigma}_{\boldsymbol{\xi}}$ afterward via Section 2 using the actual per-point $\boldsymbol{\Sigma}_{u,i}$ so the covariance reflects true pixel noise rather than heuristic weights.

---

## 4. Implementation Checklist & Convention Traps

| Aspect | What to Watch For |
| :--- | :--- |
| **Frame convention** | Confirm whether $(\mathbf{R}, \mathbf{t})$ maps world-to-camera ($\mathbf{X}_c = \mathbf{R}\mathbf{X}_w + \mathbf{t}$) or camera-to-world ($\mathbf{X}_w = \mathbf{R}_{wc}\mathbf{X}_c + \mathbf{t}_{wc}$). Invert or transform $\boldsymbol{\Sigma}_{\boldsymbol{\xi}}$ via the $\mathrm{SE}(3)$ adjoint if your downstream filter expects the opposite frame. |
| **Tangent space order** | Document whether $\delta\boldsymbol{\xi}$ is ordered $[\delta\boldsymbol{\omega}, \delta\mathbf{t}]$ or $[\delta\mathbf{t}, \delta\boldsymbol{\omega}]$, and whether $\delta\boldsymbol{\omega}$ is a **left** perturbation ($\exp([\delta\boldsymbol{\omega}]_\times)\hat{\mathbf{R}}$) or **right** perturbation ($\hat{\mathbf{R}}\exp([\delta\boldsymbol{\omega}]_\times)$). |
| **Lens distortion** | If $\mathbf{u}_i^{\text{obs}}$ is undistorted prior to OI, propagate the undistortion Jacobian $\mathbf{J}_{\text{undist},i}$ into $\boldsymbol{\Sigma}_{u,i} = \mathbf{J}_{\text{undist},i} \boldsymbol{\Sigma}_{\text{raw},i} \mathbf{J}_{\text{undist},i}^\top$. |
| **Scale / units** | Keep translation in meters and rotation increments in radians so cross-terms in the $6 \times 6$ covariance are dimensionally consistent. |