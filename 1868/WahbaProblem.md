Wahba's problem is a fundamental optimization problem in aerospace engineering, computer vision, and robotics. It asks a simple geometric question: Given two sets of corresponding vectors measured in different coordinate systems, what is the optimal rotation matrix that best aligns them?

Formulated by Grace Wahba in 1965, it is the exact mathematical problem solved under the hood by the Singular Value Decomposition (SVD) step in the Orthogonal Iteration pose estimation algorithm we discussed earlier.

## 1. The Mathematical Formulation
Suppose you have a set of $N$ vectors measured in a reference coordinate system (such as known 3D star tracker directions or 3D model points):$$\{v_1, v_2, \dots, v_n\}$$And you have the corresponding measurements of those same vectors in a body or sensor coordinate system:$$\{w_1, w_2, \dots, w_n\}$$Wahba's problem seeks to find the 3D rotation matrix $R$ that minimizes the weighted sum of squared differences between the rotated reference vectors and the sensor measurements:$$J(R) = \frac{1}{2} \sum_{i=1}^n a_i \Vert{} w_i - R v_i \Vert{}^2$$Subject to the rigid rotation constraints:$$R^T R = I \quad \text{and} \quad \det(R) = 1$$


## 2. Expanding the Cost Function
By expanding the norm squared ($\Vert{}w_i - R v_i\Vert{}^2 = w_i^T w_i + v_i^T v_i - 2 w_i^T R v_i$), and noting that the terms involving just $w_i$ and $v_i$ are constant with respect to rotation, minimizing Wahba's cost function simplifies to maximizing the trace term:$$\max_{R} \, \text{tr}(R B^T) \quad \text{or equivalently} \quad \text{tr}(R^T B)$$Where $B$ is the $3 \times 3$ cross-covariance matrix (or attitude profile matrix) accumulated from the vector pairs:$$B = \sum_{i=1}^n a_i w_i v_i^T$$


## 3. How It Is Solved
Because Wahba's problem arises in real-time attitude control (such as determining a satellite's orientation using star trackers and sun sensors), mathematicians developed several ultra-fast, closed-form solutions that bypass slow iterative optimization:
* Wahba's Original SVD Method: Compute the Singular Value Decomposition of the $3 \times 3$ matrix $B = U D V^T$. The optimal rotation matrix is extracted directly as:$$R = U \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & \det(U)\det(V) \end{bmatrix} V^T$$(This ensures the determinant is strictly $+1$, preventing improper reflections).
* Other Famous Solutions: QUEST (Quaternion Estimator), Davenport's $q$-method (which casts it as an eigenvalue problem), and ESOQ.

## 4. Connection to Pose Estimation (PnP)
In standard Perspective-n-Point algorithms like Orthogonal Iteration, the problem isn't a pure Wahba problem initially because translation ($t$) is unknown.However, once the object-space formulation removes translation using projection operators ($H_i$), the remaining rotation optimization step collapses directly into a generalized Wahba's problem—allowing SVD to instantly compute the best-fit rotation matrix.