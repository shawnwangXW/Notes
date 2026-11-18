# Ceres vs g2o
Ceres Solver and g2o (General Graph Optimization) are two of the most popular C++ open-source libraries for non-linear least squares optimization in robotics, computer vision, and SLAM (Simultaneous Localization and Mapping).

## 
Key Differences at a Glance


Feature	Ceres Solver	g2o (General Graph Optimization)
Primary Formulation	General non-linear least squares (sum of cost functions)	Graph-based optimization (vertices and edges)
Differentiation	Automatic differentiation (AutoDiff), numeric differentiation, and analytic Jacobians	Primarily manual/analytic or custom Jacobian implementation (defaults can vary)
Speed & Convergence	Faster convergence (e.g., ~3× faster or ~40% lower time in benchmark tests) with fewer iterations	Requires more iterations to reach equivalent precision in many standard setups
Ecosystem & Support	Large, active community, backed/used heavily (e.g., Google/Cartographer default), cross-platform (Android, iOS)	Smaller user community, maintenance is more limited
Manifolds / Lie Groups	Built-in flexible Manifold class for handling rotations and invariant errors	Graph-vertex/edge structure tailored around SE(3), SO(3), etc.

## Deep Dive


### 1. Ceres Solver (by Google)

* What it is: A general-purpose C++ library for modeling and solving large, complicated non-linear least squares problems.
* Strengths:
	* Automatic Differentiation: You write the residual/cost function plainly, and Ceres computes exact derivatives automatically without requiring you to manually derive Jacobians.
	* Flexibility: While famous for Bundle Adjustment (BA) and SLAM, it can solve general optimization problems far beyond graph structures.
	* Performance & Robustness: In recent benchmark comparisons (such as scan matching in Cartographer), Ceres consistently converges in fewer iterations and less wall-clock time with lower RMSE errors than g2o.
	* Community: It has a larger, well-maintained codebase with industrial backing and cross-platform support.


### 2. g2o (General Graph Optimization)

* What it is: A framework specialized in optimizing non-linear functions expressed as hyper-graphs (where nodes represent parameters to estimate and edges represent measurement constraints/factors).
* Strengths:
	* Graph Paradigm: Very intuitive if your problem naturally maps to a pose graph or network of vertices and edges (commonly used in classical SLAM backends like ORB-SLAM).
	* Customizability: Allows fine-grained control over solver backends (Gauss-Newton, Levenberg-Marquardt, Powell's Dogleg) and sparse linear algebra solvers (CSparse, Cholmod, etc.).
	* Niche Strengths: Some studies note g2o can exhibit specific advantages in certain localized geometric or obstacle-detection evaluations depending on the exact problem parameterization, though it typically needs more iterations than Ceres to converge.
* Weaknesses:
  * Smaller active community and slower maintenance cadence compared to Ceres.
  * Less out-of-the-box convenience for automatic differentiation, often requiring custom Jacobian math for complex cost terms.

### Which one should you pick?

* Choose Ceres Solver if: You want strong automatic differentiation, faster convergence, a well-supported active library, and flexibility for general non-linear least squares or complex bundle adjustment / sensor fusion tasks. Read more from a programmer's perspective on the Medium Overview on g2o vs GTSAM vs Ceres.
* Choose g2o if: You are maintaining an older SLAM codebase (like early versions of ORB-SLAM) or your pipeline is built natively around a vertex-and-edge hyper-graph architecture that you prefer not to refactor.