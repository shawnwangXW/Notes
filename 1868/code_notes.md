
## void ApriltagDetector::HandleImage


## void GpuDetector::DecodeTags()  
GpuDetector::DecodeTags() takes the detected quad corners and decodes them into AprilTag detections.

* Choose a chunk size: It divides the quads into batches, aiming for about APRILTAG_TASKS_PER_THREAD_TARGET tasks per worker thread. The + 1 ensures chunksize is at least 1.
* Prepare task storage: tasks is sized to hold the batches. Each task will handle a range of quad indices.
* Clear old results: Existing detections are destroyed and the detections array is emptied so this run starts fresh.
* Describe the image: im_orig wraps the grayscale image pointer and its dimensions/stride in the format expected by the decoder.
* Queue decoding work: For each chunk, the code fills in the task’s index range and shared inputs (quads, detector, image, camera parameters), then submits it to the worker pool.
* Wait and finalize: workerpool_run() executes the queued tasks and waits for them to finish. The code then reconciles overlapping detections and sorts the results.
The local im_orig remains alive while the worker tasks use it because the function waits for those tasks before returning.


It is still not clear.

Homography for each quad:
GpuDetector::DecodeTags ==> workerpool_add_task ==> GpuDetector::QuadDecodeTask ==> quad_decode_index


## Calculate H
quad_update_homographies constructs a projective transform from the four ideal tag-square corners to the four detected image corners. The calculation is in apriltag.c.

For each corner index i, it pairs a point in tag coordinates with quad->p[i] in image pixels:

Tag coordinates: (-1,-1), (1,-1), (1,1), (-1,1)
Image coordinates: quad->p[0] through quad->p[3]
The homography H maps between them:

$$
\lambda
\left[\begin{array}{c}u\\v\\1\end{array}\right] =
H\left [
\begin{array}{c}x\\y\\1\end{array}\right]
$$

Here (x,y) is on the ideal tag square, (u,v) is in the image, and λ is a scale factor. A 3×3 homography has eight independent values, since its overall scale does not matter. homography_compute2 fixes the bottom-right value to 1, builds eight equations from the four point pairs, and solves them with Gaussian elimination.

quad_update_homographies then inverts H to get Hinv. If either transform cannot be computed, it reports failure; otherwise, the pair can map points in either direction. The tag’s decoded bit orientation is applied later, when the detection homography is formed.

Function **homography_compute2** actually implements the algorithm. It computes a 3×3 projective homography mapping the four corners of an ideal square to the four observed image corners. In this caller, each c[i] is {source_x, source_y, image_x, image_y}: the source coordinates are the square’s corners (±1, ±1), and the image coordinates come from the detected quad.

A homography maps a point (x, y) to (u, v) as:

$$
u=\frac{h_{00}x+h_{01}y+h_{02}}{h_{20}x+h_{21}y+1},\qquad
v=\frac{h_{10}x+h_{11}y+h_{12}}{h_{20}x+h_{21}y+1}
$$

The final 1 ($h_{22}$) fixes the homography’s otherwise arbitrary scale, leaving eight unknowns. Each corner correspondence contributes two equations, so four corners give eight equations.

A is the flattened 8×9 augmented matrix for those equations. Each group of two rows corresponds to one corner: the first solves for u, the second for v. The ninth value in each row is the known image coordinate (the right-hand side).

The elimination loop uses Gaussian elimination with partial pivoting: it picks the largest available pivot in each column, swaps rows, then eliminates below the pivot.
If a pivot is smaller than epsilon, the system is treated as singular and the function returns NULL.

The back-solve loop finds the eight homography coefficients. The returned matrix places them in row-major order and uses 1 as the bottom-right coefficient.

So, the returned matrix transforms coordinates on the ideal square into image coordinates. The caller stores it as quad->H and computes its inverse for the reverse mapping.


## quad_decode 
It reads the bits of a detected AprilTag, decides whether each is black or white, and looks up the resulting codeword.

Estimate local black and white brightness. patterns describes lines through the tag’s known black and white border cells. The function samples those cells in the image, mapping tag coordinates to pixels with quad->H. It fits a simple brightness plane to each color, so the estimated levels can vary across the tag if lighting is uneven.

Check border polarity. It compares the estimated black and white levels at the tag center against family->reversed_border. If they don’t match, it returns -1, rejecting the candidate.

Sample each data bit. For every bit listed in family->bit_x and family->bit_y, it projects the cell center into the image and samples its pixel value. It subtracts the local midpoint between the estimated black and white brightnesses:

Positive result → white bit
Zero or negative result → black bit
The resulting signed values are stored in values; their magnitude represents how far each sample is from the threshold.

Optionally sharpen the samples. sharpen adds a scaled Laplacian filter to values, controlled by td->decode_sharpening. This can emphasize contrast between neighboring cells.

Build and look up the codeword. The function shifts each bit into rcode and calls quick_decode_codeword, which checks the codeword and its four rotations against the family’s known tags.

Return a confidence score. It averages the positive margins and the magnitudes of the negative margins, then returns the smaller average. A larger score means both white and black bits were, on average, well separated from the threshold. The optional im_samples image is used to visualize sample classifications.

In short, this function turns the detected quad into a sampled bit pattern, checks its border, decodes the tag ID and orientation, and reports how confidently the samples separated into black and white. It’s defined in apriltag.c.


## homography_to_pose - inputs: H, fx, fy, cx, cy
It converts a 3×3 homography—which maps points on the tag’s flat plane into image coordinates—into a 4×4 pose matrix describing the tag’s rotation and translation relative to the camera.

The homography is only defined up to an arbitrary scale. The code first removes the camera intrinsics (fx, fy, cx, cy) from its entries, recovering scaled estimates of two rotation columns and the translation. It then finds a scale s that makes those rotation columns approximately unit length. The sign is chosen so the tag is in front of the camera, according to the coordinate convention assumed here.

A rotation matrix has three mutually perpendicular columns. Since the homography only provides the first two, the code computes the third with their cross product. Because noisy measurements can make the result slightly non-orthogonal, the SVD-based polar decomposition adjusts it to a proper rotation matrix.

Finally, it returns a homogeneous 4×4 transform:

$$
\begin{bmatrix}
R_{00}&R_{01}&R_{02}&T_X\\
R_{10}&R_{11}&R_{12}&T_Y\\
R_{20}&R_{21}&R_{22}&T_Z\\
0&0&0&1
\end{bmatrix}
$$

The pose caller then scales the translation by half the tag size, since this homography’s plane coordinates use a unit-sized tag convention.

Note that the coordinate convention used here assumes the tag is in front the camera and the camera looks in the negative z-axis direction, which is opposite to the OpenCV convention.

### Theory
Calculating the camera pose (rotation matrix $R$ and translation vector $T$) from a homography matrix $H$ is a fundamental problem in computer vision, commonly used in planar marker tracking, augmented reality (AR), and camera calibration (such as Zhang's method).

Given a homography $H$ estimated from point correspondences between a planar object and its image, $H$ is defined up to a scale factor $\lambda$ as:$$H = K \begin{bmatrix} r_1 & r_2 & t \end{bmatrix}$$where $K$ is the known camera intrinsic matrix, $r_1, r_2$ are the first two columns of the rotation matrix $R$, and $t$ is the translation vector.
Step-by-Step Decomposition Process
* 1. **Normalize the Homography:** Divide the homography matrix $H$ by the norm of its first column (or second column) to **eliminate the arbitrary scale factor $\lambda$**, ensuring the columns of the rotation part behave properly.$$H' = \frac{1}{\Vert{}K^{-1} h_1\Vert{}} H$$
* 2. **Compute the Rotation Columns:** Multiply the normalized homography by the inverse of the camera intrinsic matrix $K^{-1}$ to recover the scaled metric rotation vectors:$$h_a = K^{-1} h_1, \quad h_b = K^{-1} h_2$$Since the true rotation columns $r_1$ and $r_2$ must be orthogonal and have unit length, $h_a$ and $h_b$ are often slightly distorted by noise. Gram-Schmidt orthonormalization or Singular Value Decomposition (SVD) is applied to project them onto the closest valid rotation matrix columns.
* 3. **Determine the Third Rotation Column:** The third column of the rotation matrix $r_3$ is calculated as the cross product of the first two orthonormal columns to ensure a right-handed coordinate system:$$r_3 = r_1 \times r_2$$This constructs the complete rotation matrix $R = \begin{bmatrix} r_1 & r_2 & r_3 \end{bmatrix}$.
* 4. **Calculate the Translation Vector:** Recover the translation vector $T$ by scaling the third column of the decomposition product:$$T = \frac{1}{\Vert{}K^{-1} h_1\}} K^{-1} h_3$$
* 5. **Resolve Ambiguities (Sign Selection):** Because homography decomposition typically yields up to 4 mathematical solutions for $(R, T)$ (due to sign flips), you must apply the positive depth constraint: the points observed must lie in front of the camera ($z > 0$). Test the candidate poses on at least one known point to select the physically valid solution.