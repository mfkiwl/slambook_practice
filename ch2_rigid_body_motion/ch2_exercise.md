1. Verify that the rotation matrix is an orthogonal matrix.  
   a orthohonal matrix has a property of $\mathbf{M}^{-1}=\mathbf{M}^T, \quad \mathbf{M}\mathbf{M}^T = \mathbf{I}$  
  ```math
  \mathbf{R}_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos \theta & -\sin \theta \\ 0 & \sin \theta & \cos \theta \end{bmatrix}
  ```
  ```math
  \mathbf{R}^T_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos \theta & \sin \theta \\ 0 & -\sin \theta & \cos \theta \end{bmatrix}
  ```
```math
  \mathbf{R}_x(\theta)\mathbf{R}^T_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos \theta & -\sin \theta \\ 0 & \sin \theta & \cos \theta \end{bmatrix}\begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos \theta & \sin \theta \\ 0 & -\sin \theta & \cos \theta \end{bmatrix}
  = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos^2 \theta + \sin^2 \theta & \cos \theta \sin \theta - \sin\theta \cos \theta \\ 0 & \sin\theta\cos\theta - \cos\theta\sin\theta & \sin^2\theta\cos^2\theta  \end{bmatrix}
  = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} = \mathbf{I}
  ```
6. When does a general linear equation $\mathbf{Ax = b}$ has a unique solution of $\mathbf{x}$? How to solve it numerically? Can you implement it in Eigen?  
   The matrix $\mathbf{A}$ must be square and invertible which means the determinant of $\mathbf{A}$ must be non zero value. we find the inverse of $\mathbf{A}$ and use $\mathbf{x = A^{-1}b}$ to solve $\mathbf{x}$.
   if matrix $\mathbf{A}$ is not square , we can use SVD to decomposee $\mathbf{A}$ that find a $\mathbf{x}$ that minimizes the least squares error $\| \mathbf{Ax - b} \|$. 
