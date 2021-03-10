## Four-order continuous path through the path points and trajectory optimization
### B-spline curve
The main point is to generate the control points to let the path as close to path points as possible. The method is least-square. However, when the number of control points increase, the parts near the edge will have unexpected shape. It should be noticed that with more control points, our line will be closer to the path points. This is a trade-off. In order to overcome this shortcoming, I introduced constrainted least-square method. I contrained the derivative should be parallel to the line through the first two path points and last two path points. To be easier, I just set the length of the derivative as the length between the two points (or you can adjust). So there are three methods to generate control points:
- Least square 
- Weighted least square with constrained derivative (both direction and norm)
- Weighted least square with parallel derivative (only direction)  

Here are brief mathematical derivations:    
Path point: $\bm{Q}_k, k=1,...,m$  
Control point: $\bm{P}_i, i=1,...,n$  
B-spline: $ \bm{C}(u)=\sum_{i=0}^{n}N_{i,p}(u)\bm{P}_i, u\in[0,1]$  
Derivative of B-spline: $ \bm{C}(u)=\sum_{i=0}^{n}N'_{i,p}(u)\bm{P}_i =\sum_{i=0}^{n}M_{i,p}(u)\bm{P}_i,u\in[0,1]$  

#### Least square
![Image discription](https://github.com/YixiaoWang7/Motion_Planning_for_Drones/blob/master/path1.jpg)   
File: generate_control_point1.m  
Aim: $\bm{C}(\bar{u}_k)=\bm{Q}_k, k=1,...,m$ or $\bm{N}\bm{P} = \bm{Q}$  
Least-square: $\sum_{k=1}^{m}|\bm{Q}_k-\bm{C}(\bar{u}_k)|^2$ or $|\bm{Q}-\bm{N}\bm{P}|^2$  
The result of least-square: $\bm{P} = (\bm{N}^T\bm{N})^{-1}(\bm{N}^T\bm{Q})$  
#### Weighted least square with constrained derivative (both direction and norm)
![Image discription](https://github.com/YixiaoWang7/Motion_Planning_for_Drones/blob/master/path2.jpg)   
File: generate_control_point2.m  
Contraint of derivation (both direction and norm): $\bm{M}\bm{P}=\bm{T}$  
Weight matrix: $\bm{W}$  
Lagrangain multiplier: $\bm{R}$  
Lagrangain function: $(\bm{Q}^T-\bm{P}^T\bm{N}^T)\bm{W}(\bm{Q}-\bm{N}\bm{P})+\bm{R}^T(\bm{M}\bm{P}-\bm{T})$  
#### Weighted least square with constrained derivative (only direction)
![Image discription](https://github.com/YixiaoWang7/Motion_Planning_for_Drones/blob/master/path3.jpg)   
File: generate_control_point3.m  
The point is 3-dimensional. $\bm{P} = \begin{bmatrix} p_1&p_2&p_3\end{bmatrix}$.$\bm{Q} = \begin{bmatrix} q_1&q_2&q_3\end{bmatrix}$.$\bm{T} = \begin{bmatrix} t_1&t_2&t_3\end{bmatrix}$.  
$$
\bm{N}\bm{P}=\bm{Q} \Leftrightarrow \begin{bmatrix} \bm{N}&\bm{0}&\bm{0}\\\bm{0}&\bm{N}&\bm{0}\\\bm{0}&\bm{0}&\bm{N}\end{bmatrix}\begin{bmatrix} p_1\\p_2\\p_3\end{bmatrix}=\begin{bmatrix} q_1\\q_2\\q_3\end{bmatrix}
$$  
The contraints of the derivatives
$$
\begin{bmatrix} diag(t_2)\cdot\bm{M}&-diag(t_1)\cdot\bm{M}&\bm{0}\\diag(t_3)\cdot\bm{M}&\bm{0}&-diag(t_1)\cdot\bm{M}\\\bm{0}&diag(t_3)\cdot\bm{M}&-diag(t_2)\cdot\bm{M}\end{bmatrix}\begin{bmatrix} p_1\\p_2\\p_3\end{bmatrix}=\bm{0}
$$  
The following step is to establish the lagrangain function and solve it. It should be noticed that the there are redundant constraints. Convert it into a ladder matrix and it can be solved.
### Trajectory optimization
See the docs in the personal page. https://github.com/YixiaoWang7/YixiaoWang7.github.io/blob/master/research/Motion_Planning_for_Drones/Report.pdf
![Image discription](https://github.com/YixiaoWang7/Motion_Planning_for_Drones/blob/master/V_TO.jpg)
![Image discription](https://github.com/YixiaoWang7/Motion_Planning_for_Drones/blob/master/A_TO.jpg)
![Image discription](https://github.com/YixiaoWang7/Motion_Planning_for_Drones/blob/master/J_TO.jpg)
### Time interpolation
See the docs in the personal page. https://github.com/YixiaoWang7/YixiaoWang7.github.io/blob/master/research/Motion_Planning_for_Drones/Report.pdf

The problem lies in the start and end of flight. The drone gets stuck at the start and in the end. The reason is that when interpolating at the start and in the end, the velocity is very small, amplifying the numerical errors which leads to more consuming time. I replaned the trajectory at the start and in the end of flight. I set that there are three constant jerk motion procedures of equal time and tried to find the minimum time.
The result is:

![Image discription](https://github.com/YixiaoWang7/Motion_Planning_for_Drones/blob/master/time_interpolation.jpg)
