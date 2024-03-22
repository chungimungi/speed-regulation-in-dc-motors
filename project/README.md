The main.m file is the main code that is to be used but I can't get the system to be stable under the given LQR control. 

The eigenvalues of the Lyapunov matrix P_new are all negative, which is a good sign indicating convergence towards a positive definite solution. However, the eigenvalues of the derivative of the Lyapunov function V_dot are not all negative, violating the stability condition. 

The control gain K_lqr is not providing sufficient stability for the system. We need to adjust the LQR weighting matrices Q and R_lqr to find a control gain that stabilizes the system. Increasing the weight on the state variables in Q or decreasing the weight on the control input in R_lqr may improve stability.

Currently, I'm using the solution to the Riccati equation as the initial Lyapunov matrix. Maybe we could go about a another approach and get different results. 

I have added comments next to each block of code for better understanding of the working process. Let me know if you have any queries or any changes to be made.