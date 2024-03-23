% Define system matrices A and B
A = [a11, a12; a21, a22]; % State matrix
B = [b1; b2]; % Input matrix

% Define state and control input weighting matrices Q and R
Q = [q11, q12; q21, q22]; % State weighting matrix
R = r; % Control input weighting scalar

% Compute the optimal feedback gain matrix K using lqr function
[K, P, ~] = lqr(A, B, Q, R);

% Compute the time derivative of the Lyapunov function
dotV = x' * (dotP + 2 * P * A) * x;
