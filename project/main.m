% System parameters
R = 1; % Resistance
L = 0.5; % Inductance
K = 0.01; % Motor constant
J = 0.01; % Inertia
b = 0.1; % Damping coefficient

% State-space representation
A = [0 1 0; -K/J -b/J 0; 0 -K/L -R/L];
B = [0; 0; 1/L];
C = [1 0 0];
D = 0;

% LQR weighting matrices
Q = diag([1,1,1]);
R = 2;

% Solving Riccati equation to get the LQR gain
[K_lqr,~,~] = lqr(A, B, Q, R);

% Lyapunov function weighting matrix
Q_lyap = diag([1, 1, 1]); % Adjust the diagonal values if needed

% Initialize Lyapunov matrix using the solution to the Riccati equation
P = care(A, B, Q_lyap);

% Main loop
for t = 0:1: % Only 10 iterations for debugging purposes

    % Evaluate Lyapunov function derivative
    V_dot = A'*P + A*P - P*B*(R\B')*P + Q_lyap;
    
    % Update Lyapunov matrix using Lyapunov equation
    P_dot = -P*A - A'*P - Q_lyap + P*B*(R\B')*P;
    
    % Check for non-finite values in P_dot
    if any(~isfinite(P_dot))
        disp('Non-finite value encountered in P_dot');
        break;
    end
    
    % Update Lyapunov matrix using Riccati equation
    try
        P_new = lyap(A', P_dot);
    catch
        disp('Error encountered in Lyapunov matrix update');
        break;
    end
    
    if isequal(P, P') && all(eig(P) > 0) && all(eig(P_dot) < 0)
        disp('System is stable');
    else
        disp('System is not stable');
    end

    % Update Lyapunov matrix
    P = P_new;
end