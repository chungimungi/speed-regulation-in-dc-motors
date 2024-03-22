% Define system parameters
R = 1; % Resistance
L = 0.5; % Inductance
K = 0.01; % Motor constant
J = 0.01; % Inertia
b = 0.1; % Damping coefficient

% Define system matrices for the DC motor
A = [0 1 0; -K/J -b/J 0; 0 -K/L -R/L];
Q = eye(3);

% Solve Lyapunov equation
P = lyap(A, Q);

% Check stability
if isequal(P, P') && all(eig(P) > 0)
    disp('System is stable (Positive definite)');
else
    disp('System is not stable (Not positive definite)');
end


