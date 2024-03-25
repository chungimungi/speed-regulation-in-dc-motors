% System parameters for DC motor speed regulation
R = 1;     % Resistance
L = 0.5;   % Inductance
K = 0.01;  % Motor constant
J = 0.01;  % Inertia
b = 0.2;   % Damping coefficient

% State-space representation
A = [-b/J K/J; -K/L -R/L];
B = [0; 1/L];
C = [1 0];
D = 0;

% LQR weighting matrices for speed regulation
Q_lqr = diag([1, 1]);
R_lqr = 0.1;

% Solving Riccati equation to get the LQR gain
[K_lqr, ~, ~] = lqr(A, B, Q_lqr, R_lqr);

% Lyapunov function weighting matrix
Q_lyap = eye(2); % Default unity weighting matrix

% Initialize Lyapunov matrix using the solution to the Riccati equation
P_lyap = care(A', C', Q_lyap);

% Simulation parameters
t_final = 10;  % Final time for simulation
dt = 0.01;     % Time step

% Initialize variables to store stability results
is_stable_lqr = false;
is_stable_lyap = false;

% Initialize arrays to store data for plotting
time_steps = 0:dt:t_final;
lyapunov_values = zeros(size(time_steps));
lqr_eigenvalues = zeros(length(time_steps), size(A,1));

% Main loop
for idx = 1:length(time_steps)
    % Simulate LQR control for speed regulation
    x = zeros(2, 1);   % Initial state
    u = -K_lqr * x;     % Compute control input (voltage)

    % Update state using system dynamics
    x_dot = A * x + B * u;
    x = x + x_dot * dt;

    % Compute Lyapunov function value
    lyapunov_values(idx) = x' * P_lyap * x;

    % Check stability condition for LQR
    lqr_eigenvalues(idx,:) = eig(A - B * K_lqr);

    % Break out of loop if both LQR and Lyapunov stability conditions are met
    if all(lyapunov_values(idx) <= lyapunov_values(1)) && all(real(lqr_eigenvalues(idx,:)) < 0)
        disp('Both LQR and Lyapunov stability conditions are met for speed regulation');
        is_stable_lqr = true;
        is_stable_lyap = true;
        break;
    end
end

% Plot Lyapunov function evolution
figure;
plot(time_steps, lyapunov_values);
xlabel('Time');
ylabel('Lyapunov Function Value');
title('Lyapunov Function Evolution');
grid on;

% Plot eigenvalues of A - BK over time
figure;
plot(time_steps, real(lqr_eigenvalues), 'LineWidth', 1.5);
hold on;
plot(time_steps, imag(lqr_eigenvalues), 'LineWidth', 1.5);
xlabel('Time');
ylabel('Eigenvalues');
title('Eigenvalues of A - BK over Time');
legend('Real Part', 'Imaginary Part');
grid on;

% Display stability results
if is_stable_lqr
    disp('LQR Control: System is stable ');
else
    disp('LQR Control: System is unstable');
end

if is_stable_lyap
    disp('Lyapunov Stability Analysis: System is stable ');
else
    disp('Lyapunov Stability Analysis: System is unstable');
end
