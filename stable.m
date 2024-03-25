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
t_final = 100;  % Final time for simulation
dt = 0.01;     % Time step

% Initialize variables to store stability results
is_stable_lqr = false;
is_stable_lyap = false;

% Initialize arrays to store data for plotting
time_steps = 0:dt:t_final;
lyapunov_values = zeros(size(time_steps));
lqr_eigenvalues = zeros(length(time_steps), size(A,1));
lqr_control_input = zeros(length(time_steps), 1);

% Main loop
for idx = 1:length(time_steps)
    % Simulate LQR control for speed regulation
    x = zeros(2, 1);   % Initial state
    u = -K_lqr * x;     % Compute control input (voltage)
    lqr_control_input(idx) = u; % Store control input for plotting

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

% Plot LQR control input
figure;
plot(time_steps, lqr_control_input, 'LineWidth', 1.5);
xlabel('Time');
ylabel('LQR Control Input (Voltage)');
title('LQR Control Input over Time');
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


% Initialize arrays to store stability results
lqr_stability_times = [];
lyapunov_stability_times = [];

% Main loop for LQR stability analysis
for idx = 1:length(time_steps)
    % Check stability condition for LQR
    if all(real(lqr_eigenvalues(idx,:)) < 0)
        lqr_stability_times = time_steps(idx);
        break;
    end
end

% Main loop for Lyapunov stability analysis
for idx = 1:length(time_steps)
    % Check stability condition for Lyapunov
    if all(lyapunov_values(idx) <= lyapunov_values(1))
        lyapunov_stability_times = time_steps(idx);
        break;
    end
end

% Compare stability times
fprintf('Time taken to achieve stability:\n');
fprintf('LQR Control: %.2f seconds\n', lqr_stability_times);
fprintf('Lyapunov Stability Analysis: %.2f seconds\n', lyapunov_stability_times);

% Compare stability margins
lqr_margin = min(real(lqr_eigenvalues(end,:)));
lyapunov_margin = min(lyapunov_values);
fprintf('\nStability Margins:\n');
fprintf('LQR Control: %.6f\n', lqr_margin);
fprintf('Lyapunov Stability Analysis: %.6f\n', lyapunov_margin);
