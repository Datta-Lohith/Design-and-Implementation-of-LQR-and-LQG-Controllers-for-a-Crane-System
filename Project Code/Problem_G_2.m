% Clearing all previous outputs
clc; clear;

% Setting initial conditions and time span for the simulation
initial_conditions = [0; 0; 30; 0; 60; 0; 0; 0; 0; 0; 0; 0];
time_span = 0:0.1:100;

% Solving the system using the ode45 solver
[t, x] = ode45(@twoload_lqg, time_span, initial_conditions);

% Plotting the results in a single window
figure;
subplot(2, 1, 1); % Upper subplot for the system states
plot(t, x(:, 1:6));
title('System States');
xlabel('Time (s)');
ylabel('States');
legend('x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot');
grid on;

subplot(2, 1, 2); % Lower subplot for the estimated states
plot(t, x(:, 7:12));
title('Estimated States');
xlabel('Time(s)');
ylabel('Estimates');
legend('x_hat', 'x_dot_hat', 'theta1_hat', 'theta1_dot_hat', 'theta2_hat', 'theta2_dot_hat');
grid on;

% Defining the twoload_lqg function for nonlinear LQG control
function dy_dt = twoload_lqg(t, y)
    % Defining system parameters
    M = 1000;
    mass_1 = 100; 
    mass_2 = 100; 
    length_1 = 20; 
    length_2 = 10; 
    g = 9.81;
    
    % Constructing state space matrices
    A = [0 1 0 0 0 0; 
         0 0 -(mass_1*g)/M 0 -(mass_2*g)/M 0;
         0 0 0 1 0 0;
         0 0 -((M+mass_1)*g)/(M*length_1) 0 -(mass_2*g)/(M*length_1) 0;
         0 0 0 0 0 1;
         0 0 -(mass_1*g)/(M*length_2) 0 -(g*(M+mass_2))/(M*length_2) 0];
    B = [0; 1/M; 0; 1/(M*length_1); 0; 1/(M*length_2)];
    C_matrix = [1 0 0 0 0 0]; % Defining output measurement for x component
    
    % Designing LQR parameters
    Q = diag([1000 1000 100 10 1000 100]);
    R = 0.01;
    [K, S, eigen_values] = lqr(A, B, Q, R);
    controlled_force = -K * y(1:6); % Calculating control input
    
    % Designing Kalman filter
    v_d = 0.3 * eye(6);
    v_n = 1;
    K_pop = lqr(A', C_matrix', v_d, v_n)'; % Calculating observer gain
    
    % Formulating estimator dynamics
    dx_hat = A * y(7:12) + B * controlled_force + K_pop * (C_matrix * y(1:6) - C_matrix * y(7:12));
    
    % Formulating system dynamics (controlled system)
    dx = A * y(1:6) + B * controlled_force;
    
    % Concatenating the system dynamics and observer dynamics
    dy_dt = [dx; dx_hat];
end