% Clearing previous outputs and variables
clc; clear;

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

% Defining LQR parameters
Q = diag([100 100 100 100 100 100]);
R = 0.001;

% Defining observable output matrices
C_matrices = {[1, 0, 0, 0, 0, 0];                         % Observing x component
              [0, 0, 1, 0, 0, 0; 1, 0, 0, 0, 0, 0];       % Observing theta1 and theta2
              [1, 0, 0, 0, 0, 0; 1, 0, 1, 0, 0, 0; 0, 0, 0, 0, 1, 0]};
D = 0;

% Setting initial conditions for the Luenberger Observer
initial_conditions = [4; 0; 30; 0; 60; 0; 0; 0; 0; 0; 0; 0];

% Designing LQR and Kalman Filter
[K, S, eigen_values] = lqr(A, B, Q, R);
v_d = 0.3 * eye(6);
v_n = 1;

% Choosing colors for each plot
colors = {'b', 'r', 'g', 'c', 'm', 'y', 'k'};

% Analyzing systems for each observable configuration
num_of_systems = length(C_matrices);

% Creating a single figure window
figure;

% Looping through each configuration
for i = 1:num_of_systems
    C = C_matrices{i};
    K_pop = lqr(A', C', v_d, v_n)';

    % Constructing state-space model
    sys = ss([(A-B*K) B*K; zeros(size(A)) (A-K_pop*C)], [B; zeros(size(B))], [C zeros(size(C))], D);

    % Plotting initial response
    subplot(num_of_systems, 2, 2*i-1);
    initial(sys, initial_conditions, colors{i});
    title(sprintf('Initial Response for System %d', i));

    % Plotting step response
    subplot(num_of_systems, 2, 2*i);
    step(sys, colors{i});
    title(sprintf('Step Response for System %d', i));
end

grid on;

% Adjusting the figure size for better visibility
set(gcf, 'Position', [100, 100, 1200, 800]);