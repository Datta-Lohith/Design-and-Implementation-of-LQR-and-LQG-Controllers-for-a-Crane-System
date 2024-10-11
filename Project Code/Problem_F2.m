% Non-Linear
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

% Defining output matrices for observable configurations
C_matrices = {[1, 0, 0, 0, 0, 0];                         % Observing x component
              [0, 0, 1, 0, 0, 0; 1, 0, 0, 0, 0, 0];       % Observing theta1 and theta2
              [1, 0, 0, 0, 0, 0; 1, 0, 1, 0, 0, 0; 0, 0, 0, 0, 1, 0]};   % Observing x and theta2
D = 0; % Defining D matrix as zero

% Setting LQR parameters
Q = diag([1000 100 1000 1000 1000 1000]);
R = 0.01;
[K, S, eigen_values] = lqr(A, B, Q, R); % Computing LQR gain matrix

% Setting initial conditions for the observer
initial_conditions = [0, 0, 30, 0, 60, 0, 0, 0, 0, 0, 0, 0];

% Defining desired poles for the observer
poles = [-1; -2; -3; -4; -5; -6];

% Choosing colors for each system
colors = ['b','g','r','c','m','y']; % Adjusting as needed for more systems

% Constructing and analyzing systems for each observer configuration
num_of_systems = length(C_matrices);
figure; % Creating a single figure window for all plots

for i = 1:num_of_systems
    C = C_matrices{i};
    L = place(A', C', poles)'; % Computing observer gain matrix

    % Formulating Luenberger state-space matrices
    A_q = [(A-B*K) B*K; zeros(size(A)) (A-L*C)];
    B_q = [B; zeros(size(B))];
    C_q = [C zeros(size(C))];

    % Creating state-space model
    sys = ss(A_q, B_q, C_q, D);

    % Plotting initial response (Left column)
    subplot(num_of_systems, 2, 2*i-1);
    initial(sys, initial_conditions, colors(i));
    title(sprintf('Initial Response for System %d', i));
    grid on;
    
    % Plotting step response (Right column)
    subplot(num_of_systems, 2, 2*i);
    step(sys, colors(i));
    title(sprintf('Step Response for System %d', i));
    grid on;
end

% Adjusting the figure size for better visibility
set(gcf, 'Position', [100, 100, 1200, num_of_systems*200]);