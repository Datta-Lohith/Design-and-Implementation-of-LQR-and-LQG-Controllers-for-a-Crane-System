% Linear
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

D = 0;

% Setting LQR parameters
Q = diag([1000 100 1000 1000 100 100]);
R = 0.01;
[K, S, eigen_values] = lqr(A, B, Q, R);

% Defining observer output matrices and corresponding L matrices
C_matrices = {[1, 0, 0, 0, 0, 0];                         % Observing x component
              [0, 0, 1, 0, 0, 0; 1, 0, 0, 0, 0, 0];       % Observing theta1 and theta2
              [1, 0, 0, 0, 0, 0; 1, 0, 1, 0, 0, 0; 0, 0, 0, 0, 1, 0]}; % Observing x and theta
poles = -1:-1:-6;
L_matrices = arrayfun(@(i) place(A', C_matrices{i}', poles)', 1:length(C_matrices), 'UniformOutput', false);

% Setting initial conditions for the observer
initial_conditions = [10, 40, 10, 0, 60, 0, 0, 0, 0, 0, 0, 0];

% Determining the number of systems to analyze
number_of_systems = length(C_matrices);

% Choosing colors for each system
colors = ['b','g','r','c','m','y']; % Adjust as needed for more systems

% Constructing and analyzing each system
figure; % Creating a single figure window

for i = 1:number_of_systems
    A_q = [(A-B*K) B*K; zeros(size(A)) (A-L_matrices{i}*C_matrices{i})];
    B_q = [B; zeros(size(B))];
    C_q = [C_matrices{i} zeros(size(C_matrices{i}))];
    sys = ss(A_q, B_q, C_q, D);
    
    % Plotting initial response of the system
    subplot(number_of_systems, 2, 2*i-1);
    initial(sys, initial_conditions, colors(i));
    title(sprintf('Initial Response for System %d', i));
    grid on;

    % Plotting step response of the system
    subplot(number_of_systems, 2, 2*i);
    step(sys, colors(i));
    title(sprintf('Step Response for System %d', i));
    grid on;
end

% Adjusting the figure size for better visibility
set(gcf, 'Position', [100, 100, 1200, 800]);