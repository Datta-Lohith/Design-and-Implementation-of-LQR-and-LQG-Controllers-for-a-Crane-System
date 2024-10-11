% Clearing the workspace and the command window
clc; clear;

% Defining system parameters
M = 1000; 
mass_1 = 100; 
mass_2 = 100; 
length_1 = 20;  
length_2 = 10;  
g = 9.81; 

% Defining state space matrices A and B with the assigned values
A = [0, 1, 0, 0, 0, 0;
     0, 0, -(mass_1*g)/M, 0, -(mass_2*g)/M, 0;
     0, 0, 0, 1, 0, 0;
     0, 0, -((M+mass_1)*g)/(M*length_1), 0, -(mass_2*g)/(M*length_1), 0;
     0, 0, 0, 0, 0, 1;
     0, 0, -(mass_1*g)/(M*length_2), 0, -(g*(M+mass_2))/(M*length_2), 0];
B = [0; 1/M; 0; 1/(M*length_1); 0; 1/(M*length_2)];

% Defining different C matrices for various outputs
C_matrices = {[1, 0, 0, 0, 0, 0];                         % Defining for observing x component
              [0, 0, 1, 0, 0, 0; 1, 0, 0, 0, 0, 0];       % Defining for observing theta1 and theta2
              [1, 0, 0, 0, 0, 0; 0, 0, 0, 0, 1, 0];       % Defining for observing x and theta2
              [1, 0, 0, 0, 0, 0; 1, 0, 1, 0, 0, 0; 0, 0, 0, 0, 1, 0]}; % Defining for observing x, theta1, theta2

% Checking the observability for each C matrix
output_descriptions = {'x(t)', 'theta_1(t) and theta_2(t)', 'x(t) and theta_2(t)', 'x(t), theta_1(t) and theta_2(t)'};
for i = 1:length(C_matrices)
    Observability_matrix = obsv(double(A), double(C_matrices{i})); % Calculating the observability matrix
    if rank(Observability_matrix) == 6
        fprintf('System is observable with outputs: %s\n', output_descriptions{i});
    else
        fprintf('System is not observable with outputs: %s\n', output_descriptions{i});
    end
end