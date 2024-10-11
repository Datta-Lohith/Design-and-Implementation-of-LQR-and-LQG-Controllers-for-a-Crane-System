% Linear System
clc; clear;

% Defining the system parameters
M=1000;  % Defining the mass of the crane in kg
mass_1=100;  % Defining the mass of the first load in kg
mass_2=100;  % Defining the mass of the second load in kg
length_1=20;   % Defining the length of the first cable in meters
length_2=10;   % Defining the length of the second cable in meters
g=9.81;  % Defining the acceleration due to gravity in m/s^2

% Substituting the physical parameters into the state matrices A and B
A=[0 1 0 0 0 0;
     0 0 -(mass_1*g)/M 0 -(mass_2*g)/M 0;
     0 0 0 1 0 0;
     0 0 -((M+mass_1)*g)/(M*length_1) 0 -(mass_2*g)/(M*length_1) 0;
     0 0 0 0 0 1;
     0 0 -(mass_1*g)/(M*length_2) 0 -(g*(M+mass_2))/(M*length_2) 0];
B=[0; 1/M; 0; 1/(M*length_1); 0; 1/(M*length_2)];

% Checking the controllability of the system
controllability_matrix=ctrb(A, B);
if rank(controllability_matrix) == 6
    disp('The system is controllable.');
else
    disp('The system is uncontrollable.');
end

% Setting initial conditions for the system
initial_state=[0;0;10;0;30;0];

% Setting LQR weighting matrices
Q=diag([10 100 1000 1000 100 100]); % Emphasizing the importance of state regulation
R=0.001;            % Representing the cost of control input

% Designing LQR controller
[K,S,eigen_values]=lqr(A, B, Q, R);

% Displaying the LQR results
disp('LQR Gain Matrix K:');
disp(K);
disp('Solution to Riccati Equation S:');
disp(S);
disp('Closed-loop eigen_values:');
disp(eigen_values);

% Defining the new A matrix for the closed-loop system
A_q=A-B*K;

% Creating the state-space models
system_state_space=ss(A,B,eye(6),zeros(6, 1));
controlled_system_state_space=ss(A_q,B,eye(6),zeros(6,1));

% Setting time span for the simulation
t=0:0.01:100;

% Simulating the uncontrolled system response
[Y_uncontrolled,T_uncontrolled,X_uncontrolled]=lsim(system_state_space,zeros(length(t),1),t,initial_state);
% Simulating the controlled system response
[Y_controlled,T_controlled,X_controlled]=lsim(controlled_system_state_space,zeros(length(t),1),t,initial_state);

% Preparing graphs for each state variable in a single window
figure;
colors=['b','g', 'r','c','m','y']; % Assigning different colors for each plot

for i=1:6
    % Plotting uncontrolled system response
    subplot(6,2,2*i-1);
    plot(T_uncontrolled,X_uncontrolled(:,i),colors(i));
    title(sprintf('Uncontrolled: State %d',i));
    xlabel('Time(s)');
    ylabel(sprintf('State %d',i));
    grid on;

    % Plotting controlled system response
    subplot(6,2,2*i);
    plot(T_controlled,X_controlled(:,i),colors(i));
    title(sprintf('Controlled: State %d',i));
    xlabel('Time(s)');
    ylabel(sprintf('State %d',i));
    grid on;
end

% Adjusting the figure size for better visibility
set(gcf,'Position',[100,100,1200,800]);