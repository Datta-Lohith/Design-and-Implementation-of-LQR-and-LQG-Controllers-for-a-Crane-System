% Non - Linear System
clc; clear;

% Initializing state variables for the system
initial_state = [0; 0; 0; 0; 30; 0];  % Defining [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]

% Setting time span for the simulation
simulation_time_span = 0:0.01:10000;

% Solving the system using the ode45 solver
[t, state_evolution] = ode45(@system_dynamics, simulation_time_span, initial_state);

% Plotting each state variable in separate plots
figure;
subplot(3,2,1);
plot(t, state_evolution(:,1), 'b'); % Plotting x in blue
title('x');
xlabel('Time(s)');
ylabel('x');
grid on;

subplot(3,2,2);
plot(t, state_evolution(:,2), 'g'); % Plotting x_dot in green
title('x\_dot');
xlabel('Time (s)');
ylabel('x\_dot');
grid on;

subplot(3,2,3);
plot(t, state_evolution(:,3), 'r'); % Plotting theta1 in red
title('theta1');
xlabel('Time(s)');
ylabel('theta1');
grid on;

subplot(3,2,4);
plot(t, state_evolution(:,4), 'c'); % Plotting theta1_dot in cyan
title('theta1\_dot');
xlabel('Time(s)');
ylabel('theta1\_dot');
grid on;

subplot(3,2,5);
plot(t, state_evolution(:,5), 'm'); % Plotting theta2 in magenta
title('theta2');
xlabel('Time(s)');
ylabel('theta2');
grid on;

subplot(3,2,6);
plot(t, state_evolution(:,6), 'y'); % Plotting theta2_dot in yellow
title('theta2\_dot');
xlabel('Time(s)');
ylabel('theta2\_dot');
grid on;

% Defining the system dynamics as a function
function state_derivatives = system_dynamics(t, state)
    % Defining the constants and system parameters
    M = 1000; % Defining the mass of the crane in kg
    mass_1 = 100; % Defining the mass of Load 1 in kg
    mass_2 = 100; % Defining the mass of Load 2 in kg
    length_1 = 20;  % Defining the cable length of Load 1 in meters
    length_2 = 10;  % Defining the cable length of Load 2 in meters
    g = 9.81; % Defining the acceleration due to gravity in m/s^2

    % Constructing system matrices for state-space representation
    A = [0 1 0 0 0 0;
         0 0 -(mass_1*g)/M 0 -(mass_2*g)/M 0;
         0 0 0 1 0 0;
         0 0 -((M+mass_1)*g)/(M*length_1) 0 -(mass_2*g)/(M*length_1) 0;
         0 0 0 0 0 1;
         0 0 -(mass_1*g)/(M*length_2) 0 -(g*(M+mass_2))/(M*length_2) 0];
    B = [0; 1/M; 0; 1/(M*length_1); 0; 1/(M*length_2)];

    % Designing LQR weighting matrices
    Q = diag([1000 100 1000 1000 100 100]); % Emphasizing the importance of state regulation
    R = 0.01;

    % Computing the LQR gain matrix
    [K, S, eigen_values] = lqr(A, B, Q, R);

    % Calculating the control force based on LQR gain and current state
    controlled_force = -K * state;

    % Formulating differential equations describing system dynamics
    state_derivatives = zeros(6, 1); % Preallocating for speed
    state_derivatives(1) = state(2); % Computing x_dot
    state_derivatives(2) = (controlled_force - (g/2)*(mass_1*sind(2*state(3)) + mass_2*sind(2*state(5))) - (mass_1*length_1*state(4)^2*sind(state(3))) - (mass_2*length_2*state(6)^2*sind(state(5)))) / (M + mass_1*sind(state(3))^2 + mass_2*sind(state(5))^2); % Computing x_double_dot
    state_derivatives(3) = state(4); % Computing theta1_dot
    state_derivatives(4) = (state_derivatives(2)*cosd(state(3)) - g*sind(state(3))) / length_1; % Computing theta1_double_dot
    state_derivatives(5) = state(6); % Computing theta2_dot
    state_derivatives(6) = (state_derivatives(2)*cosd(state(5)) - g*sind(state(5))) / length_2; % Computing theta2_double_dot
end