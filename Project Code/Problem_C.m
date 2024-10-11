clc; clear;

% Defining the symbolic variables for the system parameters
syms M mass_1 mass_2 length_1 length_2 g;

% Constructing the 'A' matrix of the state-space representation for the linearized system
A = [0, 1, 0, 0, 0, 0;
     0, 0, -(mass_1*g)/M, 0, -(mass_2*g)/M, 0;
     0, 0, 0, 1, 0, 0;
     0, 0, -((M+mass_1)*g)/(M*length_1), 0, -(mass_2*g)/(M*length_1), 0;
     0, 0, 0, 0, 0, 1;
     0, 0, -(mass_1*g)/(M*length_2), 0, -(g*(M+mass_2))/(M*length_2), 0];
disp('Matrix A:');
disp(A);

% Constructing the 'B' matrix of the state-space representation
B = [0; 1/M; 0; 1/(M*length_1); 0; 1/(M*length_2)];
disp('Matrix B:');
disp(B);

% Calculating the controllability matrix of the system
controllability_matrix = [B, A*B, A^2*B, A^3*B, A^4*B, A^5*B];
disp('Controllability Matrix:');
disp(controllability_matrix);

% Calculating the determinant of the controllability matrix
det_controllability_matrix = simplify(det(controllability_matrix));
disp('Determinant of the Controllability Matrix:');
disp(det_controllability_matrix);

% Checking the rank of the controllability matrix for system controllability
rank_controllability_matrix = rank(controllability_matrix);
disp('Rank of the Controllability Matrix:');
disp(rank_controllability_matrix);

% Displaying whether the system is controllable or not based on the rank
if rank_controllability_matrix < 6
    disp('The system is not controllable.');
else
    disp('The system is controllable.');
end