% Clear all variables and command window
clear all; clc;

% Define symbolic variables for joint positions, velocities, and accelerations
syms q_1 q_2 q_3 q_4 real;
syms qd_1 qd_2 qd_3 qd_4 real;
syms qdd_1 qdd_2 qdd_3 qdd_4 real;

q_sym = [q_1 q_2 q_3 q_4]';
qd_sym = [qd_1 qd_2 qd_3 qd_4]';
qdd_sym = [qdd_1 qdd_2 qdd_3 qdd_4]';

% Define symbolic Jacobians for the links and motors
Jpl = sym('j_%d_', [3, 4, 4], 'real'); 
Jol = sym('j_%d_', [3, 4, 4], 'real');
Jpm = sym('j_%d_', [3, 4, 4], 'real');
Jom = sym('j_%d_', [3, 4, 4], 'real');

% Define symbolic matrices for inertia, Coriolis, friction, and motor masses
syms B C Fv mm real;

B = zeros(4, 4, 'sym');
C = zeros(4, 4, 'sym');
Fv = zeros(1, 4, 'sym');
mm = zeros(1, 4, 'sym');

% Define symbolic parameters for masses and inertias of the links and motors
syms ml [1 4] real;
syms Il [1 4] real;
syms Im [1 4] real;
syms Fm [1 4] real;

% Define numerical values for the robot parameters
ml = [20, 20, 10, 0]; % Link masses (last link mass is zero for SCARA)
Il = [4, 4, 1, 0]; % Link inertias
Im = [0.01, 0.01, 0.005, 0.001]; % Motor inertias
Fm = [0.00005, 0.00005, 0.01, 0.005]; % Friction coefficients
kr = [1, 1, 50, 20]; % Gear ratios

% Lengths
l1 = 0.25; % m
l2 = 0.25; % m
a1 = 0.5; % m
a2 = 0.5; % m

% Define symbolic Jacobians for the SCARA robot
Jpl1 = [-l1*sin(q_1) 0 0 0;
         l1*cos(q_1) 0 0 0;
              0      0 0 0];
           
Jpl2 = [-a1*sin(q_1)-l2*sin(q_1+q_2)  -l2*sin(q_1+q_2)  0  0;
         a1*cos(q_1)+l2*cos(q_1+q_2)   l2*cos(q_1+q_2)  0  0;
                   0                     0              0  0];

Jpl3 = zeros(3,4, 'sym');

Jpl4 = [-a1*sin(q_1)-a2*sin(q_1+q_2)  -a2*sin(q_1+q_2)  0  0;
         a1*cos(q_1)+a2*cos(q_1+q_2)   a2*cos(q_1+q_2)  0  0;
               0                         0              1  0];
       
Jpl(:,:,1) = Jpl1;
Jpl(:,:,2) = Jpl2;
Jpl(:,:,3) = Jpl3;
Jpl(:,:,4) = Jpl4;

Jol1 = zeros(3, 4, 'sym'); Jol1(end,1)=1;
Jol2 = Jol1; Jol2(end,2)=1;
Jol3 = zeros(3, 4, 'sym');
Jol4 = Jol2; Jol4(end,4)=1;

Jol(:,:,1) = Jol1;
Jol(:,:,2) = Jol2;
Jol(:,:,3) = Jol3;
Jol(:,:,4) = Jol4;

Jpm1 = zeros(3, 4, 'sym');

Jpm2 = [ -a1*sin(q_1)  0  0  0;
          a1*cos(q_1)  0  0  0;
               0       0  0  0];
            
Jpm3 = [ -a1*sin(q_1) - a2*sin(q_1+q_2)  0  0  0;
          a1*cos(q_1) + a2*cos(q_1+q_2)  0  0  0;
                        0                0  0  0];
                   
Jpm4 = [ -a1*sin(q_1) - a2*sin(q_1+q_2)  0  0  0;
          a1*cos(q_1) + a2*cos(q_1+q_2)  0  0  0;
                        0                0  1  0];

Jpm(:,:,1) = Jpm1;
Jpm(:,:,2) = Jpm2;
Jpm(:,:,3) = Jpm3;
Jpm(:,:,4) = Jpm4;

Jom1 = zeros(3, 4, 'sym'); Jom1(end,:) = [kr(1) 0 0 0];
Jom2 = zeros(3, 4, 'sym'); Jom2(end,:) = [1 kr(2) 0 0];
Jom3 = zeros(3, 4, 'sym'); Jom3(end,:) = [1 1 kr(3) 0];
Jom4 = zeros(3, 4, 'sym'); Jom4(end,:) = [1 1 0 kr(4)];

Jom(:,:,1) = Jom1;
Jom(:,:,2) = Jom2;
Jom(:,:,3) = Jom3;
Jom(:,:,4) = Jom4;

% Calculate inertia matrix B
for i = 1:4
    B = B + (ml(i) * Jpl(:,:,i)' * Jpl(:,:,i)) ...
          + (Jol(:,:,i)' * Il(i) * Jol(:,:,i)) ...
          + (mm(i) * Jpm(:,:,i)' * Jpm(:,:,i)) ...
          + (Jom(:,:,i)' * Im(i) * Jom(:,:,i));
end
B = simplify(B);

% Calculate Coriolis and centrifugal matrix C
for i= 1:4
    for j= 1:4
        for k= 1:4
            cijk = 0.5*( diff(B(i,j),q_sym(k)) + diff(B(i,k),q_sym(j)) - diff(B(j,k),q_sym(i)));
            C(i,j) = C(i,j) + cijk*qd_sym(k);
        end
    end
end
C = simplify(C);

% Calculate viscous friction matrix Fv
for i= 1:4
    Fv(i) = Fm(i) * (kr(i))^2;
end
Fv = diag(Fv);

% Calculate gravitational vector g
g = [0  0  9.81*(ml(3)+ml(4))  0]';

% Calculate joint torques tau
tau = B*qdd_sym + C*qd_sym + Fv*qd_sym + g;
tau = simplify(tau);

% Calculate dynamic terms without inertia (n)
n = C*qd_sym + Fv*qd_sym + g;
n = simplify(n);

% Display the results
disp('Inertia Matrix (B):');
disp(B);

disp('Coriolis and Centrifugal Matrix (C):');
disp(C);

disp('Viscous Friction Matrix (Fv):');
disp(Fv);

disp('Gravitational Vector (g):');
disp(g);

disp('Joint Torques (tau):');
disp(tau);

disp('Dynamic Terms without Inertia (n):');
disp(n);