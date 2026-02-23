%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();

disp('iTj_0')
disp(iTj_0);
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
q = [pi/2, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';

%% Define the tool frame rigidly attached to the end-effector
% Tool frame definition
eRt = YPRToRot(pi/10, 0, pi/6);
e_r_te = [0.3; 0.1; 0];
eTt = [eRt, e_r_te; 0, 0, 0, 1];
%% Initialize Geometric Model (GM) and Kinematic Model (KM)

% Initialize geometric model with q0
gm = geometricModel(iTj_0,jointType,eTt);

% Update direct geoemtry given q0
gm.updateDirectGeometry(zeros(gm.jointNumber));

% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm);

bTt0 = gm.getToolTransformWrtBase();

disp("eTt");
disp(eTt);
disp('bTt q = 0');
disp(bTt0);

gm.updateDirectGeometry(q);
km = kinematicModel(gm);
bTt = gm.getToolTransformWrtBase();

disp('bTt');
disp(bTt);


%% Define the goal frame and initialize cartesian control
% Goal definition 
bOg = [0.2; -0.8; 0.3];
bRg = YPRToRot(0, 1.57, 0);
bTg = [bRg bOg;0 0 0 1]; 
disp('bTg')
disp(bTg)

% control proportional gain 
k_a = 0.8;
k_l = 0.8;

% Cartesian control initialization
cc = cartesianControl(gm, k_a, k_l);

% 2.1
e = cc.cartesianError(bTg);
disp("Cartesian error:");
disp(e);

% 2.2
x_dot = cc.getCartesianReference(bTg);
disp("Desired angular and linear reference velocities for the tool:");
disp(x_dot);

% 2.3 
J_t = km.getJacobianOfToolWrtBase;
q_dot = pinv(J_t) * x_dot;
disp("Desired joint velocities:");
disp(q_dot);


%% Initialize control loop



% Simulation variables
samples = 100;
t_start = 0.0;
t_end = 5.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

% preallocation variables
bTi = zeros(4, 4, gm.jointNumber);
bri = zeros(3, gm.jointNumber+1);

% joints upper and lower bounds 
qmin = -3.14 * ones(7,1);
qmin(6) = 0;
qmax = +3.14 * ones(7,1);
qmax(6) = 1;

show_simulation = true;
pm = plotManipulators(show_simulation);
pm.initMotionPlot(t, bTg(1:3,4));

% Initialize tool and end-effector velocity arrays
toolVelocity = zeros(6, length(t));
endEffectorVelocity = zeros(6, length(t));
j = 1;
%%%%%%% Kinematic Simulation %%%%%%%
for i = t
    % Updating transformation matrices for the new configuration 
    gm.updateDirectGeometry(q);
    % Get the cartesian error given an input goal frame
    x_dot = cc.getCartesianReference(bTg);
    % Update the jacobian matrix of the given model
    J_t = km.getJacobianOfToolWrtBase;
    km.updateJacobian();
    %% INVERSE KINEMATICS
    % Compute desired joint velocities 
    q_dot = pinv(J_t) * x_dot;

    % simulating the robot
    q = KinematicSimulation(q, q_dot, dt, qmin, qmax);
    
    pm.plotIter(gm, km, i, q_dot);

    % Tool velocity
    toolVelocity(:, j) = J_t * q_dot;

    % EE velocity
    endEffectorVelocity(:, j) = km.getJacobianOfLinkWrtBase(gm.jointNumber)* q_dot;

    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        break
    end
    j = j + 1;

end

pm.plotFinalConfig(gm);

%% Q2.5
figure()
grid on; hold on;
plot(t, toolVelocity', 'LineWidth', 1.5);
xlabel('Time [s]');
title('Tool velocity');
legend({'\omega_x', '\omega_y', '\omega_z', 'x\_dot', 'y\_dot', 'z\_dot'});

figure()
grid on; hold on;
plot(t, endEffectorVelocity', 'LineWidth', 1.5);
xlabel('Time [s]');
title('EE velocity');
legend({'\omega_x', '\omega_y', '\omega_z', 'x\_dot', 'y\_dot', 'z\_dot'});