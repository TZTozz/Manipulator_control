%% Template February Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 
addpath('include/utils'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();

jointType = [0, 0, 0, 0, 0, 0, 1]; % specify two possible link type: Rotational, Prismatic.

q = [pi/4, pi/3, 0, pi/4, 0, 0, 0.03]';

% control proportional gain 
k_a = 0.6;
k_l = 0.6;

% joints upper and lower bounds 
qmin = -6.28 * ones(7,1);
qmax = 6.28 * ones(7,1);
qmin(7) = -0.8;
qmax(7) = 0.8;
 
%% Define the first and the second tool frame rigidly attached to the end-effector
% First tool
eRt1 = YPRToRot(pi/10, 0, pi/6);
ert1 = [0.1; 0.06; 0];
eTt1 = [eRt1, ert1; 0, 0, 0, 1];

% Second tool 
t1Rt2 = YPRToRot(0, 0, 0);
t1rt2 = [0.05; 0; 0];
t1Tt2 = [t1Rt2, t1rt2; 0, 0, 0, 1];

descriptions = ["first goal", "second goal"]; % DO NOT MODIFY
agents = ["TOOL 1", "TOOL 2"]; % DO NOT MODIFY

%% Simulation variables
t_start = 0.0; % DO NOT MODIFY
t_end = 20.0; % DO NOT MODIFY
samples = 200;
dt = (t_end-t_start)/samples;
time = t_start:dt:t_end;

%% Define the goals
% First goal
bRg1 = YPRToRot(2.4630, -1.57, -0.785);
bOg1 = [0.8; 0.4437; 0.8149];
bTg1 = [bRg1, bOg1; 0, 0, 0, 1];

% Second goal
g1Rg2 = YPRToRot(0, 0, 0);
g1Og2 = [-0.09; 0; 0.1];
g1Tg2 = [g1Rg2, g1Og2; 0, 0, 0, 1];
  
%% Initialize Geometric Model (GM) and Kinematic Model (KM)
gm = geometricModel(iTj_0,jointType,eTt1);
gm.updateDirectGeometry(q);

%% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm);

%% Define the first goal frame and initialize cartesian control
% Cartesian control initialization
cc = cartesianControl(gm, k_a, k_l);

firstGoalReached = 0;
bTg = bTg1;

show_simulation = true; % DO NOT MODIFY
pm = plotManipulators(show_simulation); % DO NOT MODIFY
pm.initMotionPlot(time, bTg(1:3,4), descriptions(1)); % DO NOT MODIFY
    
%%%%%%% Kinematic Simulation %%%%%%%
for t = time
    % Updating geometric model 
    gm.updateDirectGeometry(q);

    % Get the cartesian error given an input goal frame
    x_dot = cc.getCartesianReference(bTg);

    % Update the jacobian matrix of the given model
    km.updateJacobian();

    % Inverse Kinematics: compute desired joint velocities 
    q_dot = pinv(km.J) * x_dot;

    % simulating the robot
    q = KinematicSimulation(q, q_dot, dt, qmin, qmax);
    
    % Plot
    pm.plotIter(gm, km, t, q_dot); % DO NOT CHANGE

    % Check if goal is reached
    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        disp(t)
        if(firstGoalReached == 0)
            firstGoalReached = 1;
            bTg = bTg1 * g1Tg2;
            pm.initMotionPlot(time, bTg(1:3,4), descriptions(2));
            eTt = eTt1 * t1Tt2;
            gm = geometricModel(iTj_0,jointType,eTt);
            gm.updateDirectGeometry(q);
            km = kinematicModel(gm);
            km.updateJacobian();
            cc = cartesianControl(gm,k_a,k_l);
        else
            break;
        end
    end
end

%% Final plots
pm.plotFinalConfig(gm, "", "TOOL"); % CHANGE THIS FUNCTION CALL IF IT IS NECESSARY
