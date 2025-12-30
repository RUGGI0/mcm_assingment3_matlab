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

eTt = zeros(4);
eTt(1:3, 1:3) = eRt;
eTt(1:3, 4) = e_r_te;
eTt(4, 4) = 1;

%% Initialize Geometric Model (GM) and Kinematic Model (KM)

% Initialize geometric model with q0
gm = geometricModel(iTj_0,jointType,eTt);

% Update direct geoemtry given q0
gm.updateDirectGeometry(q);

% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm);

bTt = gm.getToolTransformWrtBase();

disp("eTt");
disp(eTt);
disp('bTt q = 0');
disp(bTt);

%% Define the goal frame and initialize cartesian control
% Goal definition 
bOg = [0.2;-0.7;0.3];
bRg = YPRToRot(0,1.57,0);
bTg = [bRg bOg;0 0 0 1]; 
disp('bTg')
disp(bTg)

% control proportional gain 
k_a = 0.8;
k_l = 0.8;

% Cartesian control initialization
cc = cartesianControl(gm, k_a, k_l);

%% Initialize control loop 

% Simulation variables
samples = 100;
t_start = 0.0;
t_end = 10.0;
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

%%%%%%% Kinematic Simulation %%%%%%%
for i = t
    % Updating transformation matrices for the new configuration 

    % Get the cartesian error given an input goal frame

    % Update the jacobian matrix of the given model

    %% INVERSE KINEMATICS

    km = kinematicModel(gm);
    km.updateJacobian();

    x_dot = cc.getCartesianReference(bTg);

    %bJe = km.getJacobianOfJointWrtBase(7);
    bJe = km.J;

    % ---------------------------------
    bTe = gm.getTransformWrtBase(gm.jointNumber);
    bRe = bTe(1:3, 1:3);
    e_r_tb = bRe' * e_r_te;

    skew_e_r_te = [   0        -e_r_tb(3)  e_r_tb(2);
                   e_r_tb(3)     0        -e_r_tb(1);
                  -e_r_tb(2)  e_r_tb(1)     0       ];
    temp = [eye(3) zeros(3);
           skew_e_r_te eye(3)];
    bJt = temp * bJe;
    % ---------------------------------

    % Compute desired joint velocities 
    q_dot = pinv(bJt) * x_dot;

    % simulating the robot
    q = KinematicSimulation(q,q_dot,dt,qmin,qmax);

    gm.updateDirectGeometry(q);
    
    pm.plotIter(gm, km, i, q_dot);

    % ---------------------------------
    x_dot_T   = bJt * q_dot;   % tool actual twist
    x_dot_E   = bJe * q_dot;   % end-effector actual twist
    % ---------------------------------

    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        break
    end

end

pm.plotFinalConfig(gm);
