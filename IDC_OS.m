%==============================================================================
% Author: Carl Larsson
% Description: Inverse dynamics control in operational space
% Date: 18-03-2024
%==============================================================================
%% Clean up
clear
close all
disp("==============================================================================")
%% Define robot
%------------------------------------------------------------------------------

% Set sample time
sampleTime = 0.001;

% Load 6R robot
n_joints = 6;
robot = loadrobot("universalUR5");
robot.DataFormat = 'column';
robot.Gravity = [0,0,-9.81]';

showdetails(robot)

%------------------------------------------------------------------------------
%% Matrices
%------------------------------------------------------------------------------

% Weights
weights = [0.1, 0.1, 0.1, 1, 1, 1]';
initialguess = robot.homeConfiguration;

% Proportional matrix
K_P = 0.5*eye(n_joints);
K_P(4,4) = 3;
K_P(5,5) = 3;
K_P(6,6) = 3;

% Derivative matrix
K_D = 0.5*eye(n_joints);
K_D(4,4) = 3;
K_D(5,5) = 3;
K_D(6,6) = 3;

%------------------------------------------------------------------------------
%% Define positions (via points)
% A = 0, B = 1, C = 2
%------------------------------------------------------------------------------

% 0 initial velocity and acceleration
xd = zeros(n_joints,1);
xdd = zeros(n_joints,1);

% Initial, A
x0 = transpose([0.10, 0.10, 0.10, 0, 0, 0]);
x0_pose = trvec2tform([x0(1), x0(2), x0(3)]) * eul2tform([x0(4), x0(5), x0(6)]);
t0 = 0;

% B
x1 = transpose([0.15, 0.10, 0.10, 0, 0, 0]);
x1_pose = trvec2tform([x1(1), x1(2), x1(3)]) * eul2tform([x1(4), x1(5), x1(6)]);
t1 = 3;

% C
x2 = transpose([0.15, 0.15, 0.10, 0, 0, 0]);
x2_pose = trvec2tform([x2(1), x2(2), x2(3)]) * eul2tform([x2(4), x2(5), x2(6)]);
t2 = 6;

% Final, A
t3 = 9;

%------------------------------------------------------------------------------
%% Perform simulation in simulink
%------------------------------------------------------------------------------

out = sim('IDC_OS.slx');

%------------------------------------------------------------------------------
%% Create figure
%------------------------------------------------------------------------------

close all
figure(); 

%% Redraw

numSamples = size(out.q,3);
jointsValueMat = reshape(out.q,[n_joints,numSamples]);

handtrace = true;
showRobStates(robot,jointsValueMat,sampleTime,'handtrace',handtrace,'showedFPS',60,'PreservePlot',false);

%------------------------------------------------------------------------------