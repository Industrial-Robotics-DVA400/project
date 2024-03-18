%==============================================================================
% Author: Carl Larsson
% Description: Inverse dynamics control in operational space
% Date: 18-03-2024
%==============================================================================
%% Clean up
clc
clear
close all
disp("==============================================================================")
%% Define robot
%------------------------------------------------------------------------------
% Load 6R robot
n_joints = 6;
robot = loadrobot("universalUR5");
robot.DataFormat = 'column';
robot.Gravity = [0,0,-9.81]';

showdetails(robot)
%------------------------------------------------------------------------------
%% Matrices
%------------------------------------------------------------------------------

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
%% Define initial and final
% A = 0, B = 1, C = 2
%------------------------------------------------------------------------------

% Initial, A
x0 = transpose([0.25, 0.25, 0.25]);
xd0 = zeros(n_joints,1);
xdd0 = zeros(n_joints,1);
t0 = 0;

% B
x1 = transpose([0.5, 0.5, 0.5]);
xd1 = zeros(n_joints,1);
xdd1 = zeros(n_joints,1);
t1 = 5;

% C
x2 = transpose([0.4, 0.4, 0.4]);
xd2 = zeros(n_joints,1);
xdd2 = zeros(n_joints,1);
t2 = 10;

% Final, A
t3 = 15;
%------------------------------------------------------------------------------
%% Visualize robot and trajectory
%------------------------------------------------------------------------------
% All via points, together forming entire trajectory A->B->C->A
close all
show(robot, q0)
hold on
show(robot, q1)
show(robot, q2)
show(robot, q0);
%% Perform simulation in simulink
out = sim('IDC_OS.slx');
%% Redraw
sampleTime = 0.001;
numSamples = size(out.q,3);
jointsValueMat = reshape(out.q,[nJoints,numSamples]);
showRobStates(robot,jointsValueMat,sampleTime);
handtrace = true;
showRobStates(robot,jointsValueMat,sampleTime,'handtrace',handtrace,'showedFPS',60,'PreservePlot',false);
%------------------------------------------------------------------------------