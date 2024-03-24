%==============================================================================
% Author: Carl Larsson, Pontus Svensson
% Description: Inverse dynamics control in operational space
% Date: 18-03-2024
%
% This software is licensed under the MIT License
% Refer to the LICENSE file for details
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
% Based on testing, first 3 elements are for orientation, last 3 for position
weights = transpose([0.1, 0.1, 0.1, 1, 1, 1]);
initialguess = robot.homeConfiguration;

% Proportional matrix
% Position
K_P = 3*eye(n_joints);
% Orientation
K_P(4,4) = 0.1*K_P(4,4);
K_P(5,5) = 0.1*K_P(5,5);
K_P(6,6) = 0.1*K_P(6,6);

% Derivative matrix
% Position
K_D = 2*eye(n_joints);
% Orientation
K_D(4,4) = 0.1*K_D(4,4);
K_D(5,5) = 0.1*K_D(5,5);
K_D(6,6) = 0.1*K_D(6,6);

%------------------------------------------------------------------------------
%% Define positions (via points)
% A = 0, B = 1, C = 2
%------------------------------------------------------------------------------

% 0 initial velocity
x_dot = zeros(n_joints,1);

% Initial point, A
x0 = transpose([0.30, 0.30, 0.30, 0, 0, 0]);
t0 = 0;
% For setting initial condition of integrator
x0_pose = trvec2tform([x0(1), x0(2), x0(3)]) * eul2tform([x0(4), x0(5), x0(6)]);

% B
x1 = transpose([0.40, 0.30, 0.30, 0, 0, 0]);
t1 = 10;

% C
x2 = transpose([0.40, 0.40, 0.30, 0, 0, 0]);
t2 = 20;

% Final point, A
x0;
t3 = 30;

%------------------------------------------------------------------------------
%% Perform simulation in simulink
%------------------------------------------------------------------------------

% Alternative way to run simulation in simulink
out = sim('IDC_OS.slx');

%------------------------------------------------------------------------------
%% Create figure
%------------------------------------------------------------------------------

close all
figure(); 

%% Draw/redraw
% Execute this section again to redraw motion

numSamples = size(out.q,3);
jointsValueMat = reshape(out.q,[n_joints,numSamples]);

handtrace = true;
showRobStates(robot,jointsValueMat,sampleTime,'handtrace',handtrace,'showedFPS',60,'PreservePlot',false);

%------------------------------------------------------------------------------