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
weights = transpose([1, 1, 1, 1, 1, 1]);
initialguess = robot.homeConfiguration;

% Proportional matrix
K_P = 1*eye(n_joints);

% Derivative matrix
K_D = 1*eye(n_joints);

%------------------------------------------------------------------------------
%% Define positions (via points)
% A = 0, B = 1, C = 2
%------------------------------------------------------------------------------

% 0 initial velocity and acceleration (unused)
xd = zeros(n_joints,1);
xdd = zeros(n_joints,1);

% Initial, A
x0 = transpose([0.20, 0.20, 0.20, 0, 0, 0]);
x0_pose = makehgtform('translate', x0(1:3,:) ,'xrotate',x0(4),'yrotate',x0(5),'zrotate',x0(6));
t0 = 0;

% B
x1 = transpose([0.23, 0.20, 0.20, 0, 0, 0]);
t1 = 5;

% C
x2 = transpose([0.23, 0.23, 0.20, 0, 0, 0]);
t2 = 10;

% Final, A
t3 = 100;

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