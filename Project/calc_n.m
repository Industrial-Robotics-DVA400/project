%==============================================================================
% Author: Carl Larsson
% Description: Calculates n(q,q.) matrix
% Date: 18-03-2024
%==============================================================================
function n = calc_n(q,q_dot)

C = coriolisMatrix(robot, q, q_dot);
G = gravityTorque(robot, q);

n = C*q_dot + G;
end