%==============================================================================
% Author: Carl Larsson
% Description: Calculates B(q)*y
% Date: 18-03-2024
%==============================================================================
function tau = calc_tau(y,q)

B = inertiaMatrix(robot,q);

tau = B*y;

end