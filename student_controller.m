% Student supplied function to compute the control input at each instant of time
% Input parameters:
%   t  -  Time (in seconds) since start of simulation
%   x - state of the rocket, x=[y,z,th,psi,dy,dz,dth,dpsi,m]^T
%   consts - structure that contains various system constants
%   ctrl  -  any student defined control parameters
% Output parameters:
%   u  -  [thrust; torque] - two inputs to the rocket
function u = student_controller(t, x, consts, ctrl)
    % Replace the code below with your controller
    % Ex: u = -ctrl.K * x ;
    % A (bad) state-feedback controller is given below.
    
    m = x(end) ;
    x_d = [0;consts.L;0;0; 0;0;0;0;  m] ;
    u_ff = [m*consts.g / consts.gamma ; 0] ;
    
    % Output control input [thrust; torque]
    u = u_ff - ctrl.K*(x - x_d) ;
end