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
    m_d = x(end) ;
    x_d = [0;consts.L;0;0; 0;0;0;0;  m_d] ;
    u_ff = [m_d*consts.g / consts.gamma ; 0] ;
    
    
    % Output control input [thrust; torque]
    u =  u_ff - ctrl.K*(x - x_d) ;
    
    %% trial
%     K = ctrl.K;
%     u1 =(-K(1,1)*x(1)-K(1,5)*x(5))/cos(x(1))...
%         -K(1,2)*x(2)-K(1,3)*x(3)-K(1,4)*x(4)-K(1,6)*x(6)...
%         -K(1,7)*x(7)-K(1,8)*x(8)-K(1,9)*x(9);
%     u2 =(-K(2,1)*x(1)-K(2,5)*x(5))/cos(x(1))...
%         -K(2,2)*x(2)-K(2,3)*x(3)-K(2,4)*x(4)-K(2,6)*x(6)...
%         -K(2,7)*x(7)-K(2,8)*x(8)-K(2,9)*x(9);
%     u = u_ff + [u1;u2];
end