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
    
    % extract attitude
    th = x(3); psi = x(4); dth = x(7); dpsi = x(8); m = x(9);
    u1 = u_ff(1); u2 = u_ff(2);
    x_d_attd = [0;0;0;0;m_d];
    % Output control input [thrust; torque]
    if (sqrt(th^2+dth^2) > 2 && sqrt(th^2+dth^2) <= 2.5 )
        u = u_ff - ctrl.K_attd1*([th;psi;dth;dpsi;m] - x_d_attd) ;
    elseif (sqrt(th^2+dth^2) > 2.5  )
        u = u_ff - ctrl.K_attd2*([th;psi;dth;dpsi;m] - x_d_attd) ;
    elseif (x(2) < 25)
        u =  u_ff - ctrl.K*(x - x_d) ;
    elseif (x(2) >= 25 && x(2) <= 50)
        u = u_ff - ctrl.K*(x - [0;20;0;0; 0;0;0;0;  m_d]) ;
    else
        u = u_ff - ctrl.K*(x - [0;x(2)/2;0;0; 0;0;0;0;  m_d]) ;
    end
%     if (sqrt(th^2+dth^2) > 0.2 && isempty(FLAG))
%         u = u_ff - ctrl.K_attd*([th;psi;dth;dpsi;m]-x_d_attd) ;
%         1;
%     else
%         u =  u_ff - ctrl.K*(x - x_d) ;
%         FLAG = 1;
%         2;
%     end
    
end