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
    
    u1 = u_ff(1);
    u2 = u_ff(2);
    % Output control input [thrust; torque]
%     u = u_ff - ctrl.K*(x - x_d) ;
    
    %% self defined controller
    % linearized system
%     A = jacobian(f,x)
%     B = jacobian(f,u)
    
    y = x(1) ;
    z = x(2) ;
    th = x(3) ; 
    psi = x(4) ;
    
    dy = x(5) ;
    dz = x(6) ;
    dth = x(7) ;
    dpsi = x(8) ;
    
    m = x(9) ;

    A = [ 0, 0,                          0,                          0, 1, 0, 0, 0,                            0;
          0, 0,                          0,                          0, 0, 1, 0, 0,                            0;
          0, 0,                          0,                          0, 0, 0, 1, 0,                            0;
          0, 0,                          0,                          0, 0, 0, 0, 1,                            0;
          0, 0, -(1000*u1*cos(psi + th))/m, -(1000*u1*cos(psi + th))/m, 0, 0, 0, 0,  (1000*u1*sin(psi + th))/m^2;
          0, 0, -(1000*u1*sin(psi + th))/m, -(1000*u1*sin(psi + th))/m, 0, 0, 0, 0, -(1000*u1*cos(psi + th))/m^2;
          0, 0,                          0,      -(3*u1*cos(psi))/1750, 0, 0, 0, 0,                            0;
          0, 0,                          0,                          0, 0, 0, 0, 0,                            0;
          0, 0,                          0,                          0, 0, 0, 0, 0,                            0];
      
      B = [                       0,      0;
                                  0,      0;
                                  0,      0;
                                  0,      0;
            -(1000*sin(psi + th))/m,      0;
             (1000*cos(psi + th))/m,      0;
                 -(3*sin(psi))/1750,      0;
                                  0, 64/403;
                                 -1,      0];

    Q = blkdiag(1,1,1,1,1,1,1,1,1);
    R = eye(2);
    K = lqr(A,B,Q,R)
    u = u_ff - K*(x - x_d) ;
end