% Function to setup and perform any one-time computations
% Input parameters
%   x - state of the rocket, x=[y,z,th,psi,dy,dz,dth,dpsi,m]^T
%   consts - structure that contains various system constants
% Output parameters
%   ctrl  -  any student defined control parameters
function ctrl = student_setup(x0, consts)
    % We provide possibility to switch the ode solver if needed.
    % Default ode_type should work in almost all cases.  Don't change this unless you know what you are doing.
    ctrl.ode_type = 0 ; % zero => ode45, non-zero=> ode15s.

    
    % Replace code below with any one time control design computation if needed.
    % Ex: ctrl.K = lqr(A, B, Q, R) ;
%     ctrl.K = [0.0000    0.0018    0.0997    0.4690   -0.0005    0.0353    0.4850    1.8646    0.0000;
%               0.0000    0.0000   -0.0036    1.0045    0.0000    0.0000   -0.0469    3.8980    0.0000] ;
    
    y = 0 ;
    z = consts.L ;
    th = 0 ; 
    psi = 0 ;
    
    dy = 0 ;
    dz = 0 ;
    dth = 0 ;
    dpsi = 0 ;
    
    m = x0(9) ;

    u_ff = [m*consts.g / consts.gamma ; 0] ;
    
    u1 = u_ff(1);
    u2 = u_ff(2);

    A = [ 0, 0,                          0,                          0, 1, 0, 0, 0,                            0;
          0, 0,                          0,                          0, 0, 1, 0, 0,                            0;
          0, 0,                          0,                          0, 0, 0, 1, 0,                            0;
          0, 0,                          0,                          0, 0, 0, 0, 1,                            0;
          0, 0, -(1000*u1*cos(psi + th))/m, -(1000*u1*cos(psi + th))/m, 0, 0, 0, 0,  (1000*u1*sin(psi + th))/m^2;
          0, 0, -(1000*u1*sin(psi + th))/m, -(1000*u1*sin(psi + th))/m, 0, 0, 0, 0, -(1000*u1*cos(psi + th))/m^2;
          0, 0,                          0,        -(12*u1*cos(psi))/7, 0, 0, 0, 0,                            0;
          0, 0,                          0,                          0, 0, 0, 0, 0,                            0;
          0, 0,                          0,                          0, 0, 0, 0, 0,                            0];
      
      B = [                       0,      0;
                                  0,      0;
                                  0,      0;
                                  0,      0;
            -(1000*sin(psi + th))/m,      0;
             (1000*cos(psi + th))/m,      0;
                   -(12*sin(psi))/7,      0;
                                  0, 64/403;
                                 -1,      0];

    Q = blkdiag(4e-2,20,500,500,1,500,750,750,800);
%     Q = blkdiag(10,1000,1,1,10,1000,1,300,1);
%     Q = blkdiag(1,1000,1,1,1,1000,1,100,1);
    R = [1e4 0;
         0 1];
    K = lqr(A,B,Q,R)
    ctrl.K = lqr(A,B,Q,R);
   
end