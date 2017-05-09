clear; clc;

consts = get_consts();
M = consts.max.m_fuel + consts.m_nofuel;
gam = consts.gamma;
g = consts.g;
J = consts.J;
JT = consts.JT;
L = consts.L;
syms y z th psi dy dz dth dpsi m u1 u2 real;
% whole dynamic system
f = [dy; dz; dth; dpsi; -gam/m*sin(th+psi)*u1; -g+gam/m*cos(th+psi)*u1; -L/J*gam*sin(psi)*u1; 
   1/JT*u2; -u1];
x = [y;z;th;psi;dy;dz;dth;dpsi;m];
u = [u1;u2];
% linearize
A = jacobian(f,x)
B = jacobian(f,u)

% A = subs(A,{y,z,th,psi,dy,dz,dth,dpsi,m,u1,u2},{0,L,0,0,0,0,0,0,M,M*g,0});
% B = subs(B,{y,z,th,psi,dy,dz,dth,dpsi,m,u1,u2},{0,L,0,0,0,0,0,0,M,M*g,0});
% A = eval(A)
% B = eval(B)
% Q = blkdiag(1,1,1000,1000,10,1000,1,1,1);
% R = eye(2);
% K = lqr(A,B,Q,R);