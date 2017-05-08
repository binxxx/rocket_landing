% Function that computes the score
% Input parameters:
%  x - landing state of the rocket
%  consts - constant parameters for the problem
% Output parameters:
% J - score for the problem - max 100
function J = compute_score(x, consts)
    x(3) = acos(cos(x(3))) ; % Do NOT penalize multiple rotations of 2*pi
    p = [x(end)-consts.m_nofuel-consts.max.m_fuel ;
         x(1) ;
         x(2)-consts.L ;
         x(3) ;
         norm(x(5:6)) ;
         x(7)] ;
    max = [consts.max.m_fuel ;
      consts.max.y ;
      consts.max.z ;
      consts.max.theta ;
      consts.max.speed ;
      consts.max.dtheta] ;
    alpha = [10  30  20  20  10 10]' ;

    if(any(abs(p(2:end)) > max(2:end)))
        J = 0 ;
    else
        J = sum(alpha.*(max-abs(p))./max ) ;
    end
end