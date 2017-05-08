% Function that returns various constants of the system
function consts = get_consts()

    % System Constants
    % distances
    consts.L = 10 ;  %  Distance from COM of rocket to bottom of rocket  (Rocket length is 2*L)
    consts.r = 2 ;  %  Radius of rocket

    % mass
    consts.m_nofuel = 25 ; % Mass without fuel
    consts.max.m_fuel = 6*consts.m_nofuel ;  % Max fuel mass
    
    % inertia
    consts.J = 1/12*(consts.m_nofuel+consts.max.m_fuel)*(2*consts.L)^2 ;  %  Inertia of the rocket in body frame
    consts.JT = (83/320 + 1)*0.05*consts.m_nofuel*consts.r^2 ;  %  Inertial of the gimballed thruster
    
    % other constants
    consts.g = 9.81 ;  %  Acceleration due to gravity
    consts.gamma = 1000 ;  % Output velocity of the gas

    % Max parameters
    consts.max.y = 20 ;  %  Max y (horizontal position) at landing - landing pad goes from -y to +y
    consts.max.z = consts.L ;
    consts.max.theta = pi/6 ; %  Max theta at landing (radians)
    consts.max.speed = 5 ; % Max speed at landing
    consts.max.dtheta = 1 ; % Max rotational speed at landing
    consts.max.tau = 10 ; % Max thrust vectoring gimbal torque
    consts.max.fT = 25 ; % Max fuel burn rate - with gamma=1000, this corresponds to 14g
    consts.min.fT = 0.1 ; % Min fuel burn rate - Rocket engines shut off with zero thrust.
end