% Function to animate rocket
function animate_rocket(t_orig, x_orig, u_orig)
    RATE = 25*0.05 ; % Change this to speed up / slow down animation [Matlab R2014b and onwards appears to have slowed down graphics rendering!].  If someone finds a fix for this, please let me know.
    % RATE = 25 ;  % Use this for Matlab R2014a and below.
    
    t_vec = linspace(t_orig(1), t_orig(end), round( (t_orig(end)-t_orig(1))*RATE ))' ;
    x_vec = interp1(t_orig, x_orig, t_vec, 'linear');
    u_vec = interp1(t_orig, u_orig, t_vec, 'linear');

    % Setup figure
    [fig1 axes1] = prepare_fig() ;

    consts = get_consts() ;
    
    % Animation loop
    for i=1:length(t_vec)
        y = x_vec(i,1) ;
        z = x_vec(i,2) ;
        th = x_vec(i,3) ;
        psi = x_vec(i,4) ;
        m = x_vec(i,end) ;
        u = u_vec(i,1) ;
        delete(get(axes1,'Children'));
        plot(x_vec(1:i,1), x_vec(1:i,2), 'b') ;
        draw_rocket(y,z,th,psi,m, u,  x_vec(i,:),consts) ;
        drawnow ;
    end
    J = compute_score(x_vec(end,:)', consts) ;
    if (J == 0)
        load('anim_data') ;
        yrange = y+[-10 10] ;
        zrange = z+[-10 10]-5 ;
        if(min(zrange) < -5)
            zrange = zrange - min(zrange) - 5 ;
        end
        imagesc(yrange, zrange, flipdim(anim_data,1), 'AlphaData', (1-flipud(sum(anim_data, 3)==3*255))*1) ;
    end
end


% Draw a rocket + thruster + FIRE
function draw_rocket(y,z,th,psi,m, u,  x_vec,consts)
    x_len = consts.r ;
    y_len = consts.L ;
    points = [-x_len -y_len ;
              -x_len 0.8*y_len ;
              0 y_len ;
              x_len 0.8*y_len ;
              x_len -y_len ;
              -x_len -y_len] ;

    R_th = rot(th) ;
    for j=1:length(points)
        points(j,:) = (R_th*points(j,:)')' ;
    end
    
    % Draw rocket + thruster
    patch(y+points(:,1),z+points(:,2), 'b'); hold on ;
    p = [y;z] + R_th*[0; -y_len] ;
    plot_ellipse(p(1), p(2), 1.5*x_len, 0.2*y_len, th+psi, 'k', 1, [0 pi], y_len) ;
    plot_ellipse(p(1), p(2), 1.5*x_len, 0.05*y_len, th+psi, 'k', 1, [0 2*pi], y_len) ;

    % Draw fire
    R_th_psi = rot(th+psi) ;
    for j=linspace(-1,1,50)
        lw_length = rand*1.5*y_len*u ;
        p1 = p + R_th_psi*[j*1.5*x_len; -0.2*y_len] ;
        p2 = p + R_th_psi*[j*1.5*x_len; -0.2*y_len-lw_length] ;
        pp = plot([p1(1);p2(1)],[p1(2);p2(2)]) ;
        set(pp, 'Color', [1 abs(j) 0]) ;
    end
    
    % Draw landing pad
    plot(consts.max.y*[-1 1], -1*[1 1], 'k', 'LineWidth', 2) ;
    
    axis([y-100  y+100  max([-10, z-60])  max([110 z+60])]) ;
    v = axis() ;
    
    % Text
    x = v(1)+0.9*(v(2)-v(1)) ;
    y = v(3)+0.9*(v(4)-v(3)) ;
    text(x,y, ['Fuel=' num2str((m-consts.m_nofuel)/consts.max.m_fuel*100, '%.1f') '%'], 'HorizontalAlignment','right') ;
    text(x,y-5, ['Speed=' num2str(norm(x_vec(5:6)), '%.1f') ' m/s'], 'HorizontalAlignment','right') ;
    
    xlabel('y') ; ylabel('z') ;
    camlight;
    grid on;
end

% Function to plot a ellipse
function plot_ellipse(x0, y0, a, b, phi, clr, lw, range, y_len)
    num = 100;      % #points -> smoothness 

    theta = linspace(range(1),range(2),num);
    p(1,:) = a*cos(theta);
    p(2,:) = -0.1*y_len+b*sin(theta);
    R = rot(phi) ;
    for j=1:length(p)
        p(:,j) = R*p(:,j) ;
    end
    plot(x0+p(1,:), y0+p(2,:),clr,'LineWidth',lw)
end

% Function to create a 2D rotation matrix
function R = rot(th)
    R = [cos(th)   -sin(th) ;
         sin(th)    cos(th)] ;
end

% Function to set up the initial figure
function [fig1 axes1] = prepare_fig()
    fig1 = figure;

    set(0,'Units','pixels')
    scnsize = get(0,'ScreenSize');

    screen_width = scnsize(3);
    screen_height = scnsize(4);

    figure_x_limits = [-100 100];
    figure_y_limits = [-10 100];

    % find the minimum scaling factor

    figure_x_size = figure_x_limits(2) - figure_x_limits(1);
    figure_y_size = figure_y_limits(2) - figure_y_limits(1);

    xfactor = screen_width/figure_x_size;
    yfactor = screen_height/figure_y_size;

    if (xfactor < yfactor)
      screen_factor = 0.5*xfactor;
    else
      screen_factor = 0.5*yfactor;
    end

    % calculate screen offsets
    screen_x_offset = (screen_width - screen_factor*figure_x_size)/2;
    screen_y_offset = (screen_height - screen_factor*figure_y_size)/2;

    % draw figure and axes
    set(fig1,'Position', [screen_x_offset screen_y_offset screen_factor*figure_x_size screen_factor*figure_y_size]);
    axes1 = axes;
    set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
    set(axes1,'Color','w');
end