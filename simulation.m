function simulation
    close all;
    clc;

    k   = 10;      % Spring const, N/m
    m   = 1;     % Mass of system, kg
    l0  = 0.1;      % Uncompressed spring length, in m
    g   = 9.8;      % Grav. const, m/s^2

    l_dot_0     = -3;       % Initial speed of spring compression, m/s
    phi_dot_0   = 20;       % Initial ang vel of spring, rad/s
    l_0         = l0;       % Initial length of spring, m
    phi_0       = -pi/3;    % Initial ang of spring (to vertical), rad
    x_dot_0     = 0;        % Change in x-coord of spring base, m/s
    y_dot_0     = 0;        % Change in y-coord of spring base, m/s
    x_0         = 0.1;      % x-coord of spring base, m
    y_0         = 0;        % y-coord of spring base, m


    Ts = [0];
    VALS = [0 0 0 0 0 0 0 0];
    % VALS(:,1) is l_dot
    % VALS(:,2) is phi_dot
    % VALS(:,3) is l
    % VALS(:,4) is phi
    % VALS(:,5) is x_dot
    % VALS(:,6) is y_dot
    % VALS(:,7) is x
    % VALS(:,8) is y

    %% STANDING PHASE
    func_stand = stand_sim_gen (k, m, l0);
    function [val, isterm, dir] = events_stand(t, Vals)
        % Function ends when spring force is 0 (i.e. when spring returns to original length)
        val     = Vals(3) - l0;
        isterm  = 1;
        dir     = 1;
    end
    % In this part of simulation, x_dot,y_dot,x,y are unchanging
    [Ts1 VALS1] = ode45(func_stand, [Ts(end) 10], [l_dot_0; phi_dot_0; l_0; phi_0; x_dot_0; y_dot_0; x_0; y_0], ...
                        odeset('Events', @events_stand));
    
    % End values from stand phase
    l_dot_f     = VALS1(end,1);
    l_f         = VALS1(end,3);
    phi_f       = VALS1(end,4);
    x_f         = VALS1(end,7); % Should remain unchanged
    y_f         = VALS1(end,8); % Should remain unchanged

    % Starting values for flight phase
    x_dot_0 = l_dot_f*sin(phi_f);
    y_dot_0 = l_dot_f*cos(phi_f);
    % Assumption made in paper: coords of spring top are same as at end of flight phase
    % but coords of spring bottom reset themselves so that phi_0, l_0 are the same as
    % at the beginning of the stand phase
    x_0         = l_f*sin(phi_f) - l0*sin(phi_0) + x_f;
    y_0         = l_f*cos(phi_f) - l0*cos(phi_0) + y_f;
    l_dot_0     = 0;
    phi_dot_0   = 0;
    l_0         = l0;
    %phi_0      = phi_0;


    %% FLIGHT PHASE
    func_flight = flight_sim_gen (g);
    function [val, isterm, dir] = events_flight(t, Vals)
        val     = Vals(8);
        isterm  = 1;
        dir     = -1;
    end
    % In this part of simulation, l_dot,phi_dot,l,phi are unchanging
    [Ts2 VALS2] = ode45(func_flight, [Ts1(end) 10], [l_dot_0; phi_dot_0; l0; phi_0; x_dot_0; y_dot_0; x_0; y_0], ...
                        odeset('Events', @events_flight));

    % End values from flight phase
    l_dot_f     = VALS2(end,1); % Should be 0 
    phi_dot_f   = VALS2(end,2); % Should be 0
    l_f         = VALS2(end,3); % Should remain unchanged
    phi_f       = VALS2(end,4); % Should remain unchanged 
    x_dot_f     = VALS2(end,5);
    y_dot_f     = VALS2(end,6); 
    x_f         = VALS2(end,7);
    y_f         = VALS2(end,8);

    % Starting values for stand phase
    l_dot_0     = x_dot_f*sin(phi_f) + y_dot_f*cos(phi_f)
    phi_dot_0   = (x_dot_f*cos(phi_f) + y_dot_f*sin(phi_f)) / l_f
    l_0         = l0
    phi_0       = phi_0
    x_dot_0     = 0
    y_dot_0     = 0
    x_0         = x_f
    y_0         = y_f


    %% OUTPUT VALUES
    Ts      = [Ts; Ts1; Ts2];
    VALS    = [VALS; VALS1; VALS2];


    %% PLOT RESULTS
    Ls      = VALS(:,3);
    Phis    = VALS(:,4);
    Xs      = VALS(:,7);
    Ys      = VALS(:,8);

    figure();
    hold on;
    xlim ([-1.5*l0, 10*l0]);
    ylim ([0, 5*l0]);
    line([Xs(1), Xs(1)+Ls(1)*sin(Phis(1))], [Ys(1), Ys(1)+Ls(1)*cos(Phis(1))]);
    plot(Xs(1)+Ls(1)*sin(Phis(1)), Ys(1)+Ls(1)*cos(Phis(1)), 'ro');
    for i=2:length(Ts)
        pause ((Ts(i)-Ts(i-1)));
        cla;
        line([Xs(i), Xs(i)+Ls(i)*sin(Phis(i))], [Ys(i), Ys(i)+Ls(i)*cos(Phis(i))]);
        plot(Xs(i)+Ls(i)*sin(Phis(i)), Ys(i)+Ls(i)*cos(Phis(i)), 'ro');
    end
    hold off;

    figure();
    plot(Phis, Ls, 'o-');
    figure();
    plot(Xs, Ys, 'o-');
end