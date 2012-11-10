function simulation
    close all;
    clc;

    k   = 10;      % Spring const, N/m
    m   = 1;     % Mass of system, kg
    l0  = 0.1;      % Uncompressed spring length, in m
    g   = 9.8;      % Grav. const, m/s^2

    l_dot_0     = -3;       % Initial speed of spring compression, m/s
    phi_dot_0   = 15;       % Initial ang vel of spring, rad/s
    l_0         = l0;       % Initial length of spring, m
    phi_0       = -pi/3;    % Initial ang of spring (to vertical), rad


    func_stand = stand_sim_gen (k, m, l0);
    function [val, isterm, dir] = events_stand(t, Vals)
        val     = Vals(3) - l0;
        isterm  = 1;
        dir     = 1;
    end
    [Ts1 VALS1] = ode45(func_stand, [0 10], [l_dot_0; phi_dot_0; l_0; phi_0; 0; 0], ...
                        odeset('Events', @events_stand));
    L_dots  = VALS1(:,1);
    Ls      = VALS1(:,3);
    Phis    = VALS1(:,4);

    x_dot_0 = L_dots(end) * sin(Phis(end));
    y_dot_0 = L_dots(end) * cos(Phis(end));
    x_0     = Ls(end) * sin(Phis(end));
    y_0     = Ls(end) * cos(Phis(end));

    func_flight = flight_sim_gen (x_dot_0, y_dot_0, g);
    function [val, isterm, dir] = events_flight(t, Vals)
        val     = Vals(6) - l_0*cos(phi_0);
        isterm  = 1;
        dir     = -1;
    end
    [Ts2 VALS2] = ode45(func_flight, [0 10], [0; 0; 0; 0; x_0; y_0], ...
                        odeset('Events', @events_flight));
    Xs      = VALS2(:,5);
    Ys      = VALS2(:,6);

    figure();
    hold on;
    xlim ([-1.5*l0, 10*l0]);
    ylim ([0, 5*l0]);
    for i=1:length(Ts1)
        cla;
        line([0, Ls(i)*sin(Phis(i))], [0, Ls(i)*cos(Phis(i))]);
        plot(Ls(i)*sin(Phis(i)), Ls(i)*cos(Phis(i)), 'ro')
        pause (Ts1(i)*0.01);
    end    
    for i=1:length(Ts2)
        cla;
        line([Xs(i), Xs(i)-l0*sin(phi_0)], [Ys(i), Ys(i)-l0*cos(phi_0)]);
        plot(Xs(i), Ys(i), 'ro');
        pause (Ts2(i)*0.1);
    end
    hold off;

    figure();
    plot(Ts1, VALS1, 'o-');
    figure();
    plot(Ts2, VALS2, 'o-');
end