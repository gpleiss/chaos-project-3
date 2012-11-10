function simulation
    close all;
    clc;

    k   = 1;
    m   = 0.05;
    l0  = 0.1;

    l_dot_0     = -0.5;
    l_0         = l0;
    phi_dot_0   = 1;
    phi_0       = -pi/3;


    func = standing_sim_gen (k, m, l0);
    function [val, isterm, dir] = events(t,X)
        val = X(2)-l0;
        isterm = 1;
        dir = 1;
    end
    [Ts XS] = ode45(func, [0 10], [l_dot_0; l_0; phi_dot_0; phi_0], odeset('Events', @events));
    Ls = XS(:,2);
    Phis = XS(:,4);

    figure();
    hold on;
    xlim ([-1.5*l0, 1.5*l0]);
    ylim ([0, 1.5*l0]);
    for i=1:length(Ls)
        cla;
        line([0, Ls(i)*sin(Phis(i))], [0, Ls(i)*cos(Phis(i))]);
        Ts(i)
        pause (Ts(i)*0.01);
    end    
    hold off;

    figure();
    plot(Ts, XS, 'o-');
end