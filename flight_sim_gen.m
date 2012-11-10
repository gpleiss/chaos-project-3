function func = flight_sim_gen (x_dot_0, y_dot_0, g)
    func = @(t, Vars) flight_sim (t, Vars, x_dot_0, y_dot_0, g);
end

function Res = flight_sim (t, Vars, x_dot_0, y_dot_0, g)
    % Vars(1:4) are not used
    x       = Vars(5);
    y       = Vars(6);

    Res = zeros(6,1);
    Res(5) = d_x_dt (x_dot_0);
    Res(6) = d_y_dt (t, y_dot_0, g);
end

function res = d_x_dt (x_dot_0)
    res = x_dot_0;
end

function res = d_y_dt (t, y_dot_0, g)
    res = y_dot_0 - g*t;
end