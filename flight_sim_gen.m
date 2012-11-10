function func = flight_sim_gen (g)
    func = @(t, Vars) flight_sim (t, Vars, g);
end

% Vars(1) is l_dot
% Vars(2) is phi_dot
% Vars(3) is l
% Vars(4) is phi
% Vars(5) is x_dot
% Vars(6) is y_dot
% Vars(7) is x
% Vars(8) is y
function Res = flight_sim (t, Vars, g)
    % Vars(1:4) are not used
    x_dot   = Vars(5);
    y_dot   = Vars(6);
    x       = Vars(7);
    y       = Vars(8);

    Res = zeros(8,1);
    Res(5) = d2_x_dt2 ();
    Res(6) = d2_y_dt2 (g);
    Res(7) = d_x_dt (x_dot);
    Res(8) = d_y_dt (y_dot);
end

function res = d2_x_dt2 ()
    res = 0;
end

function res = d2_y_dt2 (g)
    res = -g;
end

function res = d_x_dt (x_dot)
    res = x_dot;
end

function res = d_y_dt (y_dot)
    res = y_dot;
end