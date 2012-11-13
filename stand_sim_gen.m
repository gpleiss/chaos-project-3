function func = stand_sim_gen (k, m, l0, g)
    func = @(t, Vars) stand_sim (Vars, k, m, l0, g);
end

% Vars(1) is l_dot
% Vars(2) is phi_dot
% Vars(3) is l
% Vars(4) is phi
% Vars(5) is x_dot
% Vars(6) is y_dot
% Vars(7) is x
% Vars(8) is y
function Res = stand_sim (Vars, k, m, l0, g)
    l_dot   = Vars(1);
    phi_dot = Vars(2);
    l       = Vars(3);
    phi     = Vars(4);
    % Vars(5:8) are not used

    Res = zeros(8,1); % The derivatives of x_dot, y_dot, x, and y are zero
    Res(1) = d2_l_dt2(l, phi_dot, phi, k, m, l0, g);
    Res(2) = d2_phi_dt2(l_dot, l, phi_dot, phi, g);
    Res(3) = d_l_dt(l_dot);
    Res(4) = d_phi_dt(phi_dot);
end

function res = d2_l_dt2 (l, phi_dot, phi, k, m, l0, g)
    res = (l * phi_dot^2) - ((k/m) * (l-l0)) - g * cos(phi);
end

function res = d2_phi_dt2 (l_dot, l, phi_dot, phi, g)
    res = (-2 * l_dot * phi_dot + g * sin(phi))/ l;
end

function res = d_l_dt (l_dot)
    res = l_dot;
end

function res = d_phi_dt (phi_dot)
    res = phi_dot;
end
