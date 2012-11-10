function func = stand_sim_gen (k, m, l0)
    func = @(t, Vars) stand_sim (t, Vars, k, m, l0);
end

% Vars(1) is l_dot
% Vars(2) is phi_dot
% Vars(3) is l
% Vars(4) is phi
% Vars(5) is x_dot
% Vars(6) is y_dot
% Vars(7) is x
% Vars(8) is y
function Res = stand_sim (t, Vars, k, m, l0)
    l_dot   = Vars(1);
    phi_dot = Vars(2);
    l       = Vars(3);
    phi     = Vars(4);
    % Vars(5:8) are not used

    Res = zeros(8,1);
    Res(1) = d2_l_dt2(l_dot, l, phi_dot, phi, k, m, l0);
    Res(2) = d2_phi_dt2(l_dot, l, phi_dot, phi, k, m, l0);
    Res(3) = d_l_dt(l_dot, l, phi_dot, phi, k, m, l0);
    Res(4) = d_phi_dt(l_dot, l, phi_dot, phi, k, m, l0);
end

function res = d2_l_dt2 (l_dot, l, phi_dot, phi, k, m, l0)
    res = (l * phi_dot^2) - ((k/m) * (l-l0));
end

function res = d2_phi_dt2 (l_dot, l, phi_dot, phi, k, m, l0)
    res = -2 * l_dot * phi_dot / l;
end

function res = d_l_dt (l_dot, l, phi_dot, phi, k, m, l0)
    res = l_dot;
end

function res = d_phi_dt (l_dot, l, phi_dot, phi, k, m, l0)
    res = phi_dot;
end
