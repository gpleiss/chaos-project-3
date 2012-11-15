prep;
clear all;

num_iterations = 20;
% Parameters of system
k   = 1000;     % Spring const, N/m
m   = 1;        % Mass of system, kg
l0  = 0.1;      % Uncompressed spring length, in m
g   = 9.8;      % Grav. const, m/s^2

% Fixed initial conditions
l_dot_0     = -0.5;       % Initial speed of spring compression, m/s
phi_dot_0   = 9.2;       % Initial ang vel of spring, rad/s

l_0         = l0;       % Initial length of spring, m
phi_0       = -9*pi/32;    % Initial ang of spring (to vertical), rad
x_dot_0     = 0;        % Change in x-coord of spring base, m/s
y_dot_0     = 0;        % Change in y-coord of spring base, m/s
x_0         = 0;        % x-coord of spring base, m
y_0         = 0;        % y-coord of spring base, m

num_bounces = 5; % How many steps/strides/bounces should system make?


f = poincare_gen(k, m, l0, g, ...
                  l_0, phi_0, ...
                  x_dot_0, y_dot_0,   x_0, y_0);
for i=1:num_iterations
    phi_dot_0 = fzero(@(phi_dot)( phi_dot - function_reducer(f,[l_dot_0;phi_dot],2)  ), phi_dot_0);

    l_dot_0 = fzero(@(l_dot)( l_dot - function_reducer(f,[l_dot;phi_dot_0],1)  ), l_dot_0);
end