prep;
clear all;

% Parameters of system
k   = 1000;      % Spring const, N/m
m   = 1;     % Mass of system, kg
l0  = 0.1;      % Uncompressed spring length, in m
g   = 9.8;      % Grav. const, m/s^2

% Fixed initial conditions
l_0         = l0;       % Initial length of spring, m
phi_0       = -pi/4;    % Initial ang of spring (to vertical), rad
x_dot_0     = 0;        % Change in x-coord of spring base, m/s
y_dot_0     = 0;        % Change in y-coord of spring base, m/s
x_0         = 0;        % x-coord of spring base, m
y_0         = 0;        % y-coord of spring base, m

% Initial conditions to vary
L_dot_0_vals    = linspace(-1.5, -2.5, 7);   % Initial speed of spring compression, m/s
Phi_dot_0_vals  = linspace(8, 12, 7);        % Initial ang vel of spring, rad/s
[L_DOT_0S, PHI_DOT_0S] = meshgrid(L_dot_0_vals, Phi_dot_0_vals);
NUM_BOUNCES_COMPLETED = zeros(size(L_DOT_0S));

num_bounces = 25; % How many steps/strides/bounces should system make to determine stability

%% RUN SIMULATION
for i=1:length(Phi_dot_0_vals)
    for j=1:length(L_dot_0_vals)
        l_dot_0     = L_DOT_0S(i,j);
        phi_dot_0   = PHI_DOT_0S(i,j);
        [Ts VALS num_bounces_completed] = simulation (k, m, l0, g, ...
                                                      l_dot_0, phi_dot_0, l_0, phi_0, ...
                                                      x_dot_0, y_dot_0,   x_0, y_0, ...
                                                      num_bounces);
        NUM_BOUNCES_COMPLETED(i,j) = num_bounces_completed;
    end
end

pcolor (L_DOT_0S, PHI_DOT_0S, NUM_BOUNCES_COMPLETED);
colorbar;
title ('Number of Bounces Completed');
xlabel ('l-dot_0');
ylabel ('\phi-dot_0');