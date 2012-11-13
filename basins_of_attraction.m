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

% Failure types
no_failure = 0;
fallen_backwards = 1;
fallen_forwards = 2;

% Initial conditions to vary
L_dot_0_vals    = linspace(0, -3, 101);   % Initial speed of spring compression, m/s
Phi_dot_0_vals  = linspace(0, 30, 101);        % Initial ang vel of spring, rad/s
[L_DOT_0S, PHI_DOT_0S] = meshgrid(L_dot_0_vals, Phi_dot_0_vals);
NUM_BOUNCES_COMPLETED = zeros(size(L_DOT_0S));
FAILURE_MODE = zeros(size(NUM_BOUNCES_COMPLETED));

num_bounces = 25; % How many steps/strides/bounces should system make to determine stability

%% RUN SIMULATION
for i=1:length(Phi_dot_0_vals)
    for j=1:length(L_dot_0_vals)
        l_dot_0     = L_DOT_0S(i,j);
        phi_dot_0   = PHI_DOT_0S(i,j);
        [Ts VALS num_bounces_completed failure_mode] = simulation (k, m, l0, g, ...
                                                                   l_dot_0, phi_dot_0, l_0, phi_0, ...
                                                                   x_dot_0, y_dot_0,   x_0, y_0, ...
                                                                   num_bounces);
        NUM_BOUNCES_COMPLETED(i,j) = num_bounces_completed;
        
        switch failure_mode
            case 'f'
                FAILURE_MODE(i,j) = fallen_forwards;
            case 'b'
                FAILURE_MODE(i,j) = fallen_backwards;
            case 'n'
                FAILURE_MODE(i,j) = no_failure;
            otherwise
                error ('Unknown failure type');
        end
    end
end

%% STORE RESULTS
savename = sprintf('basins_result_ldot_%d_to_%d_phidot_%d_to_%d.mat', min(L_dot_0_vals), max(L_dot_0_vals), ...
                                                                     min(Phi_dot_0_vals), max(Phi_dot_0_vals));
save (savename, 'L_DOT_0S', 'PHI_DOT_0S', 'NUM_BOUNCES_COMPLETED', 'FAILURE_MODE');


%% PLOT RESULTS
plot_basin_results (savename);
