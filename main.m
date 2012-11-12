close all;
clear all;
clc;

% Parameters of system
k   = 1000;     % Spring const, N/m
m   = 1;        % Mass of system, kg
l0  = 0.1;      % Uncompressed spring length, in m
g   = 9.8;      % Grav. const, m/s^2

% Fixed initial conditions
l_dot_0     = -2;       % Initial speed of spring compression, m/s
phi_dot_0   = 10;       % Initial ang vel of spring, rad/s
l_0         = l0;       % Initial length of spring, m
phi_0       = -pi/4;    % Initial ang of spring (to vertical), rad
x_dot_0     = 0;        % Change in x-coord of spring base, m/s
y_dot_0     = 0;        % Change in y-coord of spring base, m/s
x_0         = 0;        % x-coord of spring base, m
y_0         = 0;        % y-coord of spring base, m

num_bounces = 25; % How many steps/strides/bounces should system make?


%% RUN SIMULATION
[Ts VALS] = simulation (k, m, l0, g, ...
                        l_dot_0, phi_dot_0, l_0, phi_0, ...
                        x_dot_0, y_dot_0,   x_0, y_0, ...
                        num_bounces);
% VALS(:,1) is l_dot
% VALS(:,2) is phi_dot
% VALS(:,3) is l
% VALS(:,4) is phi
% VALS(:,5) is x_dot
% VALS(:,6) is y_dot
% VALS(:,7) is x
% VALS(:,8) is y


%% PLOT RESULTS
Ls      = VALS(:,3);
Phis    = VALS(:,4);
Xs      = VALS(:,7);
Ys      = VALS(:,8);

% This is the running simulation
figure();
hold on;
xlim ([min(Xs)-l0, max(Xs)+l0]);
% ylim ([0, max(Ys)+l0]);
ylim ([min(Xs)-l0, max(Xs)+l0]);
line([Xs(1), Xs(1)+Ls(1)*sin(Phis(1))], [Ys(1), Ys(1)+Ls(1)*cos(Phis(1))]);
plot(Xs(1)+Ls(1)*sin(Phis(1)), Ys(1)+Ls(1)*cos(Phis(1)), 'ro');
for i=2:length(Ts)
    pause ((Ts(i)-Ts(i-1)));
    cla;
    line([Xs(i), Xs(i)+Ls(i)*sin(Phis(i))], [Ys(i), Ys(i)+Ls(i)*cos(Phis(i))]);
    plot(Xs(i)+Ls(i)*sin(Phis(i)), Ys(i)+Ls(i)*cos(Phis(i)), 'ro');
end
hold off;

% Plot the phase plane
figure();
plot(Phis, Ls, 'o-');
xlabel('\phi');
ylabel('Length of spring');

% Plot the Xs and Ys.
figure();
plot(Xs, Ys, 'o-');
xlabel('X');
ylabel('Y');