clear all
close all

%Setup initial set of stable parameters
m = 1;
L0 = 1.5;
k = 600;
g = 9.81;
beta = 7*pi/16;
foot = 0; % x position of the foot
debug = 1;


q0 = 2; % start the model in the ballistic phase
Ne = 100; % simulate up to 100 events (or the end of the time interval)


x0 = -L0*cos(beta); % x position of the mass
y0 = L0*sin(beta) + 0.1; %y position of the mass (+0.1 moves it up to bring the foot off the ground)
xdot0 = 5; % initial x component of velocity
ydot0 = -0.2; % initial y component of velocity

p = [m L0 k g beta foot debug];
y0 = [x0 y0 xdot0 ydot0];
t = [0 60]; % simulate for 60 seconds


data = slipIntegrate(t, y0, q0, p, 500);

plot(data.o(:,1), data.o(:,2)); %plot the trajectory of the center of mass
