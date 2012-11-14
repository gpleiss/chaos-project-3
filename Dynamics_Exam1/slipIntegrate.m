% [T,Y,Q,P,O,Te,Ye,Ie,Qe,Oe] = slipIntegrate(t, y0, q0, p, Ne)
% 
% Spring-Loaded Inverted Pendulum model for locomotion
%
% Inputs:
% t - 1 x Nt - times to sample state
% y0 - 1 x Ny - initial condition
% q0 - scalar - initial state
% p - 1 x Np - parameters
% Ne - scalar - number of events
% 
% Outputs:
% slip - struct - contains trajectory data
%   .T - times
%   .Y - states
%   .Q - discrete modes
%   .P - parameters
%   .O - observations
%   .Te - event times
%   .Ye - event states
%   .Qe - event discrete modes
%   .Oe - event observations
%
% $Revision: $
% by Sam Burden, Berkeley 2009
% by Aaron Hoover, Olin College 2012

function slip = slipIntegrate(t, y0, q0, p, Ne)

m = p(1);
L0 = p(2);
k = p(3);
g = p(4);
beta = p(5);
foot = p(6);
debug = p(7);

t0 = t(1);
tf = t(end);
ts = t;
tol = 1e-6;

if length(ts) == 2
  dt = 0.01;
  ts = t0:dt:tf;
end

% Initialize trajectory
T = t0;
Ts = t0;
Y = y0;
Q = q0;
P = p;
% Use the array, O, to store quantities you'd like to compute/save during the
% simulation like potential and kinetic energy, x-y positions and
% velocities of the mass, and x-y positions of the foot.
O = observation(T,Y,Q,p); 
Os = O;
Te = [];
Ye = [];
Ie = [];
Qe = [];
Oe = [];

% Integrate dynamics
while T(end) < tf && length(Qe) < 2*Ne && Q(end)
  
  t0 = T(end);
  q0 = Q(end);
  y0 = Y(end,:);
  p0 = P(end,:);

  options = odeset('Events',@(t,y) events(t,y,q0,p0),'RelTol',1e-7);
  
  
  if debug
    disp(['t0 = ',num2str(t0),', q0 = ',num2str(q0),', y0 = ',num2str(y0)]);
    disp(['p0 = ',num2str(p0)]);
  end
  
  [t,y,te,ye,ie] = ode45(@(t,y) dynamics(t,y,q0,p0), [t0,ts(ts > t0)], y0, options);

  T = [T; t];
  Ts = [Ts; t(2:end-1)];
  Y = [Y; y];
  Q = [Q; q0*ones(size(t))];
  for n = 1:length(t)
    o = observation(t(n),y(n,:),q0,p0);
    O = [O; o];
    
    if n > 1 && n < length(t)
      Os = [Os; o];
    end
    
    if abs(tf-t(n)) < tol
      Ts = [Ts; tf];
      Os = [Os; o];
    end
  end
  
  if ~isempty(te)
    Te = [Te; te];
    Ye = [Ye; ye];
    Ie = [Ie; ie];
    Qe = [Qe; q0];
    Oe = [Oe; observation(te, ye, q0,p0)];
  end
  
  if T(end) < tf
    % Apply state transition
    [tr,y,q,p] = transition(t(end),y,q0,p0,te,ye,ie);
    T = [T; nan*tr; tr]; % Add nan's to indicate discontinuities
    Y = [Y; nan*y; y];
    Q = [Q; nan*q; q];
    P = [P; p];
    o = observation(tr,y,q,p);
    O = [O; nan*o; o];
    Te = [Te; tr];
    Ye = [Ye; y];
    Ie = [Ie; nan];
    Qe = [Qe; q];
    Oe = [Oe; o];
  end

end % while

% keyboard

slip = struct('t',T,'ts',Ts,'y',Y,'q',Q,'p',P,'o',O,'os',Os,'te',Te,'ye',Ye,'ie',Ie,'qe',Qe,'oe',Oe);

end % slipIntegrate


% Trajectory observation - use this function to compute values for energy,
% positions, velocities, etc.
% All observations are in Cartesian coordinates
function o = observation(t,z,q,p)

m = p(1);
L0 = p(2);
k = p(3);
g = p(4);
beta = p(5);
foot = p(6);
debug = p(7);

switch q
  
  % Stance
  % State represents polar coordinates of COM around foot
  case 1
    
    L = z(1);
    psi = z(2);
    dL = z(3);
    dpsi = z(4);
    
    x = ??? %The x position of the mass 
    y = ??? %The y position of the mass
    dx = ??? %Magnitude of the x velocity of the mass
    dy = ??? %Magnitude of the y velocity of the mass 
    fx = ??? %The x position of the foot
    fy = ??? %The y position of the foot
    PEspring = ??? %Energy stored in the spring
    PE = ??? %Total potential energy
    KE = ??? %Kinetic energy
    E_tot = PE + KE;

    o = [x,y,dx,dy,fx,fy,PEspring,PE,KE,E_tot];
    
  % Flight
  % State represents cartesian coordinates of COM
  case 2

    x = z(1);
    y = z(2);
    dx = z(3);
    dy = z(4);

    x = ??? %The x position of the mass 
    y = ??? %The y position of the mass
    dx = ??? %Magnitude of the x velocity of the mass
    dy = ??? %Magnitude of the y velocity of the mass 
    fx = ??? %The x position of the foot
    fy = ??? %The y position of the foot
    PEspring = ??? %Energy stored in the spring
    PE = ??? %Total potential energy
    KE = ??? %Kinetic energy
    E_tot = PE + KE;

    o = [x,y,dx,dy,fx,fy,PEspring,PE,KE,E_tot];
    
  otherwise
     disp(['Warning: (obs) unknown discrete mode']);
    o = nan*zeros(1,10);
end % switch

end % observation
  

% SLIP Dynamics
% From Gilgliazza, Koditschek, Holmes 2006
function dz = dynamics(t,z,q,p)

m = p(1);
L0 = p(2);
k = p(3);
g = p(4);
beta = p(5);
foot = p(6);
debug = p(7);

switch q
  
  % Stance
  % polar coordinates of COM around foot
  case 1

    if debug, disp(['(dyn) stance, z = ',num2str(z')]);, end
    
    L  = z(1);
    psi   = z(2);
    dL = z(3);
    dpsi  = z(4);
   
    %---- INSERT YOUR POLAR EQUATIONS OF MOTION TO BE INTEGRATED HERE ----%

    dz = [%dz(1); ...
          %dz(2); ...
          %dz(3); ...
          %dz(4)];


    %---- END POLAR EQUATIONS OF MOTION ----%
        
  % Flight
  % cartesian coordinates of COM
  case 2

    if debug, disp(['(dyn) flight, z = ',num2str(z')]);, end

    x  = z(1);
    y  = z(2);
    dx = z(3);
    dy = z(4);
    
    %---- INSERT YOUR CARTESIAN EQUATIONS OF MOTION TO BE INTEGRATED HERE ----%
   
    dz = [%dz(1); ...
          %dz(2); ...
          %dz(3); ...
          %dz(4)];

    %---- END CARTESIAN EQUATIONS OF MOTION ----%

  otherwise
    error(['Warning: (dyn) unknown discrete mode']);
    dz = nan*z;
end % switch

end % dynamics

                 
% Event detection
function [value,isterminal,direction] = events(t,z,q,p)

m = p(1);
L0 = p(2);
k = p(3);
g = p(4);
beta = p(5);
foot = p(6);
debug = p(7);

switch q
  
  % Stance
  % polar coordinates of COM around foot
  case 1
    
    if debug, disp(['(events) stance, z = ',num2str(z')]);, end

    L = z(1);
    psi = z(2);
    dL = z(3);
    dpsi = z(4);
    
    % Toe leaves ground plane, 
    % body penetrates ground plane
    value = [L0 - L, L > 0, (psi > -pi/2 && psi < pi/2)];
    isterminal = [1, 1, 1];
    direction = [-1, 0, 0];
        
  % Flight
  % cartesian coordinates of COM
  case 2
    
    if debug, disp(['(events) flight, z = ',num2str(z')]);, end

    x = z(1);
    y = z(2);
    dx = z(3);
    dy = z(4);
    
    % Hip height
    h = L0*sin(beta);

    % Toe penetrates ground plane, 
    % body penetrates ground plane
    value = [y - h, y > 0];
    isterminal = [1, 1];
    direction = [-1, 0];
    
  otherwise
    error(['Warning: (event) unknown discrete mode']);
    value = 0;
    isterminal = 1;
    direction = 0;
end % switch

end % events


% Transition between discrete modes
function [t,z,q,p] = transition(t,z,q,p,te,ze,ie)

m = p(1);
L0 = p(2);
k = p(3);
g = p(4);
beta = p(5);
foot = p(6);
debug = p(7);

switch q
  
  % Stance
  % polar coordinates of COM around foot
  case 1
    
    % Body penetrated ground plane
    if any(ie == 2) || any(ie == 3)
      t = nan*te;
      z = nan*ze;
      q = nan;
       
    % Toe left ground plane
    elseif any(ie == 1)
      
      L =  ze(1);
      psi =   ze(2);
      dL = ze(3);
      dpsi =  ze(4);

      t = te;
      %---- TRANSFORM STATE VARIABLES FROM POLAR COORDINATES TO CARTESIAN HERE ----%

      z = [%z_cart(1), ...
           %z_cart(2), ...
           %z_cart(3), ...
           %z_cart(4)
          ];

      %---- END POLAR TO CARTESIAN COORDINATE TRANSFORM ----%
      q = 2; % To flight
      
    else
      error(['Warning: (trans) unknown event during stance']);
      t = nan*te;
      z = nan*ze;
      q = nan;
    end
      
        
  % Flight
  % cartesian coordinates of COM
  case 2
    
    % Body penetrated ground plane
    if any(ie == 2)
      t = nan*te;
      z = nan*ze;
      q = nan;
      return;

    % Toe penetrated ground plane
    elseif any(ie == 1)
      
      x =  ze(1);
      y =  ze(2);
      dx = ze(3);
      dy = ze(4);


      foot = x + L0*cos(beta);
      L = sqrt((x-foot)^2+y^2);
      psi = atan2(x-foot,y);

      t = te;
      
      %---- TRANSFORM STATE VARIABLES FROM CARTESIAN TO POLAR COORDINATES HERE ----%
     
      z = [%z_pol(1), ...
           %z_pol(2), ...
           %z_pol(3), ...
           %z_pol(4)

      %---- END CARTESIAN TO POLAR COORDINATE TRANSFORM ----%

      q = 1; % To stance
      
    else
      error(['Warning: (trans) unknown event during flight']);
      t = nan*te;
      z = nan*ze;
      q = nan;
    end
    
  otherwise
    error(['Warning: (trans) unknown discrete mode']);
    t = nan*te;
    z = nan*ze;
    q = nan;
end % switch

% Update parameter vector
p = [m,L0,k,g,beta,foot,debug];

end % transition
