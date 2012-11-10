function [Ts VALS] = simulation(k, m, l0, g, ...
                                l_dot_0, phi_dot_0, l_0, phi_0, ...
                                x_dot_0, y_dot_0,   x_0, y_0, ...
                                num_bounces)

    Ts = [0];
    VALS = [0 0 0 0 0 0 0 0];
    % VALS(:,1) is l_dot
    % VALS(:,2) is phi_dot
    % VALS(:,3) is l
    % VALS(:,4) is phi
    % VALS(:,5) is x_dot
    % VALS(:,6) is y_dot
    % VALS(:,7) is x
    % VALS(:,8) is y

    func_stand = stand_sim_gen (k, m, l0);
    func_flight = flight_sim_gen (g);

    function [val, isterm, dir] = events_stand(t, Vals)
        % Standing ends when spring force is 0 (i.e. when spring returns to original length)
        % Ends sim if the spring falls over (i.e. phi > pi/2 or phi < -pi/2)
        val     = [ Vals(3) - l0; ...       % is spring force 0?
                    abs(Vals(4)) - pi/2];   % has spring fallen over?
        isterm  = [ Vals(2) > 0; ...
                    1];
        dir     = [ 1; ...
                    1];
    end
    function [val, isterm, dir] = events_flight(t, Vals)
        % Standing ends when base of spring makes contact with the ground again (i.e. when y=0)
        val     = Vals(8);
        isterm  = 1;
        dir     = -1;
    end

    for i=1:num_bounces

        %% STANDING PHASE
        % In this part of simulation, x_dot,y_dot,x,y are unchanging
        [Ts1 VALS1] = ode45(func_stand, [Ts(end) 100], [l_dot_0; phi_dot_0; l_0; phi_0; x_dot_0; y_dot_0; x_0; y_0], ...
                            odeset('Events', @events_stand));

        % End values from stand phase
        l_dot_f     = VALS1(end,1);
        phi_dot_f   = VALS1(end,2);
        l_f         = VALS1(end,3);
        phi_f       = VALS1(end,4);
        x_f         = VALS1(end,7); % Should remain unchanged
        y_f         = VALS1(end,8); % Should remain unchanged

        Ts      = [Ts; Ts1];
        VALS    = [VALS; VALS1];

        disp (i);
        [phi_f, pi/2 - 0.1]
        abs(phi_f) >= pi/2 - 0.1
        if (abs(phi_f) >= pi/2 - 0.1) % Some tolerance included
            disp ('breaking');
            break;
        end


        % Starting values for flight phase
        x_dot_0 = l_dot_f*sin(phi_f);
        y_dot_0 = l_dot_f*cos(phi_f);
        % Assumption made in paper: coords of spring top are same as at end of flight phase
        % but coords of spring bottom reset themselves so that phi_0, l_0 are the same as
        % at the beginning of the stand phase
        x_0         = l_f*sin(phi_f) - l0*sin(phi_0) + x_f;
        y_0         = l_f*cos(phi_f) - l0*cos(phi_0) + y_f;
        l_dot_0     = 0;
        phi_dot_0   = 0;
        l_0         = l0;
        %phi_0      = phi_0;

        %% FLIGHT PHASE
        % In this part of simulation, l_dot,phi_dot,l,phi are unchanging
        [Ts2 VALS2] = ode45(func_flight, [Ts1(end) 100], [l_dot_0; phi_dot_0; l0; phi_0; x_dot_0; y_dot_0; x_0; y_0], ...
                            odeset('Events', @events_flight));

        % End values from flight phase
        l_dot_f     = VALS2(end,1); % Should be 0 
        phi_dot_f   = VALS2(end,2); % Should be 0
        l_f         = VALS2(end,3); % Should remain unchanged
        phi_f       = VALS2(end,4); % Should remain unchanged 
        x_dot_f     = VALS2(end,5);
        y_dot_f     = VALS2(end,6); 
        x_f         = VALS2(end,7);
        y_f         = VALS2(end,8);

        disp(i);
        % Starting values for stand phase
        if (x_dot_f >= 0) % It's still traveling forward, will continue bouncing
            l_dot_0     = x_dot_f*sin(phi_f) + y_dot_f*cos(phi_f);
            phi_dot_0   = (x_dot_f*cos(phi_f) + y_dot_f*sin(phi_f)) / l_f;
            disp ('if');
        else % It's moving backwards, going to fall over
            l_dot_0 = 0; % Spring won't compress
            disp ('else');
            phi_dot_0   = (x_dot_f*cos(-phi_f) + y_dot_f*sin(-phi_f));
        end
        l_0         = l0;
        phi_0       = phi_0;
        x_dot_0     = 0;
        y_dot_0     = 0;
        x_0         = x_f;
        y_0         = y_f;

        %% OUTPUT VALUES
        Ts      = [Ts; Ts2];
        VALS    = [VALS; VALS2];

    end
end