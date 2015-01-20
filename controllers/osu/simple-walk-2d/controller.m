function [eStop, u, userOut] = controller(q, dq, userIn)
%CONTROLLER Simple ATRIAS walking controller.
%
% Description:
%   This controller unifies a number of fundamental concepts learned
%   through observation of the SLIP model and past failed experiments.
%   The controller is heuristically built up in layers and includes a few
%   key concepts:
%     * Passive stance leg behavior (zero hip torque, only axial forces)
%     * Stance leg push off during second half of stance to inject energy
%     * Slave swing leg trajectory to stance leg (with very loose gains)
%     * Ground speed matching and fixed stride length before impact
%     * Both legs stabilize torso but with a weight based on leg force
%     * Leg relabeling occurs when swing leg force exceed stance leg force
%
% Notes:
%   * Anything under 0.75 uses dynamic standing controller
%   * Anything above 0.75 uses walk controller
%   * If going from stand to walk, (same direction) do one step transistion
%   * If going from stand to walk, (other direction) first ramp speed to
%   zero then do one step transistion
%   * If going from walk to stand, slow down, switch to stand
%   * If switching direction of walk, first slow down then switch to stand,
%   come to stop, then take single step in new direction
%
% Plan:
%   * Verify walk works over obstacles and can reliable control speed
%   * Verify stand can stand in place and stagger around under 0.75
%   * Confirm simple stagger stand is best way to do things, being closer
%   to walk controller would be better
%   * Get fast walk to slow down and transition to stand smoothly.
%   * Get stand to walk to work smoothly
%   * Then think about othercase (velocity change rate limit when there is
%   a sign flip
%
% Copyright 2015 Mikhail S. Jones

    %% INITIALIZE =========================================================

    % Set default emergency stop to false
    eStop = false;

    % Parse user input arguments into convient local variables
    state = userIn(1); % Main controller state
    u_lim = clamp(userIn(2), 0, 600); % Motor torque limit (N*m)
    kp_leg = clamp(userIn(3), 0, 5000); % Leg motor proportional gain (N*m/rad)
    kd_leg = clamp(userIn(4), 0, 500); % Leg motor differential gain (N*m*s/rad)
    kp_hip = clamp(userIn(5), 0, 2000); % Hip motor proportional gain (N*m/rad)
    kd_hip = clamp(userIn(6), 0, 200); % Hip motor differential gain (N*m*s/rad)
    v_cmd = clamp(userIn(7), -1.5, 1.5); % Velocity (m/s)
    l_ret = clamp(userIn(8), 0, 0.25); % Leg retraction (m)

    % Gait parameters
    ks_leg = 2950; % Leg rotational spring constant (N*m/rad)
    l0 = 0.9; % Nominal leg length (m)
    q0_torso = 0; % Target torso pitch (rad)
    s_leg = 0.5; % Scale leg actuator gains for swing phase
    s_torso = 0.5; % Scale leg actuator gains for torso stabilization
    threshold = 50; % Spring torque threshold for scaling and switching (N*m)

    % Simulation time and velocity profile
%     persistent T; if isempty(T); T = 0; else T = T + 0.001; end % if
%     v_cmd = 1.2*sign(sin(T/pi));
    
    % Persistent variable to keep track of current stance leg
    persistent stanceLeg; if isempty(stanceLeg); stanceLeg = 1; end % if

    % Persistent variable to keep track of extra swing leg clearence
    persistent l_clr; if isempty(l_clr); l_clr = 0; end % if
    
    % Persistent variable to keep track of states at last switch
    persistent x_st_e; if isempty(x_st_e); x_st_e = 0; end % if
    persistent x_sw_e; if isempty(x_sw_e); x_sw_e = 0; end % if
    
    % Persistent variable to keep track controller type
    persistent v_tgt; if isempty(v_tgt); v_tgt = v_cmd; end % if
    persistent isStand; if isempty(isStand); isStand = true; end % if

    % Persistent variable to keep track of time since last switch
    persistent t; if isempty(t); t = 0; else t = t + 0.001; end % if
    
    % Setup stance and swing indexes
    if stanceLeg == 1 % Left
        leg_m = [8 6 4 2]; leg_l = [7 5 3 1]; leg_u = [5 4 2 1];
        hip_m = [10 9]; hip_u = [6 3];
    else % Right
        leg_m = [4 2 8 6]; leg_l = [3 1 7 5]; leg_u = [2 1 5 4];
        hip_m = [9 10]; hip_u = [3 6];
    end % if

    % Initialize input vector to zeros
    u = zeros(6,1);

    % Initialize user output vector
    userOut = zeros(1,1);

    %% HIP CONTROLLER =====================================================

    % Hip target position to counteract boom rotation
    q0_hip = (0.1*[-stanceLeg; stanceLeg] - q(11) + 0.1271);

    % Hip target velocity
    dq0_hip = zeros(2,1);

    % Hip feed-forward torque for gravity compensation
    u0_hip = 35*[stanceLeg; -stanceLeg];

    % Hip actuator torques from PD controller with feed-forward term
    u(hip_u) = u0_hip + (q0_hip - q(hip_m))*kp_hip + (dq0_hip - dq(hip_m))*kd_hip;

    %% MAIN CONTROLLER ====================================================

    switch state
    case 0 % STAND --------------------------------------------------------
        % Target leg actuator positions
        q0_leg = pi + [-1; 1; -1; 1]*acos(l0);

        % Target leg actuator velocities
        dq0_leg = zeros(4,1);

        % Leg actuator torques from PD controller
        u(leg_u) = (q0_leg - q(leg_m))*kp_leg + (dq0_leg - dq(leg_m))*kd_leg;

    case 1 % WALK ---------------------------------------------------------
        % Cartesian position of toes relative to hip in world frame
        x_st = sum(sin(q(13) + q(leg_l(1:2)))/2);
        y_st = sum(cos(q(13) + q(leg_l(1:2)))/2);
        x_sw = sum(sin(q(13) + q(leg_l(3:4)))/2);
        y_sw = sum(cos(q(13) + q(leg_l(3:4)))/2);

        % Forward velocity computed from stance leg velocities
        dx = cos((q(leg_l(2)) - q(leg_l(1)))/2)*mean(dq(13) + dq(leg_l(1:2)));

        % Stance leg push-off is proportional to desired speed and error
        l_ext = clamp(...
            1/30*abs(v_tgt) + ...
            1/20*sign(v_tgt)*(v_tgt - dx), ...
            0, 0.05)*(sign(v_tgt) == sign(dx));
            
        % Tune parameters for desired speed
        if isStand
            % Step length is proportional to current velocity and error
            l_step = clamp(...
                0.2*dx - ...
                clamp(0.4*(v_tgt - dx), -0.1, 0.1) - ...
                x_st, -0.5, 0.5);
            
            % Set leg swing trigger point
            trig = 0.8;
            
            % Define a time variant parameter
            s = t/0.5;
        else
            % Step length is constant and in direction of target velocity
            l_step = sign(v_tgt)*0.5;
            
            % Set leg swing trigger point
            trig = 0.6;
            
            % Define time invariant parameter based on hip position
            s = (x_st_e - x_st)/(x_st_e + l_step/2);
        end % if
        
        % Swing leg retraction policy (immediately retract then extend once
        % past defined trigger point)
        l_sw = l0 - (l_ret + l_clr)*(s < trig);

        % Swing leg swing policy (use cubic spline to interpolate target
        % ground projection point of the toe and find the corresponding leg
        % angle given a desired length)
        d_sw = cubic(0, 0.7, x_sw_e - x_st_e, l_step, 0, 0, s, 1);
        r_sw = pi/2 + acos((x_st + d_sw)/l_sw) - q(13);

        % Target swing leg actuator positions
        q_sw = r_sw + [-1; 1]*acos(l_sw);

        % Target swing leg actuator velocities
        dq_sw = zeros(2,1);

        % Swing leg actuator torques from PD controller
        u(leg_u(3:4)) = (q_sw - q(leg_m(3:4)))*kp_leg*s_leg + (dq_sw - dq(leg_m(3:4)))*kd_leg*s_leg;

        % Stance leg push off policy (extend leg after mid stance linearly)
        l_st = l0 + l_ext*clamp(2*s - 1, 0, 1);

        % Stance leg step down policy (retract leg if drop is detected)
        l_st = l_st + clamp(y_sw - y_st, -l_ret, 0);

        % Target stance leg actuator positions
        q_st = mean(q(leg_l(1:2))) + [-1; 1]*acos(l_st);

        % Target stance leg actuator velocities
        dq_st = [1; 1]*mean(dq(leg_l(1:2)));

        % Stance leg actuator torques from PD controller
        u(leg_u(1:2)) = (q_st - q(leg_m(1:2)))*kp_leg + (dq_st - dq(leg_m(1:2)))*kd_leg;

        % Scaling terms for torso stabilization and state switching is
        % based on absolute mean torque in springs scaled and clamped
        % between 0 and 1
        s_st = clamp(ks_leg/threshold*mean(abs(q(leg_m(1:2)) - q(leg_l(1:2)))), 0, 1);
        s_sw = clamp(ks_leg/threshold*mean(abs(q(leg_m(3:4)) - q(leg_l(3:4)))), 0, 1);

        % Add additional torque commands to leg actuators to stabilize
        % scaled based on the "force" felt in the leg
        u(leg_u(1:2)) = u(leg_u(1:2)) + ...
            s_st*s_torso*((q(13) - q0_torso)*kp_leg + dq(13)*kd_leg);
        u(leg_u(3:4)) = u(leg_u(3:4)) + ...
            s_sw*s_torso*((q(13) - q0_torso)*kp_leg + dq(13)*kd_leg);

        % Detect when swing leg force exceeds stance leg force
        if (s_sw > s_st && t > 0.2) || (isStand && s >= 1)
            % Switch stance legs
            stanceLeg = -stanceLeg;

            % Estimate extra required swing leg clearence in case of step
            l_clr = ~isStand*clamp(abs(y_sw - y_st), 0, 0.15); % TODO: check if isStand is needed

            % Reset time since last switch
            t = 0;

            % Exit conditions
            x_st_e = x_sw;
            x_sw_e = x_st;

            % Check standing speed or direction flip of commanded velocity
            if abs(v_cmd) < 0.75 || sign(v_cmd) ~= sign(dx)
                % Walk slower
                if abs(dx) > 1 && ~isStand
                    v_tgt = sign(dx)*0.75;
                % Stand
                else
                    v_tgt = sign(v_cmd)*0.25;
                    isStand = true;
                end % if
            else
                % Walk normal
                v_tgt = v_cmd;
                isStand = false;
            end % if
        end % if
        
        % Error catch for falling backwards
        if abs(dx) < 0.01; isStand = true; end % if

        % User outputs
        userOut = dx;

    otherwise % RELAX -----------------------------------------------------
        % Leg actuator torques computed to behave like virtual dampers
        u(leg_u) = (0 - dq(leg_m))*kd_leg;
        u(hip_u) = (0 - dq(hip_m))*kd_hip;
    end % switch

    % Limit absolute torque commands
	u = clamp(u, -u_lim, u_lim);
end % controller

%% LOCAL FUNCTIONS ========================================================

function b = clamp(a, lim1, lim2)
%CLAMP Clamp value between two bounds.

	% Find which limit is min and max
	a_min = min(lim1, lim2);
	a_max = max(lim1, lim2);

	% Clamp value between limits
	b = max(min(a, a_max), a_min);
end % clamp

function [y, dy] = cubic(x1, x2, y1, y2, dy1, dy2, x, dx)
%CUBIC Cubic interpolation between values.

    % Limit range since curve fit is only valid within range
    x = clamp(x, x1, x2);

    % Interpolate
    a0 = 2*(y1 - y2) + (dy1 + dy2)*(x2 - x1);
    a1 = y2 - y1 - dy1*(x2 - x1) - a0;
    a2 = dy1*(x2 - x1);
    a3 = y1;
    s = (x - x1)/(x2 - x1);
    y = a0*s^3 + a1*s^2 + a2*s + a3;
    dy = dx*(-3*a0*(x - x1)^2/(x1 - x2)^3 + 2*a1*(x - x1)/(x1 - x2)^2 - a2/(x1 - x2));
end % cubic
