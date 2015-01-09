function [eStop, u, userOut] = controller(q, dq, userIn)
%CONTROLLER Simple ATRIAS walking controller.
%
% Description:
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
    l_step = clamp(userIn(7), 0, 1); % Step length (m)
    l_ret = clamp(userIn(8), 0, 0.25); % Leg retraction (m)
    l_ext = clamp(userIn(9), 0, 0.05); % Leg push off (m)
    
    % Gait parameters
    ks_leg = 2950; % Leg rotational spring constant (N*m/rad)
%     l_step = 0.55; % Step length (m) - Note: Average human step is 0.76m
    l_trig = -l_step/5; % Stance foot position that triggers swing leg extension (m)
%     l_ret = 0.1; % Leg retraction (m)
    l0 = 0.9; % Nominal leg length (m)
%     l_ext = 0.03; % Push off length (m)
    q0_torso = 0; % Target torso pitch (rad)
    s_leg = 0.5; % Scale leg actuator gains for swing phase
    s_torso = 0.5; % Scale leg actuator gains for torso stabilization
    threshold = 50; % Spring torque threshold for scaling and switching (N*m)
            
    % Decalre persistent variables
    persistent stanceLeg;
    if isempty(stanceLeg); stanceLeg = 1; end % if
        
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
    
    % Initialize user ouput vector
    userOut = zeros(1,1);
    
    %% HIP CONTROLLER =====================================================

    % Hip target position to counteract boom rotation
    q0_hip = (-q(11) + 0.1271);
    
    % Hip target velocity
    dq0_hip = zeros(2,1);
    
    % Hip feedforward torque for gravity compensation
    u0_hip = 35*[stanceLeg; -stanceLeg];
    
    % Hip actuator torques from PD controller with feedforward term
    u(hip_u) = u0_hip + (q0_hip - q(hip_m))*kp_hip + (dq0_hip - dq(hip_m))*kd_hip;
    
    %% MAIN CONTROLLER ====================================================

    switch state
    case 0 % STAND --------------------------------------------------------
        % Target leg actuator positions (standing with legs split)
        q0_leg = pi + [-0.2; -0.2; 0.2; 0.2] + [-1; 1; -1; 1]*acos(l0);
        
        % Target leg actuator velocities
        dq0_leg = zeros(4,1);
        
        % Leg actuator torques from PD controller
        u(leg_u) = (q0_leg - q(leg_m))*kp_leg + (dq0_leg - dq(leg_m))*kd_leg;
            
    case 1 % WALK ---------------------------------------------------------            
        % Cartesian position of stance toe relative to hip in world frame
        x_st = sum(sin(q(13) + q(leg_l(1:2)))/2);
        
        % Swing leg retraction policy (immediately retract and extend once
        % past defined trigger point)
        l_sw = l0 - l_ret*(x_st > l_trig/2);
        
        % Swing leg swing policy (use cubic spline to interpolate target
        % ground projection point of the toe and find the corresponding leg
        % angle given a desired length)
        d_sw = cubic(l_step/2, l_trig, -l_step, l_step, 0, 0, x_st, 1);
        r_sw = pi/2 + acos(x_st + d_sw) - q(13);
        
        % Target swing leg actuator positions
        q_sw = r_sw + [-1; 1]*acos(l_sw);
        
        % Target swing leg actuator velocities
        dq_sw = zeros(2,1);
        
        % Swing leg actuator torques from PD controller
        u(leg_u(3:4)) = (q_sw - q(leg_m(3:4)))*kp_leg*s_leg + (dq_sw - dq(leg_m(3:4)))*kd_leg*s_leg;
        
        % Stance leg push off policy (extend leg after mid stance linearly)
        l_st = l0 + l_ext*clamp(-x_st/(l_step/2), 0, 1);
        
        % Target stance leg actuator positions
        q_st = mean(q(leg_l(1:2))) + [-1; 1]*acos(l_st);
        
        % Target stance leg actuator velocities
        dq_st = [1; 1]*mean(dq(leg_l(1:2)));
        
        % Stance leg actuator torques from PD controller
        u(leg_u(1:2)) = (q_st - q(leg_m(1:2)))*kp_leg + (dq_st - dq(leg_m(1:2)))*kd_leg;
        
        % Scaling terms for torso stabilization and state switching is
        % based on absolute mean torque in springs scaled and clamped 
        % between 0 and 1
        s_st = clamp(ks_leg*mean(abs(q(leg_m(1:2)) - q(leg_l(1:2))))/threshold, 0, 1);
        s_sw = clamp(ks_leg*mean(abs(q(leg_m(3:4)) - q(leg_l(3:4))))/threshold, 0, 1);
        
        % Add additional torque commands to leg actuators to stabilize
        % scaled based on the "force" felt in the leg
        u(leg_u(1:2)) = u(leg_u(1:2)) + ...
            s_st*s_torso*((q(13) - q0_torso)*kp_leg + dq(13)*kd_leg);
        u(leg_u(3:4)) = u(leg_u(3:4)) + ...
            s_sw*s_torso*((q(13) - q0_torso)*kp_leg + dq(13)*kd_leg);
        
        % Switch stance legs when swing leg force exceeds stance leg force
        if s_sw > s_st && x_st < 0; stanceLeg = -stanceLeg; end % if
        
        % User outputs
        userOut = x_st;
        
    otherwise % RELAX -----------------------------------------------------
        % Leg actuator torques computed to behave like virtual dampers
        u(leg_u) = (0 - dq(leg_m))*kd_leg;
        u(hip_u) = (0 - dq(hip_m))*kd_hip;
    end % switch

    % Limit absolute torque commands
	u = clamp(u, -u_lim, u_lim);
end % simpleWalk

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
