function [eStop, u, userOut] = controller(q, dq, userIn)
%CONTROLLER HZD style ATRIAS walking controller.
%
% Description:
%   A single support walking gait was found using COALESCE minimizing cost
%   of transport. The time varying trajectories are then converted to a HZD
%   style controller by slaving the motor trajectories to the stance leg
%   trajectories making the controller time invariant.
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

    % Declare persistent variables
    persistent stanceLeg;
    if isempty(stanceLeg); stanceLeg = 1; end % if
    persistent xs dxs us s0;
    if isempty(xs)
      % Load gait trajectories found in optimization
      data = load('gait.mat');
      s0 = data.s0;

      % Generate spline objects
      tmp = (data.s0 - data.s0(1))/(data.s0(end) - data.s0(1));
      xs = spline(tmp, data.x0);
      dxs = spline(tmp, data.dx0);
      us = spline(tmp, data.u0);
    end % if

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

    % Hip feed-forward torque for gravity compensation
    u0_hip = 35*[stanceLeg; -stanceLeg];

    % Hip actuator torques from PD controller with feed-forward term
    u(hip_u) = u0_hip + (q0_hip - q(hip_m))*kp_hip + (dq0_hip - dq(hip_m))*kd_hip;

    %% MAIN CONTROLLER ====================================================

    switch state
    case 0 % STAND --------------------------------------------------------
        % Target leg actuator positions (standing with legs split)
        q0_leg = pi + [-0.2; -0.2; 0.2; 0.2] + [-1; 1; -1; 1]*acos(0.9);

        % Target leg actuator velocities
        dq0_leg = zeros(4,1);

        % Leg actuator torques from PD controller
        u(leg_u) = (q0_leg - q(leg_m))*kp_leg + (dq0_leg - dq(leg_m))*kd_leg;

    case 1 % WALK ---------------------------------------------------------
        % Time invariant parameter
        s = (pi/2 + (q(13) + mean(q(leg_l(1:2)))) - s0(1))/(s0(end) - s0(1));

        % Bound time invariant parameter between 0 and 1
        s = clamp(s, 0, 1);

        % Target leg actuator states and inputs
        q0_leg = ppval(xs, s);
        dq0_leg = ppval(dxs, s);
        u0_leg = 0*ppval(us, s);

        % Leg actuator torques from PD controller
        u(leg_u) = u0_leg + (q0_leg - q(leg_m))*kp_leg + (dq0_leg - dq(leg_m))*kd_leg;

        % Switch stance leg once time invariant parameter reaches 1
        if s >= 1; stanceLeg = -stanceLeg; end % if

        % User outputs
        userOut = s;

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
