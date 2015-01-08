function [eStop, u, userOut] = controller(q, dq, userIn)
%CONTROLLER ATRIAS example controller.

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
    u0_hip = 35*[1; -1];
    
    % Hip actuator torques from PD controller with feedforward term
    u([3 6]) = u0_hip + (q0_hip - q([9 10]))*kp_hip + (dq0_hip - dq([9 10]))*kd_hip;

    %% MAIN CONTROLLER ====================================================

    switch state
    case 0 % STAND --------------------------------------------------------
        % Target leg actuator positions (standing with legs split)
        q0_leg = pi + [-0.2; -0.2; 0.2; 0.2] + [-1; 1; -1; 1]*acos(0.9);

        % Target leg actuator velocities
        dq0_leg = zeros(4,1);

        % Leg actuator torques from PD controller
        u([2 1 5 4]) = (q0_leg - q([4 2 8 6]))*kp_leg + (dq0_leg - dq([4 2 8 6]))*kd_leg;

    otherwise % RELAX -----------------------------------------------------
        % Leg actuator torques computed to behave like virtual dampers
        u([2 1 5 4]) = (0 - dq([4 2 8 6]))*kd_leg;
        u([3 6]) = (0 - dq([9 10]))*kd_hip;
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
