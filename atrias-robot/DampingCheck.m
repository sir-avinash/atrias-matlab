classdef DampingCheck < handle
    % Check to see if an encoder is decelerating

    properties (Access = private)
        v0   = 0; % Initial velocity [rad/s]
        t    = -0.005; % Time decelerating [s]
        vMin = 0.3; % Minimum velocity [rad/s]
    end % properties

    properties (Access = private, Constant = true)
        gearRatio = 50;   % [unitless]
        torqueConstant = 0.119; % [N*m/A]
        maxCurrent = 100; % [A]
        reflectedMotorInertia = 0.0019*DampingCheck.gearRatio^2; % [kg*m^2]
        expectedDecelConst = DampingCheck.gearRatio*DampingCheck.maxCurrent*DampingCheck.torqueConstant/DampingCheck.reflectedMotorInertia; % [rad/s^2]
    end

    methods
        function o = DampingCheck(vMin)
            % Set the minimum velocity (0.3 rad/s is reasonable)
            o.vMin = abs(vMin);
        end % dampingCheck

        function initialVelocity(o,v0)
            % Time starts when v0 is set
            o.v0 = v0;
            o.t  = -0.005;
        end % initialVelocity

        % Check if deceleration has failed
        % Given
        %   v: Current velocity
        % Return
        %   decelFail: true if failing to decelerate
        function decelFail = checkDeceleration(o,v)
            % Increment time
            o.t = o.t + 0.001;

            % Boundary equation
            vBound = abs(o.v0) + o.vMin - DampingCheck.expectedDecelConst*o.t*0.75;

            % Set a minimum bounding velocity
            vBound = max(abs(vBound), abs(o.vMin));

            % If we're not slowing down, return true
            decelFail = (abs(v) > vBound);
        end % checkDeceleration

    end % methods

end % dampingCheck