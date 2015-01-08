%ATRIASPOSTPROCESS

classdef AtriasPostProcess < Atrias
  % PROTECTED PROPERTIES ==================================================
  properties (SetAccess = protected)
    time@double vector % Time
  end % properties

  % PUBLIC PROPERTIES =====================================================
  properties
    plotLeftLeg@logical scalar = true;  % Choose to plot the left leg
    plotRightLeg@logical scalar = true; % Choose to plot the right leg
  end % properties

  % PUBLIC METHODS ========================================================
  methods
    function obj = AtriasPostProcess(varargin)
    %ATRIASPOSTPROCESS ATRIAS data processing toolkit constructor.
    %
    % Required Input Arguments:
    %  states - Robot log state
    %  time   - Robot log time
    %
    % Optional Input Arguments:
    %  indices - Vector of data indices to use

      switch nargin
      case 2 % Use all of the input data
        states = varargin{1};
        time   = varargin{2};
      case 3 % Crop the data
        indices = varargin{3};
        states  = varargin{1}(indices,:);
        time    = varargin{2}(indices);
      otherwise
        error('Invalid number of input arguments.')
      end

      % Call superclass constructor
      obj = obj@Atrias(states);

      % % Set object properties
      obj.time = time;
    end % AtriasPostProcess

    function plotLegAngularVelocity(obj)
      % Create figure window and set default parameters
      figure;

      % Set plot properties
      hold on; grid on; box on;

      % Plot
      if obj.plotLeftLeg
        plot(obj.time, obj.left.legVelocityA, 'DisplayName', 'Left Leg Angular Velocity A');
        plot(obj.time, obj.left.legVelocityB, 'DisplayName', 'Left Leg Angular Velocity B');
      end
      if obj.plotRightLeg
        plot(obj.time, obj.right.legVelocityA, 'DisplayName', 'Right Leg Angular Velocity A');
        plot(obj.time, obj.right.legVelocityB, 'DisplayName', 'Right Leg Angular Velocity B');
      end

      % Set plot properties
      title(' Leg Angular Velocities');
      xlabel('Time [s]');
      ylabel('Angular Velocity [rad/s]');
      legend('show')
    end % plotLegAngularVelocity

    function plotLegAngles(obj)
      % Create figure window and set default parameters
      figure;

      % Set plot properties
      hold on; grid on; box on;

      % Plot
      if obj.plotLeftLeg
        plot(obj.time, obj.left.legAngleA, 'DisplayName', 'Left Leg Angle A');
        plot(obj.time, obj.left.legAngleB, 'DisplayName', 'Left Leg Angle B');
      end
      if obj.plotRightLeg
        plot(obj.time, obj.right.legAngleA, 'DisplayName', 'Right Leg Angle A');
        plot(obj.time, obj.right.legAngleB, 'DisplayName', 'Right Leg Angle B');
      end

      % Set plot properties
      title('Leg Angles');
      xlabel('Time [s]');
      ylabel('Angle [rad]');
      legend('show')
    end % plotLegAngles

    function plotLegMotorTorques(obj)
      % Create figure window and set default parameters
      figure;

      % Set plot properties
      hold on; grid on; box on;

      % Plot
      if obj.plotLeftLeg
        plot(obj.time, obj.left.motorTorqueA, 'DisplayName', 'Left Leg Motor Torque A');
        plot(obj.time, obj.left.motorTorqueB, 'DisplayName', 'Left Leg Motor Torque B');
      end
      if obj.plotRightLeg
        plot(obj.time, obj.right.motorTorqueA, 'DisplayName', 'Right Leg Motor Torque A');
        plot(obj.time, obj.right.motorTorqueB, 'DisplayName', 'Right Leg Motor Torque B');
      end

      % Set plot properties
      title('Leg Motor Torques');
      xlabel('Time [s]');
      ylabel('Torque [N*m]');
      legend('show')
    end % plotLegMotorTorques

    function plotHipMotorTorques(obj)
      % Create figure window and set default parameters
      figure;

      % Set plot properties
      hold on; grid on; box on;

      % Plot
      if obj.plotLeftLeg
        plot(obj.time, obj.left.hipTorque, 'DisplayName', 'Left Hip Motor Torque');
      end
      if obj.plotRightLeg
        plot(obj.time, obj.right.hipTorque, 'DisplayName', 'Right Hip Motor Torque');
      end

      % Set plot properties
      title('Hip Motor Torques');
      xlabel('Time [s]');
      ylabel('Torque [N*m]');
      legend('show')
    end % plotHipMotorTorques

    function plotMotorVelocities(obj)
      % Create figure window and set default parameters
      figure;

      % Set plot properties
      hold on; grid on; box on;

      % Plot
      if obj.plotLeftLeg
        plot(obj.time, obj.left.motorVelocityA, 'DisplayName', 'Left Leg Motor Velocity A');
        plot(obj.time, obj.left.motorVelocityB, 'DisplayName', 'Left Leg Motor Velocity B');
      end
      if obj.plotRightLeg
        plot(obj.time, obj.right.motorVelocityA, 'DisplayName', 'Right Leg Motor Velocity A');
        plot(obj.time, obj.right.motorVelocityB, 'DisplayName', 'Right Leg Motor Velocity B');
      end

      % Set plot properties
      title('Motor Velocities');
      xlabel('Time [s]');
      ylabel('Velocity [rad/s]');
      legend('show')
    end % plotMotorVelocities

    function plotToeSensors(obj)
      % Create figure window and set default parameters
      figure;

      % Set plot properties
      hold on; grid on; box on;

      % Plot
      if obj.plotLeftLeg
        plot(obj.time, obj.left.toe, 'DisplayName', 'Left Leg Toe Sensor');
      end
      if obj.plotRightLeg
        plot(obj.time, obj.right.toe, 'DisplayName', 'Right Leg Toe Sensor');
      end

      % Set plot properties
      title('Toe Sensors');
      xlabel('Time [s]');
      ylabel('Value');
      legend('show')
    end % plotToeSensors

    function plotLegSpringTorques(obj)
      % Create figure window and set default parameters
      figure;

      % Set plot properties
      hold on; grid on; box on;

      % Plot
      if obj.plotLeftLeg
        plot(obj.time, obj.left.springTorqueA, 'DisplayName', 'Left Leg Spring Torque A');
        plot(obj.time, obj.left.springTorqueB, 'DisplayName', 'Left Leg Spring Torque B');
      end
      if obj.plotRightLeg
        plot(obj.time, obj.right.springTorqueA, 'DisplayName', 'Right Leg Spring Torque A');
        plot(obj.time, obj.right.springTorqueB, 'DisplayName', 'Right Leg Spring Torque B');
      end

      % Set plot properties
      title('Leg Spring Torques');
      xlabel('Time [s]');
      ylabel('Torque [N*m]');
      legend('show')
    end % plotLegSpringTorques

    function plotLegMotorCurve(obj)
      % Properties
      kg = obj.left.legGearRatio;
      dqMax = obj.left.legMotorVelocityLimit;
      tauMax = kg*obj.left.legMotorTorqueLimit;

      % Create figure window and set default parameters
      figure;

      % Plot left A motor curve
      subplot(2, 2, 1); hold on; grid on; box on;
      plot(kg*obj.left.motorVelocityA, obj.left.motorTorqueA);
      plot([0, dqMax, 0, -dqMax, 0], [tauMax, 0, -tauMax, 0, tauMax], '--k');
      xlim([-dqMax dqMax]); ylim([-tauMax tauMax]);
      title('Left Motor A');

      % Plot left B motor curve
      subplot(2, 2, 2); hold on; grid on; box on;
      plot(kg*obj.left.motorVelocityB, obj.left.motorTorqueB);
      plot([0, dqMax, 0, -dqMax, 0], [tauMax, 0, -tauMax, 0, tauMax], '--k');
      xlim([-dqMax dqMax]); ylim([-tauMax tauMax]);
      title('Left Motor B');

      % Plot right A motor curve
      subplot(2, 2, 3); hold on; grid on; box on;
      plot(kg*obj.right.motorVelocityA, obj.right.motorTorqueA);
      plot([0, dqMax, 0, -dqMax, 0], [tauMax, 0, -tauMax, 0, tauMax], '--k');
      xlim([-dqMax dqMax]); ylim([-tauMax tauMax]);
      title('Right Motor A');

      % Plot right B motor curve
      subplot(2, 2, 4); hold on; grid on; box on;
      plot(kg*obj.right.motorVelocityB, obj.right.motorTorqueB);
      plot([0, dqMax, 0, -dqMax, 0], [tauMax, 0, -tauMax, 0, tauMax], '--k');
      xlim([-dqMax dqMax]); ylim([-tauMax tauMax]);
      title('Right Motor B');
    end % plotLegMotorCurve

    function plotHipMotorCurve(obj)
      % Properties
      kg = obj.left.hipGearRatio;
      dqMax = obj.left.hipMotorVelocityLimit;
      tauMax = kg*obj.left.hipMotorTorqueLimit;

      % Create figure window and set default parameters
      figure;

      % Plot left hip motor curve
      subplot(1, 2, 1); hold on; grid on; box on;
      plot(kg*obj.left.hipVelocity, obj.left.hipTorque);
      plot([0, dqMax, 0, -dqMax, 0], [tauMax, 0, -tauMax, 0, tauMax], '--k');
      xlim([-dqMax dqMax]); ylim([-tauMax tauMax]);
      title('Left Hip Motor');

      % Plot left hip motor curve
      subplot(1, 2, 2); hold on; grid on; box on;
      plot(kg*obj.right.hipVelocity, obj.right.hipTorque);
      plot([0, dqMax, 0, -dqMax, 0], [tauMax, 0, -tauMax, 0, tauMax], '--k');
      xlim([-dqMax dqMax]); ylim([-tauMax tauMax]);
      title('Right Hip Motor');
    end % plotHipMotorCurve

    function plotCartesianForces(obj)
      % Create figure window and set default parameters
      figure;

      % Set plot properties
      hold on; grid on; box on;

      % Plot
      if obj.plotLeftLeg
        plot(obj.time, obj.left.forceX, 'DisplayName', 'Left Leg X Force');
        plot(obj.time, obj.left.forceZ, 'DisplayName', 'Left Leg Z Force');
      end
      if obj.plotRightLeg
        plot(obj.time, obj.right.forceX, 'DisplayName', 'Right Leg X Force');
        plot(obj.time, obj.right.forceZ, 'DisplayName', 'Right Leg Z Force');
      end

      % Set plot properties
      title('Cartesian Leg Forces');
      xlabel('Time [s]');
      ylabel('Force [N]');
      legend('show')
    end % plotCartesianForces

    function plotLegAngleTorque(obj)
      % Create figure window and set default parameters
      figure;

      % Set plot properties
      hold on; grid on; box on;

      % Plot
      if obj.plotLeftLeg
        plot(obj.time, obj.left.legAngleTorque, 'DisplayName', 'Left Leg');
      end
      if obj.plotRightLeg
        plot(obj.time, obj.right.legAngleTorque, 'DisplayName', 'Right Leg');
      end

      % Set plot properties
      title('Leg Angle Torque');
      xlabel('Time [s]');
      ylabel('Torque [N*m]');
      legend('show')
    end % plotLegAngleTorque

    function plotAxialLegForce(obj)
      % Create figure window and set default parameters
      figure;

      % Set plot properties
      hold on; grid on; box on;

      % Plot
      if obj.plotLeftLeg
        plot(obj.time, obj.left.axialLegForce, 'DisplayName', 'Left Leg');
      end
      if obj.plotRightLeg
        plot(obj.time, obj.right.axialLegForce, 'DisplayName', 'Right Leg');
      end

      % Set plot properties
      title('Axial Leg Force');
      xlabel('Time [s]');
      ylabel('Force [N]');
      legend('show')
    end % plotAxialLegForce

    function plotForceLengthCurve(obj)
      % Create figure window and set default parameters
      figure;

      % Set plot properties
      hold on; grid on; box on;

      % Plot
      if obj.plotLeftLeg
        plot(obj.left.legLength, obj.left.axialLegForce, 'DisplayName', 'Left Leg');
      end
      if obj.plotRightLeg
        plot(obj.right.legLength, obj.right.axialLegForce, 'DisplayName', 'Right Leg');
      end

      % Set plot properties
      title('Axial Leg Force vs Leg Length');
      xlabel('Length [m]');
      ylabel('Force [N]');
      legend('show')
    end % plotForceLengthCurve

    function plotToeHeight(obj)
      % Create figure window and set default parameters
      figure;

      % Set plot properties
      hold on; grid on; box on;

      % Left toe z position (Boom yaw, boom roll, boom pitch, left hip angle, left leg A, left leg B)
      lToeZ = @(q1,q2,q3,q7,q8,q9) sin(q2).*2.000648572463481+cos(q2).*sin(q1).*1.225043935243062e-16+cos(q1).*sin(q3).*2.12585554664817e-17-cos(pi.*(1.0./2.0)+q7).*(sin(q2).*1.267183643900547e-1+cos(q2).*sin(q1).*7.759261967173419e-18-cos(q1).*sin(q3).*6.073872990423343e-17+cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*9.919387360751214e-1).*(1.83e2./1.0e3)+sin(pi.*(1.0./2.0)+q8).*(cos(pi.*(1.0./2.0)+q7).*(sin(q2).*9.919387360751214e-1+cos(q2).*sin(q1).*6.073872990423343e-17+cos(q1).*sin(q3).*7.759261967173419e-18-cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*1.267183643900547e-1)+sin(pi.*(1.0./2.0)+q7).*(sin(q2).*1.267183643900547e-1+cos(q2).*sin(q1).*7.759261967173419e-18-cos(q1).*sin(q3).*6.073872990423343e-17+cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*9.919387360751214e-1)).*(1.0./2.0)+sin(pi.*(1.0./2.0)+q7).*(sin(q2).*9.919387360751214e-1+cos(q2).*sin(q1).*6.073872990423343e-17+cos(q1).*sin(q3).*7.759261967173419e-18-cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*1.267183643900547e-1).*(1.83e2./1.0e3)+cos(pi.*(1.0./2.0)+q8).*(cos(q1).*cos(q3).*6.123233995736766e-17-cos(pi.*(1.0./2.0)+q7).*(sin(q2).*1.267183643900547e-1+cos(q2).*sin(q1).*7.759261967173419e-18-cos(q1).*sin(q3).*6.073872990423343e-17+cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*9.919387360751214e-1).*6.123233995736766e-17+sin(pi.*(1.0./2.0)+q7).*(sin(q2).*9.919387360751214e-1+cos(q2).*sin(q1).*6.073872990423343e-17+cos(q1).*sin(q3).*7.759261967173419e-18-cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*1.267183643900547e-1).*6.123233995736766e-17+sin(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17)).*(1.0./2.0)-cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*3.471785576262925e-1-sin(q2).*sin(q7).*sin(q9).*3.879630983586709e-18-cos(q1).*cos(q3).*sin(q9).*3.061616997868383e-17+cos(q7).*cos(q9).*sin(q2).*6.335918219502733e-2-cos(q2).*sin(q3).*sin(q9).*(1.0./2.0)-cos(q7).*sin(q2).*sin(q9).*3.036936495211671e-17-cos(q9).*sin(q2).*sin(q7).*4.959693680375607e-1+cos(q2).*cos(q3).*cos(q7).*cos(q9).*4.959693680375607e-1+cos(q2).*cos(q7).*cos(q9).*sin(q1).*3.879630983586709e-18-cos(q1).*cos(q7).*cos(q9).*sin(q3).*3.036936495211671e-17+cos(q2).*cos(q3).*cos(q7).*sin(q9).*3.879630983586709e-18+cos(q2).*cos(q3).*cos(q9).*sin(q7).*6.335918219502733e-2-cos(q2).*cos(q7).*sin(q1).*sin(q9).*1.859587279037377e-33-cos(q2).*cos(q9).*sin(q1).*sin(q7).*3.036936495211671e-17-cos(q1).*cos(q7).*sin(q3).*sin(q9).*2.375588832961181e-34-cos(q1).*cos(q9).*sin(q3).*sin(q7).*3.879630983586709e-18-cos(q2).*cos(q3).*sin(q7).*sin(q9).*3.036936495211671e-17-cos(q2).*sin(q1).*sin(q7).*sin(q9).*2.375588832961181e-34+cos(q1).*sin(q3).*sin(q7).*sin(q9).*1.859587279037377e-33+sin(q1).*sin(q2).*sin(q3).*sin(q9).*3.061616997868383e-17-cos(q3).*cos(q7).*cos(q9).*sin(q1).*sin(q2).*3.036936495211671e-17-cos(q3).*cos(q7).*sin(q1).*sin(q2).*sin(q9).*2.375588832961181e-34-cos(q3).*cos(q9).*sin(q1).*sin(q2).*sin(q7).*3.879630983586709e-18+cos(q3).*sin(q1).*sin(q2).*sin(q7).*sin(q9).*1.859587279037377e-33+2.01e2./2.0e2;
      % Right toe z position (Boom yaw, boom roll, boom pitch, right hip angle, right leg A, right leg B)
      rToeZ = @(q1,q2,q3,q4,q5,q6) sin(q2).*2.000648572463481+cos(q2).*sin(q1).*1.225043935243062e-16+cos(q1).*sin(q3).*2.12585554664817e-17-cos(pi.*(-1.0./2.0)+q4).*(sin(q2).*1.267183643900547e-1+cos(q2).*sin(q1).*7.759261967173419e-18-cos(q1).*sin(q3).*6.073872990423343e-17+cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*9.919387360751214e-1).*(1.83e2./1.0e3)+sin(pi.*(-1.0./2.0)+q5).*(cos(pi.*(-1.0./2.0)+q4).*(sin(q2).*9.919387360751214e-1+cos(q2).*sin(q1).*6.073872990423343e-17+cos(q1).*sin(q3).*7.759261967173419e-18-cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*1.267183643900547e-1)+sin(pi.*(-1.0./2.0)+q4).*(sin(q2).*1.267183643900547e-1+cos(q2).*sin(q1).*7.759261967173419e-18-cos(q1).*sin(q3).*6.073872990423343e-17+cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*9.919387360751214e-1)).*(1.0./2.0)+sin(pi.*(-1.0./2.0)+q4).*(sin(q2).*9.919387360751214e-1+cos(q2).*sin(q1).*6.073872990423343e-17+cos(q1).*sin(q3).*7.759261967173419e-18-cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*1.267183643900547e-1).*(1.83e2./1.0e3)-cos(pi.*(-1.0./2.0)+q5).*(cos(q1).*cos(q3).*6.123233995736766e-17+cos(pi.*(-1.0./2.0)+q4).*(sin(q2).*1.267183643900547e-1+cos(q2).*sin(q1).*7.759261967173419e-18-cos(q1).*sin(q3).*6.073872990423343e-17+cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*9.919387360751214e-1).*6.123233995736766e-17-sin(pi.*(-1.0./2.0)+q4).*(sin(q2).*9.919387360751214e-1+cos(q2).*sin(q1).*6.073872990423343e-17+cos(q1).*sin(q3).*7.759261967173419e-18-cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*1.267183643900547e-1).*6.123233995736766e-17+sin(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17)).*(1.0./2.0)-cos(q3).*(cos(q2)-sin(q1).*sin(q2).*6.123233995736766e-17).*3.471785576262925e-1-sin(q2).*sin(q4).*sin(q6).*3.879630983586709e-18-cos(q1).*cos(q3).*sin(q6).*3.061616997868383e-17+cos(q4).*cos(q6).*sin(q2).*6.335918219502733e-2-cos(q2).*sin(q3).*sin(q6).*(1.0./2.0)-cos(q4).*sin(q2).*sin(q6).*3.036936495211671e-17-cos(q6).*sin(q2).*sin(q4).*4.959693680375607e-1+cos(q2).*cos(q3).*cos(q4).*cos(q6).*4.959693680375607e-1+cos(q2).*cos(q4).*cos(q6).*sin(q1).*3.879630983586709e-18-cos(q1).*cos(q4).*cos(q6).*sin(q3).*3.036936495211671e-17+cos(q2).*cos(q3).*cos(q4).*sin(q6).*3.879630983586709e-18+cos(q2).*cos(q3).*cos(q6).*sin(q4).*6.335918219502733e-2-cos(q2).*cos(q4).*sin(q1).*sin(q6).*1.859587279037377e-33-cos(q2).*cos(q6).*sin(q1).*sin(q4).*3.036936495211671e-17-cos(q1).*cos(q4).*sin(q3).*sin(q6).*2.375588832961181e-34-cos(q1).*cos(q6).*sin(q3).*sin(q4).*3.879630983586709e-18-cos(q2).*cos(q3).*sin(q4).*sin(q6).*3.036936495211671e-17-cos(q2).*sin(q1).*sin(q4).*sin(q6).*2.375588832961181e-34+cos(q1).*sin(q3).*sin(q4).*sin(q6).*1.859587279037377e-33+sin(q1).*sin(q2).*sin(q3).*sin(q6).*3.061616997868383e-17-cos(q3).*cos(q4).*cos(q6).*sin(q1).*sin(q2).*3.036936495211671e-17-cos(q3).*cos(q4).*sin(q1).*sin(q2).*sin(q6).*2.375588832961181e-34-cos(q3).*cos(q6).*sin(q1).*sin(q2).*sin(q4).*3.879630983586709e-18+cos(q3).*sin(q1).*sin(q2).*sin(q4).*sin(q6).*1.859587279037377e-33+2.01e2./2.0e2;

      % Plot
      if obj.plotLeftLeg
        plot(obj.time, lToeZ(obj.boomYawAngle,obj.boomRollAngle,obj.boomPitchAngle,obj.left.hipAngle,obj.left.legAngleA,obj.left.legAngleB), 'DisplayName', 'Left Leg');
      end
      if obj.plotRightLeg
        plot(obj.time, rToeZ(obj.boomYawAngle,obj.boomRollAngle,obj.boomPitchAngle,obj.right.hipAngle,obj.right.legAngleA,obj.right.legAngleB), 'DisplayName', 'Right Leg');
      end

      % Set plot properties
      title('Toe Height vs Time')
      xlabel('Time [s]')
      ylabel('Height [m]')
      legend('show')

    end % plotToeHeight
  end % methods
end % classdef
% vim: set shiftwidth=2 tabstop=2 softtabstop=2 expandtab :
