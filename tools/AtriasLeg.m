%ATRIASLEG

classdef AtriasLeg < handle
  % PROTECTED PROPERTIES ==================================================
  properties (SetAccess = protected)
    % Parent object reference
    parent@Atrias scalar

    % Position states
    legAngleB@double vector
    motorAngleB@double vector
    legAngleA@double vector
    motorAngleA@double vector
    hipAngle@double vector

    % Velocity states
    legVelocityB@double vector
    motorVelocityB@double vector
    legVelocityA@double vector
    motorVelocityA@double vector
    hipVelocity@double vector

    % Control inputs
    motorTorqueB@double vector
    motorTorqueA@double vector
    hipTorque@double vector

    % Toe data
    toe@double vector
  end % properties

  % CONSTANT PROPERTIES ===================================================
  properties (Constant = true)
    % Leg spring properties
    springStiffness = 2950 % Leg spring stiffness [N*m/rad]
    springDamping = 1.49 % Leg spring damping [N*m*s/rad]

    % Leg motor properties
    legMotorConstant = 0.0987 % Leg motor torque constant [N*m/A]
    legGearRatio = 50 % Leg gear ratio []
    legMotorDamping = 19 % Motor damping [N*m*s/rad]
    legMotorVelocityLimit = 393.755 % Maximum rated angular velocity [rad/s]
    legMotorTorqueLimit = 23.281 % Maximum rated torque [N*m]
    legMotorCurrentLimit = 193.166 % Maximum rated current [A]

    % Leg geometric properties
    thighLength = 0.5 % Leg thigh segment length [m]
    shinLength = 0.5 % Leg shin segment length [m]

    % Hip motor properties
    hipMotorConstant = 0.184 % Motor torque constant [N*m/A]
    hipGearRatio = 56.335 % Gear ratio []
    hipMotorVelocityLimit = 252.446 % Maximum rated angular velocity [rad/s]
    hipMotorTorqueLimit = 10.921 % Maximum rated torque [N*m]
    hipMotorCurrentLimit = 59.513 % Maximum rated current [A]
  end % properties

  % PUBLIC METHODS ========================================================
  methods
    function obj = AtriasLeg(atrias, q, dq, u, toe)
    %ATRIASLEG ATRIAS leg data processing toolkit constructor.

      % Set position state properties
      obj.legAngleB = q(:,1);
      obj.motorAngleB = q(:,2);
      obj.legAngleA = q(:,3);
      obj.motorAngleA = q(:,4);
      obj.hipAngle = q(:,5);

      % Set velocity state properties
      obj.legVelocityB = dq(:,1);
      obj.motorVelocityB = dq(:,2);
      obj.legVelocityA = dq(:,3);
      obj.motorVelocityA = dq(:,4);
      obj.hipVelocity = dq(:,5);

      % Set toe sensor property
      obj.toe = toe;

      % Set control input properties
      obj.motorTorqueB = u(:,1);
      obj.motorTorqueA = u(:,2);
      obj.hipTorque = u(:,3);

      % Set parent object
      obj.parent = atrias;
    end % AtriasLeg

    function deltaA = springDeflectionA(obj)
      deltaA = obj.motorAngleA - obj.legAngleA;
    end % springDeflectionA

    function deltaRateA = springDeflectionRateA(obj)
      deltaRateA = obj.motorVelocityA - obj.legVelocityA;
    end % springDeflectionRateA

    function deltaB = springDeflectionB(obj)
      deltaB = obj.motorAngleB - obj.legAngleB;
    end % springDeflectionB

    function deltaRateB = springDeflectionRateB(obj)
      deltaRateB = obj.motorVelocityB - obj.legVelocityB;
    end % springDeflectionRateB

    function l = legLength(obj)
      l = cos((obj.legAngleA - obj.legAngleB)/2);
    end % legLength

    function dl = legLengthRate(obj)
      dl = -(sin((obj.legAngleA - obj.legAngleB)/2).*(obj.legVelocityA - obj.legVelocityB))/2;
    end % legLengthRate

    function q = legAngle(obj)
      q = (obj.legAngleA + obj.legAngleB)/2;
    end % legAngle

    function dq = legAngleRate(obj)
      dq = (obj.legVelocityA + obj.legVelocityB)/2;
    end % legAngleRate

    function l = motorLength(obj)
      l = cos((obj.motorAngleA - obj.motorAngleB)/2);
    end % motorLength

    function dl = motorLengthRate(obj)
      dl = -(sin((obj.motorAngleA - obj.motorAngleB)/2).*(obj.motorVelocityA - obj.motorVelocityB))/2;
    end % motorLengthRate

    function q = motorAngle(obj)
      q = (obj.motorAngleA + obj.motorAngleB)/2;
    end % motorAngle

    function dq = motorAngleRate(obj)
      dq = (obj.motorVelocityA + obj.motorVelocityB)/2;
    end % motorAngleRate

    function tauA = springTorqueA(obj)
      tauA = obj.springStiffness*obj.springDeflectionA;
    end % springTorqueA

    function tauB = springTorqueB(obj)
      tauB = obj.springStiffness*obj.springDeflectionB;
    end % springTorqueB

    function fx = forceX(obj)
      l1 = obj.thighLength;
      l2 = obj.shinLength;
      tausA = obj.springTorqueA;
      tausB = obj.springTorqueB;
      qlA = obj.legAngleA;
      qlB = obj.legAngleB;
      qb = obj.parent.boomPitchAngle;

      fx = -(l2*tausB.*sin(qb + qlA) - l1*tausA.*sin(qb + qlB))./(l1*l2*sin(qlA - qlB));
    end % forceX

    function fz = forceZ(obj)
      l1 = obj.thighLength;
      l2 = obj.shinLength;
      tausA = obj.springTorqueA;
      tausB = obj.springTorqueB;
      qlA = obj.legAngleA;
      qlB = obj.legAngleB;
      qb = obj.parent.boomPitchAngle;

      fz = -(l2*tausB.*cos(qb + qlA) - l1*tausA.*cos(qb + qlB))./(l1*l2*sin(qlA - qlB));
    end % forceZ

    function fq = legAngleTorque(obj)
      fq = obj.springTorqueA + obj.springTorqueB;
    end % legAngleTorque

    function fa = axialLegForce(obj)
      ks = obj.springStiffness;
      lm = obj.motorLength;
      ll = obj.legLength;

      fa = 2*ks*(acos(lm)-acos(ll))./(1-ll.^2).^0.5;
    end % axialLegForce
  end % methods
end % classdef
% vim: set shiftwidth=2 tabstop=2 softtabstop=2 expandtab :
