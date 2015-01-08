%ATRIAS

classdef Atrias < handle
  % PROTECTED PROPERTIES ==================================================
  properties (SetAccess = protected)
    % Leg properties
    left@AtriasLeg scalar
    right@AtriasLeg scalar

    % Boom properties
    boomRollAngle@double vector
    boomRollVelocity@double vector
    boomYawAngle@double vector
    boomYawVelocity@double vector
    boomPitchAngle@double vector
    boomPitchVelocity@double vector

    % Robot state
    q
    dq

    % Controller data (optional)
    controllerData
  end % properties

  % CONSTANT PROPERTIES ===================================================
  properties (Constant = true)
    % World properties
    gravity = 9.81 % Acceleration of gravity [m/s/s]

    % Boom properties
    baseHeight = 1.007 % Boom height to center of rotation [m]
    boomLength = 2.006 % Boom length from center of rotation to torso center-plane [m]
    boomBodyOffset = 1.6968 % Boom-body angle offset [rad]

    % Control properties
    sampleRate = 1000 % Sampling frequency [Hz]
  end % properties

  % PUBLIC METHODS ========================================================
  methods
    function obj = Atrias(varargin)
    %ATRIAS ATRIAS data processing toolkit constructor.
    %
    % Required Input Arguments:
    %   q - State position information
    %   dq - State velocity information
    %   u - Commanded torques
    %   toe - Toe sensor data
    %
    % Notes:
    %   - State information vectors in following order
    %     [rBl rBm rAl rAm lBl lBm lAl lAm rHm lHm bR bY bP]

      switch nargin
      case 1 % For post-processing data
        q = varargin{1}(:,1:13);
        dq = varargin{1}(:,14:26);
        u = varargin{1}(:,27:32);
        toe = varargin{1}(:,33:34);
        % If more data is available, save it
        if size(varargin{1},2) > 34
          obj.controllerData = varargin{1}(:,35:end);
        end
      case 3 % For use in controllers
        q = varargin{1};
        dq = varargin{2};
        toe = varargin{3};
        u = [0; 0; 0; 0; 0; 0];
      otherwise
        error('Invalid number of input arguments.')
      end % switch

      % Set object properties
      obj.q = q;
      obj.dq = dq;
      obj.left  = AtriasLeg(obj, q(:,[5:8 10]), dq(:,[5:8 10]), u(:,1:3), toe(:,2));
      obj.right = AtriasLeg(obj, q(:,[1:4 9]), dq(:,[1:4 9]), u(:,4:6), toe(:,1));
      obj.boomRollAngle = q(:,11);
      obj.boomRollVelocity = dq(:,11);
      obj.boomYawAngle = q(:,12);
      obj.boomYawVelocity = dq(:,12);
      obj.boomPitchAngle = q(:,13);
      obj.boomPitchVelocity = dq(:,13);
    end % Atrias
  end % methods
end % classdef
% vim: set shiftwidth=2 tabstop=2 softtabstop=2 expandtab :
