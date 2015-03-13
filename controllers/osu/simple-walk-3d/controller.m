function [eStop, u, userOut] = controller(q, dq, userIn, controlIn)
%CONTROLLER Simple ATRIAS walking controller.
%
% Description:
%   This controller unifies a number of fundamental concepts learned
%   through observation of the SLIP model and past failed experiments.
%   The controller is heuristically built up in layers and includes a few
%   key concepts:
%     * Walking
%       * Passive stance leg behavior (zero hip torque, only axial forces)
%       * Stance leg push off during second half of stance to inject energy
%       * Slave swing leg trajectory to stance leg (with very loose gains)
%       * Ground speed matching and fixed stride length before impact
%       * Both legs stabilize torso but with a weight based on leg 'force'
%         - Sum absolute spring torque is used to capture all TD conditions
%       * Leg relabeling occurs when swing leg exceeds stance leg force
%     * Standing
%       * Step length is proportional to CoM velocity plus some error term
%         - CoM velocity is updated using a weighted filter based on force
%       * Trajectories are time based unless velocity exceeds some value
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
% Copyright 2015 Mikhail S. Jones

  %% INITIALIZE ===========================================================

  % Set default emergency stop flag to false
  eStop = false;

  % Parse user input arguments into convient local variables
  state = userIn(1); % Main controller state
  u_lim = clamp(userIn(2), 0, 600); % Motor torque limit (N*m)
  kp_leg = clamp(userIn(3), 0, 5000); % Leg motor proportional gain (N*m/rad)
  kd_leg = clamp(userIn(4), 0, 500); % Leg motor differential gain (N*m*s/rad)
  kp_hip = clamp(userIn(5), 0, 2000); % Hip motor proportional gain (N*m/rad)
  kd_hip = clamp(userIn(6), 0, 200); % Hip motor differential gain (N*m*s/rad)
  l_ret = clamp(userIn(7), 0, 0.25); % Leg retraction (m)
  thres_lo = clamp(userIn(8), 0, 100); % Lower spring torque threshold (N*m)
  thres_hi = clamp(userIn(9), 0, 100); % Upper spring torque threshold (N*m)
  filter_gain = clamp(userIn(10), 0, 1); % Filter coefficient
  t_step = clamp(userIn(11), 0, 1); % Step duration (s)
  dx_gain = clamp(userIn(12), 0, 1); % Transverse velocity gain
  dy_gain = clamp(userIn(13), 0, 1); % Transverse velocity gain
  dx_err_gain = clamp(userIn(14), 0, 1); % Transverse velocity error gain
  dy_err_gain = clamp(userIn(15), 0, 1); % Transverse velocity error gain
  d0_hip = clamp(userIn(16), -0.2, 0.2); % Hip offset (m)
  l0_leg = clamp(userIn(17), 0.85, 0.95); % Nominal leg length (m)
  s_torso = clamp(userIn(18), 0, 1); % Scale leg actuator gains for torso stabilization
  s_leg = clamp(userIn(19), 0, 1); % Scale leg actuator gains for swing phase
  d_offset = clamp(userIn(20), -0.05, 0.05); % Torso CoM offset (m)
  yaw_offset = clamp(userIn(21), -0.05, 0.05); % Torso yaw offset (rad)
  d0_offset_gain = clamp(userIn(22), -0.1, 0.1); % Toe offset gain

  % Controller input
  dx_cmd = -1.2*clamp(controlIn(2), -1, 1); % X Velocity (m/s)
  dy_cmd = -0*clamp(controlIn(1), -1, 1); % Y Velocity (m/s)

  % Gait parameters
  ks_leg = 2950; % Leg rotational spring constant (N*m/rad)

  % Controller parameters
  dt = 0.001; % Sample time (sec)
  q0_pitch = 0; % Torso pitch (rad)
  q0_roll = 0; % Torso roll (rad)

  % Persistent variable to keep track of current stance leg
  persistent stanceLeg; if isempty(stanceLeg); stanceLeg = 1; end % if

  % Persistent variable to keep track of extra swing leg clearence
  persistent l_clr; if isempty(l_clr); l_clr = 0; end % if

  % Persistent variable to keep track of states at last switch
  persistent x_st_e; if isempty(x_st_e); x_st_e = 0; end % if
  persistent x_sw_e; if isempty(x_sw_e); x_sw_e = 0; end % if

  % Persistent variable to keep track controller type
  persistent isStand; if isempty(isStand); isStand = true; end % if

  % Persistent variable to keep track of time since last switch
  persistent t; if isempty(t); t = 0; else t = t + 0.001; end % if

  % Persistent variable to keep track of target X velocity
  persistent dx_tgt; if isempty(dx_tgt); dx_tgt = dx_cmd; end % if
  persistent dy_tgt; if isempty(dy_tgt); dy_tgt = dy_cmd; end % if

  persistent d; if isempty(d); d = 0; end % if

  % Persistent variable to keep track of estimated position
  persistent x_est; if isempty(x_est); x_est = 0; end % if
  persistent y_est; if isempty(y_est); y_est = 0; end % if

  % Persistent variable to keep track of estimated/filtered velocity
  persistent dx_est; if isempty(dx_est); dx_est = 0; end % if
  persistent dy_est; if isempty(dy_est); dy_est = 0; end % if

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
  userOut = zeros(6,1);

  % SIMULATION ONLY -------------------------------------------------------
  persistent T; if isempty(T); T = 0; else T = T + 0.001; end % if

  % % Figure eight
  % dx_cmd = 0.5*sin(T*2*pi/15);
  % dy_cmd = 0.5*cos(T*2*pi/7.5);

  % % Forward walk
  % dx_cmd = clamp(0.2*round(T/4), 0, 1.2);
  % dy_cmd = 0;

  % % Side step
  % dx_cmd = 0;
  % dy_cmd = clamp(0.1*round(T/5), 0, 0.5);

  % Force standing controller
  isStand = true;
  dx_tgt = dx_cmd;
  dy_tgt = dy_cmd;
  % -----------------------------------------------------------------------

  %% MAIN CONTROLLER ======================================================

  switch state
  case 0 % STAND ----------------------------------------------------------
    % Reset persistent variables
    l_clr = 0; x_st_e = 0; x_sw_e = 0; dx_tgt = 0;
    t = 0; d = 0; x_est = 0; y_est = 0; dx_est = 0; dy_est = 0;

    % Target leg actuator positions
    q0_leg = pi + [-1; 1; -1; 1]*acos(l0_leg);

    % Target leg actuator velocities
    dq0_leg = zeros(4,1);

    % Leg actuator torques from PD controller
    u(leg_u) = (q0_leg - q(leg_m))*kp_leg + (dq0_leg - dq(leg_m))*kd_leg;

    % Target hip actuator positions
    q0_hip = d0_hip*stanceLeg*[-1; 1];

    % Target hip actuator velocities
    dq0_hip = zeros(2,1);

    % Hip actuator torques from PD controller
    u(hip_u) = (q0_hip - q(hip_m))*kp_hip + (dq0_hip - dq(hip_m))*kd_hip;

  case 1 % WALK -----------------------------------------------------------
    % Cartesian position of toes relative to hip in world frame
    x_st = sum(sin(q(13) + q(leg_l(1:2)))/2);
    y_st = sum(cos(q(13) + q(leg_l(1:2)))/2);
    x_sw = sum(sin(q(13) + q(leg_l(3:4)))/2);
    y_sw = sum(cos(q(13) + q(leg_l(3:4)))/2);

    % Scaling terms for torso stabilization and state switching is
    % based on absolute mean torque in springs scaled and clamped
    % between 0 and 1
    s_st = scaleFactor(ks_leg*mean(abs(q(leg_m(1:2)) - q(leg_l(1:2)))), thres_lo, thres_hi);
    s_sw = scaleFactor(ks_leg*mean(abs(q(leg_m(3:4)) - q(leg_l(3:4)))), thres_lo, thres_hi);

    % Forward kinematic lengths
    l_l = cos((q(leg_l(2)) - q(leg_l(1)))/2);
    l_h = 0.1831*stanceLeg;
    l_t = 0.335*22.2/60;

    % Compute CoM velocities
    dx = -mean(cos(q(13) + q(leg_l(1:2))).*(dq(13) + dq(leg_l(1:2)))) + l_t*cos(q(13))*dq(13);
    dy = (l_l*cos(q(hip_m(1)) + q(11)) - l_h*sin(q(hip_m(1)) + q(11)))*(dq(hip_m(1)) + dq(11)) + l_t*cos(q(11))*dq(11);

    % Update CoM states when we are 'confident' stance leg is on the ground
    if s_st >= 1
      % Filter velocity and ignore bad values
      if abs(dx) < 2
        % Use filter, but only update when velocity is reasonable
        dx_est = dx_est + filter_gain*(dx - dx_est);
      end % if

      % Filter velocity and ignore bad values
      if abs(dy) < 2
        % Use filter, but only update when velocity is reasonable
        dy_est = dy_est + filter_gain*(dy - dy_est);
      end % if
    end % if

    % Compute CoM positions
    x_est = x_est + dx_est*dt;
    y_est = y_est + dy_est*dt;

    % Stance leg push-off is proportional to desired speed and error
    l_ext = clamp(...
      1/30*abs(dx_tgt) + ...
      1/20*sign(dx_tgt)*(dx_tgt - dx_est), ...
      0, 0.02)*(sign(dx_tgt) == sign(dx_est));

    % Bring toes in as speed increases
    d0_hip = d0_hip - d0_offset_gain*dx_tgt;

    % Tune parameters for desired speed
    if isStand
      % Step length is proportional to current velocity
      l_step = clamp(...
        dx_est*dx_gain + ...
        (dx_est - dx_tgt)*dx_err_gain + ...
        l_t*sin(q(13)) + ...
        stanceLeg*yaw_offset, ...
        -0.3, 0.3);

      % Set leg swing trigger point
      trig = 0.8;

      % Define a time variant parameter
      s = clamp(t/t_step, 0, 1);
    else
      % Step length is constant and in direction of target velocity
      l_step = sign(dx_tgt)*0.2;

      % Set leg swing trigger point
      trig = 0.6;

      % Define time invariant parameter based on hip position
      s = (x_st_e - x_st)/(x_st_e + l_step/2);
    end % if

    % Swing leg retraction policy (immediately retract then extend once
    % past defined trigger point)
    if s < 0.5
      [l_sw, dl_sw] = cubic(0, 0.5, l0_leg, l0_leg - (l_ret + l_clr), 0, 0, s, 1);
    else
      [l_sw, dl_sw] = cubic(0.5, 1, l0_leg - (l_ret + l_clr), l0_leg, 0, 0, s, 1);
    end
    % l_sw = l0_leg - (l_ret + l_clr)*(s < trig);

    % Swing leg swing policy (use cubic spline to interpolate target
    % ground projection point of the toe and find the corresponding leg
    % angle given a desired length)
    [d_sw, dd_sw] = cubic(0, 0.7, x_sw_e, l_step, 0, 0, s, 1);
    r_sw = pi/2 + real(acos(d_sw/l_sw)) - q(13);
    dr_sw = - dq(13) - (dd_sw/l_sw - (dl_sw*d_sw)/l_sw^2)/sqrt(1 - d_sw^2/l_sw^2);

    % Target swing leg actuator positions
    q_sw = r_sw + [-1; 1]*real(acos(l_sw));

    % Target swing leg actuator velocities
%     dq_sw = zeros(2,1);
    dq_sw = dr_sw + [1; -1]*dl_sw/sqrt(1 - l_sw^2);

    % Swing leg actuator torques from PD controller
    u(leg_u(3:4)) = s_leg*((q_sw - q(leg_m(3:4)))*kp_leg + (dq_sw - dq(leg_m(3:4)))*kd_leg);

    % Stance leg push off policy (extend leg after mid stance linearly)
    l_st = l0_leg + l_ext*clamp(2*s - 1, 0, 1);

    % Stance leg step down policy (retract leg if drop is detected)
    l_st = l_st + clamp(y_sw - y_st, -l_ret, 0);

    % Target stance leg actuator positions
    q_st = mean(q(leg_l(1:2))) + [-1; 1]*real(acos(l_st));

    % Target stance leg actuator velocities
    dq_st = [1; 1]*mean(dq(leg_l(1:2)));

    % Stance leg actuator torques from PD controller
    u(leg_u(1:2)) = (q_st - q(leg_m(1:2)))*kp_leg + (dq_st - dq(leg_m(1:2)))*kd_leg;

    % Add additional torque commands to leg actuators to stabilize
    % scaled based on the "force" felt in the leg
    u(leg_u(1:2)) = u(leg_u(1:2)) + ...
      s_st*s_torso*((q(13) - q0_pitch)*kp_leg + dq(13)*kd_leg);
    u(leg_u(3:4)) = u(leg_u(3:4)) + ...
      s_sw*s_torso*((q(13) - q0_pitch)*kp_leg + dq(13)*kd_leg);

    % Lateral foot placement
    d = - d0_hip*stanceLeg - ...
      dy_est*dy_gain - ...
      (dy_est - dy_cmd)*dy_err_gain - ...
      d_offset*22.2/60 - ...
      l_t*sin(q(11));

    % Inverse kinematics
    L = sqrt(l_l^2 + l_h^2);
    q1 = real(asin(d/L));
    q2 = real(asin(-l_h/L));
    q_h = q1 - q2 - q(11);
    q_h = clamp(q_h, -0.15*stanceLeg, 0.3*stanceLeg);
    dq_h = 0;

    % Hip feed-forward torque for gravity compensation
    u(hip_u) = 35.*[stanceLeg; -stanceLeg];

    % Swing leg PD controller
    u(hip_u(2)) = u(hip_u(2)) + ...
      s*(1 - s_sw)*(q_h - q(hip_m(2)))*kp_hip + (dq_h - dq(hip_m(2)))*kd_hip;

    % Torso stabilization weighted PD controller
    u(hip_u) = u(hip_u) + ...
      [s_st; s_sw].*s_torso.*((q(11) - q0_roll)*kp_hip + dq(11)*kd_hip);

    % Detect when swing leg force exceeds stance leg force
    if (s_sw > s_st && t > 0.2) || (isStand && s >= 1)
      % Switch stance legs
      stanceLeg = -stanceLeg;

      % Estimate extra required swing leg clearence in case of step
      %l_clr = clamp(abs(y_sw - y_st), 0, 0.15);

      % Reset time since last switch
      t = 0; x_est = 0; y_est = 0;

      % Exit conditions
      x_st_e = x_sw;
      x_sw_e = x_st;

      % Check standing speed or direction flip of commanded velocity
      if abs(dx_cmd) < 0.75 || sign(dx_cmd) ~= sign(dx_est)
        if abs(dx_est) > 1 && ~isStand
          % Walk slower
          dx_tgt = sign(dx_est)*0.75;
          dy_tgt = 0;
        else
          % Stand
          dx_tgt = dx_cmd;
          dy_tgt = dy_cmd;
          isStand = true;
        end % if
      else
        % Walk normal
        dx_tgt = dx_cmd;
        dy_tgt = 0;
        isStand = false;
      end % if
    end % if

    % Error catch for falling backwards
    if abs(dx_est) < 0.05; isStand = true; end % if

    % User outputs
    userOut = [q(12); 0; dx_est; dy_est; dx_cmd; dy_cmd];

  otherwise % RELAX -------------------------------------------------------
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

function s = scaleFactor(f, tl, tu)
%SCALEFACTOR Compute scalar (0 to 1) representing forces in leg.

  s = (clamp(f, tl, tu) - tl)/(tu - tl);
end % scaleFactor
