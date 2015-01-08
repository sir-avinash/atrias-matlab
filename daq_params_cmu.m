% This function generates:
%       calibration parameters 
%       motor parameters
%
% It is primarily based on http://code.google.com/p/atrias/source/browse/robot_definitions/
%
% Notes:
%   CMU biped = type 1
%   CMU left leg = type 1
%   CMU right leg = type 2
%   1 | A = shin
%   2 | B = thigh
%   leg = shin/thigh encoders
%   trans = motor encoders

% Update rates
sample_time = 0.001; % seconds
update_freq = 1/sample_time; % Hz

% Medulla state parameters
MEDULLA_STATE_IDLE  = 0;
MEDULLA_STATE_RUN   = 2;
MEDULLA_STATE_HALT  = 4;
MEDULLA_STATE_ERROR = 5;
MEDULLA_STATE_RESET = 6;

% Commands sent from the Instrument Panel to the system.
% These become parameters for the Simulink model
gui_enable_cmd  = 0; % Whether to enable the robot
gui_disable_cmd = 0; % Whether to disable the robot
gui_reset_cmd   = 0; % Whether to reset the robot (after a E-Stop)

% Low-level Medulla parameters
MEDULLA_ADC_OFFSET_COUNTS = 172;  % Ticks
MEDULLA_ADC_MAX_VOLTS     = 2.70; % Volts

% Renishaw Absolute 32-bit encoders
LEG_A_CALIB_LOC = 0.959931088596881; % Radians
LEG_B_CALIB_LOC = 2.181661564992912; % Radians

% Ticks to radians constants
BOOM_PITCH_ENCODER_RAD_PER_TICK = 2*pi / (2^17 * 2); % Ticks to rad constant = rad_per_rev / (ticks_per_rev * gear_ratio)
BOOM_ROLL_ENCODER_RAD_PER_TICK = 2*pi / (2^17 * 7); % Ticks to rad constant = rad_per_rev / (ticks_per_rev * gear_ratio)
BOOM_YAW_ENCODER_RAD_PER_TICK = 2*pi / (2^17 * 9.6); % Ticks to rad constant = rad_per_rev / (ticks_per_rev * gear_ratio)
HIP_INC_ENCODER_RAD_PER_TICK =  2*pi / (2500 * 4 * 57);  % Ticks to rad constant = rad_per_rev / (lines_per_rev * quadrature * gear_ratio)
HIP_ABS_ENCODER_RAD_PER_TICK = 2*pi / 2^13; % Ticks to rad constant = rad_per_rev / ticks_per_rev              

% Motor parameters
LEFT_MOTOR_HIP_DIRECTION = 1.0; %The direction for the left hip motor.
RIGHT_MOTOR_HIP_DIRECTION = 1.0; %The direction for the right hip motor.
MTR_MAX_COUNT = 19900; % The maximum commanded amplifier value. This is the maximum value sent to the Medullas for the amplifier command.
LEG_MOTOR_CONSTANT = 0.119;
HIP_MOTOR_CONSTANT = 0.184;
HIP_MTR_GEAR_RATIO = 57;
LEG_MTR_GEAR_RATIO = 50;

HIP_INC_MAX_ENCODER_TICKS = 2^16 - 1; % Max value of the incremental hip encoder (16-bit)
HIP_ABS_MAX_ENCODER_TICKS = 2^13 - 1; % Max value of the absolute hip encoder (13-bit)
BOOM_MAX_ENCODER_TICKS = 2^17 - 1; % Max value of the absolute boom encoders (17-bit)

% From biped1_variant_defs.h
% Note: Left and right are swapped for CMU
RIGHT_HIP_CALIB_VAL = 4444;   %Calibration encoder value in ticks (OSU convention)
LEFT_HIP_CALIB_VAL = 1359;  %Calibration encoder value in ticks (OSU convention)
RIGHT_HIP_CALIB_POS = -pi/2;  %Calibration angle in radians (OSU convention)
LEFT_HIP_CALIB_POS = 3*pi/2; %Calibration angle in radians (OSU convention)
MTR_MAX_CURRENT = 60.0; %Maximum motor current for scaling
MTR_MAX_CONT_CURRENT = 30.0; %Maximum continuous amplifier current
MTR_HIP_MAX_CURRENT = 60.0; %Maximum hip motor current for scaling
BOOM_LENGTH = 2.006; % meters, center of rotation to hip center
BOOM_X_METERS_PER_TICK = -0.00000937522094511213193198; %The meters of boom motion per encoder tick. This is calculated from the boom's length, the number of encoder ticks per encoder revolution, and the gear ratio between the boom and the encoder.
LEG_MTR_MAX_TORQUE = MTR_MAX_CURRENT*LEG_MOTOR_CONSTANT*LEG_MTR_GEAR_RATIO;
LEG_MTR_MAX_CONT_TORQUE = MTR_MAX_CONT_CURRENT*LEG_MOTOR_CONSTANT*LEG_MTR_GEAR_RATIO;
LEG_MTR_MAX_VELOCITY = 393.755 / LEG_MTR_GEAR_RATIO; % Rad/s
HIP_MTR_MAX_TORQUE = MTR_HIP_MAX_CURRENT*HIP_MOTOR_CONSTANT*HIP_MTR_GEAR_RATIO;


% From leg1_definitions.h (corresponds to CMU right leg)
%(Note that A=shin and B=thigh for OSU convention)
LEG1_LEG_A_CALIB_VAL = 227806296+18240-105494; %Calibration encoder value in ticks
LEG1_LEG_B_CALIB_VAL = 227981518-27550+100498; %Calibration encoder value in ticks
LEG1_LEG_A_RAD_PER_CNT = 9.8039216e-09; %Ticks to rad constant
LEG1_LEG_B_RAD_PER_CNT = -9.8039216e-09; %Ticks to rad constant
LEG1_TRAN_A_CALIB_VAL = 162551054; %Calibration encoder value in ticks
LEG1_TRAN_B_CALIB_VAL = 163658926; %Calibration encoder value in ticks
LEG1_TRAN_A_RAD_PER_CNT = 9.8039216e-09; %Ticks to rad constant
LEG1_TRAN_B_RAD_PER_CNT = -9.8039216e-09; %Ticks to rad constant
LEG1_MOTOR_A_DIRECTION = 1.0; 
LEG1_MOTOR_B_DIRECTION = -1.0;

% From leg2_definitions.h (corresponds to CMU left leg)
%(Note that A=shin and B=thigh for OSU convention)
LEG2_LEG_A_CALIB_VAL = 208914429-56335+74466; %Calibration encoder value in ticks
LEG2_LEG_B_CALIB_VAL = 229010442-9300+102681; %Calibration encoder value in ticks
LEG2_LEG_A_RAD_PER_CNT = -9.8039216e-09; %Ticks to rad constant
LEG2_LEG_B_RAD_PER_CNT = -9.8039216e-09; %Ticks to rad constant
LEG2_TRAN_A_CALIB_VAL = 162423925; %Calibration encoder value in ticks
LEG2_TRAN_B_CALIB_VAL = 177684487; %Calibration encoder value in ticks
LEG2_TRAN_A_RAD_PER_CNT = 9.8039216e-09; %Ticks to rad constant
LEG2_TRAN_B_RAD_PER_CNT = 9.8039216e-09; %Ticks to rad constant
LEG2_MOTOR_A_DIRECTION = 1.0;
LEG2_MOTOR_B_DIRECTION = -1.0;

% From measurements at CMU
BOOM_PITCH_CAL_VALUE_RAD = 0;
BOOM_PITCH_CAL_VALUE_TICKS  = 63515; % ticks, pitch encoder reading when robot is vertical (64109 when pitch locked)
BOOM_ROLL_CAL_VALUE_RAD = 6*pi/180; %Roll angle (in radians) for calibration point
BOOM_ROLL_CAL_VALUE_TICKS = 68304; %Roll encoder value for calibration point
BOOM_ROBOT_VERTICAL_OFFSET = 0.3434; % meters, 
BOOM_HEIGHT = 0.895; % meters, top of the center of rotation to robot ground level

%% params for DAQ functions
fcut_smooth = 16*(2*pi); % Hz, low pass filter cutoff frequency for removing encoder dropouts
fcut_unwrap = 8*(2*pi); % Hz, low pass filter cutoff frequency for unwrapping angles
max_encoder_velocity = 200; % rad/s, max rate which encoder values can change
sensor_start_time = 8; % s
% two pole low-pass filter params for velocity estimation
fcut_velocity = 100*(2*pi); % Hz, low pass filter cutoff frequency for velocities
lpf_damping = sqrt(2)/2;
B1_lpf_velocity = -2*exp(-lpf_damping*fcut_velocity*sample_time)*cos(fcut_velocity*sample_time*sqrt(1-lpf_damping^2));
B2_lpf_velocity = exp(-2*lpf_damping*fcut_velocity*sample_time);
A_lpf_velocity = 1 + B1_lpf_velocity + B2_lpf_velocity;

% Parameters related to incremental encoder decoding
INC_ENC_RAD_PER_TICK = 2*pi/14000/LEG_MTR_GEAR_RATIO;
INC_ENC_DIRS         = -[ LEG1_MOTOR_A_DIRECTION
                          LEG1_MOTOR_B_DIRECTION
                          LEG2_MOTOR_A_DIRECTION
                          LEG2_MOTOR_B_DIRECTION ];
MEDULLA_TIMER_FREQ = 32e6;

% The distance between the hip pivot and the leg motors pivot (the center of the leg)
HIP_WIDTH = .18;

% Whether or not to enable sensor decoding for each medulla in the DAQ
imu_daq_enable = true;
