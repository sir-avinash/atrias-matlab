%SIMULATION_PARAMS Simulation properties of ATRIAS.

%% TIMINGS ================================================================
t_end = 30;
% output_sample_time = 1/30; % For video renders
output_sample_time = 1/1000; % For more detailed plots and animation

% Sensors
sample_freq = 1000;
sample_time = 1/sample_freq;
lpf_damping = sqrt(2)/2; % Butterworth filter damping
fcut_velocity = 100*(2*pi); % Cutoff frequency for velocities
B1_lpf_velocity = -2*exp(-lpf_damping*fcut_velocity*sample_time)*cos(fcut_velocity*sample_time*sqrt(1-lpf_damping^2));
B2_lpf_velocity = exp(-2*lpf_damping*fcut_velocity*sample_time);
A_lpf_velocity = 1 + B1_lpf_velocity + B2_lpf_velocity;
