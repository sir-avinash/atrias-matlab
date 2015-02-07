% Class to encompass all of the IMU system configuration parameters

classdef IMUParams
	properties
		align_gyro_tol  = 1e-6 % Motion tolerance during alignment, rads/millisecond
		align_accel_tol = .02  % Motion tolerance during alignment, Gs
		align_time_ms   = 5 * 1000;           % Alignment time, in milliseconds
		earth_rot_rate  = 7.292115e-5 * .001; % Earth's rotation rate, rad/millisecond. From WolframAlpha
		bad_data_tol    = 10                  % The number of cycles (sequence counter updates) we'll tolerate missing.
		max_gyro_rate   = 490 * pi/180; % The maximum gyro input rate, rad/s
		max_accel       = 10;           % The accelerometers's maximum input, Gs
		nom_status      = 119  % The nominal status byte value for the IMU
	end
end
