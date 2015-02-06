classdef IMUFailReason < uint8
	enumeration
		NONE(0)       % We're still working (there's not actually a failure).
		MOTION(1)     % The robot moved too much during alignment
		BAD_GACCEL(2) % The gravitational acceleration's direction was unexpected (upside-down robot?).
		WATCHDOG(3)   % The data from the IMU stopped updating
		IMU_STATUS(4) % The IMU's built in test thinks the IMU is broken. This could be *very bad*
		GYRO_MAG(5)   % One or more gyros reported an angular rate that exceeds its maximum specified limit -- likely bad data
		ACCEL_MAG(6)  % One or more accelerometers reported an acceleration that exceeds its upper limit -- likely bad data
		NOTFINITE(7)  % A gyro or accelerometer reading was not finite.
	end
end
