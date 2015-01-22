classdef IMUFailReason < uint8
	enumeration
		NONE(0)       % We're still working (there's not actually a failure).
		MOTION(1)     % The robot moved too much during alignment
		BAD_GACCEL(2) % The gravitational acceleration's direction was unexpected (upside-down robot?).
		WATCHDOG(3)   % The data from the IMU stopped updating
		IMU_STATUS(4) % The IMU's built in test thinks the IMU is broken. This could be *very bad*
	end
end
