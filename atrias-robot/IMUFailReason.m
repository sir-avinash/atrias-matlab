classdef IMUFailReason < uint8
	enumeration
		NONE(0)       % We're still working (there's not actually a failure).
		MOTION(1)     % The robot moved too much during alignment
		BAD_GACCEL(2) % The gravitational acceleration's direction was unexpected (upside-down robot?).
		BIG_BIAS(3)   % A large bias was computed. Is the latitude correct?
		WATCHDOG(4)   % The data from the IMU stopped updating
	end
end
