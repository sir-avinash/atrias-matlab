classdef IMUFailReason < uint8
	enumeration
		NONE(0)       % We're still working (there's not actually a failure).
		MOTION(1)     % The robot moved too much during alignment
		BAD_GACCEL(2) % The gravitational acceleration's direction was unexpected (upside-down robot?).
	end
end
