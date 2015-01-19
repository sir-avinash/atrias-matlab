classdef IMUSysState < uint8
	enumeration
		INIT(1)  % Waiting for first data
		ALIGN(2) % Conducting the alignment process
		RUN(3)   % Running (producing accurate output)
		FAIL(4)  % Something's broken. Failed alignment? Missing data?
	end
end
