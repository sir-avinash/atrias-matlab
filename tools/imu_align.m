% This function performs a long alignment of the IMU based on log data.
%
% Parameters:
%     app      An instance of AtriasPostProcess with the relevant data
%     latitude The alignment location's latitude
%
% Returns:
%     heading The robot's heading, in radians.

function heading = imu_align(app, latitude)
	% Pick out the necessary data
	imu_data = app.controllerData(:, 1:7).';
	gyros    = imu_data(1:3, :);
	accels   = imu_data(4:6, :);
	seqs     = imu_data(7,   :);

	% Find the first sample of actual data
	firstSmpl = min(find(seqs));

	% Find all of the times where the robot is stationary
	is_stationary                = ~imu.checkMotion(imu.IMUParams, gyros, accels);
	is_stationary(1:firstSmpl-1) = 0;

	% Also, for consistency, indicate that it is not stationary at the end of the simulation.
	% This will keep the risings and fallings variables (in the next section) the same length.
	is_stationary(end) = 0;

	% Find the longest consecutive sequence of stationary samples
	edges    = diff(is_stationary);        % Do edge detection on is_stationary
	risings  = find(edges > 0);            % Pick out the locations of the rising edges
	fallings = find(edges < 0);            % Pick out the locations of the falling edges
	runlens  = fallings - risings;         % The length of each run of 1's is just the difference in the corresponding rising and falling points.
	[~, startEdge] = max(runlens);         % Identify the index of the longest run (or the first longest run, if there are multiple).
	startTime      = risings(startEdge)+1; % Find the starting time for this run; the +1 compensates for diff(is_stationary) being 1 smaller than is_stationary
	endTime        = fallings(startEdge);  % Similarly, find the ending time.
end
