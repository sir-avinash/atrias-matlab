% This function performs a long alignment of the IMU based on log data.
%
% Parameters:
%     app      An instance of AtriasPostProcess with the relevant data
%     latitude The alignment location's latitude (radians)
%
% Returns:
%     heading The robot's heading, in radians.

function heading = imu_align(app, latitude)
	% Pick out the necessary data
	imu_data = app.controllerData(:, 1:8).';
	gyros    = imu_data(1:3, :);
	accels   = imu_data(4:6, :);
	statuses = imu_data(8,   :);

	% Find the first sample of actual data
	firstSmpl = min(find(statuses));

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
	startTick      = risings(startEdge)+1; % Find the starting time for this run; the +1 compensates for diff(is_stationary) being 1 smaller than is_stationary
	endTick        = fallings(startEdge);  % Similarly, find the ending time.

	% Throw out the first and last 10 seconds of data.
	% This is to avoid including that motion in the alignment calculations
	discardCycles = 10 * 1000; % 10 seconds, 1 kHz
	startTick     = startTick + discardCycles;
	endTick       = endTick   - discardCycles;

	% Throw an error if we have no good data remaining
	if startTick > endTick
		error('Robot not stationary for long enough!')
	end

	% Output some nice user diagnostics
	disp(['Found ' num2str(app.time(endTick) - app.time(startTick), '%.3f') ' seconds of stationary data beginning at time ' ...
	      num2str(app.time(startTick), '%.3f') ' and ending at time ' num2str(app.time(endTick), '%.3f')])

	% Spit out a warning if less than 3 minutes of data was given
	minDur = 3 * 60;
	if app.time(endTick) - app.time(startTick) < minDur
		warning('The robot was not stationary for very long. The results may be inaccurate')
	end

	% Zero out the data from cycles where the status byte was incorrect.
	% This further cleans the data, preventing bad readings from skewing the results
	params = imu.IMUParams;
	gyros(:,  statuses ~= params.nom_status) = 0;
	accels(:, statuses ~= params.nom_status) = 0;

	% Begin the actual alignment code.
	% Here, we copy what imu.IMUSys does
	imu_orient = imu.init_imu_orient;

	% Integrate the data
	align_rm = sum(gyros(:, startTick:endTick),  2);
	align_gm = sum(accels(:, startTick:endTick), 2);

	% Another diagnostic: compute the latitude
	computedLat = pi/2 - acos(dot(align_rm/norm(align_rm), align_gm/norm(align_gm)));
	disp(['Computed latitude of ' num2str(rad2deg(computedLat)) ' degrees. Deviation from given latitude: ' num2str(rad2deg(computedLat - latitude)) ' degrees.'])

	% Step 1: leveling
	[imu_orient, fail_reas] = imu.align_lvl(imu_orient, align_gm);

	if fail_reas ~= imu.IMUFailReason.NONE
		error(['Alignment failure. Reason: ' fail_reas])
	end

	% Step 2: Heading correction via gyros
	imu_orient = imu.align_rothdg(imu_orient, align_rm);

	% No error checking is necessary after step 2.
	% This completes the alignment.

	% To find the heading, we rotate the IMU's Z vector into the world frame then do some trig.
	imuZ_world = imu_orient.rot([0; 0; 1]);
	heading    = atan2(imuZ_world(2), -imuZ_world(1));

	% Last, bring the heading into the [0, 2*pi) range (mainly to fit standard heading conventions)
	heading = mod(heading, 2*pi);
end
