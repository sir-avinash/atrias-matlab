classdef IMUSys < handle
	methods
		% Constructor -- does some basic init that can't directly be done to the properties themselves
		function this = IMUSys(sample_time)
			% This IMU is two simple rotations away from the local orientation.
			imu_rel_local      = Quat([0; pi/4; 0]) * Quat([-pi/2; 0; 0]);
			this.local_rel_imu = imu_rel_local.conj();

			% The local orientation is initially equal to the world orientation.
			this.imu_orient = imu_rel_local;

			% Just copy over the same time; it's specified in daq_params and passed through
			this.sample_time = sample_time;

			% Compute the number of cycles we want to wait after a failed align and before the first alignment
			this.align_reset_wait    = 3.0 / sample_time;     % Desired wait time in seconds converted into samples
			this.align_reset_counter = this.align_reset_wait; % Switch to align on the first packet, since the robot should be stationary during model startup
		end

		% This checks if our motion is within the alignment tolerances.
		function is_moving = checkMotion(this, gyros, accels)
			is_moving = (norm(gyros) >= this.align_gyro_tol || abs(norm(accels) - 1) >= this.align_accel_tol);
		end

		% This waits until align_reset_wait cycles of zero motion before attempting to align.
		function init(this, gyros, accels)
			% Throw out the first sample
			if ~this.has_data
				this.has_data = true;
				return
			end

			% Make sure the robot's stationary before beginning or re-trying alignment
			if this.checkMotion(gyros, accels)
				this.fail_reas           = IMUFailReason.MOTION;
				this.align_reset_counter = 0; % We're moving; reset the timer
			else
				this.align_reset_counter = this.align_reset_counter + 1; % We're stationary, continue (or begin) counting

				% Show that we've stopped detecting motion.
				this.fail_reas = IMUFailReason.NONE;

				% Switch to align if it's been long enough
				if this.align_reset_counter >= this.align_reset_wait
					this.state     = IMUSysState.ALIGN;
					this.align_ticks = 0;
				end
			end
		end

		% Alignment update function. This accumulates accelerometer readings in order to properly level the IMU at startup.
		function align(this, gyros, accels, latitude, heading)
			this.align_gm    = this.align_gm + accels;
			this.align_ticks = this.align_ticks + 1;

			% If the gyros or accelerometers read something too large,
			% terminate alignment and indicate the error
			if this.checkMotion(gyros, accels)
				this.state               = IMUSysState.INIT;
				this.align_reset_counter = 0;
				this.fail_reas           = IMUFailReason.MOTION;
				return
			end

			% Return early if the alignment isn't done yet.
			if this.align_ticks < this.align_time_ms
				return
			end

			% Alignment complete!


			% Run through the alignment steps. Check for errors in between each step and terminate if an error was found.
			% For simplicity, we'll set the FAIL state here so that an alignment step only needs to set fail_reas to signal an error

			this.align_step1

			if this.fail_reas ~= IMUFailReason.NONE
				this.state = IMUSysState.FAIL;
				return
			end

			this.align_step2(heading)

			% No need to re-check here, as align_step2 has no failure modes

			this.align_step3(latitude)

			% Again, align_step3 has no failure modes

			% Alignment was successful!
			% Set the state for the next iteration.
			this.state = IMUSysState.RUN;
		end

		% Step 1 of the alignment process (leveling)
		function align_step1(this)
			% First, rotate the acceleration vector using the previous-known IMU orientation.
			ghat = this.imu_orient.rot(this.align_gm);

			% Next, normalize the accumulated acceleration vector
			u_g = ghat / norm(ghat);

			% If a correction greater than (or equal to) 90 degrees is necessary, fail.
			% (robot upside down?)
			if u_g(3) <= 0
				this.fail_reas = IMUFailReason.BAD_GACCEL;
				return
			end

			% Use a cross product to compute the rotation axis for the
			% orientation correction. This will have a length which is the sin
			% of the necessary correction rotation angle.
			a = cross(u_g, [0; 0; 1]);

			% Catch the (rare!) case where no correction is necessary.
			if all(a == 0)
				return
			end

			% Turn the previous cross product into a true axis and angle
			theta = asin(norm(a));
			axis  = a / norm(a);

			% Apply the correction to the orientation
			this.imu_orient = Quat(theta * axis) * this.imu_orient;
		end

		% Step 2 of the alignment process (heading)
		function align_step2(this, heading)
			% Rotate the IMU's Z vector into world coordinates
			imuZ_world = this.imu_orient.rot([0; 0; 1]);

			% Compute the rotation angle for the correction
			theta = atan2(imuZ_world(2), -imuZ_world(1)) - heading;

			% Perform the update
			this.imu_orient = Quat(theta * [0; 0; 1]) * this.imu_orient;
		end

		% Step 3 of the alignment process (earth rotation and bias)
		function align_step3(this, latitude)
			% Compute the Earth's rotation vector for rotation cancellation
			this.earth_rot = this.earth_rot_rate * [0; cos(latitude); sin(latitude)];
		end

		% Main IMU operation state.
		function run(this, gyros, seq)
			% Compute the size of the seq increment (note that seq wraps modulo 128).
			dseq = mod(int16(seq) - int16(this.prevSeq), int16(128));

			% Rotate the delta angles from IMU coordinates into world coordinates
			gyros_world = this.imu_orient.rot(gyros);

			% Remove the Earth's rotation from the gyro measurements
			gyros_world = gyros_world - this.earth_rot;

			% Compute the angular velocities
			this.ang_vel = gyros_world / this.sample_time;

			% Rescale the gyro delta angles using dseq to make up for any missed cycles
			gyros_world = double(dseq) * gyros_world;

			% Execute the update to imu_orient
			this.imu_orient = Quat(gyros_world) * this.imu_orient;
		end

		% Function to check if the data seems valid or not...
		% Should be only called once per iteration
		function is_valid = checkValidity(this, gyros, accels, seq, status)
			% Assume it's bad. We'll change this if all checks are good
			is_valid = false;

			% Check gyro magnitudes
			if any(abs(gyros) >= 420 * pi / 180 * this.sample_time)
				return
			end

			% Check accelerometer magnitudes
			if any(abs(accels) >= 12)
				return
			end

			% Check for nonfinite gyro or accelerometer values
			if any(~isfinite(gyros)) || any(~isfinite(accels))
				return
			end

			% Check for a properly increasing sequence counter
			dseq = mod(seq - this.prevSeq, 128);
			if dseq ~= 1 && dseq ~= 2
				return
			end

			% IMU BIT result check
			if status ~= this.nom_status
				return
			end

			% Update our stored values for the next iteration
			this.prevSeq = seq;

			% Every check passed! The data seems valid.
			is_valid = true;
		end

		% The main IMU update loop; contains a state machine to handle alignment
		% ang_vel is in IMU coordinates!
		function [local_orient, ang_vel, state, fail_reas] = ...
			update(this, gyros, accels, seq, status, latitude, heading)
		%
			% Run the state machine iff we have new data
			if this.checkValidity(gyros, accels, seq, status)
				switch this.state
					case IMUSysState.INIT
						this.init(gyros(:), accels(:))

					case IMUSysState.ALIGN
						this.align(gyros(:), accels(:), latitude, heading)

					case IMUSysState.RUN
						this.run(gyros(:), seq)
				end

				% Update the missed sequence counter with this new information
				if this.missed_seq_cntr > 0
					this.missed_seq_cntr = this.missed_seq_cntr - 1;
				end
			else
				% No new data this cycle. If we've previously gotten new data, this is an error.

				% First, ignore the case where we have not yet gotten new IMU data or have already failed
				if this.state == IMUSysState.ALIGN || this.state == IMUSysState.RUN
					% Record the missed sequence
					this.missed_seq_cntr = this.missed_seq_cntr + 1;

					% Check if this surpasses our missed sequence tolerance
					if this.missed_seq_cntr > this.missed_seq_tol
						% Uh oh... watchdog failure. Fail and indicate the failure reason
						this.state     = IMUSysState.FAIL;
						this.fail_reas = IMUFailReason.WATCHDOG;
					end
				end
			end

			% Update the function outputs
			local_orient = this.imu_orient * this.local_rel_imu;
			ang_vel      = this.ang_vel;
			state        = this.state;
			fail_reas    = this.fail_reas;
		end
	end

	properties
		% Various parameters and constants
		align_gyro_tol  = 1e-6 % Motion tolerance during alignment, rads/millisecond
		align_accel_tol = .02  % Motion tolerance during alignment, Gs
		align_reset_wait       % The number of cycles to wait between the last robot motion and re-trying a failed alignment. Calculated in the constructor
		align_time_ms   = 5 * 1000;           % Alignment time, in milliseconds
		earth_rot_rate  = 7.292115e-5 * .001; % Earth's rotation rate, rad/millisecond. From WolframAlpha
		missed_seq_tol  = 10   % The number of cycles (sequence counter updates) we'll tolerate missing.
		nom_status      = 119  % The nominal status byte value for the IMU
		sample_time

		% Init state's state.
		align_reset_counter % Current counter for the re-align-attempt delay

		% Accumulators for accelerometers and gyros for alignment
		align_gm = [0; 0; 0]

		% Elapsed alignment duration, in units of sample_time
		align_ticks = 0

		% Current angular velocity, in world coordinates
		ang_vel = [0; 0; 0]

		% Earth's angular velocity, radians per cycle
		earth_rot = [0; 0; 0]

		% Explanation for an alignment failure
		fail_reas = IMUFailReason.NONE

		% Whether or not we've gotten data yet. Used for throwing
		% out the first IMU packet, which usually contains a spike.
		has_data = false;

		% The current orientation of the IMU coordinate frame (Quat)
		imu_orient

		% Similarly, the orientation of the ATRIAS coordinate frame in the IMU frame
		local_rel_imu

		% A counter which records missed sequences; used for a watchdog.
		missed_seq_cntr = uint8(0)

		% Previous sequence value; kept to detect when new data is available and
		% to begin alignment at the correct time.
		prevSeq = uint8(0)

		% Current alignment state
		state = IMUSysState.INIT;
	end
end % classdef
