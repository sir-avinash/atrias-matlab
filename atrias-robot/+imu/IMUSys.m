classdef IMUSys < handle
	methods
		% Constructor -- does some basic init that can't directly be done to the properties themselves
		function this = IMUSys(sample_time)
			% This IMU is two simple rotations away from the local orientation.
			imu_rel_local      = imu.init_imu_orient();
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
			is_moving = (norm(gyros) >= this.params.align_gyro_tol || abs(norm(accels) - 1) >= this.params.align_accel_tol);
		end

		% This waits until align_reset_wait cycles of zero motion before attempting to align.
		function init(this, gyros, accels)
			% Throw out the first samples (until they stabilize; the first few samples are often very large).
			if ~this.has_data
				if ~this.checkMotion(gyros, accels)
					this.has_data = true;
				end

				return
			end

			% Make sure the robot's stationary before beginning or re-trying alignment
			if this.checkMotion(gyros, accels)
				this.fail_reas           = imu.IMUFailReason.MOTION;
				this.align_reset_counter = 0; % We're moving; reset the timer
			else
				this.align_reset_counter = this.align_reset_counter + 1; % We're stationary, continue (or begin) counting

				% Show that we've stopped detecting motion.
				this.fail_reas = imu.IMUFailReason.NONE;

				% Switch to align if it's been long enough
				if this.align_reset_counter >= this.align_reset_wait
					this.state     = imu.IMUSysState.ALIGN;
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
				this.state               = imu.IMUSysState.INIT;
				this.align_reset_counter = 0;
				this.fail_reas           = imu.IMUFailReason.MOTION;
				return
			end

			% Return early if the alignment isn't done yet.
			if this.align_ticks < this.params.align_time_ms
				return
			end

			% Alignment complete!


			% Run through the alignment steps. Check for errors in between each step and terminate if an error was found.
			% For simplicity, we'll set the FAIL state here so that an alignment step only needs to set fail_reas to signal an error

			[this.imu_orient, this.fail_reas] = imu.align_lvl(this.imu_orient, this.align_gm);

			if this.fail_reas ~= imu.IMUFailReason.NONE
				this.state = imu.IMUSysState.FAIL;
				return
			end

			this.imu_orient = imu.align_corrhdg(this.imu_orient, heading);

			% No need to re-check here, as align_step2 has no failure modes

			this.earth_rot = imu.align_biases(this.params.earth_rot_rate, latitude);

			% Again, align_step3 has no failure modes

			% Alignment was successful!
			% Set the state for the next iteration.
			this.state = imu.IMUSysState.RUN;
		end

		% Main IMU operation state.
		function run(this, gyros, dseq)
			% Rotate the delta angles from IMU coordinates into world coordinates
			gyros_world = this.imu_orient.rot(gyros);

			% Remove the Earth's rotation from the gyro measurements
			gyros_world = gyros_world - this.earth_rot;

			% Compute the angular velocities
			this.ang_vel = gyros_world / this.sample_time;

			% Rescale the gyro delta angles using dseq to make up for any missed cycles
			gyros_world = double(dseq) * gyros_world;

			% Execute the update to imu_orient
			this.imu_orient = imu.Quat(gyros_world) * this.imu_orient;
		end

		% Function to check if the data seems valid or not...
		function fail_reas = checkValidity(this, gyros, accels, dseq, status)
			% Assume it's good. We'll change this later if we find that the data was actually bad
			fail_reas = imu.IMUFailReason.NONE;

			% Check gyro magnitudes
			if any(abs(gyros) >= this.params.max_gyro_rate * this.sample_time)
				fail_reas = imu.IMUFailReason.GYRO_MAG;
				return
			end

			% Check accelerometer magnitudes
			if any(abs(accels) >= this.params.max_accel)
				fail_reas = imu.IMUFailReason.ACCEL_MAG;
				return
			end

			% Check for nonfinite gyro or accelerometer values
			if any(~isfinite(gyros)) || any(~isfinite(accels))
				fail_reas = imu.IMUFailReason.NOTFINITE;
				return
			end

			% Check for a properly increasing sequence counter
			if dseq ~= 1
				fail_reas = imu.IMUFailReason.WATCHDOG;
				return
			end

			% IMU BIT result check
			if status ~= this.params.nom_status
				fail_reas = imu.IMUFailReason.IMU_STATUS;
				return
			end
		end

		% The main IMU update loop; contains a state machine to handle alignment
		% ang_vel is in IMU coordinates!
		function [local_orient, ang_vel, state, fail_reas] = ...
			update(this, gyros, accels, seq, status, latitude, heading)
		%
			% Compute the size of the seq increment (note that seq wraps modulo 128).
			dseq = mod(int16(seq) - int16(this.prevSeq), int16(128));
			this.prevSeq = seq;

			% If we have yet to fail, do our integrity checks.
			if this.state ~= imu.IMUSysState.FAIL
				this.fail_reas = this.checkValidity(gyros, accels, dseq, status);
			end

			% Run the state machine iff we have new, valid data
			if this.fail_reas == imu.IMUFailReason.NONE
				switch this.state
					case imu.IMUSysState.INIT
						this.init(gyros(:), accels(:))

					case imu.IMUSysState.ALIGN
						this.align(gyros(:), accels(:), latitude, heading)

					case imu.IMUSysState.RUN
						this.run(gyros(:), dseq)
				end

				% Update the missed sequence counter with this new information
				if this.bad_data_cntr > 0
					this.bad_data_cntr = this.bad_data_cntr - 1;
				end
			else
				% No new data this cycle. If we've previously gotten new data, this is an error.

				% First, ignore the case where we have not yet gotten new IMU data or have already failed
				if this.state == imu.IMUSysState.ALIGN || this.state == imu.IMUSysState.RUN
					% Record the missed sequence
					this.bad_data_cntr = this.bad_data_cntr + 1;

					% Check if this surpasses our missed sequence tolerance
					if this.bad_data_cntr > this.params.bad_data_tol
						% Uh oh... watchdog failure. Fail and indicate the failure reason
						this.state = imu.IMUSysState.FAIL;
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
		% Parameters
		align_reset_wait    % The number of cycles to wait between the last robot motion and re-trying a failed alignment. Calculated in the constructor
		params = imu.IMUParams
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
		fail_reas = imu.IMUFailReason.NONE

		% Whether or not we've gotten data yet. Used for throwing
		% out the first IMU packet, which usually contains a spike.
		has_data = false;

		% The current orientation of the IMU coordinate frame (Quat)
		imu_orient

		% Similarly, the orientation of the ATRIAS coordinate frame in the IMU frame
		local_rel_imu

		% A counter which records bad data; used for the watchdog and data integrity functionality
		bad_data_cntr = uint8(0)

		% Previous sequence value; kept to detect when new data is available and
		% to begin alignment at the correct time.
		prevSeq = uint8(0)

		% Current alignment state
		state = imu.IMUSysState.INIT;
	end
end % classdef
