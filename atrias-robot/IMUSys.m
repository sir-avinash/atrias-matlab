classdef IMUSys < handle
	methods
		% Constructor -- does some basic init that can't directly be done to the properties themselves
		function this = IMUSys
			% This IMU is two simple rotations away from the local orientation.
			this.imu_rel_local = Quat([0; pi/4; 0]) * Quat([-pi/2; 0; 0]);
			this.local_rel_imu = this.imu_rel_local.conj();

			% The local orientation is initially equal to the world orientation.
			this.imu_orient = this.imu_rel_local;
		end

		% Initialization state update function. Waits until we have new data.
		% Note that alignment should not begin until the second cycle of data
		% as the first cycle can have a large spike.
		function init(this, seq)
			% We wish to remain in init if seq is stuck at any value.
			% Since we initialize prevSeq to 0, if seq starts at a nonzero (constant)
			% value, then we seq \neq prevSeq and prevSeq \neq 0.
			% This was chosen in order to remain conservative and not accidentally perform a bad
			% align.
			if this.prevSeq ~= 0
				this.state = IMUSysState.ALIGN;
			end
		end

		% Alignment update function. This accumulates accelerometer readings in order to properly level the IMU at startup.
		function align(this, gyros, accels, sample_time, align_time_ms, align_gyro_tol, align_accel_tol, align_bias_tol, earth_rot_rate, latitude)
			this.align_gm    = this.align_gm + accels;
			this.align_rm    = this.align_rm + gyros;
			this.align_ticks = this.align_ticks + 1;

			% If the gyros or accelerometers read something too large,
			% terminate alignment and indicate the error
			if norm(gyros) >= align_gyro_tol || abs(norm(accels) - 1) >= align_accel_tol
				this.state     = IMUSysState.FAIL_ALIGN;
				this.fail_reas = IMUFailReason.MOTION;
				return
			end

			% Return early if the alignment isn't done yet.
			if this.align_ticks < align_time_ms
				return
			end


			% Alignment complete!


			% Run through the alignment steps. Check for errors in between each step and terminate if an error was found.
			% For simplicity, we'll set the FAIL_ALIGN state here so that an alignment step only needs to set fail_real to signal an error

			this.align_step1

			if this.fail_reas ~= IMUFailReason.NONE
				this.state = IMUSysState.FAIL_ALIGN;
				return
			end

			this.align_step2

			% No need to re-check here, as align_step2 has no failure modes

			this.align_step3(earth_rot_rate, latitude, align_bias_tol)

			if this.fail_reas ~= IMUFailReason.NONE
				this.state = IMUSysState.FAIL_ALIGN;
				return
			end


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
		function align_step2(this)
			% Rotate the rotation vector into world coordinates
			rhat = this.imu_orient.rot(this.align_rm);

			% Compute the rotation angle for the correction
			theta = atan2(rhat(1), rhat(2));

			% Check that the angle is not zero (unlikely, but possible).
			if theta == 0
				return
			end

			% Perform the update
			this.imu_orient = Quat(theta * [0; 0; 1]) * this.imu_orient;
		end

		% Step 3 of the alignment process (earth rotation and bias)
		function align_step3(this, earth_rot_rate, latitude, align_bias_tol)
			% Compute the Earth's rotation vector
			this.earth_rot = earth_rot_rate * [0; cos(latitude); sin(latitude)];

			% Rotate the Earth's rotation vector into the IMU frame for computing the bias
			earth_rot_imu = this.imu_orient.invRot(this.earth_rot);

			% Compute the bias and store it for use in the integrator
			this.gyro_bias = this.align_rm/this.align_ticks - earth_rot_imu;

			% Check if the bias is acceptable
			if norm(this.gyro_bias) > align_bias_tol
				this.fail_reas = IMUFailReason.BIG_BIAS;
				return
			end
		end

		% Main IMU operation state.
		function run(this, gyros, seq, sample_time)
			% Compute the size of the seq increment (note that seq wraps modulo 128).
			dseq = mod(int16(seq) - int16(this.prevSeq), int16(128));

			% Remove the gyro bias from the measurements
			gyros = gyros - this.gyro_bias;

			% Rotate the delta angles from IMU coordinates into world coordinates
			gyros_world = this.imu_orient.rot(gyros);

			% Remove the Earth's rotation from the gyro measurements
			gyros_world = gyros_world - this.earth_rot;

			% Compute the angular velocities
			this.ang_vel = gyros_world / sample_time;

			% Rescale the gyro delta angles using dseq to make up for any missed cycles
			gyros_world = double(dseq) * gyros_world;

			% Execute the update to imu_orient
			this.imu_orient = Quat(gyros_world) * this.imu_orient;
		end

		% The main IMU update loop; contains a state machine to handle alignment
		% ang_vel is in IMU coordinates!
		function [imu_orient, local_orient, ang_vel, state, fail_reas] = ...
			update(this, gyros, accels, seq, sample_time, align_time_ms, align_gyro_tol, align_accel_tol, align_bias_tol, earth_rot_rate, latitude)
		%
			% Run the state machine iff we have new data
			if seq ~= this.prevSeq
				switch this.state
					case IMUSysState.INIT
						this.init(seq)

					case IMUSysState.ALIGN
						this.align(gyros(:), accels(:), sample_time, align_time_ms, align_gyro_tol, align_accel_tol, align_bias_tol, earth_rot_rate, latitude)

					case IMUSysState.RUN
						this.run(gyros(:), seq, sample_time)
				end

				% Update our stored values for the next iteration
				this.prevSeq = seq;
			end

			% Update the function outputs
			imu_orient   = this.imu_orient;
			local_orient = imu_orient * this.local_rel_imu;
			ang_vel      = this.ang_vel;
			state        = this.state;
			fail_reas    = this.fail_reas;
		end
	end

	properties
		% Accumulators for accelerometers and gyros for alignment
		align_gm = [0; 0; 0]
		align_rm = [0; 0; 0]

		% Elapsed alignment duration, in units of sample_time
		align_ticks = 0

		% Current angular velocity, in world coordinates
		ang_vel = [0; 0; 0]

		% Earth's angular velocity, radians per cycle
		earth_rot = [0; 0; 0]

		% Explanation for an alignment failure
		fail_reas = IMUFailReason.NONE

		% Gyro bias (radians/cycle)
		gyro_bias = [0; 0; 0]

		% The current orientation of the IMU coordinate frame (Quat)
		imu_orient

		% Orientation of the IMU within the ATRIAS coordinate frame
		imu_rel_local

		% Similarly, the orientation of the ATRIAS coordinate frame in the IMU frame
		local_rel_imu

		% Previous sequence value; kept to detect when new data is available and
		% to begin alignment at the correct time.
		prevSeq = uint8(0);

		% Current alignment state
		state = IMUSysState.INIT;
	end
end % classdef
