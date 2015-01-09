classdef BoomCoords < handle
	methods
		function [roll,pitch,yaw,droll,dpitch,dyaw] = update(this, imu_orient, local_orient, ang_vel, state, boomRoll, boomPitch, boomYaw)
			% Get the local orientation quaternion as 4 real numbers
			orient = local_orient.getVals;

			% We can trivially and directly grab roll from the corresponding rotation matrix.
			roll = asin(2 * dot(orient([1 3]), orient([2 4])));

			% Grab pitch next.
			pitch = -asin(2 * (orient(2)*orient(4) - orient(1)*orient(3)) / cos(roll));

			% Yaw's more difficult, because we need to de-wrap it. Produce the horizontally-projected yaw vector
			yaw_vec = [[1, -1, 1, -1] * orient.^2; 2*(orient(1)*orient(4) - orient(2)*orient(3)); 0] / cos(roll);

			% Perform the yaw update using a cross product
			yawCross = cross(yaw_vec, [cos(this.yaw); -sin(this.yaw); 0]);
			this.yaw = this.yaw + asin(yawCross(3));

			% Set the final yaw output
			yaw = this.yaw;

			% The angular velocity in IMU coordinates is a linear function of the individual joint
			% velocities. This is the inverse function.
			ang_vel_world = imu_orient.rot(ang_vel);
			jvels = [ cos(yaw),           -sin(yaw),            0
			          sin(yaw)/cos(roll),  cos(yaw)/cos(roll),  0
			          sin(yaw)*tan(roll),  cos(yaw)*tan(roll), -1 ] * ang_vel_world;

			% Grab the joint rates from the linear equation solution
			droll = jvels(1);
			dpitch = jvels(2);
			dyaw   = jvels(3);

			% Save some offsets to match closely with the boom's angles themselves.
			if this.prevState == IMUSysState.ALIGN
				this.rollOff  = boomRoll - roll;
				this.pitchOff = boomPitch - pitch;
				this.yawOff   = boomYaw   - yaw;
			end
			roll  = roll + this.rollOff;
			pitch = pitch + this.pitchOff;
			yaw   = yaw   + this.yawOff;
			this.prevState = state;
		end
	end

	properties
		prevState = 0
		yaw = 0
		rollOff = 0
		pitchOff = 0
		yawOff   = 0
	end
end
