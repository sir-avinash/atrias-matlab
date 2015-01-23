classdef BoomCoords < handle
	methods
		function [roll,pitch,yaw,droll,dpitch,dyaw] = update(this, local_orient, ang_vel)
			% Get the local orientation quaternion as 4 real numbers
			orient = local_orient.getVals;

			% We can trivially and directly grab roll from the corresponding rotation matrix.
			roll = asin(2 * dot(orient([1 3]), orient([2 4])));

			% Grab pitch next.
			pitch = -asin(2 * (orient(2)*orient(4) - orient(1)*orient(3)) / cos(roll));

			% Yaw's more difficult, because we need to de-wrap it. Produce the horizontally-projected yaw vector
			yaw_vec = [[1, -1, 1, -1] * orient.^2; 2*(orient(1)*orient(4) - orient(2)*orient(3))] / cos(roll);

			% Perform the yaw update
			yaw_vec = [ cos(this.yaw) -sin(this.yaw)
			            sin(this.yaw)  cos(this.yaw) ] * yaw_vec;
			yawTheta = atan2(yaw_vec(1), yaw_vec(2));
			this.yaw = this.yaw + yawTheta;

			% Set the final yaw output
			yaw = this.yaw;

			% The angular velocity in IMU coordinates is a linear function of the individual joint
			% velocities. This is the inverse function.
			jvels = [ sin(yaw),           cos(yaw),            0
			         -cos(yaw)/cos(roll), sin(yaw)/cos(roll),  0
			         -cos(yaw)*tan(roll), sin(yaw)*tan(roll), -1 ] * ang_vel;

			% Grab the joint rates from the linear equation solution
			droll = jvels(1);
			dpitch = jvels(2);
			dyaw   = jvels(3);
			roll  = roll + this.boomRollOffset;
		end
	end

	properties
		yaw = 0
		boomRollOffset  = 0.127      % Angle between the physical boom and the virtual boom. Units: radians
	end
end
