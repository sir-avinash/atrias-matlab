classdef CmuCoords < handle
	methods
		function [roll,pitch,yaw,droll,dpitch,dyaw] = update(this, local_orient, ang_vel)
			% Get the local rotation matrix
			R = local_orient.toRotMat;

			% Transform the rotation matrix to yaw-pitch-roll
			yaw   = atan2( R(2,1), R(1,1));
			pitch = atan2(-R(3,1), sqrt(R(3,2)^2+R(3,3)^2));
			roll  = atan2( R(3,2), R(3,3));

			% TODO: Unwrap Yaw

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
% vim: noexpandtab
