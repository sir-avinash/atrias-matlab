classdef CmuCoords < handle
	methods
		function [roll,pitch,yaw,droll,dpitch,dyaw] = update(this, local_orient, ang_vel)
			% Get the local rotation matrix
			R = local_orient.toRotMat;

			% Transform the rotation matrix to yaw-pitch-roll
			yaw   = atan2( R(2,1), R(1,1));
			pitch = atan2(-R(3,1), sqrt(R(3,2)^2+R(3,3)^2));
			roll  = atan2( R(3,2), R(3,3));

			% Unwrap the yaw value
			this.curyaw = this.curyaw + mod(yaw - this.curyaw + pi, 2*pi) - pi;

			% The angular velocity in IMU coordinates is a linear function of the individual joint
			% velocities. This is the inverse function.
			jvels = [ cos(y)*tan(p), tan(p)*sin(y), 1
			          -sin(y),       cos(y),        0
			          cos(y)/cos(p), sin(y)/cos(p), 0 ] * ang_vel;

			% Grab the joint rates from the linear equation solution
			dyaw   = jvels(1);
			dpitch = jvels(2);
			droll  = jvels(3);
		end
	end

	properties
		curyaw = 0 % Unwrapped yaw value
	end
end
% vim: noexpandtab
