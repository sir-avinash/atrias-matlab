% Step 2 of the alignment process (heading)

function imu_orient = align_corrhdg(imu_orient, heading)
	% Rotate the IMU's Z vector into world coordinates
	imuZ_world = imu_orient.rot([0; 0; 1]);

	% Compute the rotation angle for the correction
	theta = atan2(imuZ_world(2), -imuZ_world(1)) - heading;

	% Perform the update
	imu_orient = imu.Quat(theta * [0; 0; 1]) * imu_orient;
end
