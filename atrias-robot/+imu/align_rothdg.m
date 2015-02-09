% Step 2 of the full alignment: heading determination

function imu_orient = align_rothdg(imu_orient, align_rm)
	% Compute the world-relative rotation into the world frame using the current orientation estimate
	rm_world = imu_orient.rot(align_rm);

	% Find the rotation angle for the correction
	theta = atan2(rm_world(1), rm_world(2));

	% Build and apply the correction rotation
	imu_orient = imu.Quat(theta * [0; 0; 1]) * imu_orient;
end
