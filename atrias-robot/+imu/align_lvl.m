% Step 1 of the alignment process, leveling

function [imu_orient, fail_reas] = align_step1(imu_orient, align_gm)
	% Make MATLAB not error out if we return early
	imu_orient = imu_orient;
	fail_reas  = imu.IMUFailReason.NONE;

	% First, rotate the acceleration vector using the previous-known IMU orientation.
	ghat = imu_orient.rot(align_gm);

	% Next, normalize the accumulated acceleration vector
	u_g = ghat / norm(ghat);

	% If a correction greater than (or equal to) 90 degrees is necessary, fail.
	% (robot upside down?)
	if u_g(3) <= 0
		fail_reas = imu.IMUFailReason.BAD_GACCEL;
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
	imu_orient = imu.Quat(theta * axis) * imu_orient;
end
