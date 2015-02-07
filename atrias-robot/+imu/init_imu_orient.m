% This returns the initial value for the IMU's orientation quaternion

function imu_orient = init_imu_orient()
	imu_orient = imu.Quat([0; pi/4; 0]) * imu.Quat([-pi/2; 0; 0]);
end
