% This checks if our motion is within the alignment tolerances.

function is_moving = checkMotion(params, gyros, accels)
	is_moving = (sum(gyros.^2) >= params.align_gyro_tol^2 | abs(sqrt(sum(accels.^2)) - 1) >= params.align_accel_tol);
end
