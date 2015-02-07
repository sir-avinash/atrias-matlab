% This checks if our motion is within the alignment tolerances.

function is_moving = checkMotion(params, gyros, accels)
	is_moving = (norm(gyros) >= params.align_gyro_tol || abs(norm(accels) - 1) >= params.align_accel_tol);
end
