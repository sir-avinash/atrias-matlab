% Step 3 of the alignment process (earth rotation and bias)

function earth_rot = align_biases(earth_rot_rate, latitude)
	% Compute the Earth's rotation vector for rotation cancellation
	earth_rot = earth_rot_rate * [0; cos(latitude); sin(latitude)];
end
