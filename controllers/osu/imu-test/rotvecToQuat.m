% This function converts a rotation vector into a rotation quaternion
function quat = rotvecToQuat(vec)
	% The rotation angle
	ang = norm(vec);

	% The real and nonreal component multipliers
	m_real    = cos(ang/2);
	m_nonreal = sin(ang/2);

	% Assemble the quaternion
	quat = [m_real; m_nonreal*ones(3,1)] .* [1; vec/ang];
end
