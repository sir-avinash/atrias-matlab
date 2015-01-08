% This function rotates a vector by the given quaternion
function out = quatRot(quat, vec)
	out = quatMult(quatMult(quat, [0; vec]), quatConj(quat));
	out = out(2:end);
end
