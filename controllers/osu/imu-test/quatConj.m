% This function evaluates the conjugate of a quaternion, given as a 4x1 vector.
function conj = quatConj(quat)
	conj = [1; -1; -1; -1] .* quat;
end
