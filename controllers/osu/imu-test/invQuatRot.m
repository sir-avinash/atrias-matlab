% This function rotates by the negative of
% the given rotation quaternion's rotation.
function out = invQuatRot(quat, vec)
	out = quatMult(quatConj(quat), quatMult([0; vec], quat));
	out = out(2:end);
end
