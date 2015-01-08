% This function initializes a variable for complex step differentiation.
% You give it the point to differentiate it, the direction to differentiate in,
% and the (relative) step size with which to differentiate

function cx_x = cxdiff_init(x, dx, ssize)
	cx_x = x + dx * ssize * 1i;
end
