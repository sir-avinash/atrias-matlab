% This function evaluate the derivative of a expression
% using complex-step differentiation

function [val, dval] = cxdiff_eval(expr, ssize)
	val  = real(expr);
	dval = imag(expr) / ssize;
end
