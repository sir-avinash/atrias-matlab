% Time: 142.518
classdef Quat < handle
	methods
		% Basic constructor. Can construct by giving the 4 values or by giving a rotation
		% vector (which will be made into a rotation quaternion)
		function this = Quat(vec)
			switch numel(vec)
				case 3
					angle = norm(vec);
					vals = [cos(angle/2); sin(angle/2) * vec(:) / angle];

				case 4
					vals = vec;

				otherwise
					error(['Input vector size ' num2str(numel(vec)) ' is invalid. Must be 3 or 4'])
			end

			if size(vals) ~= [2 2]
				this.mat = [1 0; 0 1] * vals(1) + [i 0; 0 -i] * vals(2) + [0 1; -1 0] * vals(3) + [0 i; i 0] * vals(4);
			else
				this.mat = vals;
			end
		end

		% Quaternion conjugation function
		function C = conj(this)
			C = Quat(this.mat');
		end

		% Quaternion multiplication function
		function C = mtimes(A, B)
			C = Quat(A.mat * B.mat);
		end

		% Function to perform a vector rotation by the inverse of the rotation represented by this quaternion.
		function vec = invRot(this, vec)
			vecmat  = [i 0; 0 -i] * vec(1) + [0 1; -1 0] * vec(2) + [0 i; i 0] * vec(3);
			outQuat = this.mat' * vecmat * this.mat;
			vec     = [imag(outQuat(1)); real(outQuat(3)); imag(outQuat(3))];
		end

		% Function to rotate the given vector by the rotation represented
		% by this quaternion
		function vec = rot(this, vec)
			vecmat  = [i 0; 0 -i] * vec(1) + [0 1; -1 0] * vec(2) + [0 i; i 0] * vec(3);
			outQuat = this.mat * vecmat * this.mat';
			vec     = [imag(outQuat(1)); real(outQuat(3)); imag(outQuat(3))];
		end

		% Grab this quaternion as a vector of 4 real values
		function vals = getVals(this)
			vals = [real(this.mat(1)); imag(this.mat(1)); real(this.mat(3)); imag(this.mat(3))];
		end
	end

	properties
		% The numeric values for this Quaternion
		mat
	end
end
