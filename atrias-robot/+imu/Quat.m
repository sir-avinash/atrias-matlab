classdef Quat
	methods
		% Basic constructor. Can construct by giving the 4 values or by giving a rotation
		% vector (which will be made into a rotation quaternion)
		function this = Quat(vec)
			switch numel(vec)
				case 3
					angle = norm(vec);

					% Check for a zero rotation angle to avoid a divide by zero
					if angle > 0
						this.vals = [cos(angle/2); sin(angle/2) * vec(:) / angle];
					else
						this.vals = [1; 0; 0; 0];
					end

				case 4
					this.vals = vec(:);

				otherwise
					error(['Input vector size ' num2str(numel(vec)) ' is invalid. Must be 3 or 4'])
			end
		end

		% Quaternion conjugation function
		function this = conj(this)
			this.vals = this.vals .* [1; -1; -1; -1];
		end

		% Quaternion multiplication function
		function A = mtimes(A, B)
			A.vals = (A.vals([1 2 3 4; 2 1 4 3; 3 4 1 2; 4 3 2 1]) .* [1 -1 -1 -1; 1 1 -1 1; 1 1 1 -1; 1 -1 1 1]) * B.vals;
		end

		% Function to perform a vector rotation by the inverse of the rotation represented by this quaternion.
		function vec = invRot(this, vec)
			vec = (this.vals([2 3 4; 1 4 3; 4 1 2; 3 2 1]) .* [1 1 1; 1 1 -1; -1 1 1; 1 -1 1]) * vec;
			vec = (vec([2 1 4 3; 3 4 1 2; 4 3 2 1]) .* [1 1 -1 1; 1 1 1 -1; 1 -1 1 1]) * this.vals;
		end

		% Function to rotate the given vector by the rotation represented
		% by this quaternion
		function vec = rot(this, vec)
			vec = (this.vals([2 3 4; 1 4 3; 4 1 2; 3 2 1]) .* [-1 -1 -1; 1 -1 1; 1 1 -1; -1 1 1]) * vec;
			vec = (vec([2 1 4 3; 3 4 1 2; 4 3 2 1]) .* [1 -1 1 -1; 1 -1 -1 1; 1 1 -1 -1]) * this.vals;
		end

		% Grab this quaternion as a vector of 4 real values
		function vals = getVals(this)
			vals = this.vals;
		end

		% Convert this quaternion into a rotation matrix
		% This was derived (and verified) symbolically
		function rotMat = toRotMat(this)
			vals   = this.vals;
			rotMat = [ vals(1)^2 + vals(2)^2 - vals(3)^2 - vals(4)^2, 2*vals(2)*vals(3) - 2*vals(1)*vals(4),         2*vals(2)*vals(4) + 2*vals(1)*vals(3)
			           2*vals(2)*vals(3) + 2*vals(1)*vals(4),         vals(1)^2 - vals(2)^2 + vals(3)^2 - vals(4)^2, 2*vals(3)*vals(4) - 2*vals(1)*vals(2)
			           2*vals(2)*vals(4) - 2*vals(1)*vals(3),         2*vals(3)*vals(4) + 2*vals(1)*vals(2),         vals(1)^2 - vals(2)^2 - vals(3)^2 + vals(4)^2 ];
		end
	end

	properties
		% The numeric values for this Quaternion
		vals
	end
end
