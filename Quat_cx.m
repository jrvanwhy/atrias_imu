% Time: 263.675
classdef Quat < handle
	methods
		% Basic constructor. Can construct by giving the 4 values or by giving a rotation
		% vector (which will be made into a rotation quaternion)
		function this = Quat(vec)
			switch numel(vec)
				case 2
					this.vals = vec;

				case 3
					angle = norm(vec);
					vals = [cos(angle/2); sin(angle/2) * vec(:) / angle];
					this.vals = [1 i] * vals([ 1 3
					                           2 4 ]);

				case 4
					this.vals = [1 i] * vec([ 1 3
					                          2 4 ]);

				otherwise
					error(['Input vector size ' num2str(numel(vec)) ' is invalid. Must be 3 or 4'])
			end
		end

		% Quaternion conjugation function
		function C = conj(this)
			C = Quat([conj(this.vals(1)), -this.vals(2)]);
		end

		% Quaternion multiplication function
		function C = mtimes(A, B)
			C = Quat([A.vals(1) * B.vals(1) - conj(B.vals(1)) * A.vals(2), B.vals(2) * A.vals(1) + A.vals(2) * conj(B.vals(1))]);
		end

		% Function to perform a vector rotation by the inverse of the rotation represented by this quaternion.
		function vec = invRot(this, vec)
			outQuat = this.conj * Quat([0; vec(:)]) * this;
			vec     = [imag(outQuat.vals(1)); real(outQuat.vals(2)); imag(outQuat.vals(2))];
		end

		% Function to rotate the given vector by the rotation represented
		% by this quaternion
		function vec = rot(this, vec)
			outQuat = this * Quat([0; vec(:)]) * this.conj;
			vec     = [imag(outQuat.vals(1)); real(outQuat.vals(2)); imag(outQuat.vals(2))];
		end
	end

	properties
		% The numeric values for this Quaternion
		vals
	end
end
