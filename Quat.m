classdef Quat < handle
	methods
		% Basic constructor. Can construct by giving the 4 values or by giving a rotation
		% vector (which will be made into a rotation quaternion)
		function this = Quat(vec)
			switch numel(vec)
				case 3
					angle = norm(vec);
					this.vals = [cos(angle/2); sin(angle/2) * vec(:) / angle];

				case 4
					this.vals = vec;

				otherwise
					error(['Input vector size ' num2str(numel(vec)) ' is invalid. Must be 3 or 4'])
			end
		end

		% Quaternion conjugation function
		function C = conj(this)
			C = Quat(this.vals .* [1; -1; -1; -1]);
		end

		% Quaternion multiplication function
		function C = mtimes(A, B)
			C = Quat([ A.vals(1)*B.vals(1) - A.vals(2)*B.vals(2) - A.vals(3)*B.vals(3) - A.vals(4)*B.vals(4)
			           A.vals(1)*B.vals(2) + A.vals(2)*B.vals(1) + A.vals(3)*B.vals(4) - A.vals(4)*B.vals(3)
			           A.vals(1)*B.vals(3) - A.vals(2)*B.vals(4) + A.vals(3)*B.vals(1) + A.vals(4)*B.vals(2)
			           A.vals(1)*B.vals(4) + A.vals(2)*B.vals(3) - A.vals(3)*B.vals(2) + A.vals(4)*B.vals(1) ]);
		end

		% Function to rotate the given vector by the rotation represented
		% by this quaternion
		function vec = rot(this, vec)
			outQuat = this * Quat([0; vec(:)]) * this.conj;
			vec     = outQuat.vals(2:4);
		end
	end

	properties
		% The numeric values for this Quaternion
		vals
	end
end
