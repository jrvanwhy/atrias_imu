classdef Quat < handle
	methods
		% Basic constructor. Can construct by giving the 4 values or by giving a rotation
		% vector (which will be made into a rotation quaternion)
		function this = Quat(vec)
			switch numel(vec)
				case 3
					error('TODO: This') % TODO

				case 4
					error('TODO: This') % TODO
				default
					error(['Input vector size ' num2str(numel(vec)) ' is invalid. Must be 3 or 4'])
			end
		end
	end

	properties
		% The numeric values for this Quaternion
		vals
	end
end
