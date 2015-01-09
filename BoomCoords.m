classdef BoomCoords < handle
	methods
		function [yaw] = update(this, local_orient, state, boomYaw)
			% Do a bit more calibration during alignment.
			if state == IMUSysState.ALIGN
				this.yawOff = boomYaw - this.yaw;
			end

			% For yaw, we rotate the local X vector into the world frame.
			% We use a cross product to smoothly update the yaw value without jumping
			localX_world = local_orient.rot([1; 0; 0]);
			localX_hproj = localX_world;
			localX_hproj(3) = 0;
			localX_hproj    = localX_hproj / norm(localX_hproj);
			yawupsin        = cross(localX_hproj, [cos(this.yaw); -sin(this.yaw); 0]);
			this.yaw        = this.yaw + asin(yawupsin(3));

			% Set this function's yaw output, as it's not directly derived.
			yaw = this.yaw + this.yawOff;
		end
	end

	properties
		yaw    = 0
		yawOff = 0
	end
end
