classdef BoomCoords < handle
	methods
		function [roll,pitch,yaw] = update(this, local_orient, state, boomRoll, boomYaw)
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

			% Roll section. Simply project the local Y vector into the world frame,
			% then look at the resulting Z coordinate.
			localY_world = local_orient.rot([0; 1; 0]);
			imu_roll     = asin(localY_world(3));
			roll         = imu_roll + this.rollOff;

			% Do a bit more calibration during alignment.
			if this.prevState == IMUSysState.ALIGN
				this.yawOff = boomYaw - this.yaw;
				this.rollOff = boomRoll - imu_roll;
			end

			% Save the state for the next iteration
			this.prevState = state;

			% Pitch section. Take the local Z coordinate, un-roll it, then un-yaw it to get the pitch.
			pitch_trans  = Quat([0; 0; this.yaw]) * Quat(-imu_roll * [cos(this.yaw); -sin(this.yaw); 0]) * local_orient;
			localZ_pitch = pitch_trans.rot([0; 0; 1]);
			pitch        = atan2(localZ_pitch(1), localZ_pitch(3));
		end
	end

	properties
		prevState = 0
		yaw    = 0
		yawOff = 0
		rollOff = 0
	end
end
