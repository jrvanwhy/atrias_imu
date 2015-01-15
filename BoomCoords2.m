classdef BoomCoords2 < handle
	methods
		function [roll,pitch,yaw,droll,dpitch,dyaw] = update(this, imu_orient, local_orient, ang_vel, state, boomRoll, boomPitch, boomYaw)
			% Get the local orientation quaternion as 4 real numbers
			orient = local_orient.getVals;

			% We can trivially and directly grab roll from the corresponding rotation matrix.
			roll = asin(2 * dot(orient([1 3]), orient([2 4])));

			% Grab pitch next.
			pitch = -asin(2 * (orient(2)*orient(4) - orient(1)*orient(3)) / cos(roll));

			% Yaw's more difficult, because we need to de-wrap it. Produce the horizontally-projected yaw vector
			yaw_vec = [[1, -1, 1, -1] * orient.^2; 2*(orient(1)*orient(4) - orient(2)*orient(3))] / cos(roll);

			% Perform the yaw update
			yawTheta = atan2(dot(yaw_vec, [-sin(this.yaw); -cos(this.yaw)]), dot(yaw_vec, [cos(this.yaw); -sin(this.yaw)]));
			this.yaw = this.yaw + yawTheta;

			% Set the final yaw output
			yaw = this.yaw;

			% The angular velocity in IMU coordinates is a linear function of the individual joint
			% velocities. This is the inverse function.
			jvels = [ cos(yaw),           -sin(yaw),            0
			          sin(yaw)/cos(roll),  cos(yaw)/cos(roll),  0
			          sin(yaw)*tan(roll),  cos(yaw)*tan(roll), -1 ] * ang_vel;

			% Grab the joint rates from the linear equation solution
			droll = jvels(1);
			dpitch = jvels(2);
			dyaw   = jvels(3);

			% Save some offsets to match closely with the boom's angles themselves.
			if this.prevState == IMUSysState.ALIGN
				this.rollOff  = boomRoll - roll;
				this.yawOff   = boomYaw   - yaw;
			end
			roll  = roll + this.rollOff;
			yaw   = yaw   + this.yawOff;
			this.prevState = state;
		end
	end

	properties
		prevState = IMUSysState.INIT
		yaw = 0
		rollOff = 0
		yawOff   = 0
	end
end
