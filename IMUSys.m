classdef IMUSys < handle
	methods
		% Constructor -- does some basic init that can't directly be done to the properties themselves
		function this = IMUSys
			% The local orientation is initially equal to the world orientation.
			% This IMU is two simple rotations away from the local orientation.
			this.imu_orient = Quat([0; pi/4; 0]) * Quat([-pi/2; 0; 0]);
		end

		% Initialization state update function. Waits until we have new data.
		% Note that alignment should not begin until the second cycle of data
		% as the first cycle can have a large spike.
		function init(this, seq)
			% We wish to remain in init if seq is stuck at any value.
			% Since we initialize prevSeq to 0, if seq starts at a nonzero (constant)
			% value, then we seq \neq prevSeq and prevSeq \neq 0. This will likely add a millisecond
			% to the startup time, but was chosen in order to remain conservative and not accidentally perform a bad
			% align.
			if this.prevSeq ~= 0
				this.state = IMUSysState.ALIGN;
			end
		end

		% Alignment update function. This accumulates accelerometer readings in order to properly level the IMU at startup.
		function align(this, accels, sample_time)
			this.align_accum = this.align_accum + accels;

			% TODO: Catch a bad alignment ("large" accelerations or angular velocities)
			% and enter an "invalid alignment" state.

			% Quit if the alignment isn't done yet.
			% To avoid needing an extra state variable, just accumulate
			% until the accumulated acceleration magnitude exceeds 1 g * align time
			align_time = 10; % seconds
			if sample_time * norm(this.align_accum) < align_time
				return
			end

			% Alignment complete!

			% Set the state for the next iteration
			this.state = IMUSysState.RUN;

			% First, normalize the accumulated acceleration vector
			this.align_accum = this.align_accum / norm(this.align_accum);

			% Rotate the vector using the previous-known IMU orientation.
			this.align_accum = this.imu_orient.rot(this.align_accum);

			% Use a cross product to compute the rotation axis for the
			% orientation correction. This will have a length which is the sin
			% of the necessary correction rotation angle.
			orient_corr = cross(this.align_accum, [0; 0; 1]);

			% Catch the (rare!) case where no correction is necessary.
			if orient_corr == 0
				return
			end

			% Turn the previous cross product into a true axis and angle
			angle = asin(norm(orient_corr));
			axis  = orient_corr / norm(orient_corr);

			% Apply the correction to the orientation
			this.imu_orient = Quat(angle * axis) * this.imu_orient;
		end

		% Main IMU operation state. TODO
		function run(this)
			% TODO: this
		end

		% The main IMU update loop; contains a state machine to handle alignment
		function update(this, gyros, accels, seq, sample_time)
			% Run the state machine iff we have new data
			if seq ~= this.prevSeq
				switch this.state
					case IMUSysState.INIT
						this.init(seq)

					case IMUSysState.ALIGN
						this.align(accels(:), sample_time)

					case IMUSysState.RUN
						this.run
				end

				% Update our stored values for the next iteration
				this.prevSeq = seq;
			end
		end
	end

	properties
		% Accumulator for the accelerometer-based leveling for aligment
		align_accum = zeros(3, 1);

		% The current orientation of the IMU coordinate frame (Quat)
		imu_orient

		% Previous sequence value; kept to detect when new data is available and
		% to begin alignment at the correct time.
		prevSeq = uint8(0);

		% Current alignment state
		state = IMUSysState.INIT;
	end
end % classdef
