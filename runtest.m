clear all
close all

imu = IMUSys;
bc  = BoomCoords;

latitude = 44.5673031 * pi/180; % The DRL's latitude, according to Google Maps
earth_rot_rate  = 7.292115e-5 * .001; % Earth's rotation rate, rad/millisecond. From WolframAlpha
align_bias_tol  = 2e-8;               % Tolerance on the gyro bias. Radians per millisecond

load('~/imu_test_walk')

startTime = 2.023e6 - 2 * 60 * 1000;
state = state(startTime:end, :);
time  = time(startTime:end);
len = 4 * 60 * 1000;
state = state(1:len, :);
time  = time(1:len);

A = AtriasPostProcess(state, time);

% Figure out when our data cuts out
%len = 1 + find(diff(A.controllerData(:, 7)), 1, 'last');
%len = numel(time);

% Pre-allocate things
imu_states   = zeros(len, 1  );
test_out     = zeros(3,   len);
rolls        = zeros(len, 1  );
pitches      = zeros(len, 1  );
yaws         = zeros(len, 1  );
drolls       = zeros(len, 1  );
dpitches     = zeros(len, 1  );
dyaws        = zeros(len, 1  );

% Simulate!!!
for iter = 1:len
	if mod(iter, 1000) == 0
		disp(['Completion: ' num2str(iter / len)])
	end
	[imu_orient,local_orient,ang_vel,state2] = ...
		imu.update(A.controllerData(iter, 1:3), A.controllerData(iter, 4:6), A.controllerData(iter,7), .001, 1 * 60 * 1000, 1e-6, .02, align_bias_tol, earth_rot_rate, latitude);
	imu_states(iter) = imu.state;
	[rolls(iter),pitches(iter),yaws(iter),drolls(iter),dpitches(iter),dyaws(iter)] = ...
		bc.update(imu_orient, local_orient, ang_vel, state2, A.boomRollAngle(iter), A.boomPitchAngle(iter), A.boomYawAngle(iter));
end
