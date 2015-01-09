clear all
close all

imu = IMUSys;
bc  = BoomCoords;

load('imu_test_walk')
A = AtriasPostProcess(state, time);

% Figure out when our data cuts out
len = 1 + find(diff(A.controllerData(:, 7)), 1, 'last');

% Pre-allocate things
imu_states   = zeros(len, 1  );
align_accums = zeros(3,   len);
accum_norms  = zeros(len, 1  );
test_out     = zeros(3,   len);
rolls        = zeros(len, 1  );
pitches      = zeros(len, 1  );
yaws         = zeros(len, 1  );

% Simulate!!!
for iter = 1:len
	if mod(iter, 1000) == 0
		disp(['Completion: ' num2str(iter / len)])
	end
	[~,local_orient,state2] = imu.update(A.controllerData(iter, 1:3), A.controllerData(iter, 4:6), A.controllerData(iter,7), .001);
	imu_states(iter) = imu.state;
	align_accums(:, iter) = imu.align_accum;
	accum_norms(iter)     = norm(imu.align_accum);
	[rolls(iter),pitches(iter),yaws(iter)] = bc.update(local_orient, state2, A.boomRollAngle(iter), A.boomYawAngle(iter));
end
