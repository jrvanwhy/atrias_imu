clear all
close all

imu = IMUSys;
bc  = BoomCoords;

load('imu_test_walk')
A = AtriasPostProcess(state, time);

% Pre-allocate things
imu_states = zeros(numel(time), 1);
align_accums = zeros(3, numel(time));
accum_norms  = zeros(numel(time), 1);
test_out     = zeros(3, numel(time));
yaws         = zeros(numel(time), 1);

% Simulate!!!
for iter = 1:numel(time)
	if mod(iter, 1000) == 0
		disp(['Completion: ' num2str(iter / numel(time))])
	end
	[~,local_orient,state] = imu.update(A.controllerData(iter,1:3), A.controllerData(iter,4:6), A.controllerData(iter,7), .001);
	imu_states(iter) = imu.state;
	align_accums(:, iter) = imu.align_accum;
	accum_norms(iter)     = norm(imu.align_accum);
	yaws(iter) = bc.update(local_orient, state, A.boomYawAngle(iter));
end
