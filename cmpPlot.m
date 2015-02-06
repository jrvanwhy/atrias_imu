% This function creates a comparison plot between the boom data and the IMU data.
% Note that it will not create a new figure for you, and will leave hold on!
%
% Parameters:
%     app    An AtriasPostProcess instance with the test's data. Must contain imu_data, boomQ, and boomDQ (in that order) in the controller data (as Mikhail's controller does).
%     coord  Numeric coordinate. 1 = roll, 2 = yaw, 3 = pitch, 4 = roll rate, 5 = yaw rate, 6 = pitch rate
%     boomLs Boom's line style (for plot())
%     imuLs  IMU's line style (for plot())
%
% Usage examples:
%     cmpPlot(AtriasPostProcess(state, time), 1, '.-', '.-r') % Plot the rolls, with line style '.-' for the boom data and '.-r' for the IMU data
%     cmpPlot(A, 5, '.-', '.-r')                              % Using an existing AtriasPostProcess instance, plot the yaw rate with the sameline styles as the previous example

function cmpPlot(app, coord, boomLs, imuLs)
	% Pick out the boom data to plot based on the specified coordinate
	boomDataOrd = {'RollAngle', 'YawAngle', 'PitchAngle', 'RollVelocity', 'YawVelocity', 'PitchVelocity'};
	boomData = app.(['boom' boomDataOrd{coord}]);

	% Pick out the imu data to plot based on the specified coordinate
	imuData = app.controllerData(:, end-6 + coord);

	% Plot the boom data
	plot(app.time, boomData, boomLs)

	% Plot the IMU data
	hold on
	plot(app.time, imuData, imuLs)
end
