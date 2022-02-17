function [sensor_x_pos_mul,sensor_y_pos_mul] = getRingOfFire(nSensors,rSensors)
%GETRINGOFFIRE Get ring of fire positions and orientations for sensors 
%   Detailed explanation goes here

rSensors = 0.5*rSensors;

anglePerSensor = 2*pi/nSensors;
radialPosition = rSensors/tan(anglePerSensor/2);
indices = 1:(nSensors);
angularPositions = indices*anglePerSensor;
[xPositions, yPositions] = pol2cart(angularPositions,radialPosition);
orientations = angularPositions+pi/2;

% Defining sensor lines
sensor_x_coord1 = xPositions + rSensors.*cos(orientations);
sensor_x_coord2 = xPositions - rSensors.*cos(orientations);
sensor_y_coord1 = yPositions + rSensors.*sin(orientations);
sensor_y_coord2 = yPositions - rSensors.*sin(orientations);

sensor_x_pos_mul = horzcat(sensor_x_coord1',sensor_x_coord2');
sensor_y_pos_mul = horzcat(sensor_y_coord1',sensor_y_coord2');

end

%% view what this looks like
% [xP,yP,ori] = getRingOfFire(10,10)
% figure
% scatter(xP,yP)
% hold on
% line = [-10, 10;0 0];
% for i = 1:length(ori)
% rotatedLine = rotMat(ori(i))*line;
% plot(rotatedLine(1,:)+xP(i),rotatedLine(2,:)+yP(i))
% end