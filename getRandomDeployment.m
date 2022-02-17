function [sensor_x_pos_mul,sensor_y_pos_mul] = getRandomDeployment(nSensors,rLimit,sens_range)
%GETRANDOMDEPLOYMENT Get random deployment of sensors with a sensible
%orientation (where the normal of the line points to the LKP)
sens_range = 0.5*sens_range;
xPositions = unifrnd(-rLimit,rLimit,1,nSensors);
yPositions = unifrnd(-rLimit,rLimit,1,nSensors);
rSensor = vecnorm([xPositions;yPositions]);

% while any(rSensor>rLimit)
%     outOfBounds = rSensor>rLimit;
%     xPositions(outOfBounds) = unifrnd(-rLimit,rLimit,nnz(outOfBounds),1);
%     yPositions(outOfBounds) = unifrnd(-rLimit,rLimit,nnz(outOfBounds),1);
%     rSensor = vecnorm([xPositions';yPositions']);
% end


thetaSensor = atan2(yPositions,xPositions);
oriSensor = thetaSensor+pi/2;

% Defining sensor lines
sensor_x_coord1 = xPositions + sens_range.*cos(oriSensor);
sensor_x_coord2 = xPositions - sens_range.*cos(oriSensor);
sensor_y_coord1 = yPositions + sens_range.*sin(oriSensor);
sensor_y_coord2 = yPositions - sens_range.*sin(oriSensor);

sensor_x_pos_mul = horzcat(sensor_x_coord1',sensor_x_coord2');
sensor_y_pos_mul = horzcat(sensor_y_coord1',sensor_y_coord2');





end

%% view what this looks like
% [xP,yP,ori] = getRandomDeployment(10,10)
% figure
% scatter(xP,yP)
% hold on
% line = [-10, 10;0 0];
% for i = 1:length(ori)
% rotatedLine = rotMat(ori(i))*line;
% plot(rotatedLine(1,:)+xP(i),rotatedLine(2,:)+yP(i))
% plot([0,xP(i)],[0,yP(i)],':')
% end
% axis equal