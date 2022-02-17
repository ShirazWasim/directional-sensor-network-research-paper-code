function [sensor_x_pos_mul,sensor_y_pos_mul] = getUniformDeployment(nSensors,rSensors,rLimit)
rSensors = 0.5*rSensors;

%GETUNIFORMDEPLOYMENT Get a close to uniform deployment
%   Detailed explanation goes here
x0 = rand(nSensors,2); x0(:,2) = x0(:,2)*2*pi;
ub = ones(size(x0)); ub(:,2) = inf;
lb = -ones(size(x0)); lb(:,2) = -inf;
% settleCount = 0;
% bestRT = 0; bestF = 0;
options = optimoptions('fmincon','MaxFunctionEvaluations',60e3);
% while settleCount < 10
    [rt,f] = fmincon(@squareSumMin6ObjFuncRT,x0,[],[],[],[],lb,ub,[],options);
%     if f < bestF
%         bestF = f;
%         bestRT = rt;
%     else
%         settleCount = settleCount+1;
%     end
% end
xy = zeros(size(rt));
[xPositions, yPositions] = pol2cart(rt(:,2),rt(:,1));
orientations = rt(:,2)+pi/2;

xPositions = rLimit*xPositions;
yPositions =rLimit*yPositions;

% Defining sensor lines
sensor_x_coord1 = xPositions + rSensors.*cos(orientations);
sensor_x_coord2 = xPositions - rSensors.*cos(orientations);
sensor_y_coord1 = yPositions + rSensors.*sin(orientations);
sensor_y_coord2 = yPositions - rSensors.*sin(orientations);

sensor_x_pos_mul = horzcat(sensor_x_coord1,sensor_x_coord2);
sensor_y_pos_mul = horzcat(sensor_y_coord1,sensor_y_coord2);
end

function [val] = squareSumMin6ObjFunc(xy)
dists = pdist2(xy,xy);
sortedDists = sort(dists,1);
errors = sortedDists(1:7,:);
sumSquareDists = sum(sum(errors.^2));
val = -sumSquareDists;
end

function [val] = squareSumMin6ObjFuncRT(rt)
xy = zeros(size(rt));
[xy(:,1), xy(:,2)] = pol2cart(rt(:,2),rt(:,1));
dists = pdist2(xy,xy);
sortedDists = sort(dists,1);
errors = sortedDists(1:3,:);
sumSquareDists = sum(sum(errors));
val = -(sumSquareDists);
end

function [val] = minObjFunc(xy)
dists = pdist(xy);
val = -min(dists);
end

function [val] = minObjFuncRT(rt)
xy = zeros(size(rt));
[xy(:,1), xy(:,2)] = pol2cart(rt(:,2),rt(:,1));
dists = pdist(xy);
val = -min(dists);
end