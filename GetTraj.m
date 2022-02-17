function [xdata ydata] = TrajGen(nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y)
% A function that outputs the simulated target trajectories based on the defined motion parameters


%% Create motion parameters
xyT = zeros(2,nT); % x and y position of target T
travelSinceLastChangeT = zeros(1,nT); % distance travelled by target T since last change in direction
vT = normrnd(vTmean,vTstd,1,nT); % velocity of target T

% Code to correct if velocity out of bounds
outOfBounds = or(vT>vTmean+3*vTstd,vT<0); % determine if velocity of target T is out of bounds (2*vTmean)
while any(outOfBounds)
    vT(outOfBounds) = normrnd(vTmean,vTstd,size(vT(outOfBounds)));
    outOfBounds = or(vT>2*vTmean,vT<0);
end

moveT = movementModel(xyT); % Gives the movement direction and distance target T will travel (using normal distribution)

%% Create target paths
xdata = zeros(size(xyT,2),length(times)); 
ydata = zeros(size(xyT,2),length(times));
    
for timeIndex = 1:length(times) % for loop on time interval steps
    
    % give new direction to the targets who need it
    needNewDirection = travelSinceLastChangeT > vecnorm(moveT); % if travel since last change is greater than 
    if nnz(needNewDirection)>0 % nnz is number of nonzero matrix elements in matrix (needNewDirection)
        moveT(:,needNewDirection) = movementModel(xyT(:,needNewDirection));
        travelSinceLastChangeT(needNewDirection) = 0;
    end
    
    % Determine new position, after time increment, for each target
    deltad = timeStep*vT; % Distance travelled in time step
    travelSinceLastChangeT = travelSinceLastChangeT + deltad; % Distance travelled since last change in direction for target T
    xyT = xyT+moveT./repmat(vecnorm(moveT),2,1).*repmat(deltad,2,1); % New position for target T
    
    % Saving positional data for target motion (based on time increments)
    xdata(:,timeIndex) = xyT(1,:); % saves the X data for all targets at the current time step as a column in xplot
    ydata(:,timeIndex) = xyT(2,:);
    
end

xdata = xdata + LKP_x;
ydata = ydata + LKP_y;

end