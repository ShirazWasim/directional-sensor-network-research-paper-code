function [ move ] = movementModel0(currentPoss)
%MOVEMENTMODEL A model for movement given the current position. The move is
%a vector of x,y with maximum magnitude 1. This will be
%multiplied by the maximum step size to give the next move
%   In
% currentPos - current position
%   Out
% move - the move to take next from the current position

[thetas, ~] = cart2pol(currentPoss(1,:),currentPoss(2,:));
centerIndices = find(vecnorm(currentPoss) == 0);
thetas(centerIndices) = unifrnd(0,2*pi,1,length(centerIndices)); % Giving angles to the trajectories still at the center position

[movexs, moveys] = pol2cart(normrnd(thetas,pi/3),unifrnd(0,100,size(thetas)));
move = [movexs; moveys];

end

