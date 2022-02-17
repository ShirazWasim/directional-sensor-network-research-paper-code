function [traj_xdata1, traj_ydata1,traj_xdata2,traj_ydata2] = GetBoundedBoxTraj2(traj_xdata, traj_ydata, sensor_x_pos_mul, sensor_y_pos_mul,margin)
% Finds the set of trajectories that pass through a bounding box around the sensor position

x_uB = max(sensor_x_pos_mul) + margin;
x_lB = min(sensor_x_pos_mul) - margin;
y_uB = max(sensor_y_pos_mul) + margin;
y_lB = min(sensor_y_pos_mul) - margin;

if x_uB == x_lB
    x_lB = max(sensor_x_pos_mul) - margin;
    x_uB = min(sensor_x_pos_mul) + margin;
end

if y_uB == y_lB
    y_lB = max(sensor_y_pos_mul) - margin;
    y_uB = min(sensor_y_pos_mul) + margin;
end

%%
% Find trajectories that have values within the required bounds
traj_checkx2 = and(x_lB <= traj_xdata , traj_xdata <= x_uB);
traj_checky2 = and(y_lB <= traj_ydata , traj_ydata <= y_uB);

a = and(traj_checkx2,traj_checky2); % Checking if a trajectory point is within both the x and y bounds
rows = sum(a,2) > 0; % Finding the rows of the trajectories that have at least one point within both bounds

%%
% Finding complete trajectories that have a value within the bounded region
row_idx = find(rows > 0);
traj_xdata1 = traj_xdata(row_idx,:);
traj_ydata1 = traj_ydata(row_idx,:);


%% 
% % Only parts of the trajectory that are within the bounds
% traj_xdata2 = zeros(size(traj_xdata,1),size(traj_xdata,2));
% traj_ydata2 = zeros(size(traj_ydata,1),size(traj_ydata,2));
% 
% % Finding indices of the nonzero values
% lin_idx = find(a > 0);
% 
% % Setting those indices in new matrix
% traj_xdata2(lin_idx) = traj_xdata(lin_idx);
% traj_ydata2(lin_idx) = traj_ydata(lin_idx);
% 
% % Removing zeroed trajectories
% traj_xdata2 = traj_xdata2(row_idx,:);
% traj_ydata2 = traj_ydata2(row_idx,:);

%%
traj_xdata2 = traj_xdata.*a;
traj_ydata2 = traj_ydata.*a;

traj_xdata2 = traj_xdata2(row_idx,:);
traj_ydata2 = traj_ydata2(row_idx,:);

end