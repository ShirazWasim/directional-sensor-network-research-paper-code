function [traj_xdata_new traj_ydata_new] = GetNewTraj(traj_xdata, traj_ydata, sensor_x_pos, sensor_y_pos, deployment_time, nT, vTmean, vTstd, times, timeStep, startTime, endTime, sensor_index, margin,LKP_x, LKP_y, obstacles_check, num_obstacles, adaptive)
% This outputs the combined matrix of previously non-intersected trajctories and resimulated trajectories

%% Storing data for trajectories that are not intersected
[traj_nonint_xdata traj_nonint_ydata] = GetNonIntData(traj_xdata, traj_ydata, sensor_x_pos, sensor_y_pos, deployment_time, nT,startTime, times,sensor_index);

% Determining the number of trajectories that need to be resimulated
numint = nT - size(traj_nonint_xdata,1);

%% Resimulating required trajectories
[traj_resim_xdata traj_resim_ydata] = GetResimTraj(numint, sensor_x_pos, sensor_y_pos,deployment_time, vTmean, vTstd, times, timeStep,startTime, endTime,margin, LKP_x, LKP_y, obstacles_check, num_obstacles, adaptive, sensor_index);


%% Create new data matrix with resimulated trajectories
% Combining matrices of the trajectories that weren't intersected and the resampled trajectories.
traj_xdata_new = vertcat(traj_nonint_xdata,traj_resim_xdata);
traj_ydata_new = vertcat(traj_nonint_ydata,traj_resim_ydata);
end