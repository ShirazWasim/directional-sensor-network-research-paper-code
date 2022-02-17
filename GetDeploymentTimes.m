function [deploy_time] = GetDeploymentTimes(traj_xdata, traj_ydata, sensor_x_pos, sensor_y_pos, startTime, timeStep, sensor_index, margin)
% This function outputs the time at which the earliest simulated trajectory would intersect the sensor in its planned pose

% Only considering the trajectories in the vicinity of the sensor
[traj_xdata_red traj_ydata_red] = GetBoundedBoxTraj(traj_xdata, traj_ydata, sensor_x_pos(sensor_index,:),sensor_y_pos(sensor_index,:),margin);

column = 0;
for sens_ind = 1:size(sensor_x_pos,2)-1
    [~, i] = IntCheck(traj_xdata,traj_ydata,sensor_x_pos(sensor_index,sens_ind:sens_ind+1),sensor_y_pos(sensor_index,sens_ind:sens_ind+1));
    column = vertcat(column,i);
end

column(1,:) = [];
time_val = min(column);

deploy_time = startTime + time_val*timeStep;
% deploy_time = startTime + i*timeStep;
%% Plotting
% a = find(p_intx);
% 
% p_intx = p_intx(a);
% p_inty = p_inty(a);
% 
% plot(traj_xdata_red', traj_ydata_red','--k')
% hold on
% 
% plot(sensor_x_pos',sensor_y_pos','k')
% hold on
% scatter(p_intx,p_inty,'xr')

end