function [traj_xdata traj_ydata] = GetNonIntData(traj_xdata, traj_ydata, sensor_xpos, sensor_ypos, deployment_time, nT,startTime, times,sensor_index)
% This function outputs the trajectories that were not intersected by the most recent sensor deploymet

%% Determining Number of Intersections
valid_idx = times >= startTime;
valid_idx2 = valid_idx.*times;
valid_idx2 = valid_idx2(valid_idx2~=0);
row2 = [];


% For loop for directional sensors made up of multiple line segments
for sensor_ind = 1:size(sensor_xpos,2)-1
    
     % Only considering portions of trajectory after the sensor deployment
     valid_idx3 = valid_idx2 > deployment_time(sensor_index,1);
     traj_xdata_time_red = traj_xdata(:,valid_idx3); 
     traj_ydata_time_red = traj_ydata(:,valid_idx3);
     
     % Intersection check
    [int] = IntCheck(traj_xdata_time_red,traj_ydata_time_red, sensor_xpos(sensor_index,sensor_ind:sensor_ind+1), sensor_ypos(sensor_index,sensor_ind:sensor_ind+1));
    
    % Finding rows that weren't intersected and saving
    row_idx = find(int == 0);
    traj_xdata = traj_xdata(row_idx,:);
    traj_ydata = traj_ydata(row_idx,:);
   
end
end



