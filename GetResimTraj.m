function [traj_resim_xdata traj_resim_ydata] = GetResimTraj(numint, sensor_x_pos, sensor_y_pos,deployment_time, vTmean, vTstd, times, timeStep,startTime,endTime, margin,LKP_x, LKP_y, obstacles_check, num_obstacles, adaptive, sensor_index)
% Function that outputs a matrix of resampled trajectories that are checked to make sure they don't intersect previously deployed sensors.

%% Resimulating required trajectories
% Prelim for loops
n = 0;
nTa = numint;
always_zero = 0;
scale = 2;

%% Remove parts of the trajectory that have already been travelled at the
% Deployment start time
if startTime > 0
    I = times >= startTime;
else
    I = 1;
end

% Defining resimulated trajectories variable to concatenate in loop
traj_resim_xdata = [];
traj_resim_ydata = [];


%% Loop that outputs a matrix containing the resampled trajectories that don't intersect previously placed sensors.
if numint > 0
    while n < numint % while the total number of rows of the output matrix is less than the number of trajectories required
        [traj_xdata traj_ydata] = GetTraj(nTa, vTmean, vTstd, times, timeStep, always_zero, endTime, LKP_x, LKP_y); % Generating a new set of trajectories
                                                            
        % Checking and resimulating trajectories to ensure they don't intersect obstacles
        switch obstacles_check
            case 'Include'
                [traj_xdata, traj_ydata] = obstaclecheck(traj_xdata, traj_ydata, vTmean, vTstd, times, timeStep, startTime, endTime, adaptive, LKP_x, LKP_y, num_obstacles);
        end 
        
        [num_int_seg, traj_xdata, traj_ydata] = GetMultiLineInt3(traj_xdata, traj_ydata, sensor_x_pos, sensor_y_pos, startTime, times, deployment_time,margin);
        
        % Only considering the portions of the trajectories after the deployment start time
        traj_xdata = traj_xdata(:,I);
        traj_ydata = traj_ydata(:,I);
       
        %% Saving trajectories that pass check
        if size(traj_xdata,1) > nTa
            traj_resim_xdata = vertcat(traj_resim_xdata, traj_xdata(1:nTa,:));
            traj_resim_ydata = vertcat(traj_resim_ydata, traj_ydata(1:nTa,:));
        else
            traj_resim_xdata = vertcat(traj_resim_xdata, traj_xdata);
            traj_resim_ydata = vertcat(traj_resim_ydata, traj_ydata);
        end
        
        % Check if size corresponds with required number of trajectories
        n = size(traj_resim_xdata,1);
        nTa = nTa - size(traj_xdata,1); % Trajectories that still need to be found
    end
end

end