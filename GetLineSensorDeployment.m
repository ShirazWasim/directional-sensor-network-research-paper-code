function  [sensor_line_x sensor_line_y, deploy_times_line, sensor_orient, robot_traj_x, robot_traj_y, robot_times] = GetLineSensorDeployment(traj_xdata,traj_ydata, sens_num, sens_range, num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y,pat_init, num_rand_starts, margin,nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y, obstacles_check, num_obstacles, adaptive);
% This function outputs the sensor network planned using the linear sensing model
sensor_type = 'Line';

%% Robot Initialisation
% Initial robot positions
robot_pos_x = robot_init_pos_x;
robot_pos_y = robot_init_pos_y;
% Time at which the robots are at the positions above (initial = 0s)
robot_pos_time = ones(size(robot_init_pos_x))*startTime;


h = waitbar(0,['Deploying Line Sensor 0 out of ' num2str(sens_num)]);
for sensor_index = 1:sens_num
    
    waitbar(sensor_index/sens_num,h,['Deploying Line Sensor ' num2str(sensor_index) ' out of ' num2str(sens_num)]);
    

    %% Sub-Region Optimization
    % Determining the optimal sub-region that contains the most amount of trajectories
    [sensor_location_line, deployment_time, robot_pos_x, robot_pos_y, robot_pos_time, traj_total] = GetPatternSearchSensorPosition(traj_xdata, traj_ydata, times,startTime, sens_range, num_robots, robot_speed, robot_pos_x, robot_pos_y, robot_pos_time, pat_init, LKP_x, LKP_y, sensor_index);
    
    traj_totals(sensor_index,:) = traj_total;
    
    %% Saving Robot Positions and Times
    robot_traj_x(sensor_index,:) = robot_pos_x;
    robot_traj_y(sensor_index,:) = robot_pos_y;
    robot_times(sensor_index,:) = robot_pos_time;
    
    
    
    %% Determining Optimal Orientation
    [sensor_line_opt_x sensor_line_opt_y val_equal, orient] = GetOptimalLineOrientation(traj_xdata, traj_ydata, sensor_location_line, startTime, times, deployment_time, sens_range, num_rand_starts ,margin);
    
    % Saving the x-y coordinates describing the sensor pose
    sensor_line_x(sensor_index,:) = sensor_line_opt_x;
    sensor_line_y(sensor_index,:) = sensor_line_opt_y;
    
    sensor_orient(sensor_index,:) = orient; % Angle of orientation for each sensor
    
    %% Deployment Times
    % Earliest Possible Intersection Method:
    %[deploy_time] = GetDeploymentTimes(traj_xdata, traj_ydata, sensor_line_x, sensor_line_y, startTime, timeStep, sensor_index, margin);
    
    % Robot Delivery Method (saving times outputted from pattern search):
    deploy_times_line(sensor_index,:) = deployment_time;
    
    
    %% Trajectory Resimulation
    [traj_xdata traj_ydata] = GetNewTraj(traj_xdata, traj_ydata, sensor_line_x, sensor_line_y, deploy_times_line, nT, vTmean, vTstd, times, timeStep, startTime, endTime,sensor_index, margin, LKP_x, LKP_y, obstacles_check, num_obstacles, adaptive);
    
end
close(h);