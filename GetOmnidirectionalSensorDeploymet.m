function [sensor_omni_x, sensor_omni_y, deploy_times_omni] = GetOmnidirectionalSensorDeploymet(traj_xdata,traj_ydata, sens_num, sens_range, sector_angle, curve_approx,num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y,pat_init, sens_orient_num, margin,nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y, obstacles_check, num_obstacles, adaptive);
% This function outputs the sensor network planned using the omnidirectional sensing model

%% Robot Initialisation
% Initial robot positions
robot_pos_x = robot_init_pos_x;
robot_pos_y = robot_init_pos_y;
% Time at which the robots are at the positions above (initial = 0s)
robot_pos_time = ones(size(robot_init_pos_x))*startTime;


h = waitbar(0,['Deploying Omnidirectional Sensor 0 out of ' num2str(sens_num)]);

for sensor_index = 1:sens_num
    
    waitbar(sensor_index/sens_num,h,['Deploying Omnidirectional Sensor ' num2str(sensor_index) ' out of ' num2str(sens_num)]);
    
    %% Sub-Region Optimization
    % Determining the optimal sub-region that contains the most amount of trajectories
    [sensor_location_omni, sensor_time, robot_pos_x, robot_pos_y, robot_pos_time] = GetPatternSearchSensorPosition(traj_xdata, traj_ydata, times,startTime, sens_range,num_robots, robot_speed, robot_pos_x, robot_pos_y, robot_pos_time,pat_init, LKP_x, LKP_y);
    
    % Creating Sensor Segments
    thetas = 0:curve_approx:2*pi;
    sensor_omni_x(sensor_index,:) = sensor_location_omni(1,1) + 0.5*sens_range.*cos(thetas);
    sensor_omni_y(sensor_index,:) = sensor_location_omni(1,2) + 0.5*sens_range.*sin(thetas);
    
    %% Saving Robot Positions
    robot_traj_x(sensor_index,:) = robot_pos_x;
    robot_traj_y(sensor_index,:) = robot_pos_y;
    robot_times(sensor_index,:) = robot_pos_time;
    
   
    %% Deployment Times
    % Earliest Possible Intersection Method:
    %[deploy_time] = GetDeploymentTimes(traj_xdata, traj_ydata, sensor_omni_x, sensor_omni_y, startTime, timeStep, sensor_index, margin);
    
    % Robot Delivery Method (saving times outputted from pattern search):
    deploy_times_omni(sensor_index,:) = sensor_time;
    
    
    %% Trajectory Resimulation
    [traj_xdata traj_ydata] = GetNewTraj(traj_xdata, traj_ydata, sensor_omni_x, sensor_omni_y,deploy_times_omni, nT, vTmean, vTstd, times, timeStep, startTime, endTime,sensor_index, margin, LKP_x, LKP_y, obstacles_check, num_obstacles, adaptive);
    
end
close(h);

end