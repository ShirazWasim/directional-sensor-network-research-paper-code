function [sensor_line_opt_x sensor_line_opt_y, val_opt, orient] = GetOptimalLineOrientationfminsearchequal(traj_xdata, traj_ydata, sensor_location, startTime, times, deploy_times_line, sens_range, num_rand_starts ,margin)
% This function outputs the optimal orientation for a linear sensor placed
% at the input deployment position by checking how many trajectories are
% intersected 

% Since the function being optimized is discrete, the optimization is started at a number of equally-spaced angles.
% Initializating equally-spaced angles
a = 2*pi/num_rand_starts;
theta_mul = (0:a:2*pi)';

for theta_index = 1:size(theta_mul,1);
    theta = theta_mul(theta_index,:);
    
    % Optimization
    func = @(theta) SensorPoseEvaluation(theta,traj_xdata,traj_ydata,sensor_location,startTime, times, deploy_times_line, sens_range, margin);
    rt = fminsearch(func,theta);
    
    % Saving Poses and Number of Intersections for the Optimal Orientation after each equally-spaced angle initialization
    sensor_xstart = sensor_location(1,1) - 0.5*(sens_range.*cos(rt));
    sensor_ystart = sensor_location(1,2) - 0.5*(sens_range.*sin(rt));
    
    sensor_xend = sensor_location(1,1) + 0.5*(sens_range.*cos(rt));
    sensor_yend = sensor_location(1,2) + 0.5*(sens_range.*sin(rt));
    
    sensor_line_opt_x(theta_index,:) = [sensor_xstart ; sensor_xend]';
    sensor_line_opt_y(theta_index,:) = [sensor_ystart ; sensor_yend]';
    
    [val,~,~] = GetMultiLineInt3(traj_xdata, traj_ydata, sensor_line_opt_x(theta_index,:), sensor_line_opt_y(theta_index,:),startTime, times, deploy_times_line, margin);
    
    val_opt_mul(theta_index,:) = val;
end

% Selecting the pose that intersects the most amount of trajectories 
[val_opt,val_opt_row] = max(val_opt_mul); % Optimal number of trajectories intersected
sensor_line_opt_x = sensor_line_opt_x(val_opt_row,:); % Pose x-data
sensor_line_opt_y = sensor_line_opt_y(val_opt_row,:); % Pose y-data
orient = rt; % Optimal orientation angle


function [num_int] = SensorPoseEvaluation(theta,traj_xdata,traj_ydata,sensor_location,startTime, times, deploy_times_line, sens_range, margin)  
%% Generating sensor poses with start point on the circumference of the circle
sensor_xstart = sensor_location(1,1) - 0.5*(sens_range.*cos(theta)); 
sensor_ystart = sensor_location(1,2) - 0.5*(sens_range.*sin(theta));

sensor_xend = sensor_location(1,1) + 0.5*(sens_range.*cos(theta));
sensor_yend = sensor_location(1,2) + 0.5*(sens_range.*sin(theta));

sensor_x_pos = [sensor_xstart ; sensor_xend]';
sensor_y_pos = [sensor_ystart ; sensor_yend]';

%% Determining number of intersections
[num_int,~,~] = GetMultiLineInt3(traj_xdata, traj_ydata, sensor_x_pos, sensor_y_pos,startTime, times, deploy_times_line, margin);
num_int = -num_int;

end
end