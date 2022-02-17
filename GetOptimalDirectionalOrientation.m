function [sensor_directional_opt_x sensor_directional_opt_y, sensor_dir_line_pos_x, sensor_dir_line_pos_y, theta] = GetOptimalDirectionalOrientation(traj_xdata, traj_ydata, sensor_location, startTime, times, sensor_time, sens_range, sector_angle, curve_approx, num_rand_starts ,margin)
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
    func = @(theta) SensorPoseEvaluation(theta,traj_xdata,traj_ydata,sensor_location, sens_range,startTime, times, sensor_time, sector_angle, curve_approx, margin);
    rt = fminsearch(func,theta);
    
    % Saving Poses and Number of Intersections for the Optimal Orientation after each equally-spaced angle initialization
    sensor_xstart = sensor_location(1,1) - 0.5*(sens_range.*cos(rt));
    sensor_ystart = sensor_location(1,2) - 0.5*(sens_range.*sin(rt));
    
    sensor_xend = sensor_location(1,1) + 0.5*(sens_range.*cos(rt));
    sensor_yend = sensor_location(1,2) + 0.5*(sens_range.*sin(rt));
    
    sensor_x_cand_pos(theta_index,:) = [sensor_xstart ; sensor_xend]';
    sensor_y_cand_pos(theta_index,:) = [sensor_ystart ; sensor_yend]';
    
    % Creating directional sensors from linear sensors
    [sensor_x_pos, sensor_y_pos] = GetDirectionalSectorGeometry(sensor_x_cand_pos(theta_index,:) , sensor_y_cand_pos(theta_index,:) , sector_angle, sens_range,curve_approx);
    
    sensor_line_opt_x(theta_index,:) = sensor_x_pos;
    sensor_line_opt_y(theta_index,:) = sensor_y_pos;
    
    val = GetMultiLineInt3(traj_xdata, traj_ydata, sensor_line_opt_x(theta_index,:), sensor_line_opt_y(theta_index,:),startTime, times, sensor_time, margin);
    
    val_opt_mul(theta_index,:) = sum(val,'All');
    thetas(theta_index,:) = rt;
end


% Selecting the pose that intersects the most amount of trajectories 
[val_opt,val_opt_row] = max(val_opt_mul); % Optimal number of trajectories intersected
sensor_directional_opt_x = sensor_line_opt_x(val_opt_row,:); % Linear sensor pose x-data
sensor_directional_opt_y = sensor_line_opt_y(val_opt_row,:); % Linear sensor pose y-data
sensor_dir_line_pos_x = sensor_x_cand_pos(val_opt_row,:); % Directional sensor pose x-data
sensor_dir_line_pos_y = sensor_y_cand_pos(val_opt_row,:); % Directional sensor pose y-data
theta = thetas(val_opt_row,:); % Optimal orientation angle

function [num_int] = SensorPoseEvaluation(theta,traj_xdata,traj_ydata,sensor_location, sens_range,startTime, times, sensor_time, sector_angle, curve_approx, margin)
%% Generating sensors with start point on the circumference of the circle
sensor_xstart = sensor_location(1,1) - 0.5*(sens_range.*cos(theta));
sensor_ystart = sensor_location(1,2) - 0.5*(sens_range.*sin(theta));

sensor_xend = sensor_location(1,1) + 0.5*(sens_range.*cos(theta));
sensor_yend = sensor_location(1,2) + 0.5*(sens_range.*sin(theta));

sensor_x_pos_mul = [sensor_xstart ; sensor_xend]';
sensor_y_pos_mul = [sensor_ystart ; sensor_yend]';

%% Getting directional poses from linear poses
[sensor_x_pos_opt, sensor_y_pos_opt] = GetDirectionalSectorGeometry(sensor_x_pos_mul, sensor_y_pos_mul, sector_angle, sens_range,curve_approx);


%% Determining number of intersections
[num_int] = GetMultiLineInt3(traj_xdata, traj_ydata, sensor_x_pos_opt, sensor_y_pos_opt, startTime, times, sensor_time, margin);

num_int = -sum(num_int,'All');
end

end