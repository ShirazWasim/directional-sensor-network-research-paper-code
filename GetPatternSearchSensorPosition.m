function [pos, time, robot_pos_x, robot_pos_y, robot_pos_time, a] = GetPatternSearchSensorPosition(traj_xdata, traj_ydata, times,startTime, sens_range, num_robots, robot_speed, robot_pos_x, robot_pos_y, robot_pos_time, pat_init, LKP_x, LKP_y, sensor_index)
% A function that determines the optimal sub-region to deploy the sensor in using a Pattern Search Optimization

% Finding the size of the region
maxim1 = [max(abs(traj_xdata),[],'all'), max(abs(traj_ydata),[],'all') ];
circ_scaling = (max(maxim1, [],'all'))/(2*sens_range);
range = circ_scaling*sens_range; % Ensures initial stencil is large enough to cover all the trajectories


% Generating the random initializations for the pattern search process
x_center_circle_multi = unifrnd(-maxim1(1,1),maxim1(1,1),pat_init,1);
y_center_circle_multi = unifrnd(-maxim1(1,2),maxim1(1,2),pat_init,1);


time_index = find(times >= startTime);
times = times(time_index:end);

for init_index = 1:pat_init % for loop for each pattern search initialization
    
    % Selecting the initialization
    x_center_circle = x_center_circle_multi(init_index,:);
    y_center_circle = y_center_circle_multi(init_index,:);
    range = circ_scaling*sens_range; % Setting the initial size of the stencil
    
    while range > 0.5*sens_range
        %% Circle
        % Hexagonal-Circle Stencil Generation
        [circ_xdata, circ_ydata] = GetHexGeometry(x_center_circle, y_center_circle,range);
        
        % Determining how many trajectories inside each circle/sub-region
        for circindex = 1:size(circ_xdata,1)
            
            % Determining the earliest time a sensor can be deployed
            earliest_deployment_time = inf;
            earliest_robot_index = 0;
            for robotindex = 1:size(robot_pos_x,1)
                dist_to_deployment = sqrt((robot_pos_x(robotindex) - circ_xdata(circindex,1)).^2 + (robot_pos_y(robotindex) - circ_ydata(circindex,1)).^2); % Distance of each robot to the potential deployment position
                deployment_time = dist_to_deployment/robot_speed + robot_pos_time(robotindex); % Time taken to reach the potential deployment position
                if deployment_time < earliest_deployment_time
                    earliest_deployment_time = deployment_time; % Earliest time that one of the robots can reach the deployment position
                    earliest_robot_index = robotindex; % Index of earliest robot
                end
            end
            
            % Only considering the trajectories near the current sub-region
            % [traj_xdata1, traj_ydata1] = GetBoundedBoxTraj2(traj_xdata, traj_ydata, circ_xdata(circindex,1), circ_ydata(circindex,1), range);
            
            % Trimming the trajectories for after the deployment time
            valid_indices = times >= earliest_deployment_time;
            traj_xdata_trimmed = traj_xdata(:,valid_indices);
            traj_ydata_trimmed = traj_ydata(:,valid_indices);
            
            % Determining how many trajectories are within the sub-region
            ineq_check = (traj_xdata_trimmed - circ_xdata(circindex,1)).^2 + (traj_ydata_trimmed-circ_ydata(circindex,1)).^2;
            check = ineq_check <= range^2;
            numint(circindex,1) = sum(sum(check,2)>0);
        end
        
        %% Plotting Pattern Search Iteration
        % Pattern Search
        % figure
        % x1 = traj_xdata_trimmed';
        % y1 = traj_ydata_trimmed';
        % x_vec = x1(:);
        % y_vec = y1(:);
        % scatter(x_vec,y_vec, 10, '.')
        % hold on
        %
        % circle = pol2cart(linspace(0,2*pi),range);
        % [xc,yc] = pol2cart(linspace(0,2*pi),range);
        %
        % circ_coordsx = xc+circ_xdata;
        % circ_coordsy = yc+circ_ydata;
        %
        % scatter(circ_xdata,circ_ydata,'x','k')
        % hold on
        % plot(circ_coordsx',circ_coordsy','k');
        % xlabel('x distance from LKP (m)')
        % ylabel('y distance from LKP (m)')
        % axis([-4000 4000 -4000 4000])
        % set(gca,'FontSize',8, 'FontName', 'Times New Roman')
        % box on
        % axis equal
        
        %% Plotting Stencil Only
        % range_plot = 100;
        % [circ_xdata_plot, circ_ydata_plot] = GetHexGeometry(x_center_circle, y_center_circle,range_plot);
        % circle = pol2cart(linspace(0,2*pi),range);
        % [xc,yc] = pol2cart(linspace(0,2*pi),range);
        %
        % circ_coordsx = xc + circ_xdata_plot - x_center_circle;
        % circ_coordsy = yc + circ_ydata_plot - y_center_circle;
        %
        % scatter(circ_xdata_plot-x_center_circle,circ_ydata_plot-y_center_circle,'x','k')
        % hold on
        % plot(circ_coordsx',circ_coordsy','k');
        % xlabel('Size (m)')
        % ylabel('Size (m)')
        % axis equal
        
 
        %% 
        % Finding the circle with the most amount of trajectories
        [val,row] = max(numint',[],'linear');
        
        
        % If optimal circle is center circle, then reduce the radius, otherwise, shift the stencil to optimal circle
        if row == 1
            x_center_circle = circ_xdata(1,1);
            y_center_circle = circ_ydata(1,1);
            range = range*0.75;
        else
            x_center_circle = circ_xdata(row,1);
            y_center_circle = circ_ydata(row,1);
        end
        
    end
    
    
    %% Repeating For Final Circle Size
    % Cross-Circle Generation
    [circ_xdata, circ_ydata] = GetHexGeometry(x_center_circle, y_center_circle, 0.5*sens_range);
    
    % Bounded Circle
    for circindex = 1:size(circ_xdata,1)
        
        % determine earliest time a sensor could be deployed
        earliest_deployment_time = inf;
        earliest_robot_index = 0;
        for robotindex = 1:size(robot_pos_x,1)
            dist_to_deployment = sqrt((robot_pos_x(robotindex) - circ_xdata(circindex,1)).^2 + (robot_pos_y(robotindex) - circ_ydata(circindex,1)).^2);
            dist(robotindex,:) = dist_to_deployment;
            deployment_time = dist_to_deployment/robot_speed + robot_pos_time(robotindex);
            d_time(robotindex,:) = round(deployment_time);
            if deployment_time < earliest_deployment_time
                earliest_deployment_time = deployment_time;
                earliest_robot_index = robotindex;
            end
        end
        
        
        % Only considering the trajectories near the current sub-region
        % [traj_xdata1, traj_ydata1] = GetBoundedBoxTraj2(traj_xdata, traj_ydata, circ_xdata(circindex,1), circ_ydata(circindex,1), range);
        
        % trim trajectory data to be after the deployment time
        valid_indices = times >= earliest_deployment_time;
        traj_xdata_trimmed = traj_xdata(:,valid_indices);
        traj_ydata_trimmed = traj_ydata(:,valid_indices);
        
        ineq_check = (traj_xdata_trimmed - circ_xdata(circindex,1)).^2 + (traj_ydata_trimmed-circ_ydata(circindex,1)).^2;
        
        check = ineq_check <= range^2;
        
        numints(circindex,1) = sum(sum(check,2)>0);
        opt_times(circindex,1) = earliest_deployment_time;
        opt_robot_index(circindex,1) = earliest_robot_index;
        
    end
    
    % Determining optimal circle/location for sensors
    [val,row] = max(numints',[],'linear');
    val_opt(init_index,:) = val; % Number of trajectories inside optimal circle
    
    % Saving the final position, time and robot index for each initialization
    [pos(init_index,:)] = [circ_xdata(row,1), circ_ydata(row,1)];
    time(init_index,:) =  opt_times(row,:);
    delivery_robot_index(init_index,:) = opt_robot_index(row,:);
    
    
end


%% Obtaining the optimal sub-region

% Comparing results for different initializations and finding the position(circle) containing the most amount of trajectories
[a,opt_init_row] = max(val_opt,[],'linear');
b = find(val_opt == a);

% Deployment position, time and robot index for sub-regions with the most trajectory intersections
pos = pos(b(1,1),:);
time = time(b(1,1),:);
earliest_robot_index = delivery_robot_index(b(1,1),:);

% Selecting the sub-region (from those that contain the most trajectories) that is the shortest distance from the LKP
distances = sqrt((pos(:,1) - LKP_x).^2 + (pos(:,2) - LKP_y).^2);
[~,opt_dist_row] = min(distances,[],'linear');

% Deployment position, time and robot index for optimal sub-region
pos = pos(opt_dist_row,:);
time = time(opt_dist_row,:);
earliest_robot_index = delivery_robot_index(opt_dist_row,:);

%% Update robot position and time given best delivery
robot_pos_x(earliest_robot_index,1) = pos(1);
robot_pos_y(earliest_robot_index,1) = pos(2);
robot_pos_time(earliest_robot_index,1) = time;

end
