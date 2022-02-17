function [num_int_line_test, deploy_times_line] = GetSimpleDeploymentResults(timeStep, startTime,runLength, endTime, times, nT, vTmean, vTstd, vTmax, margin, sens_num, sens_range, sector_angle, curve_approx, num_rand_starts, num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y, LKP_x, LKP_y, adaptive, num_obstacles, pat_init, obstacles_check, test_index, traj_type);

%% Trajectory Data Generation
switch traj_type
    % Generate new trajectories
    case 'Generated'
        [traj_xdata traj_ydata] = GetTraj(nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y);
        
        %% Obstacles Check
        switch obstacles_check
            case 'Include'
                [traj_xdata, traj_ydata] = obstaclecheck(traj_xdata, traj_ydata, vTmean, vTstd, times, timeStep, startTime, endTime,adaptive, LKP_x, LKP_y, num_obstacles);
        end
        
        % Code to remove parts of the trajectory that are prior to the deployment start time
        if startTime > 0
            [traj_xdata, traj_ydata] = GetTrajReduction(traj_xdata, traj_ydata, times, startTime);
        end
        
    % Use saved trajectories    
    case 'Saved'
        load(sprintf('%s\\data\\Trajectories\\traj_data%u.mat', pwd, startTime))
        traj_xdata = cell2mat(traj_xdata_test_comp(1,end));
        traj_ydata = cell2mat(traj_ydata_test_comp(1,end));
end


%% Get Linear Sensor Deployment
[sensor_line_x, sensor_line_y, deploy_times_line, sensor_orient, robot_traj_x, robot_traj_y, robot_times] = GetLineSensorDeployment(traj_xdata,traj_ydata, sens_num, sens_range, num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y,pat_init, num_rand_starts, margin,nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y, obstacles_check, num_obstacles, adaptive);


%% TESTING

num_test = 10;

h = waitbar(0,['Test Number 0 out of ' num2str(num_test)]);

for num_tests = 1:num_test
    waitbar(num_tests/num_test,h,['Test Number ' num2str(num_tests) ' out of ' num2str(num_test)]);
    
    %% Test Trajectories Creation
    switch traj_type
        case 'Generated'
            [traj_xdata_test traj_ydata_test] = GetTraj(nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y);
            
            % Obstacles Check
            switch obstacles_check
                case 'Include'
                    [traj_xdata_test, traj_ydata_test] = obstaclecheck(traj_xdata_test, traj_ydata_test, vTmean, vTstd, times, timeStep, startTime, endTime,adaptive, LKP_x, LKP_y, num_obstacles);
            end
            
            % Code to remove parts of the trajectory that are prior to the deployment start time
            if startTime > 0
                [traj_xdata_test, traj_ydata_test] = GetTrajReduction(traj_xdata_test, traj_ydata_test, times, startTime);
            end
            
        case 'Saved'
            traj_xdata_test = cell2mat(traj_xdata_test_comp(1,num_tests));
            traj_ydata_test = cell2mat(traj_ydata_test_comp(1,num_tests));
    end
    
    
    %% Determining Number of Intersections
    % Getting directional sensors
    [sensor_line_dir_x, sensor_line_dir_y] = GetDirectionalSectorGeometry(sensor_line_x, sensor_line_y, sector_angle, sens_range,curve_approx);

    % Checking number of intersections
    [num_int_line_test] = GetMultiLineInt3(traj_xdata_test, traj_ydata_test, sensor_line_dir_x, sensor_line_dir_y, startTime, times, deploy_times_line,margin);

    % Saving the number of intersections for each test set of trajectories
    num_int_line_test = sum(num_int_line_test,'all');
    num_int_line_tests(num_tests,:) = num_int_line_test;

end
close(h)

% Finding the average number of trajectories intersected
num_int_line_test = sum(num_int_line_tests, 'All')/num_test;

%% Saving Data
% Data Table
data_table = horzcat(deploy_times_line, sensor_line_x(:,1),sensor_line_y(:,1),sensor_orient, num_int_line_test.*ones(size(sensor_orient)));

% Save Variables
save(sprintf('%s\\data\\Simple Deployment\\SimpleDeploymentWorkspace%u.mat', pwd, test_index));

% Save Data
save(sprintf('%s\\data\\Simple Deployment\\SimpleDeploymentData%u.mat', pwd, test_index),'data_table');


%% Plotting
[simp_deploy] = obstaclesplot(obstacles_check, num_obstacles)
hold on
patch(sensor_line_dir_x', sensor_line_dir_y','k','LineWidth',1.5)
hold on
scatter(LKP_x, LKP_y,'r','x')
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal

%% Robot Trajectory Plot
robot_traj_x = vertcat(zeros(1,num_robots),robot_traj_x);
robot_traj_y = vertcat(zeros(1,num_robots),robot_traj_y);

[robot_traj_plot] = obstaclesplot(obstacles_check, num_obstacles)
hold on
plot(robot_traj_x(:,3)',robot_traj_y(:,3)','k:')
hold on
plot(robot_traj_x(:,2)',robot_traj_y(:,2)','r:')
hold on
plot(robot_traj_x(:,1)',robot_traj_y(:,1)','b:')
hold on
patch(sensor_line_dir_x',sensor_line_dir_y','k')
scatter(LKP_x, LKP_y,'r','x')
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal

% Saving Plots
saveas(figure(simp_deploy),[pwd sprintf('\\Figures\\Simple Deployment\\SimpleDeployment%u.fig', test_index)]);
end