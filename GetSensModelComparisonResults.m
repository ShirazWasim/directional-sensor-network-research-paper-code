function [num_int_line_test, num_int_line_dir, num_int_dir, num_int_omni] = GetSensModelComparisonResults(timeStep, startTime,runLength, endTime, times, nT, vTmean, vTstd, vTmax, margin, sens_num, sens_range, sector_angle, curve_approx, num_rand_starts, num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y, LKP_x, LKP_y, adaptive, num_obstacles, pat_init, obstacles_check, test_index, traj_type);

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


%% Line Sensor
[sensor_line_x sensor_line_y, deploy_times_line, sensor_orient, robot_traj_x, robot_traj_y, robot_times] = GetLineSensorDeployment(traj_xdata,traj_ydata, sens_num, sens_range, num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y,pat_init, num_rand_starts, margin,nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y, obstacles_check, num_obstacles, adaptive);


%% Directional Sensor
[sensor_dir_x, sensor_dir_y, sensor_dir_line_x, sensor_dir_line_y, deploy_times_dir, dir_sensor_orient] = GetDirectionalSensorDeployment(traj_xdata,traj_ydata, sens_num, sens_range, sector_angle,curve_approx,num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y,pat_init, num_rand_starts, margin,nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y, obstacles_check, num_obstacles, adaptive);


%% Omnidirectional Sensor
[sensor_omni_x, sensor_omni_y, deploy_times_omni] = GetOmnidirectionalSensorDeploymet(traj_xdata,traj_ydata, sens_num, sens_range, sector_angle, curve_approx,num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y,pat_init, num_rand_starts, margin,nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y, obstacles_check, num_obstacles, adaptive);


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
   
    
    %% Linear Test:
    [num_int_line_test] = GetMultiLineInt3(traj_xdata_test, traj_ydata_test, sensor_line_x, sensor_line_y, startTime, times, deploy_times_line, margin);
    
    num_int_line_test = sum(num_int_line_test,'all');
    num_int_line_tests(num_tests,:) = num_int_line_test;
    
    
    %% Linear-Directional Test:
    %Creating Directional Sensor Sector Geometry
    [x_coord, y_coord] = GetDirectionalSectorGeometry(sensor_line_x, sensor_line_y, sector_angle, sens_range,curve_approx);
    
    [num_int_line_dir] = GetMultiLineInt3(traj_xdata_test, traj_ydata_test, x_coord, y_coord,startTime, times, deploy_times_line, margin);
    
    num_int_line_dir = sum(num_int_line_dir,'all');
    num_int_line_dir_tests(num_tests,:) = num_int_line_dir;
    
    
    %% Directional Test:
    [num_int_dir] = GetMultiLineInt3(traj_xdata_test, traj_ydata_test, sensor_dir_x, sensor_dir_y,startTime, times, deploy_times_dir, margin);
    
    num_int_dir = sum(num_int_dir,'all');
    num_int_dir_tests(num_tests,:) = num_int_dir;
    
    
    %% Omni-directional Test:
    [num_int_omni] = GetMultiLineInt3(traj_xdata_test, traj_ydata_test, sensor_omni_x, sensor_omni_y, startTime, times, deploy_times_omni, margin);
    
    num_int_omni = sum(num_int_omni,'all');
    num_int_omni_tests(num_tests,:) = num_int_omni;
    

end
close(h)

num_int_line_test = sum(num_int_line_tests, 'All')/num_test;
num_int_line_dir = sum(num_int_line_dir_tests,'All')/num_test;
num_int_dir = sum(num_int_dir_tests,'All')/num_test;
num_int_omni = sum(num_int_omni_tests,'All')/num_test;


%% Data Table
% Linear Directional
data_table_line_dir = horzcat(deploy_times_line, sensor_line_x(:,1),sensor_line_y(:,1),sensor_orient);

% Directional
data_table_dir = horzcat(deploy_times_dir, sensor_dir_line_x(:,1), sensor_dir_line_y(:,1), dir_sensor_orient);



%% Saving Data 
% Save Variables
save(sprintf('%s\\data\\Sensing Model Study\\SensModelWorkspace%u.mat', pwd, test_index));

% Data
save(sprintf('%s\\data\\Sensing Model Study\\Line-DirData%u.mat', pwd, test_index),'data_table_line_dir');
save(sprintf('%s\\data\\Sensing Model Study\\DirData%u.mat', pwd, test_index),'data_table_dir');

%% Plotting
% Plotting Linear Sensor
[line_figure] = obstaclesplot(obstacles_check, num_obstacles);
hold on
plot(sensor_line_x',sensor_line_y','k','LineWidth',1.5)
hold on
scatter(LKP_x, LKP_y,'r','x')
xlabel('x distance from LKP (m)','FontName','Times New Roman','FontSize',8)
ylabel('y distance from LKP (m)','FontName','Times New Roman','FontSize',8)
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal

% Plotting Linear-Directional Sensors
[line_directional_figure] = obstaclesplot(obstacles_check, num_obstacles);
hold on
plot(x_coord',y_coord','k')
hold on
scatter(LKP_x, LKP_y,'r','x')
hold on
plot(sensor_line_x',sensor_line_y','k')
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal

% Plotting Directional Sensors
[directional_figure] = obstaclesplot(obstacles_check, num_obstacles);
hold on
plot(sensor_dir_x',sensor_dir_y','k');
hold on
scatter(LKP_x, LKP_y,'r','x')
hold on
patch(sensor_dir_x',sensor_dir_y','k');
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal

% Plotting Omnidirectional Sensor
[omni_figure] = obstaclesplot(obstacles_check, num_obstacles);
hold on
plot(sensor_omni_x',sensor_omni_y','k')
hold on
scatter(LKP_x, LKP_y,'r','x')
hold on
patch(sensor_omni_x',sensor_omni_y','k')
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal

%% Saving Plots
saveas(figure(line_directional_figure),[pwd sprintf('\\Figures\\Sensing Model Study\\Line-Dir%u.fig', test_index)]);
saveas(figure(directional_figure),[pwd sprintf('\\Figures\\Sensing Model Study\\Dir%u.fig', test_index)]);

end