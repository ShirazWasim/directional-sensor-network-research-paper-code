function [num_int_pat, num_int_uni, num_int_ring, num_int_rand] = GetComparativeStudyResults(timeStep, startTime,runLength, endTime, times, nT, vTmean, vTstd, vTmax, margin, sens_num, sens_range, sector_angle, curve_approx, num_rand_starts, num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y, LKP_x, LKP_y, pat_init, adaptive, obstacles_check, num_obstacles, test_index, traj_type)

obstacles_check = "Don't Include";

%% Trajectory Data Generation
switch traj_type
    % Generate new trajectories
    case 'Generated'
        [traj_xdata traj_ydata] = GetTraj(nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y);
           
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


% Getting the Size of the RoI
maxim = [max(traj_xdata,[],'all'), max(traj_ydata,[],'all') ];
rLimit = sens_range + max(maxim, [],'all');


%% Sensor Planning
% PATTERN SEARCH
[sensor_line_x sensor_line_y, deploy_times_line, sensor_orient, robot_traj_x, robot_traj_y, robot_times] = GetLineSensorDeployment(traj_xdata,traj_ydata, sens_num, sens_range, num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y,pat_init, num_rand_starts, margin,nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y, obstacles_check, num_obstacles, adaptive);

% Building Comparative Models
% UNIFORM
[sensor_x_pos_uni,sensor_y_pos_uni] = getUniformDeployment(sens_num,sens_range,rLimit);
sensor_x_pos_uni = sensor_x_pos_uni + LKP_x;
sensor_y_pos_uni = sensor_y_pos_uni + LKP_y;

% RING OF FIRE
[sensor_x_pos_ring,sensor_y_pos_ring] = getRingOfFire(sens_num,sens_range);
sensor_x_pos_ring = sensor_x_pos_ring + LKP_x;
sensor_y_pos_ring = sensor_y_pos_ring + LKP_y;

% RANDOM
[sensor_x_pos_rand,sensor_y_pos_rand] = getRandomDeployment(sens_num,rLimit,sens_range);
sensor_x_pos_rand = sensor_x_pos_rand + LKP_x;
sensor_y_pos_rand = sensor_y_pos_rand + LKP_y;

%% TESTING

num_test = 10;

h = waitbar(0,['Test Number 0 out of ' num2str(num_test)]);

for num_tests = 1:num_test
    
    waitbar(num_tests/num_test,h,['Test Number ' num2str(num_tests) ' out of ' num2str(num_test)]);
    
    %% Test Trajectories Creation
    switch traj_type
        % Generate new trajectories
        case 'Generated'
            [traj_xdata_test traj_ydata_test] = GetTraj(nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y);
            
            % Code to remove parts of the trajectory that are prior to the deployment start time
            if startTime > 0
                [traj_xdata_test, traj_ydata_test] = GetTrajReduction(traj_xdata_test, traj_ydata_test, times, startTime);
            end
            
            
        % Use saved trajectories
        case 'Saved'
            traj_xdata_test = cell2mat(traj_xdata_test_comp(1,num_tests));
            traj_ydata_test = cell2mat(traj_ydata_test_comp(1,num_tests));
    end
    
    
    %% PATTERN SEARCH
    % Converting to Directional
    [pat_search_planx, pat_search_plany] = GetDirectionalSectorGeometry(sensor_line_x, sensor_line_y, sector_angle, sens_range,curve_approx);
    
    % Intersection Check
    [num_int_pat] = GetMultiLineInt3(traj_xdata_test, traj_ydata_test, pat_search_planx, pat_search_plany, startTime, times,deploy_times_line, margin);
    
    num_int_pat_tests(num_tests,:) = sum(num_int_pat, 'all'); % Total number of intersections
    
    
    %% UNIFORM
    % Converting to Directional
    [uniform_planx, uniform_plany] = GetDirectionalSectorGeometry(sensor_x_pos_uni,sensor_y_pos_uni, sector_angle, sens_range,curve_approx);
    
    % Intersection Check
    [num_int_uni] = GetMultiLineInt2(traj_xdata_test, traj_ydata_test, uniform_planx, uniform_plany,margin);
    
    num_int_uni_tests(num_tests,:) = sum(num_int_uni, 'all'); % Total number of intersections
    
    %% RING OF FIRE
    % Converting to Directional
    [ring_planx, ring_plany] = GetDirectionalSectorGeometry(sensor_x_pos_ring,sensor_y_pos_ring, sector_angle, sens_range,curve_approx);
    
    % Intersection Check
    [num_int_ring] = GetMultiLineInt2(traj_xdata_test, traj_ydata_test, ring_planx, ring_plany, margin);
    
    num_int_ring_tests(num_tests,:) = sum(num_int_ring,'all');
    
    %% RANDOM
    % Converting to Directional
    [rand_planx, rand_plany] = GetDirectionalSectorGeometry(sensor_x_pos_rand, sensor_y_pos_rand, sector_angle, sens_range,curve_approx);
    
    % Intersection Check
    [num_int_rand] = GetMultiLineInt2(traj_xdata_test, traj_ydata_test, rand_planx, rand_plany, margin);
    
    num_int_rand_tests(num_tests,:) = sum(num_int_rand,'all');
    
end
close(h)

% Averaging the results of the tests
num_int_pat = sum(num_int_pat_tests, 'All')/num_test;
num_int_uni = sum(num_int_uni_tests,'All')/num_test;
num_int_ring = sum(num_int_ring_tests,'All')/num_test;
num_int_rand = sum(num_int_rand_tests,'All')/num_test;


%% Saving Data 
% Save Variables
save(sprintf('%s\\data\\Comparative Study\\CompStudyWorkspace%u.mat', pwd, test_index));


%% Plotting
% PATTERN SEARCH
pat_figure = figure;
hold on
patch(pat_search_planx', pat_search_plany','k')
hold on
scatter(LKP_x, LKP_y,'r','x')
xlim([-rLimit rLimit])
ylim([-rLimit rLimit])
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',10, 'FontName', 'Times New Roman')
box on
axis equal

% UNIFORM
uniform_figure = figure;
hold on
patch(uniform_planx', uniform_plany','k')
hold on
scatter(LKP_x, LKP_y,'r','x')
xlim([-rLimit rLimit])
ylim([-rLimit rLimit])
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal

% RING OF FIRE
ring_figure = figure;
hold on
patch(ring_planx', ring_plany','k')
hold on
scatter(LKP_x, LKP_y,'r','x')
xlim([-rLimit rLimit])
ylim([-rLimit rLimit])
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal

% RANDOM
rand_figure = figure;
hold on
patch(rand_planx', rand_plany','k')
hold on
scatter(LKP_x, LKP_y,'r','x')
xlim([-rLimit rLimit])
ylim([-rLimit rLimit])
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal

%% Saving Plots
saveas(figure(pat_figure),[pwd sprintf('\\Figures\\Comparative Study\\Pat%u.fig', test_index)]);
saveas(figure(uniform_figure),[pwd sprintf('\\Figures\\Comparative Study\\Uni%u.fig', test_index)]);
saveas(figure(ring_figure),[pwd sprintf('\\Figures\\Comparative Study\\Ring%u.fig', test_index)]);
saveas(figure(rand_figure),[pwd sprintf('\\Figures\\Comparative Study\\Rand%u.fig', test_index)]);

end
