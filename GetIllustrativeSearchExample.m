function [orig_data, new_data] = GetIllustrativeSearchExample(timeStep, startTime,runLength, endTime, times, nT, vTmean, vTstd, vTmax, margin, sens_num, sens_range, sector_angle, curve_approx, num_rand_starts, num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y, LKP_x, LKP_y, pat_init, obstacles_check, num_obstacles)
LKP_x = 0;
LKP_y = 0;
end_time_orig = endTime;

%% Trajectory Data Generation
switch obstacles_check
    case "Include"
        adaptive = 'No';
        init_time = 0;
        [traj_xdata traj_ydata] = GetTraj(nT, vTmean, vTstd, times, timeStep, init_time, endTime, LKP_x, LKP_y);
        
        [traj_xdata, traj_ydata] = obstaclecheck(traj_xdata, traj_ydata, vTmean, vTstd, times, timeStep, init_time, endTime,adaptive,LKP_x, LKP_y, num_obstacles);
        
        [traj_xdata, traj_ydata] = GetTrajReduction(traj_xdata, traj_ydata, times, startTime);
        
    case "Don't Include"
        adaptive = 'No';
        [traj_xdata traj_ydata] = GetTraj(nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y);
        
        [traj_xdata, traj_ydata] = GetTrajReduction(traj_xdata, traj_ydata, times, startTime);
end


%% Sensor Placement
[sensor_line_x sensor_line_y, deploy_times_line, sensor_orient, robot_traj_x, robot_traj_y, robot_times] = GetLineSensorDeployment(traj_xdata,traj_ydata, sens_num, sens_range, num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y,pat_init, num_rand_starts, margin,nT, vTmean, vTstd, times, timeStep, startTime, endTime, LKP_x, LKP_y, obstacles_check, num_obstacles, adaptive);
% load(sprintf('%s\\data\\Adaptive Deployment\\Adaptive Deployment(Before Replan).mat', pwd))

%% Saving Original Deployment
save(sprintf('%s\\data\\Adaptive Deployment\\Adaptive Deployment(Before Replan).mat', pwd),'sensor_line_x', 'sensor_line_y', 'deploy_times_line', 'sensor_orient', 'robot_traj_x', 'robot_traj_y', 'robot_times');

% Directional Sensor Construction
[sensor_dir_x, sensor_dir_y] = GetDirectionalSectorGeometry(sensor_line_x, sensor_line_y, sector_angle, sens_range, curve_approx);

% Original Deployment
[orig_deploy] = obstaclesplot(obstacles_check, num_obstacles)
hold on
patch(sensor_dir_x', sensor_dir_y','k')
hold on
scatter(LKP_x, LKP_y,'r','x','LineWidth', 1)
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal

%% Clue Find
% Target Trajectory Data
n_targ = 100;
adaptive = 'No';
targ_time = 0;

[targ_xdata targ_ydata] = GetTraj(n_targ, vTmean, vTstd, times, timeStep, targ_time, endTime, LKP_x, LKP_y);

switch obstacles_check
    case "Include"
        
        [targ_xdata, targ_ydata] = obstaclecheck(targ_xdata, targ_ydata, vTmean, vTstd, times, timeStep, init_time, endTime,adaptive, LKP_x, LKP_y, num_obstacles);
        
        [targ_xdata targ_ydata] = GetTargInt2(targ_xdata, targ_ydata, sensor_dir_x, sensor_dir_y, deploy_times_line, times, targ_time, margin);
        
    case "Don't Include"
        
        [targ_xdata targ_ydata] = GetTargInt2(targ_xdata, targ_ydata, sensor_dir_x, sensor_dir_y, deploy_times_line, times, targ_time, margin);     
end

% Getting a fast trajectory (for illustration only, so that the replanned sensors are spaced out)
furthest_dist = sortrows(sqrt((targ_xdata(:,end) - 0).^2 + (targ_ydata(:,end) - 0).^2));
fur_row = ceil(size(furthest_dist,1));

targ_xdata = targ_xdata(fur_row,:); % Target x-data
targ_ydata = targ_ydata(fur_row,:); % Target y-data


%% Sorting Original Planned Network
% Clue Time
clue_time = median(deploy_times_line); % Ensuring clue time is at median deployment time (so that half the sensors are deployed before replan)
clue_time_column = ceil(clue_time ./ timeStep);
clue_xpos = targ_xdata(:,clue_time_column); % x-position of clue 
clue_ypos = targ_ydata(:,clue_time_column); % y-position of clue


% Moving robots to clue position
last_deploy = robot_times <= clue_time; 
last_deploy_times = last_deploy.*robot_times; % Finding last deployment time for each robot (before clue time)

% Finding location of the last deployment position for each robot
for i = 1:size(robot_times,2)
    [de_val, row] = max(last_deploy_times(:,i)',[],'linear');
    robot_pos_time(i,:) = de_val;
    robot_pos_x(i,:) = robot_traj_x(row,i);
    robot_pos_y(i,:) = robot_traj_y(row,i);
end

% Determining the maximum time it will take for all the robots to reach the clue position
distances = sqrt((robot_pos_x - clue_xpos).^2 + (robot_pos_y - clue_xpos).^2);
recovery_time = distances./robot_speed;
[re_val, ~] = max(recovery_time',[],'linear'); % Maximum time for robots to reach clue position


% Resetting robot position (to clue position) and time (to 0)
robot_pos_x(:,1) = clue_xpos;
robot_pos_y(:,1) = clue_ypos;
robot_pos_time_orig = robot_pos_time + recovery_time; % The time at which each robot reaches the LKP
robot_pos_time(:,1) = 0;


% Determining the new deployment start time
time_est = (sqrt((0 - clue_xpos).^2 + (0 - clue_ypos).^2))/vTmax; %  Conservative estimate of how long it took the target to get to the LKP
elaps_time = clue_time - time_est; % Time elapsed since target was at the clue position
startTimeNew = elaps_time + re_val; % New start time


% Number of Sensors to Redeploy
num_deployed = sum(deploy_times_line <= clue_time,'All');
num_left = sens_num - num_deployed;


% Saving deployment positions and times of sensors already deployed and removing unused planned poses
sensor_line_x = sensor_line_x(1:num_deployed,:); % Linear-sensor data
sensor_line_y = sensor_line_y(1:num_deployed,:);
sensor_dir_x = sensor_dir_x(1:num_deployed,:); % Linear-directional sensor data
sensor_dir_y = sensor_dir_y(1:num_deployed,:);
deploy_times_line = deploy_times_line(1:num_deployed,:); % Re-setting deploy_times_line variable to remove the planned sensors that weren't deployed (by the clue time)
deploy_times_orig = deploy_times_line; % New variable for original deployment times
deploy_times_line(:,1) = 0; % Resetting original deployment times to 0 (for the re-planned simulation, these sensors are already deployed and so have a deployment time of 0)


%% Resimulating MonteCarlo Trajectories
endTime = startTimeNew + runLength; % New end time
times = 0:timeStep:endTime; % New time indices

init_time = 0;
adaptive = 'Yes'; % Setting the deployment type to adaptive (used in obstaclecheck code)
traj_xdata = []; % Resetting simulated trajectories
traj_ydata = [];

while size(traj_xdata,1) < nT
    [traj_xdata_adap traj_ydata_adap] = GetTraj(nT, vTmean, vTstd, times, timeStep, init_time, endTime, clue_xpos, clue_ypos); % Re-simulating trajectories to plan on
    
    switch obstacles_check
        case "Include"
            % Checking if the simulated trajectories intersect the obstacles and then resimulating those ones that are
            [traj_xdata_adap, traj_ydata_adap] = obstaclecheck(traj_xdata_adap, traj_ydata_adap, vTmean, vTstd, times, timeStep, init_time, endTime,adaptive,clue_xpos,clue_ypos, num_obstacles);    
        
        case "Don't Include"   
    end
    
    % Checking if the simulated trajectories intersect a previously deployed sensor 
    [traj_xdata_adap traj_ydata_adap] = GetTargInt2(traj_xdata_adap, traj_ydata_adap, sensor_dir_x, sensor_dir_y, deploy_times_line, times, startTimeNew, margin);
    
    % Reducing the trajectory portions considered (i.e. only the portion of the trajectory between the new start time and end time
    [traj_xdata_adap, traj_ydata_adap] = GetTrajReduction(traj_xdata_adap, traj_ydata_adap, times, startTimeNew);
    
    % Concatenating trajectories until more than nT (i.e. the number of trajectories to plan on) are found
    traj_xdata = vertcat(traj_xdata,traj_xdata_adap);
    traj_ydata = vertcat(traj_ydata,traj_ydata_adap);
end

% Taking only the nT (e.g. 10,000) trajectories
traj_xdata = traj_xdata(1:nT,:);
traj_ydata = traj_ydata(1:nT,:);


%% Sensor Placement
adap_deploy = waitbar(0,['Deploying Replanned Sensor 0 out of ' num2str(sens_num)]);

for sensor_index = num_deployed+1:sens_num
    
    waitbar(sensor_index/sens_num,adap_deploy,['Deploying Replanned Sensor ' num2str(sensor_index) ' out of ' num2str(sens_num)]);
    
    %% Sub-Region Optimization
    % Determining the optimal sub-region that contains the most amount of trajectories
    [sensor_location_line, deployment_time, robot_pos_x, robot_pos_y, robot_pos_time] = GetPatternSearchSensorPosition(traj_xdata, traj_ydata, times, startTimeNew, sens_range, num_robots, robot_speed, robot_pos_x, robot_pos_y, robot_pos_time, pat_init, clue_xpos,clue_ypos);
    
    %% Saving Robot Positions and Times
    robot_traj_x(sensor_index,:) = robot_pos_x;
    robot_traj_y(sensor_index,:) = robot_pos_y;
    robot_times(sensor_index,:) = robot_pos_time;
    
    %% Determining Optimal Orientation
    [sensor_line_opt_x, sensor_line_opt_y, num_int, orient] = GetOptimalLineOrientation(traj_xdata, traj_ydata, sensor_location_line, startTimeNew, times, deployment_time, sens_range, num_rand_starts, margin);
    
    % Saving the x-y coordinates describing the sensor pose
    sensor_line_x(sensor_index,:) = sensor_line_opt_x;
    sensor_line_y(sensor_index,:) = sensor_line_opt_y;
    
    sensor_orient(sensor_index,:) = orient;
    %% Deployment Times
    % Robot Delivery Method (saving times outputted from pattern search):
    deploy_times_line(sensor_index,:) = deployment_time;
    
    %% Trajectory Resimulation
    [traj_xdata traj_ydata] = GetNewTraj(traj_xdata, traj_ydata, sensor_line_x, sensor_line_y, deploy_times_line, nT, vTmean, vTstd, times, timeStep, startTimeNew, endTime,sensor_index, margin, clue_xpos, clue_ypos, obstacles_check, num_obstacles, adaptive);
    
end
close(adap_deploy);

% [figure] = obstaclesplot(obstacles_check, num_obstacles)
% hold on
% plot(traj_xdata',traj_ydata')

save(sprintf('%s\\data\\Adaptive Deployment\\Adaptive Deployment(After Replan).mat', pwd));

%% Directional Sensors
% Converting linear sensors to directional sensors
[sensor_dir_x, sensor_dir_y] = GetDirectionalSectorGeometry(sensor_line_x, sensor_line_y, sector_angle, sens_range, curve_approx);

%% Get Resim Target Trajectory
% Re-simulating the portion of the target's trajectory from the clue position onwards and ensuring it intersects a sensor
endTime = end_time_orig - clue_time;
[targ_xdata, targ_ydata, int_time, int_points, phi] = GetTargResim(targ_xdata, targ_ydata, sensor_dir_x(num_deployed+1:end,:), sensor_dir_y(num_deployed+1:end,:), deploy_times_line(num_deployed+1:end,:), vTmean, vTstd, times, timeStep, startTimeNew, endTime, init_time, clue_xpos, clue_ypos, clue_time_column,obstacles_check, num_obstacles, adaptive, margin);
targ_times = timeStep:timeStep:size(targ_xdata,2)*timeStep; % Full target trajectory indices


%% Correcting deployment times
deploy_times_line = deploy_times_line + clue_time; % Correcting the deployment times of the sensors
deploy_times_line(1:num_deployed) = deploy_times_orig; % Inputting the original sensor deployment times

% Original Deployment (Sorting deployment poses according to time)
orig_data = sortrows(horzcat(deploy_times_line(1:num_deployed), sensor_dir_x(1:num_deployed,:), sensor_dir_y(1:num_deployed,:))); 
deploy_orig_times = orig_data(:,1); % Original Sensor Deployment times in order
sensor_dir_orig_x = orig_data(:, 2:10); % Original linear-directional sensors x-positions in deployment time order
sensor_dir_orig_y = orig_data(:, 11:end); % Original linear-directional sensors y-positions in deployment time order

% Re-planned Deployment (Sorting deployment poses according to time)
new_data = sortrows(horzcat(deploy_times_line(num_deployed+1:end,:), sensor_dir_x(num_deployed+1:end,:), sensor_dir_y(num_deployed+1:end,:))); 
deploy_new_times = new_data(:,1); % Replanned Sensor Deployment times in order
sensor_dir_new_x = new_data(:, 2:10); % Replanned linear-directional sensors x-positions in deployment time order
sensor_dir_new_y = new_data(:, 11:end); % Replanned linear-directional sensors x-positions in deployment time order

% Target Trajectory Times
times = timeStep:timeStep:size(targ_xdata,2)*timeStep;

%% Saving Data
save(sprintf('%s\\data\\Adaptive Deployment\\Adaptive Deployment%u.mat', pwd, 1));


%% Plot Orig Deployment (1st Time Instance)
origtime1 = deploy_orig_times(1,1);
origtarg_col1 = find(targ_times <= origtime1 + elaps_time);


[origtime1_figure] = obstaclesplot(obstacles_check, num_obstacles)
hold on
patch(sensor_dir_x(1,:)', sensor_dir_y(1,:)','k')
hold on
plot(targ_xdata(:,origtarg_col1)',targ_ydata(:,origtarg_col1)', 'b')
hold on
scatter(LKP_x, LKP_y,'r','x','LineWidth', 1)
hold on
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal

%% Plot Orig Deployment (2nd Time Instance)
origind2 = ceil(num_deployed/3);
origtime2 = deploy_orig_times(origind2,:);
origtime2_ind = (1:origind2);
targ_col2 = find(targ_times <= origtime2 + elaps_time);


[origtime2_figure] = obstaclesplot(obstacles_check, num_obstacles)
hold on
patch(sensor_dir_x(origtime2_ind,:)', sensor_dir_y(origtime2_ind,:)','k')
hold on
plot(targ_xdata(:,targ_col2)',targ_ydata(:,targ_col2)', 'b')
hold on
scatter(LKP_x, LKP_y,'r','x','LineWidth', 1)
hold on
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal


%% Plot Orig Deployment (3rd Time Instance)
origind3 = ceil(2*num_deployed/3);
origtime3 = deploy_orig_times(origind3 ,:);
origtime3_ind = (1:origind3 );
targ_col3 = find(targ_times <= origtime3 + elaps_time);


[origtime3_figure] = obstaclesplot(obstacles_check, num_obstacles)
hold on
patch(sensor_dir_x(origtime3_ind,:)', sensor_dir_y(origtime3_ind,:)','k')
hold on
plot(targ_xdata(:,targ_col3)',targ_ydata(:,targ_col3)', 'b')
hold on
scatter(LKP_x, LKP_y,'r','x','LineWidth', 1)
hold on
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal

%% Plot Orig Deployment (4th Time Instance / Clue Time)
origtime4 = deploy_orig_times(end,:)
targ_col4 = find(targ_times <= clue_time + elaps_time);

[origtime4_figure] = obstaclesplot(obstacles_check, num_obstacles)
hold on
patch(sensor_dir_orig_x', sensor_dir_orig_y', 'k')
hold on
plot(targ_xdata(:,targ_col4)',targ_ydata(:,targ_col4)', 'b')
hold on
scatter(LKP_x, LKP_y,'r','x','LineWidth', 1)
hold on
scatter(clue_xpos,clue_ypos,'b','x','LineWidth', 1)
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal


%% Plot Replan Deployment (1st Time Instance)
newtime1 = deploy_new_times(1,:)

if newtime1 + elaps_time <= int_time
newtarg_col1 = find(targ_times <= newtime1 + elaps_time);

[newtime1_figure] = obstaclesplot(obstacles_check, num_obstacles)
hold on
patch(sensor_dir_orig_x', sensor_dir_orig_y', 'k')
hold on
patch(sensor_dir_new_x(1,:)', sensor_dir_new_y(1,:)','m')
hold on
plot(targ_xdata(:,newtarg_col1)',targ_ydata(:,newtarg_col1)', 'b')
hold on
scatter(LKP_x, LKP_y,'r','x','LineWidth', 1)
hold on
scatter(clue_xpos,clue_ypos,'b','x','LineWidth', 1)
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal
end

%% Plot Replan Deployment (2nd Time Instance)
newind2 = ceil(num_left/3);
newtime2 = deploy_new_times(newind2,:);

if newtime2 + elaps_time <= int_time
newtime2_ind = (1:newind2);
newtarg_col2 = find(targ_times <= newtime2 + elaps_time);


[newtime2_figure] = obstaclesplot(obstacles_check, num_obstacles)
hold on
patch(sensor_dir_orig_x', sensor_dir_orig_y', 'k')
hold on
patch(sensor_dir_new_x(newtime2_ind,:)', sensor_dir_new_y(newtime2_ind,:)','m')
hold on
plot(targ_xdata(:,newtarg_col2)',targ_ydata(:,newtarg_col2)', 'b')
hold on
scatter(LKP_x, LKP_y,'r','x','LineWidth', 1)
hold on
scatter(clue_xpos,clue_ypos,'b','x','LineWidth', 1)
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal
end

%% Plot Replan Deployment (3rd Time Instance)
newind3 = ceil(2*num_left/3);
newtime3 = deploy_new_times(newind3,:);


if newtime3 + elaps_time <= int_time
newtime3_ind = (1:newind3);
newtarg_col3 = find(targ_times <= newtime3 + elaps_time);


[newtime3_figure] = obstaclesplot(obstacles_check, num_obstacles)
hold on
patch(sensor_dir_orig_x', sensor_dir_orig_y', 'k')
hold on
patch(sensor_dir_new_x(newtime3_ind,:)', sensor_dir_new_y(newtime3_ind,:)','m')
hold on
plot(targ_xdata(:,newtarg_col3)',targ_ydata(:,newtarg_col3)', 'b')
hold on
scatter(LKP_x, LKP_y,'r','x','LineWidth', 1)
hold on
scatter(clue_xpos,clue_ypos,'b','x','LineWidth', 1)
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal
end


%% Plot Replan Deployment (4th Time Instance / Interception)
[newmaxtime, newmaxind] = max(deploy_new_times, [],'Linear');
newtime4 = int_time;
newtime4_ind = find(deploy_new_times <= int_time);
newtarg_col4 = find(targ_times <= int_time);


[newtime4_figure] = obstaclesplot(obstacles_check, num_obstacles)
hold on
patch(sensor_dir_orig_x', sensor_dir_orig_y', 'k')
hold on
patch(sensor_dir_new_x(newtime4_ind,:)', sensor_dir_new_y(newtime4_ind,:)','m')
hold on
plot(targ_xdata(:,newtarg_col4)',targ_ydata(:,newtarg_col4)', 'b')
hold on
scatter(LKP_x, LKP_y,'r','x','LineWidth', 1)
hold on
scatter(clue_xpos,clue_ypos,'b','x','LineWidth', 1)
hold on
scatter(int_points(1,1),int_points(2,1),'b','x','LineWidth', 1)
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal


%% Full Target Trajectory (if no intersection)
[targ_figure] = obstaclesplot(obstacles_check, num_obstacles)
hold on
plot(targ_xdata',targ_ydata', 'b')
hold on
scatter(LKP_x, LKP_y,'r','x','LineWidth', 1)
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal


%% Full Sensor Network
[fullnetwork_figure] = obstaclesplot(obstacles_check, num_obstacles)
hold on
patch(sensor_dir_orig_x', sensor_dir_orig_y', 'k')
hold on
patch(sensor_dir_new_x', sensor_dir_new_y','m')
hold on
scatter(LKP_x, LKP_y,'r','x','LineWidth', 1)
hold on
scatter(clue_xpos,clue_ypos,'b','x','LineWidth', 1)
xlabel('x distance from LKP (m)')
ylabel('y distance from LKP (m)')
set(gca,'FontSize',8, 'FontName', 'Times New Roman')
box on
axis equal

end