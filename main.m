close all
clear
rng(0)

% Close all Windows
openWindows = findall(0,'type','figure','tag','TMWWaitbar')
delete(openWindows)


%% Parameters
% Time Parameters
timeStep = 10; % Timesteps for trajectories
startTime = 1800; % Deployment start time
endTime = 5400; % Deployment end time
runLength = endTime - startTime; % Duration of search
times = 0:timeStep:endTime; % Simulation times from start time to end time

% Target Trajectory Parameters
nT = 1000; % Number of simulated targets (for planning)
vTmean = 0.7;  % Target speed distribution mean
vTstd = vTmean/3; % Target speed distribution std dev
vTmax = vTmean+3*vTstd; % Maximum target speed
traj_type = 'Generated'; % 'Saved' if using previously generated trajectories or 'Generated' if creating new ones

% Bounding Box Parameter
margin = timeStep*vTmax; % Size of margin around bounding box

% Sensor Parameters
sens_num = 10;  % Number of sensors
sens_range = 20; % Sensing Range

% Directional Sensor Paramaters
sector_angle = pi/4; % Angular sensing range
curve_approx = sector_angle/6; % Approximation of curved segments on directional and omni-directional sensors
num_rand_starts = 10; % Number of Initializations in Orientation Optimization

% Delivery Robot Parameters
num_robots = 3; % Number of delivery robots
robot_speed = 10; % Delivery robot speed
robot_init_pos_x = zeros(num_robots,1); % Robots initial x-positions
robot_init_pos_y = zeros(num_robots,1); % Robots initial y-positions

% LKP
LKP_x = 0;
LKP_y = 0;

% Pattern Search Parameters
pat_init = 10; % Number of pattern search starting positions

% Obstacles
obstacles = ["Include" , "Don't Include"];
obs_val = 2; % 1 is "Include" Obstacles and 2 is "Don't Include" Obstacles
num_obstacles = 2; % Number of Obstacles Included (Max 9)

% Adaptive Deployment
adaptive = ["Yes", "No"];


%% Simple Deployment
% Clearing Contents of Data File
rmdir(sprintf('%s\\data\\Simple Deployment',pwd),'s')
mkdir(sprintf('%s\\data\\',pwd),'Simple Deployment')

% Clearing Contents of Figures File
rmdir(sprintf('%s\\Figures\\Simple Deployment',pwd),'s')
mkdir(sprintf('%s\\Figures\\',pwd),'Simple Deployment')

for test_index = 1:size(num_robots,2);
    robot_init_pos_x = zeros(num_robots(:,test_index),1);
    robot_init_pos_y = zeros(num_robots(:,test_index),1);
    
    [num_int_line_test, deploy_times_line] = GetSimpleDeploymentResults(timeStep, startTime,runLength, endTime, times, nT, vTmean, vTstd, vTmax, margin, sens_num, sens_range, sector_angle, curve_approx, num_rand_starts, num_robots(1,test_index), robot_speed(1,test_index), robot_init_pos_x, robot_init_pos_y, LKP_x, LKP_y, adaptive(1,2), num_obstacles, pat_init, obstacles(1,obs_val), test_index, traj_type);
    num_int_line_test_res(test_index,:) = num_int_line_test;
    
end

%% Sensing Model Comparison
% % This section compares sensor network's planned using linear,linear-directional, directional and omnidirectional sensing models
% 
% % Clearing Contents of Data File
% rmdir(sprintf('%s\\data\\Sensing Model Study',pwd),'s')
% mkdir(sprintf('%s\\data\\',pwd),'Sensing Model Study')
% 
% % Clearing Contents of Figures File
% rmdir(sprintf('%s\\Figures\\Sensing Model Study',pwd),'s')
% mkdir(sprintf('%s\\Figures\\',pwd),'Sensing Model Study')
% 
% 
% for test_index = 1:size(sector_angle,2);
% %     robot_init_pos_x = zeros(num_robots(:,test_index),1);
% %     robot_init_pos_y = zeros(num_robots(:,test_index),1);
%     
%     
%     [num_int_line_test, num_int_line_dir, num_int_dir, num_int_omni] = GetSensModelComparisonResults(timeStep, startTime,runLength, endTime, times, nT, vTmean, vTstd, vTmax, margin, sens_num, sens_range, sector_angle(1,test_index), curve_approx, num_rand_starts, num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y, LKP_x, LKP_y, adaptive(1,2), num_obstacles, pat_init, obstacles(1,obs_val), test_index, traj_type);  
% 
%     num_int_line_test_res(test_index,:) = num_int_line_test;
%     num_int_line_dir_res(test_index,:) = num_int_line_dir;
%     num_int_dir_res(test_index,:) = num_int_dir;
%     num_int_omni_res(test_index,:) = num_int_omni;
% end
 
%% Comparative Study
% This code compares the proposed methodology to three comparative topologies

% Clearing Contents of Data File
rmdir(sprintf('%s\\data\\Comparative Study',pwd),'s')
mkdir(sprintf('%s\\data\\',pwd),'Comparative Study')

% Clearing Contents of Figures File
rmdir(sprintf('%s\\Figures\\Comparative Study',pwd),'s')
mkdir(sprintf('%s\\Figures\\',pwd),'Comparative Study')


for test_index = 1:size(sens_range,2);
    [no_int_pat, no_int_uni, no_int_ring, no_int_rand] = GetComparativeStudyResults(timeStep, startTime,runLength, endTime, times, nT, vTmean, vTstd, vTmax, margin, sens_num(1,test_index), sens_range(1,test_index), sector_angle, curve_approx, num_rand_starts, num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y, LKP_x, LKP_y, pat_init, adaptive(1,2), obstacles(1,obs_val), num_obstacles, test_index, traj_type);
    
    no_int_pat_res(test_index,:) = no_int_pat;
    no_int_uni_res(test_index,:) = no_int_uni;
    no_int_ring_res(test_index,:) = no_int_ring;
    no_int_rand_res(test_index,:) = no_int_rand;
end

comp_data = horzcat(no_int_pat_res, no_int_uni_res, no_int_rand_res, no_int_ring_res);

save(sprintf('%s\\data\\Comparative Study\\CompStudyVariables.mat', pwd));

%% Illustrative Search Example
% This code outputs an illustrative search example with an initial planning and subsequent replanning after a clue find

% [orig_data, new_data] = GetIllustrativeSearchExample(timeStep, startTime,runLength, endTime, times, nT, vTmean, vTstd, vTmax, margin, sens_num, sens_range, sector_angle, curve_approx, num_rand_starts, num_robots, robot_speed, robot_init_pos_x, robot_init_pos_y, LKP_x, LKP_y, pat_init, obstacles(1,obs_val),num_obstacles);
