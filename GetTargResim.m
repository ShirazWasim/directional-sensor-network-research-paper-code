function [targ_xdata, targ_ydata, int_time, int_points, ang1] = GetTargResim(targ_xdata, targ_ydata, sensor_dir_x, sensor_dir_y,deploy_times_line, vTmean, vTstd, times, timeStep, startTime, endTime,init_time, clue_xpos, clue_ypos, clue_time_column,obstacles_check, num_obstacles, adaptive, margin)

%% Resimulating trajectory from LKP
n_targ = 2000;
adaptive = 'Yes';
targ_time = 0;
targ_new_xdata1 = [];
targ_new_ydata1 = [];

while isempty(targ_new_xdata1)
[targ_new_xdata targ_new_ydata] = GetTraj(n_targ, vTmean, vTstd, times, timeStep, targ_time, endTime, clue_xpos,clue_ypos);

switch obstacles_check
    case "Include"
        [targ_new_xdata, targ_new_ydata] = obstaclecheck(targ_new_xdata, targ_new_ydata, vTmean, vTstd, times, timeStep, 0, endTime,adaptive, clue_xpos,clue_ypos, num_obstacles);
end

[targ_new_xdata targ_new_ydata] = GetTargInt3(targ_new_xdata, targ_new_ydata, sensor_dir_x, sensor_dir_y, deploy_times_line, times, targ_time, margin);

targ_new_xdata1 = vertcat(targ_new_xdata1,targ_new_xdata);
targ_new_ydata1 = vertcat(targ_new_ydata1,targ_new_ydata);


%% Finding target heading in same angular direction
ang1 = angle(clue_xpos + j*clue_ypos);

targ_last_colx = targ_new_xdata1(:,end) - clue_xpos;
targ_last_coly = targ_new_ydata1(:,end) - clue_ypos;

ang2 = angle(targ_last_colx + j.*targ_last_coly);

ang3 = find(and(ang2 <= ang1 + pi/8, ang2 >= ang1 - pi/8));

if ~isempty(ang3)
    targ_new_xdata1 = targ_new_xdata1(ang3(1),:);
    targ_new_ydata1 = targ_new_ydata1(ang3(1),:);
else
    targ_new_xdata1 = [];
    targ_new_ydata1 = [];
end

end

targ_xdata = [targ_xdata(1:clue_time_column) targ_new_xdata1];
targ_ydata = [targ_ydata(1:clue_time_column) targ_new_ydata1];

%% Finding Time and Point of Intersection
times = timeStep:timeStep:size(targ_xdata,2)*timeStep;
startTime = 0;

% Determining which sensor intercepts the target
[targ_new_xdata targ_new_ydata, sen_ind] = GetTargInt3(targ_xdata, targ_ydata, sensor_dir_x, sensor_dir_y, deploy_times_line, times, targ_time, margin);
sensor_dir_x = sensor_dir_x(sen_ind,:);
sensor_dir_y = sensor_dir_y(sen_ind,:);

col = [];
idx = [];

for sensor_ind = 1:size(sensor_dir_x,2)-1
    [int_1, column] = IntCheck(targ_xdata, targ_ydata, sensor_dir_x(1,sensor_ind:sensor_ind+1), sensor_dir_y(1,sensor_ind:sensor_ind+1));
    
    if int_1 > 0
        col = vertcat(col,column);
        idx = vertcat(idx, sensor_ind);
    end
    
end


%% Time and Position of Intersection
for index = 1:size(col,1)
    int_points = GetIntPoints([targ_xdata(col(index,:)) targ_xdata(col(index,:)+1); targ_ydata(col(index,:)) targ_ydata(col(index,:)+1)],[sensor_dir_x(idx(index,:)) sensor_dir_x(idx(index,:)+1); sensor_dir_y(idx(index,:)) sensor_dir_y(idx(index,:)+1)]);

    if isempty(int_points)
    else
    int_time = col(index,:)*timeStep;
    end

end

if size(int_points,2) > 1
    int_points = int_points(1,:);
    int_time = int_time(1,:);
end

end