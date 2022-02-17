function [traj_int_x traj_int_y, z] = GetTargInt3(traj_xdata, traj_ydata, sensor_x_pos, sensor_y_pos, deploy_times_line, times, startTime, margin)
% A function that takes in matrices for multi-segmented lines (traj_xdata)
% and multiple sensor positions (sensor_x_pos) and outputs a vector of
% trajectory intersections where each row corresponds to each sensor 

valid_idx = times >= startTime;
valid_idx2 = valid_idx.*times;
valid_idx2 = valid_idx2(valid_idx2~=0);
row2 = [];
traj_int_x = [];
traj_int_y = [];

for index = 1:size(sensor_x_pos,1)
    % Reducing trajectories  
    valid_idx3 = valid_idx2 >= deploy_times_line(index,1);
    
    if index == 1
        traj_xdata_time_red = traj_xdata(:,valid_idx3); %.*valid_idx3;
        traj_ydata_time_red = traj_ydata(:,valid_idx3); %.*valid_idx3;
       
        [traj_xdata1, traj_ydata1,traj_xdata2,traj_ydata2] = GetBoundedBoxTraj2(traj_xdata_time_red, traj_ydata_time_red, sensor_x_pos(index,:), sensor_y_pos(index,:),margin);
        
        traj_xdata_rem = [];
        traj_ydata_rem = [];
    else
        if isempty(traj_xdata_rem)
            traj_xdata_time_red = traj_xdata(:,valid_idx3); %.*valid_idx3;
            traj_ydata_time_red = traj_ydata(:,valid_idx3); %.*valid_idx3;   
            
            [traj_xdata1, traj_ydata1,traj_xdata2,traj_ydata2] = GetBoundedBoxTraj2(traj_xdata_time_red, traj_ydata_time_red, sensor_x_pos(index,:), sensor_y_pos(index,:),margin);
        else
            a = ismember(traj_xdata_time_red,traj_xdata_rem,'rows');
            row2 = find(a == 1);
            % row2 = vertcat(row2,row);
%             z = index.*ones(size(traj_xdata(row2,:),1),1);
%             traj_int_x = vertcat(traj_int_x, horzcat(traj_xdata(row2,:),z));
%             traj_int_y = vertcat(traj_int_y, horzcat(traj_ydata(row2,:),z));
             traj_int_x = vertcat(traj_int_x, traj_xdata(row2,:));
             traj_int_y = vertcat(traj_int_y, traj_ydata(row2,:));

            traj_xdata_time_red = traj_xdata(:,valid_idx3); %.*valid_idx3;
            traj_ydata_time_red = traj_ydata(:,valid_idx3); %.*valid_idx3;
            
            traj_xdata_time_red(row2,:) = [];
            traj_ydata_time_red(row2,:) = [];
            
            traj_xdata(row2,:) = [];
            traj_ydata(row2,:) = [];
            
            traj_xdata_rem = [];
            traj_ydata_rem = [];
            
            [traj_xdata1, traj_ydata1,traj_xdata2,traj_ydata2] = GetBoundedBoxTraj2(traj_xdata_time_red, traj_ydata_time_red, sensor_x_pos(index,:), sensor_y_pos(index,:),margin);
        end
    end
       
%%  Checking Intersection
    for sensor_ind = 1:size(sensor_x_pos,2)-1
        if isempty(traj_xdata2)
            num_int_seg(index,sensor_ind)= 0; % Number of intersections for an empty matrix is zero
            
            % Saving trajectories that need to be removed
            traj_xdata_rem = []; 
            traj_ydata_rem = [];
            
        else
            for traj_index = 1:size(traj_xdata2,1)
                % Getting the trajectory to check
                traj_xdata_red_1 = traj_xdata2(traj_index,:);
                traj_ydata_red_1 = traj_ydata2(traj_index,:);
                
                % Removing zero columns
                col_idx = find(traj_xdata_red_1 ~= 0);
                traj_xdata_red_2 = traj_xdata_red_1(:,col_idx);
                traj_ydata_red_2 = traj_ydata_red_1(:,col_idx);
                
                % Checking intersections
                [int_1] = IntCheck(traj_xdata_red_2,traj_ydata_red_2, sensor_x_pos(index,sensor_ind:sensor_ind+1), sensor_y_pos(index,sensor_ind:sensor_ind+1));
                int(traj_index,:) = int_1;
            end
            
            % Getting trajectory that was intersected, to remove
            row_idx = find(int == 1);
            if ~isempty(row_idx)
                traj_xdata_rem = vertcat(traj_xdata_rem, traj_xdata1(row_idx,:));
                traj_ydata_rem = vertcat(traj_ydata_rem, traj_ydata1(row_idx,:));
            end 
            
            
          
            % Adding the number of trajectories intersected by this sensor
            num_int_seg(index,sensor_ind)= sum(int,'all');
            
            if size(traj_xdata_rem,1) > 0
                traj_xdata1(row_idx,:) = [];
                traj_ydata1(row_idx,:) = [];
                
                traj_xdata2(row_idx,:) = [];
                traj_ydata2(row_idx,:) = [];
                z = index;
            end
            
            int = [];
        end
        
    end 
    

end

if ~isempty(traj_xdata_rem)
    a = ismember(traj_xdata_time_red,traj_xdata_rem,'rows');
    row2 = find(a == 1);
    traj_xdata(row2,:) = [];
    traj_ydata(row2,:) = [];

end
