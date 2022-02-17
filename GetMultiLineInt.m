function [num_int_seg] = GetMultiLineInt(traj_xdata, traj_ydata, sensor_x_pos, sensor_y_pos,margin)
% A function that takes in matrices for multi-segmented lines (traj_xdata)
% and multiple sensor positions (sensor_x_pos) and outputs a vector of
% trajectory intersections where each row corresponds to each sensor 

for index = 1:size(sensor_x_pos,1)
    % Getting only the trajectories that are near the sensor
    
    if index == 1
        [traj_xdata_red traj_ydata_red] = GetBoundedBoxTraj(traj_xdata, traj_ydata, sensor_x_pos(index,:), sensor_y_pos(index,:),margin);
    else
        for rem_index = 1:size(traj_xdata_rem,1)
            a = ismember(traj_xdata,traj_xdata_rem(rem_index,:),'rows');
            b = ismember(traj_ydata,traj_ydata_rem(rem_index,:),'rows');
            c = and(a,b);
            row = find(c == 1);
           
            traj_xdata(row,:) = [];
            traj_ydata(row,:) = [];
        end
        
        [traj_xdata_red traj_ydata_red] = GetBoundedBoxTraj(traj_xdata, traj_ydata, sensor_x_pos(index,:), sensor_y_pos(index,:),margin);
    end  
        
    for sensor_ind = 1:size(sensor_x_pos,2)-1
        [int] = IntCheck(traj_xdata_red,traj_ydata_red, sensor_x_pos(index,sensor_ind:sensor_ind+1), sensor_y_pos(index,sensor_ind:sensor_ind+1));
        
        traj_xdata_rem = traj_xdata_red.*int;
        traj_ydata_rem = traj_ydata_red.*int;
        
        traj_xdata_rem = traj_xdata_rem(any(traj_xdata_rem,2),:);
        traj_ydata_rem = traj_ydata_rem(any(traj_ydata_rem,2),:);
        
        num_int_seg(index,sensor_ind)= sum(int,'all');
    end
end
end