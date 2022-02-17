function [traj_xdata traj_ydata] = GetBoundedBoxTraj(traj_xdata, traj_ydata, sensor_x_pos_mul, sensor_y_pos_mul,margin)
% Finds the set of trajectories that pass through a bounding box around the sensor position

x_uB = max(sensor_x_pos_mul) + margin;
x_lB = min(sensor_x_pos_mul) - margin;
y_uB = max(sensor_y_pos_mul) + margin;
y_lB = min(sensor_y_pos_mul) - margin;

if x_uB == x_lB
    x_lB = max(sensor_x_pos_mul) - margin;
    x_uB = min(sensor_x_pos_mul) + margin;
end

if y_uB == y_lB
    y_lB = max(sensor_y_pos_mul) - margin;
    y_uB = min(sensor_y_pos_mul) + margin;
end

%%
% Find trajectories that have values within the required bounds
traj_checkx = sum(and(x_lB <= traj_xdata , traj_xdata <= x_uB),2);
traj_checky = sum(and(y_lB <= traj_ydata , traj_ydata <= y_uB),2);

traj_checkx = traj_checkx > 0;
traj_checky = traj_checky > 0;

rows = and(traj_checkx, traj_checky);

% Multiply by 1 if satisfy previous inequality otherwise multiply by 0
traj_xdata = traj_xdata.*rows;
traj_ydata = traj_ydata.*rows;

% Removing zeroed trajectories
traj_xdata(all(~traj_xdata,2), : ) = [];
traj_ydata(all(~traj_ydata,2), : ) = [];

end