function [sensor_x_pos sensor_y_pos, sensor_linex, sensor_liney] = GetDirectionalSensorPose(traj_xdata, traj_ydata, sensor_x_pos_mul, sensor_y_pos_mul, sector_angle,curve_approx, sens_range, margin)

[x_coord, y_coord] = GetDirectionalSectorGeometry(sensor_x_pos_mul, sensor_y_pos_mul, sector_angle, sens_range, curve_approx);

%% Determining Number of Intersections

[num_int_pat_dir] = GetMultiLineInt(traj_xdata, traj_ydata, x_coord, y_coord, margin);

% Determining the sensor pose that maximizes trajectory intersections
[val,row] = max(sum(num_int_pat_dir,2));

% Directional Sensor Position
sensor_x_pos = x_coord(row,:);
sensor_y_pos = y_coord(row,:);

% Coordinates defining line for directional sensor
sensor_linex = sensor_x_pos_mul(row,:);
sensor_liney = sensor_y_pos_mul(row,:);

%% Plotting
% figure
% plot(traj_xdata',traj_ydata')
% hold on
% plot(x_coord(2,:)',y_coord(2,:)','k');
% hold on
% plot(sensor_x_pos_mul,sensor_y_pos_mul,'k')
end