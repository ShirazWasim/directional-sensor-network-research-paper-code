function [sensor_x_pos_mul sensor_y_pos_mul] = GetSensorLines(sensor_location,sens_range,sens_orient_num, sensor_type)
% A function to generate the geometries of the multiple sensor orientations
% to be tested.

orient_step = 2*pi/sens_orient_num; % how many orientation angles to check

% directional sensors require twice the orientations of line sensors as
% both directions need to be checked for directional
switch sensor_type
    case 'Line'
        theta = 0:orient_step:pi;
        theta = theta(1:end-1);
    case 'Directional'
        theta = 0:orient_step:2*pi;
        theta = theta(1:end-1);
end

%% Generating sensors with start point on the circumference of the circle
sensor_xstart = sensor_location(1,1) - 0.5*(sens_range.*cos(theta));
sensor_ystart = sensor_location(1,2) - 0.5*(sens_range.*sin(theta));

sensor_xend = sensor_location(1,1) + 0.5*(sens_range.*cos(theta));
sensor_yend = sensor_location(1,2) + 0.5*(sens_range.*sin(theta));

sensor_x_pos_mul = [sensor_xstart ; sensor_xend]';
sensor_y_pos_mul = [sensor_ystart ; sensor_yend]';

%% Generating sensors with start point at the center of the circle
% sensor_xcenter = sensor_location(1,1);
% sensor_ycenter = sensor_location(1,2);
% 
% theta = 0:sens_orient_num:2*pi;
% 
% x_new = sensor_xcenter + sens_range.*cos(theta);
% y_new = sensor_ycenter + sens_range.*sin(theta);
% 
% x_cent = repmat(sensor_xcenter, 1,size(x_new,2));
% y_cent = repmat(sensor_ycenter, 1,size(x_new,2));
% 
% sensor_x_pos_mul = [x_cent ; x_new]';
% sensor_y_pos_mul = [y_cent ; y_new]';

%% Plotting
% figure
% plot(sensor_x_pos_mul',sensor_y_pos_mul','k')
% xlabel('x distance from LKP (m)') 
% ylabel('y distance from LKP (m)')

end