function [x_coord, y_coord] = GetDirectionalSectorGeometry(sensor_x_pos_mul, sensor_y_pos_mul, sector_angle, sens_range,curve_approx)

%% Creating Directional Sensor Sector Geometry
sensor_x_local = sensor_x_pos_mul - sensor_x_pos_mul(:,1);
sensor_y_local = sensor_y_pos_mul - sensor_y_pos_mul(:,1);

if sector_angle > 0
    start_angle = cart2pol(sensor_x_local(:,2), sensor_y_local(:,2)) - sector_angle/2;
    end_angle = cart2pol(sensor_x_local(:,2), sensor_y_local(:,2)) + sector_angle/2;
    
    for sensor_ind = 1:size(sensor_x_local,1)
        thetas(sensor_ind,:) = start_angle(sensor_ind,1): curve_approx: end_angle(sensor_ind,1);
        x_coord(sensor_ind,:) = sens_range.*cos(thetas(sensor_ind,:));
        y_coord(sensor_ind,:) = sens_range.*sin(thetas(sensor_ind,:));
        
        x_coord(sensor_ind,:) = x_coord(sensor_ind,:) + sensor_x_pos_mul(sensor_ind,1);
        y_coord(sensor_ind,:) = y_coord(sensor_ind,:) + sensor_y_pos_mul(sensor_ind,1);
        
    end
    
    x_coord = horzcat(sensor_x_pos_mul(:,1),x_coord,sensor_x_pos_mul(:,1));
    y_coord = horzcat(sensor_y_pos_mul(:,1),y_coord,sensor_y_pos_mul(:,1));
    
elseif sector_angle == 0
    x_coord = sensor_x_pos_mul;
    y_coord = sensor_y_pos_mul;
end

end