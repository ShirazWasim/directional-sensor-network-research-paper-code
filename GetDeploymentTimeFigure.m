function [deployment_time_figure] = GetDeploymentTimeFigure(circ_xdata, circ_ydata, circindex, robot_pos_x, robot_pos_y, robot_pos_time, dist, d_time) 

 % Plot
    dist1x =  [circ_xdata(circindex,:) robot_pos_x(1,1)];    
    dist1y =  [circ_ydata(circindex,:) robot_pos_y(1,1)];   
    dist2x =  [circ_xdata(circindex,:) robot_pos_x(2,1)];    
    dist2y =  [circ_ydata(circindex,:) robot_pos_y(2,1)];    
    dist3x =  [circ_xdata(circindex,:) robot_pos_x(3,1)];    
    dist3y =  [circ_ydata(circindex,:) robot_pos_y(3,1)];
    
    robot_pos_x = round(robot_pos_x);
    robot_pos_y = round(robot_pos_y);
    
    deployment_time_figure = figure;
    scatter(circ_xdata(circindex,:),circ_ydata(circindex,:),'kx')
    hold on
    scatter(robot_pos_x',robot_pos_y','ms')
    hold on
    plot(dist1x',dist1y','k:')
    hold on
    plot(dist2x',dist2y','k:')
    hold on
    plot(dist3x',dist3y','k:')
    hold on
    txt = ('\leftarrow Query Position')
    text(circ_xdata(circindex,:),circ_ydata(circindex,:), txt,'FontSize',8, 'FontName', 'Times New Roman');
    txt = (sprintf('Robot 1 \n (%u,%u), t_{1} = %us \n Distance to Deployment: %um \n Earliest Deployment Time: %us',robot_pos_x(1,1),robot_pos_y(1,1), round(robot_pos_time(1,1)), round(dist(1,1)), round(d_time(1,1))))
    text(robot_pos_x(1,1),robot_pos_y(1,1), txt,'FontSize',8, 'FontName', 'Times New Roman');
    txt = (sprintf('Robot 2 \n (%u,%u), t_{2} = %us \n Distance to Deployment: %um \n Earliest Deployment Time: %us',robot_pos_x(2,1),robot_pos_y(2,1), round(robot_pos_time(2,1)), round(dist(2,1)), round(d_time(2,1))))
    text(robot_pos_x(2,1),robot_pos_y(2,1), txt,'FontSize',8, 'FontName', 'Times New Roman');
    txt = (sprintf('Robot 3 \n (%u,%u), t_{3} = %us \n Distance to Deployment: %um \n Earliest Deployment Time: %us',robot_pos_x(3,1),robot_pos_y(3,1), round(robot_pos_time(3,1)), round(dist(3,1)), round(d_time(3,1))))
    text(robot_pos_x(3,1),robot_pos_y(3,1), txt,'FontSize',8, 'FontName', 'Times New Roman');
    xlabel('x distance from LKP (m)')
    ylabel('y distance from LKP (m)')
    set(gca,'FontSize',8, 'FontName', 'Times New Roman')
    box on
    axis equal
    
    end