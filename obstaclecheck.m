function [traj_xdata_save, traj_ydata_save] = obstaclecheck(traj_xdata, traj_ydata, vTmean, vTstd, times, timeStep, startTime, endTime,adaptive, LKP_x, LKP_y, num_obstacles)

nT = size(traj_xdata,1);

load ./data/obstacles/obstacles1
obstacles = obstacles(1:num_obstacles,:);

for i = 1:size(obstacles,1); 
    obstacles{i,1} = obstacles{i,1} + obstacles{i,3}; 
    obstacles{i,2} = obstacles{i,2} + obstacles{i,4};
end

nTa = 1;
traj_xdata_save = zeros(1,size(traj_xdata,2));
traj_ydata_save = zeros(1,size(traj_xdata,2));

while nTa > 0
    for i = 1:size(obstacles,1);
        in = inpolygon(traj_xdata,traj_ydata,obstacles{i,1},obstacles{i,2});
        in = sum(in,2) > 0;
        
        traj_xdata = traj_xdata.*~in;
        traj_ydata = traj_ydata.*~in;
        
        traj_xdata(traj_xdata(:,1)==0,:) = [] ;
        traj_ydata(traj_ydata(:,1)==0,:) = [] ;
    end
    
    traj_xdata_save = vertcat(traj_xdata_save,traj_xdata);
    traj_ydata_save = vertcat(traj_ydata_save,traj_ydata);
    
    nTa = nT - size(traj_xdata_save,1) + 1;
    
    switch adaptive
        case 'Yes'
            [traj_xdata traj_ydata] = GetTraj(nTa, vTmean, vTstd, times, timeStep, 0, endTime, LKP_x, LKP_y);
            
        case 'No'
            [traj_xdata traj_ydata] = GetTraj(nTa, vTmean, vTstd, times, timeStep, 0, endTime, LKP_x, LKP_y);
    end
    
end

traj_xdata_save(1,:) = [];
traj_ydata_save(1,:) = [];

% plot(traj_xdata_save',traj_ydata_save');
% hold on
% 
% for i = 1:size(obstacles,1); %plot obstacles
%     plot(obstacles{i,1}, obstacles{i,2},'k');
%     hold on
% end

end