function [new_figure] = obstaclesplot(obstacles_check, num_obstacles)
load ./data/obstacles/obstacles1
obstacles = obstacles(1:num_obstacles,:);

switch obstacles_check
    case 'Include'
        for i = 1:size(obstacles,1);
            obstacles{i,1} = obstacles{i,1} + obstacles{i,3};
            obstacles{i,2} = obstacles{i,2} + obstacles{i,4};
        end
        
        new_figure = figure;
        for i = 1:size(obstacles,1); %plot obstacles
            plot(obstacles{i,1}, obstacles{i,2},'k');
            hold on
        end
    otherwise
        new_figure = figure;
end



end