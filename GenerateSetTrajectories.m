function [traj_xdata_test_comp, traj_ydata_test_comp] = GenerateSetTrajectories(nT, vTmean, vTstd, times, timeStep, startTime, endTime)
% Creates and saves a set of trajectories according to the parameters specified above
% This was used to create 20 sets of trajectories for use when testing

num_set = 20;  % Number of sets of trajectories

for num_sets = 1:num_set
    
    [traj_xdata_test_comp{num_sets} traj_ydata_test_comp{num_sets}] = GetTraj(nT, vTmean, vTstd, times, timeStep, startTime, endTime);
    
end

% save(sprintf('%s\\data\\Trajectories\\traj_data%u.mat', pwd, startTime),'traj_xdata_test_comp', 'traj_ydata_test_comp');
save(sprintf('%s\\traj_data%u.mat', pwd, startTime),'traj_xdata_test_comp', 'traj_ydata_test_comp');

end