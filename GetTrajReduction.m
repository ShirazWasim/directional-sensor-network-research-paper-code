function [traj_xdata, traj_ydata] = GetTrajReduction(xdata, ydata, times, startTime)

indices = times >= startTime;
traj_xdata = xdata(:, indices);
traj_ydata = ydata(:, indices);

end