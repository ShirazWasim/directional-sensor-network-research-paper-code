function [int, column] = IntCheck(traj_xdata,traj_ydata,sensor_xpos,sensor_ypos)
% A function to determine if multiple intput trajectories (in matrix form) are intersected
% by the input sensor poses (poses are entered as lines, for directional model:
% the multiple line segments that make up the directional sensor are input
% one by one). Output is a binary vector, with 1 representing a trajectory
% that is intersected.


%% Defining paramaters and setting up matrices for operations
x1 = traj_xdata(:,1:end-1);
x2 = traj_xdata(:,2:end);
y1 = traj_ydata(:,1:end-1);
y2 = traj_ydata(:,2:end);

x3 = repmat(sensor_xpos(1,1),size(x1,1),size(x1,2));
x4 = repmat(sensor_xpos(1,2),size(x1,1),size(x1,2));

y3 = repmat(sensor_ypos(1,1),size(y1,1),size(y1,2));
y4 = repmat(sensor_ypos(1,2),size(y1,1),size(y1,2));

%% Intersection equation 'http://www.cs.swan.ac.uk/~cssimon/line_intersection.html'
t_a_num = (y3-y4).*(x1-x3) + (x4-x3).*(y1-y3);
t_a_denom =(x4-x3).*(y1-y2) - (x1-x2).*(y4-y3);

t_b_num = (y1-y2).*(x1-x3) + (x2-x1).*(y1-y3);
t_b_denom = (x4-x3).*(y1-y2) - (x1-x2).*(y4-y3);

t_a = t_a_num ./ t_a_denom;
t_b = t_b_num ./ t_b_denom;

%% Checking if the values of t_a and t_b satisfy the inequalities: 0 <= t_a <= 1 and 0 <= t_b <= 1
t_a_check = and(0 <= t_a , t_a <= 1);
t_b_check = and(0 <= t_b , t_b <= 1);

% Finding the ones that satisfy both inequalities
t = t_a_check + t_b_check;
t_sum = t > 1;

%% Determine points of intersection
p_intx = (x1 + t_a.*(x2-x1)).*t_sum;
p_inty = (y1 + t_a.*(y2-y1)).*t_sum;

[~,column] = find(p_intx);

%% Determining which trajectories (rows) are intersected

% Summing the rows, if a trajectory is intersected, it will have a non-zero
% value and its magnitude describes the number of points of intersection
% for that trajectory
t_sum = sum(t_sum,2);

int = t_sum > 0;

end