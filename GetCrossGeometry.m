function [cross_square_xdata cross_square_ydata,cross_xcoord,cross_ycoord] = GetCrossGeometry(x_center,y_center,cross_length)
%% Cross Generation

% Left side of cross
x_left = x_center - cross_length;
y_left = y_center;

% Right side of cross
x_right = x_center + cross_length;
y_right = y_center;

% Top side of cross
x_top = x_center;
y_top = y_center + cross_length;

% Bottom side of cross
x_bottom = x_center;
y_bottom = y_center - cross_length;


cross_xcoord = [x_center, x_top, x_bottom, x_right, x_left];
cross_ycoord = [y_center, y_top, y_bottom, y_right, y_left];


%% Box Generation
% Center Square Data Points
C_bottom_leftx = x_center - 0.5*cross_length; C_bottom_rightx = x_center + 0.5*cross_length;
C_bottom_lefty = y_center - 0.5*cross_length; C_bottom_righty = y_center - 0.5*cross_length;

C_top_leftx = x_center - 0.5*cross_length; C_top_rightx = x_center + 0.5*cross_length;
C_top_lefty = y_center + 0.5*cross_length; C_top_righty = y_center + 0.5*cross_length;

center_xcoord = [C_bottom_leftx, C_top_leftx, C_top_rightx, C_bottom_rightx, C_bottom_leftx];
center_ycoord = [C_bottom_lefty,  C_top_lefty,  C_top_righty, C_bottom_righty, C_bottom_lefty];


% Top Square Data Points
T_bottom_leftx = x_center - 0.5*cross_length; T_bottom_rightx = x_center + 0.5*cross_length;
T_bottom_lefty = y_center + 0.5*cross_length; T_bottom_righty = y_center + 0.5*cross_length;

T_top_leftx = x_center - 0.5*cross_length; T_top_rightx = x_center + 0.5*cross_length;
T_top_lefty = y_center + 1.5*cross_length; T_top_righty = y_center + 1.5*cross_length;

top_xcoord = [T_bottom_leftx, T_top_leftx, T_top_rightx, T_bottom_rightx, T_bottom_leftx];
top_ycoord = [T_bottom_lefty,  T_top_lefty,  T_top_righty, T_bottom_righty, T_bottom_lefty];


% Bottom Square Data Points
B_bottom_leftx = x_center - 0.5*cross_length; B_bottom_rightx = x_center + 0.5*cross_length;
B_bottom_lefty = y_center - 1.5*cross_length; B_bottom_righty = y_center - 1.5*cross_length;

B_top_leftx = x_center - 0.5*cross_length; B_top_rightx = x_center + 0.5*cross_length;
B_top_lefty = y_center - 0.5*cross_length; B_top_righty = y_center - 0.5*cross_length;

bottom_xcoord = [B_bottom_leftx, B_top_leftx, B_top_rightx, B_bottom_rightx, B_bottom_leftx];
bottom_ycoord = [B_bottom_lefty,  B_top_lefty,  B_top_righty, B_bottom_righty, B_bottom_lefty];


% Right Square Data Points
R_bottom_leftx = x_center + 0.5*cross_length; R_bottom_rightx = x_center + 1.5*cross_length;
R_bottom_lefty = y_center - 0.5*cross_length; R_bottom_righty = y_center - 0.5*cross_length;

R_top_leftx = x_center + 0.5*cross_length; R_top_rightx = x_center + 1.5*cross_length;
R_top_lefty = y_center + 0.5*cross_length; R_top_righty = y_center + 0.5*cross_length;

right_xcoord = [R_bottom_leftx, R_top_leftx, R_top_rightx, R_bottom_rightx, R_bottom_leftx];
right_ycoord = [R_bottom_lefty,  R_top_lefty,  R_top_righty, R_bottom_righty, R_bottom_lefty];

% Left Square Data Points
L_bottom_leftx = x_center - 1.5*cross_length; L_bottom_rightx = x_center - 0.5*cross_length;
L_bottom_lefty = y_center - 0.5*cross_length; L_bottom_righty = y_center - 0.5*cross_length;

L_top_leftx = x_center - 1.5*cross_length; L_top_rightx = x_center - 0.5*cross_length;
L_top_lefty = y_center + 0.5*cross_length; L_top_righty = y_center + 0.5*cross_length;

left_xcoord = [L_bottom_leftx, L_top_leftx, L_top_rightx, L_bottom_rightx, L_bottom_leftx];
left_ycoord = [L_bottom_lefty, L_top_lefty, L_top_righty, L_bottom_righty, L_bottom_lefty];

% Concatenate data points
cross_square_xdata = vertcat(center_xcoord, top_xcoord, bottom_xcoord, right_xcoord, left_xcoord);
cross_square_ydata = vertcat(center_ycoord, top_ycoord, bottom_ycoord, right_ycoord, left_ycoord);


%% Plotting

% scatter(cross_xcoord,cross_ycoord,'b')
% hold on
% plot(top_xcoord,top_ycoord,'k')
% hold on
% plot(bottom_xcoord,bottom_ycoord,'k')
% hold on
% plot(center_xcoord,center_ycoord,'k')
% hold on
% plot(right_xcoord,right_ycoord,'k')
% hold on
% plot(left_xcoord,right_ycoord,'k')

end 






