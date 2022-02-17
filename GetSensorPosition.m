function [pos] = GetSensorPosition(traj_xdata,traj_ydata,dens_res)
%% Converting target x-y matrix data into vectors (preparing for ksdensity input):
x1 = traj_xdata';
y1 = traj_ydata';

Min_radiusx = min(abs(traj_xdata),[],'all');
Max_radiusx = max(abs(traj_xdata),[],'all');
Min_radiusy = min(abs(traj_ydata),[],'all');
Max_radiusy = max(abs(traj_ydata),[],'all');

min_radius = min(Min_radiusx,Min_radiusy);
max_radius = max(Max_radiusx,Max_radiusy);

x = x1(:);
y = y1(:);
xy = horzcat(x,y);

%% Creating Rectangular Grid of Evaluation Points
% gridx1 = -5000:dens_res:5000;
% gridx2 = -5000:dens_res:5000;
% [x1,x2] = meshgrid(gridx1, gridx2);
% x1 = x1(:);
% x2 = x2(:);
% xi = [x1 x2];

%% Creating Circular Grid of Evaluation Points
M = 50;
N = 100;
R1 = min_radius ; % inner radius 
R2 = max_radius;  % outer radius
nR = linspace(R1,R2,M) ;
nT = linspace(0,2*pi,N) ;
[R, T] = meshgrid(nR,nT) ;

% Convert grid to cartesian coordintes
X = R.*cos(T); 
Y = R.*sin(T);
[m,n]=size(X);

x1 = X(:);
x2 = Y(:);
xi = [x1 x2];

% Plot grid
% figure
% set(gcf,'color','w') ;
% axis equal
% axis off
% box on
% hold on
% % Plot internal grid lines
% for i=1:m
%     plot(X(i,:),Y(i,:),'k','linewidth',1.5); 
% end
% for j=1:n
%     plot(X(:,j),Y(:,j),'k','linewidth',1.5); 
% end

%% Generating PDF and evaluating at assigned grid points
[f,~] = ksdensity(xy,xi);


%% Extracting optimal position
[~,row] = max(f);
pos = xi(row,:);

%% Plotting 
figure(102)
ksdensity(xy,xi);
end