%%  3D Splines Fit
%
%   This script is to be used to connect waypoints through a spline curve
%   
%   This script is part of the software package accompanying the
%   "Autonomous Mobile Robot Design" course
%
%   Author: 
%   Kostas Alexis (kalexis@unr.edu)
%

%   Define how many points and generate them (can also be manual)
NumPoints = 20; 
t = linspace(0,8*pi,npts);
z = linspace(-1,1,npts);
omz = sqrt(1-z.^2);
xyz = [cos(t).*omz; sin(t).*omz; z];

%   show the points
plot3(xyz(1,:),xyz(2,:),xyz(3,:),'ro','LineWidth',6);

ax = gca;
ax.XTick = [];
ax.YTick = [];
ax.ZTick = [];
box on

%   perform the spline fit
spline_curve_ = cscvn(xyz(:,[1:end 1]));

%   and plot the result
hold on
fnplt(spline_curve_,'b',4)
spline_points_ = fnplt(spline_curve_,'b',2);
plot3(spline_points_(1,:),spline_points_(2,:),spline_points_(3,:),'g','LineWidth',2)
hold off