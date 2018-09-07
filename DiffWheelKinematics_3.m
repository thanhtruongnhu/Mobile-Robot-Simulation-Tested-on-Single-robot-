function [Robot,theta] = DiffWheelKinematics_3(Robot,nb)
global L R dt
f = 1; % Downgrade/upgrade the effect (importance) of theta

for id = 1:nb
% Tinh toan goc lech so voi X-axis (so voi X_R)
if  norm(norm(Robot(id).v))>0 
    u=[1,0];
    angle=acos(dot(Robot(id).v,u)/norm(Robot(id).v)); 
else      
    angle=Robot(id).angle; 
end

if Robot(id).v(2)<0 
    angle=-angle;
end
%  angle_old= rad2deg(Robot(id).angle) %[Debug only]
%  angle_new= rad2deg(angle) %[Debug only]
% 
%  theta= rad2deg(diffangle(angle,Robot(id).angle)) %[Debug only]
theta= diffangle(angle,Robot(id).angle);

if abs(theta) <= (10*pi/180)  % Allowable angle error = 10 deg
    omega = 0;
else
    omega=theta/dt; % van toc goc 
end

%% 2-wheel angular velocity
Robot(id).w(1)=(norm(Robot(id).v)+f*omega*L/2)/R; %left wheel. (Rad/s)
Robot(id).w(2)=(norm(Robot(id).v)-f*omega*L/2)/R; %right wheel.(Rad/s)

%% Pose
% id: index of robot
% w=[wl;wr]: angular velocity of left and right wheel.
% L: distance betweem two wheels
% R: wheel radius
% dt: time-step

% As=[cos(angle),cos(angle);
%     sin(angle),sin(angle);
%     -2/L, 2/L];
% S = (R/2) * dt * As *[Robot(id).w(2);Robot(id).w(1)];
%% Position of robot
% Robot(id).x(1)  =Robot(id).x(1)+S(1);
% Robot(id).x(2)  =Robot(id).x(2)+S(2);
%% orientation of robot
% Robot(id).angle =Robot(id).angle+S(3); %rad
end
end