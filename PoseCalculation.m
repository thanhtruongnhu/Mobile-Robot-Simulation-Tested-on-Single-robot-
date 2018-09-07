function [x_o,y_o,angle] = PoseCalculation (x,angle,v,theta)

% Position of robot
x_o =  x(1)+ v(1);
y_o =  x(2)+ v(2);

% orientation of robot
angle =  angle+theta; % new angle of robot

