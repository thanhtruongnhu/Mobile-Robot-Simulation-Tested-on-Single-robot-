clear
clc
close all
global L R dt
global r_a r_nm count
%% Simulation Parameters (Need to be simplified) (Need to input into an Excel file)
% Waypoints should be input by excel files for different robots(id): is an
% N-by-(2*nb) matrix where N is number of waypoints for each robot (each
% robot must have the same number of waypoints)
waypoints = [0.7543    0.5966 ;  % If we have more robot for example 3 robots: robot 1 [column 1&2]...
             1.25    1.75 ;  % robot 2 [column 3&4], robot 3 [column 5,6];
             5.25    8.25 ;  
             7.25    8.75 ;
             11.75   10.75;
             0   0];
[nw,n] = size(waypoints);     %Where m = number of waypoints, (n/2)= nb = number of robots  

if (n/2) < 1                 %If there is only one robot, assign n = 1
    nb = 1;
else
    nb = (n/2);
end

F = zeros(nb,5);               % Create a matrix with row number = nb & col number = [x(1) x(2) angle v(1) v(2)]

% Intiating Robot class: Assign intial Pose and Velocity for each robot
for  id  = 1:nb
initPose = [waypoints(1,id*2-1), waypoints(1,id*2) 0];  % Initial pose (x y theta)
initVec  = [0,0];                                       %Assume that initial control vector equal 0
F(id,:)  = [initPose, initVec];
end

Robot_2    = Robot_2(waypoints,F,nb,nw); 

dt = 0.05;             % Sample time [s]
tVec = 0:dt:500;        % Time array        

% Define Vehicle
R = 0.0325;                % Wheel radius [m] % Excel
L = 0.17;                % Wheelbase [m]    % Excel

% Define Avoidance radius & No-manipulate radius
r_a  = 0.15;    % Excel  d_boy=13.5 cm => dist.min = 13.5 cm
r_nm = 0.4;     % Excel

% Counting variable for each robot's waypoints
count = repmat(2,1,nb); % [Robot 1, Robot 2,..., Robot nb] Create array with elements equal 2
stopflag = zeros(1,nb); % [Robot 1, Robot 2,..., Robot nb] 0:Continue loop, 1:Stop loop for that robot

%% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Main: Assume that differential time period (dt) equal 1 second (s)
% (Assume that time step Delta t = 0.05 s, v = 0.4 m/s => s = 0.02 m)
colorVec        = hsv(nb);                  % Generate nb hue-saturation-value color map for each robot
% ControlRate     = robotics.Rate(1/dt);    % Adjust movement speed (f = 1/0.05 Hz)

% hold on
% figure (1)
% xlim([0 13])
% ylim([0 13])

for p_idx = 1: numel(tVec) % pose index          
        Robot_2 = ControlVector_2(Robot_2,nb,stopflag); 
        Robot_2 = DiffWheelKinematics_3(Robot_2,nb);          
        
        viz([Robot_2(1).x(1);Robot_2(1).x(2);Robot_2(1).angle],waypoints)
%         PlotTrajectory(Robot_2,nb,colorVec); % Update visualization
        [Robot_2,stopflag] = StopCondition(Robot_2,stopflag,nb); 
%         waitfor(ControlRate);
        pause(dt)
end
% hold off
    
    
    
    
    
