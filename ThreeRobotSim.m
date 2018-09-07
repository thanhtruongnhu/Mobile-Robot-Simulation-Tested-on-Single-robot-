clear
clc
close all
global L R dt waypoints r_a r_nm
%% Simulation Parameters (Need to be simplified) (Need to input into an Excel file)
% Waypoints should be input by excel files for different robots(id): is an
% N-by-(2*nb) matrix where N is number of waypoints for each robot (each
% robot must have the same number of waypoints)
waypoints = [2.00    1.00  1  2  1  3  1  1;  % If we have more robot for example 3 robots: robot 1 [column 1&2]...
%              1.25    1.75 ;  % robot 2 [column 3&4], robot 3 [column 5,6];
%              5.25    8.25 ;  
%              7.25    8.75 ;
%              11.75   10.75;
             12.00   10.00  10  12  9  10  13 10];
[m,n] = size(waypoints);     %Where m = number of waypoints, (n/2)= nb = number of robots  

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

Robot    = Robot(F);

% Initiating Pose matrix: is a (3*nb)-by-M matrix representing the robot
% pose where M is the number of pose
% (Assume that time step Delta t = 0.05 s, v = 0.4 m/s => s = 0.02 m)
sampleTime = 0.05;             % Sample time [s]
tVec = 0:sampleTime:50;        % Time array
dt = sampleTime;           

% Define Vehicle
R = 0.2;                % Wheel radius [m] % Excel
L = 0.7;                % Wheelbase [m]    % Excel

% Define Avoidance radius & No-manipulate radius
r_a  = 0.15;    % Excel  d_boy=13.5 cm => dist.min = 13.5 cm
r_nm = 0.4;     % Excel

%% Main: Assume that differential time period (dt) equal 1 second (s)
% (Assume that time step Delta t = 0.05 s, v = 0.4 m/s => s = 0.02 m)
colorVec        = hsv(nb);                  % Generate nb hue-saturation-value color map for each robot
ControlRate     = robotics.Rate(1/0.05);    % Adjust movement speed (f = 1/0.05 Hz)


hold on
figure (1)
xlim([0 13])
ylim([0 13])
for i = 2:m
    redirect_flag = 0;
    for p_idx = 1: numel(tVec) % pose index    
        Robot = ControlVector(Robot,nb,i);
         for id = 1:nb            
            Robot(id)   = DiffWheelKinematics(Robot(id));
            
            % Update visualization
            plot(Robot(id).x(1),Robot(id).x(2),'Marker','+','Color',colorVec(id,:))
            drawnow
            waitfor(ControlRate);
            
            if  norm (Robot(id).x - waypoints(i,id*2-1 : id*2)) <= 0.01 % Stop & Redirect condition: r_ij_stop = 0.01m
                redirect_flag = 1;
                break            
            end
        end
            if redirect_flag == 1 % Stop & Redirect condition (cont.)
                break
            end
    end
end
hold off
    
    
    
    
    
