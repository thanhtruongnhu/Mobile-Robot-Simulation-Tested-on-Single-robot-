% =======================Single Robot Behavior Control===================
% 
% Instructed by:    MSc. Pham Duy Hung
% Coded by:         Thanh Truong Nhu, BKU
% Date:             September 7, 2018
% Location:         University of Engineering and Technology
% If you counter any problem regarding this code, please don't hesitate to
% contact me via my email: thanhtruong.kyle@gmail.com
% 
% =========================================================================
% Thank you & Happy coding !
% Code starts below this line:
% =========================================================================
clear
clc
close all
global L R dt
global r_a r_nm count
%% Simulation Parameters (Need to be simplified) (Need to input into an Excel file)
% Waypoints should be input by excel files for different robots(id): is an
% N-by-(2*nb) matrix where N is number of waypoints for each robot (each
% robot must have the same number of waypoints)
waypoints = [1.5    1.5;  % If we have more robot for example 3 robots: robot 1 [column 1&2]...
    %              1.25    1.75 ;  % robot 2 [column 3&4], robot 3 [column 5,6];
    %              5.25    8.25 ;
    %              7.25    8.75 ;
    %              11.75   10.75;
    %     0.997  0.877];
    0.2  0.2
    0.5  0.1];
[nw,n] = size(waypoints);     %Where m = number of waypoints, (n/2)= nb = number of robots
nb = (n/2);

F = zeros(nb,5);               % Create a matrix with row number = nb & col number = [x(1) x(2) angle v(1) v(2)]

% Intiating Robot class: Assign intial Pose and Velocity for each robot
for  id  = 1:nb
    initPose = [waypoints(1,id*2-1), waypoints(1,id*2) 0];  % Initial pose (x y theta)
    initVec  = [0,0];                                       %Assume that initial control vector equal 0
    F(id,:)  = [initPose, initVec];
end

Robot    = Robot(waypoints,F,nb,nw);

dt = 0.14;               % Sample time [s]
tVec = 0:dt:5000;        % Time array

% Define Vehicle
R = 0.0325;              % Wheel radius [m] % Excel  %3.25 cm = 0.0325 m
L = 0.17;                % Wheelbase [m]    % Excel  %17 cm = 0.17 m

% Define Avoidance radius & No-manipulate radius
r_a  = 0.15;    % Excel  d_boy=13.5 cm => dist.min = 13.5 cm
r_nm = 0.4;     % Excel

% Counting variable for each robot's waypoints
count = repmat(2,1,nb); % [Robot 1, Robot 2,..., Robot nb] Create array with elements equal 2 (Assume that robot start from Waypoint no.1 so we ignore Waypoint no.1)
stopflag = zeros(1,nb); % [Robot 1, Robot 2,..., Robot nb] 0:Continue loop, 1:Stop loop for that robot
%% Algorithm Parameters
% (Assume that time step Delta t = 0.05 s, v = 0.4 m/s => s = 0.02 m)
colorVec        = hsv(nb);                  % Generate nb hue-saturation-value color map for each robot

% Decode parameters
startMarker_in = 255;
endMarker_in = 255;
size = 8; %Create a vector to store 4 bytes
startMarker_out = 179;
endMarker_out   = 179;

newData = false;
recvInProgress = false;
ndx = 1;
receivedBytes = zeros(1,6); %There are 6 content Bytes
numReceived = 0;

%% Observation Variables Set-up
w_observe = zeros(numel(tVec),2);    %column 1 - wl; column 2 - wr
w_treated_observe = zeros(numel(tVec),2);    %column 1 - wl; column 2 - wr
angle_observe = zeros(numel(tVec),3); %col1 - init angle; col2 - desired angle; col3 -theta (control angle)

%% Reset all left-over settings of all Serial ports
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

%% Set up Plot
% hold on
% figure (1)
%% Set up COM ports
chn1 = serial('COM8');
chn2 = serial('COM6');
chn1.InputBufferSize    = 1024;     %bytes
chn2.OutputBufferSize   = 1024;     %bytes

fopen(chn1);
fopen(chn2);
disp('Start reading')
readasync(chn1)
flushinput (chn1); %Erase Input Buffer (Erase buffer received from Rasp)
flushoutput(chn2);

%% Inititiate Filter Matrix
filterAngle = zeros(1,10);
while chn1.BytesAvailable < 8 % > 0 %
    waitfor(0.001);
end

for i = 10:(-1):1 % 10:Oldest data , 1:Newest data
    A = fread(chn1,size,'uint8');
    if A(1) == startMarker_in && A(size) == endMarker_in
        filterAngle(i) = deg2rad(A(6)*254 + A(7));
    end
end

%% MAIN LOOP
for p_idx = 1: numel(tVec) % pose index
    
    tic; %Start timer
    %% Receiving data from Tracking System (Reading data from 9xstream modem)
    BytesAvailable = chn1.BytesAvailable; % (observation only)
    
    
    while (newData == false)
        rb = fread(chn1,1,'uint8');
        if recvInProgress == true
            if rb ~= endMarker_in
                receivedBytes(ndx) = rb;
                ndx = ndx + 1;
            else
                if ndx ~= 1
                    recvInProgress = false;
                    numReceived = ndx; % (observation only)
                    ndx = 1;
                    newData = true;
                end
            end
        elseif rb == startMarker_in
            recvInProgress = true;
        end
    end
    
    if  newData == true
        Robot(1).x(1) = (receivedBytes(1)*254 + receivedBytes(2))/1000; %Robot(1).x(1)
        Robot(1).x(2) = (receivedBytes(3)*254 + receivedBytes(4))/1000; %Robot(1).x(2)
        angle = deg2rad(receivedBytes(5)*254 + receivedBytes(6));
        [filterAngle,Robot(1).angle] = UpdateFilterMatrix(filterAngle,angle);
        newData = false;
        angle_observe(p_idx,1) = Robot(1).angle; %Init angle (observation only)
    end
    
    %% Calculating Control Vector and Wheels' Speed
    Robot = ControlVector(Robot,nb,stopflag);
    [Robot,angle_observe(p_idx,3)] = DiffWheelKinematics(Robot,nb);
    [Robot,stopflag] = StopCondition(Robot,stopflag,nb);
    w_observe(p_idx,:) = Robot(1).w;  % (observation only)
    angle_observe(p_idx,2) = Robot(1).angle; %Desired angle (observation only)
    
    %% Trasmitting data to robots (Writing data to 9xstream modem)
    [wl,wr] = WheelSpeedModify(Robot(1).w(1),Robot(1).w(2));
    w_treated_observe(p_idx,:) = [wl,wr]; % (observation only)
    [wl_quo, wl_rem, wl_sign, wr_quo, wr_rem, wr_sign] = ValueConverter4Transmit(wl,wr);
    toc; %Stop timer
    
    disp('Start Writing')
    fwrite(chn2,[startMarker_out wl_quo wl_rem wl_sign wr_quo wr_rem wr_sign endMarker_out],'async');
    pause(0.1) % Timer for 20Hz control rate
    
end








